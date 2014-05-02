/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */


/** \file dns.c
 *
 * DNS Lookup of ach channels via SRV records
 *
 *  \author Neil T. Dantam
 */

#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include <netinet/in.h>
#include <arpa/nameser.h>
#include <resolv.h>

#include "ach.h"
#include "ach_impl.h"

/** Wire format of dns packet header */
struct dns_packet {
    uint16_t id;
    uint16_t flags;
    uint16_t qdcount;
    uint16_t ancount;
    uint16_t nscount;
    uint16_t arcount;
    uint8_t data[1];
};

/** Wire format of dns resource record header */
struct res_rec {
    uint16_t type;
    uint16_t clazz;
    uint32_t ttl;
    uint16_t rdlen;
    uint8_t data[1];
};

/** Wire format of dns SRV record data */
struct srv_data {
    uint16_t pri;
    uint16_t weight;
    uint16_t port;
    uint8_t target[1];
};

enum ach_status
parse_dns_srv ( unsigned char *buf, size_t len,
                char *phost, size_t host_len, int *pport )
{
    if( len < sizeof(struct dns_packet) )
        return ACH_OVERFLOW;

    struct dns_packet *pkt = (struct dns_packet*) buf;
    uint8_t *msgend =  buf + len;

    /* decode header */
    /* uint16_t id = ntohs(pkt->id); */
    uint16_t qdcount = ntohs(pkt->qdcount);
    uint16_t ancount = ntohs(pkt->ancount);
    /* uint16_t nscount = ntohs(pkt->nscount); */
    /* uint16_t arcount = ntohs(pkt->arcount); */

    /* Skip question section */
    /* TODO: maybe validate question? */
    uint8_t *pkt_question = pkt->data;
    uint8_t *pkt_answer = pkt_question;
    for (size_t i = qdcount; i > 0; --i) {
        int size;
        if ((size = dn_skipname(pkt_answer, msgend)) < 0)
            return ACH_BAD_HEADER;
        pkt_answer = pkt_answer + size + QFIXEDSZ;
    }

    /* Process Answer */
    uint8_t *rr_ans = pkt_answer;
    int minpri = INT_MAX;
    for( size_t i = ancount; i > 0; --i ) {
        char name[host_len-1];
        int size = dn_expand(buf, msgend, rr_ans, name, (int)host_len-1);
        if (size < 0) {
            ach_set_errstr("dn_expand() failed");
            return ACH_BAD_HEADER;
        }
        struct res_rec *nrec = (struct res_rec*)(rr_ans + size);
        struct res_rec hrec;
        hrec.type = (uint16_t)ntohs( nrec->type );
        hrec.clazz = (uint16_t)ntohs( nrec->clazz );
        hrec.ttl = (uint32_t)ntohl( nrec->ttl );
        hrec.rdlen = (uint16_t)ntohs( nrec->rdlen );
        uint8_t *rdata = nrec->data;

        uint8_t *rr_end = rdata + hrec.rdlen;
        if( rr_end > msgend ) return ACH_BAD_HEADER;

        if( T_SRV == hrec.type ) {
            struct srv_data *ndata = (struct srv_data*)rdata;
            uint16_t pri    = (uint16_t)ntohs( ndata->pri );
            uint16_t port    = (uint16_t)ntohs( ndata->port );
            if( pri < minpri ) {
                *pport = port;
                minpri = pri;
                int size2 = dn_expand( buf, msgend, ndata->target, phost, (int)host_len );
                if( size2 < 0 ) {
                    ach_set_errstr("dn_expand() on target failed");
                    return ACH_BAD_HEADER;;
                }
            }
        }
    }
    if( INT_MAX == minpri ) {
        ach_set_errstr( "No SRV record found in DNS packet" );
        return ACH_ENOENT;
    } else {
        return ACH_OK;
    }

    /* TODO: Check additional data section for address */
}

#define ANSWER_SIZE 4096

enum ach_status
ach_srv_search( const char *channel, const char *domain,
                char *host, size_t host_len,
                int *port )
{
    ach_set_errstr( "" );

    /* Create Query */
    /* RFC 6763: channel is registered as a service "subtype" */
    const char srv_type[] = "._sub._ach._tcp.";
    char srvname[strlen(channel) + strlen(srv_type) + strlen(domain) + 2];
    srvname[0] = '_';
    srvname[1] = '\0';
    strcat(srvname, channel);
    strcat(srvname, srv_type);
    strcat(srvname, domain);

    /* Perform Query */
    unsigned char answer[ANSWER_SIZE] = {0};
    int len_a = res_search( srvname, C_IN, T_SRV, answer, ANSWER_SIZE );
    if( len_a < 0 ) {
        ach_set_errstr( "res_search() failed\n" );
        return ACH_FAILED_SYSCALL;
    }

    /* Process Result */
    return parse_dns_srv( answer, (size_t)len_a, host, host_len, port );
}
