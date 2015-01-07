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
#include <poll.h>

#include <netinet/in.h>
#include <arpa/nameser.h>
#include <resolv.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "ach.h"
#include "ach/private_posix.h"

int
mdns_res_search( const char *dname, int clazz, int type,
                 unsigned char *answer, int anslen);

int dns_query(int sock, const struct sockaddr *addr, size_t addr_size,
              const void *pkt, size_t pkt_len,
              void *ans, size_t ans_len);


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
    /* RFC 6763 service "subtype" are bogus */
    const char srv_type[] = "._ach._tcp.";
    char srvname[strlen(channel) + strlen(srv_type) + strlen(domain) + 1];
    srvname[0] = '\0';
    strcat(srvname, channel);
    strcat(srvname, srv_type);
    strcat(srvname, domain);

    /* Perform Query */
    unsigned char answer[ANSWER_SIZE] = {0};
    int len_a;
    if( 0 == strcasecmp("local", domain) ) {
        /* Magically do mdns lookup */
        len_a = mdns_res_search( srvname, C_IN, T_SRV, answer, ANSWER_SIZE );
        if( len_a < 0 ) {
            ach_set_errstr( "mdns lookup failed\n" );
            return ACH_FAILED_SYSCALL;
        }
    } else {
        /* Ye olde fashioned dns lookup */
        len_a = res_search( srvname, C_IN, T_SRV, answer, ANSWER_SIZE );
        if( len_a < 0 ) {
            ach_set_errstr( "res_search() failed\n" );
            return ACH_FAILED_SYSCALL;
        }
    }

    /* Process Result */
    return parse_dns_srv( answer, (size_t)len_a, host, host_len, port );
}


/* I'd rather reinvent the wheel than link against Avahi. */

/** A trivial MDNS Resolver */
int
mdns_res_search( const char *dname, int clazz, int type,
                 unsigned char *answer, int anslen)
{

    /* Create request packet */
    unsigned char q[ANSWER_SIZE];
    int qlen = res_mkquery( ns_o_query, dname, clazz, type,
                            NULL, 0, NULL,
                            q, ANSWER_SIZE );
    if( qlen < 0 ) {
        return -1;
    }

    /* Open Socket */
    int sock = socket( AF_INET, SOCK_DGRAM, 0 );
    if( sock < 0 ) {
        perror( "Could not create socket");
        return -1;
    }

    struct sockaddr_in recv_addr = {0};
    recv_addr.sin_family = AF_INET;
    recv_addr.sin_port = htons(0);
    recv_addr.sin_addr.s_addr = INADDR_ANY;

    u_int yes = 1;
    if( setsockopt(sock, IPPROTO_IP, IP_MULTICAST_LOOP, &yes, sizeof(yes)) ) {
        perror("setsockopt for sender failed\n");
        return -1;
    }
    if( setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) ) {
        perror("setsockopt for receiver failed\n");
        return -1;
    }

    if (bind(sock, (struct sockaddr *) &recv_addr, sizeof(recv_addr)) < 0) {
        perror("Failed to bind the socket");
        return -1;
    }

    int rlen ;
    {
        struct sockaddr_in send_addr = {0};
        send_addr.sin_port = htons(5353);
        send_addr.sin_family = AF_INET;
        send_addr.sin_addr.s_addr = inet_addr("224.0.0.251");

        rlen = dns_query( sock, (struct sockaddr *)&send_addr, sizeof(send_addr),
                          q, (size_t)qlen,
                          answer, (size_t)anslen);
    }

    /* Close Socket */
    close(sock);

    return (int)rlen;
}


static inline int
timespec_cmp( struct timespec t0, struct timespec t1 )
{
    if( t0.tv_sec > t1.tv_sec ) return 1;
    else if( t0.tv_sec == t1.tv_sec ) {
        if( t0.tv_nsec > t1.tv_nsec ) return 1;
        else if( t0.tv_nsec == t1.tv_nsec ) return 0;
        else return 1;
    } else return -1;
}


#define MDNS_TIMEOUT_SEC 2
#define MDNS_RETRY_MS 250

int dns_query(int sock, const struct sockaddr *addr, size_t addr_size,
              const void *pkt, size_t pkt_len,
              void *ans, size_t ans_len )
{
    ssize_t rlen;

    struct timespec t0;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    struct timespec t_now = t0;
    struct timespec t_resend = {0,0};
    struct timespec t_abort = t0;
    t_abort.tv_sec += MDNS_TIMEOUT_SEC;

    do {
        /* Send Query */
        if( timespec_cmp(t_now, t_resend) > 0 ) {
            ssize_t r = sendto( sock, pkt, pkt_len, 0,
                                addr, (socklen_t)addr_size );
            if( (ssize_t)pkt_len != r ) {
                perror( "could not send udp query" );
                return -1;
            }
            /* Compute resend time */
            int64_t resend_ns = t_now.tv_nsec + MDNS_RETRY_MS*1000000;
            t_resend.tv_nsec = resend_ns % 1000000000;
            t_resend.tv_sec = t_now.tv_sec + resend_ns / 1000000000;
        }

        /* Poll */
        int poll_i;
        {
            struct pollfd fd;
            fd.fd = sock;
            fd.events = POLLIN;
            fd.revents = 0;
            poll_i = poll(&fd, 1, MDNS_RETRY_MS); /* wait in ms */
        }
        if( poll_i < 0 ) return poll_i;
        if( poll_i ) {
            /* Get Response */
            rlen = recv( sock, ans, (size_t)ans_len, 0 );
            if( rlen < 0 ) {
                perror( "could not receive udp response" );
                return -1;
            }
            /* Check ID */
            /* We could also check the source address and port, but
             * since that is easily spoofed, it seems that little
             * would be gained. */
            if( rlen >= (ssize_t)sizeof(struct dns_packet) &&
                ((struct dns_packet*)ans)->id == ((struct dns_packet*)pkt)->id
                )
            {
                return (int)rlen;
            }
        }
        clock_gettime(CLOCK_MONOTONIC, &t_now);
    } while( timespec_cmp(t_now, t_abort) < 0 );

    return -1;
}
