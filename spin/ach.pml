
#define INDEX_CNT 4
#define DATA_SIZE 9
#define MAX_MSG_SIZE 3

#define ALIGN 1 /* not really implemented */

#define SIZE_BITS 6
#define SEQ_NUM_BITS 8
#define N_MSGS (2*INDEX_CNT)

#define MEMCPY( dst, dstoff, src, srcoff, length, counter_var) \
  counter_var = 0;                                          \
  do :: (counter_var < length ) ->                          \
        dst[dstoff+counter_var] = src[srcoff+counter_var];  \
        counter_var++;                                      \
     :: else -> break;                                      \
  od


#define FREE_IDX(I) \
  assert(1 == shm_idx[I].used );\
  assert(shm_idx_free < INDEX_CNT);\
  shm_data_free = shm_data_free + shm_idx[I].size; \
  shm_idx_free++; \
  shm_idx[I].size = 0; \
  shm_idx[I].offset = 0; \
  shm_idx[I].seq_num = 0; \
  shm_idx[I].used = 0;

#define OLDEST_INDEX_I ((shm_idx_head + shm_idx_free)%INDEX_CNT)

#define LAST_INDEX_I ((shm_idx_head + INDEX_CNT - 1)%INDEX_CNT)

typedef index {
  unsigned size    : SIZE_BITS;
  unsigned offset  : SIZE_BITS;
  unsigned seq_num : SEQ_NUM_BITS;
  bit used;
}

index shm_idx[INDEX_CNT] = 0
unsigned shm_last_seq  : SEQ_NUM_BITS = 0
unsigned shm_idx_head  : SIZE_BITS    = 0
unsigned shm_idx_free  : SIZE_BITS    = INDEX_CNT
unsigned shm_data_head : SIZE_BITS    = 0
unsigned shm_data_free : SIZE_BITS    = DATA_SIZE
byte shm_data[DATA_SIZE] = 0


active proctype consts() {
  assert( INDEX_CNT < (1 << SIZE_BITS) - 1 );
  assert( DATA_SIZE < (1 << SIZE_BITS) - 1 );

}


never {
  do
    /* no overflow on array indices */
    :: (shm_data_free > DATA_SIZE) -> break;
    :: (shm_idx_free > INDEX_CNT) -> break;
    :: (shm_data_head > DATA_SIZE) -> break;
    :: (shm_idx_head > INDEX_CNT) -> break;
    :: else
  od
};


/* Validate the channel structure */
active proctype monitor ()  {
  byte j;
  select( j : 0 .. (INDEX_CNT-1));
  assert( j < INDEX_CNT );
  /* check used flags */
  atomic {
    if :: (j < shm_idx_free) ->
          /* from head entry up to shm_idx_free, unused */
          assert(0 == shm_idx[(shm_idx_head+j)%INDEX_CNT].used );
          assert(0 == shm_idx[(shm_idx_head+j)%INDEX_CNT].size );
       :: else ->
          /* rest are used */
          assert(1 == shm_idx[(shm_idx_head+j)%INDEX_CNT].used );
          assert(0 < shm_idx[(shm_idx_head+j)%INDEX_CNT].size );
          assert(0 < shm_idx[(shm_idx_head+j)%INDEX_CNT].seq_num );
    fi;
  }
  /* Check alignment */
  assert( shm_idx[j].offset % ALIGN == 0 );
  /* Check free space in data */
  assert( (shm_idx[LAST_INDEX_I].offset +  
           shm_idx[LAST_INDEX_I].size +  
           shm_data_free ) % DATA_SIZE
          == shm_idx[OLDEST_INDEX_I].offset );
  /* Check free data head vs. last offset+size */
  assert( (shm_idx[LAST_INDEX_I].offset +
           shm_idx[LAST_INDEX_I].size) % DATA_SIZE ==
          shm_data_head );
  /* Check oldest/last interval */
  assert( (LAST_INDEX_I + shm_idx_free + 1)%INDEX_CNT == OLDEST_INDEX_I );

  /* Check sequence numbers and offsets */
  assert( shm_idx[LAST_INDEX_I].seq_num == shm_last_seq );
  atomic {
    if :: (shm_idx[j].used && shm_idx[(j+1)%INDEX_CNT].used) ->
          /* j and j+1 are both used */
          if :: ( shm_idx_head == (j+1)%INDEX_CNT ) ->
                /* j+1 is the next index to be free'ed and used */
                assert( (j+1)%INDEX_CNT == OLDEST_INDEX_I );
                assert( j == LAST_INDEX_I );
                assert( shm_idx[(j+1)%INDEX_CNT].seq_num + INDEX_CNT ==
                        shm_idx[j].seq_num + 1 );
                assert( shm_idx_free == 0);
                assert( shm_last_seq == shm_idx[j].seq_num);
                assert( (shm_idx[(j)].offset +  
                         shm_idx[(j)].size +
                         shm_data_free ) % DATA_SIZE
                        == shm_idx[(j+1)%INDEX_CNT].offset );
             :: else ->
                assert( shm_idx[j].seq_num + 1 ==
                        shm_idx[(j+1)%INDEX_CNT].seq_num );
                assert( (shm_idx[j].offset + shm_idx[j].size) % DATA_SIZE ==
                        shm_idx[(j+1)%INDEX_CNT].offset );
          fi
       :: else -> assert( shm_idx_free > 0 ); /* must be some free indices */
    fi;
  }
  /* Check data validity */
  /* This PML model writes the sequence number as the first byte of every frame,
   * so we can just check that here. */
  atomic {
    if :: (shm_idx[j].used) ->
          assert( shm_data[ shm_idx[j].offset ] == shm_idx[j].seq_num);
       :: else -> skip;
    fi
  }
}



active proctype putter()
{
  byte buf[MAX_MSG_SIZE];
  byte buflen = 0; 
  byte i;
  do
    :: (shm_last_seq < N_MSGS) ->
       
       /* generate some data */
       select( buflen : 1 .. MAX_MSG_SIZE);
       buf[0] = shm_last_seq + 1; /* that's the current sequence number */
       i = 1;
       do
         :: (i < buflen ) ->
            buf[i] = shm_last_seq - i;
            i++;
         :: else -> break
       od;

       assert( buflen < DATA_SIZE);

       /* ach_put() function */
       atomic {
         
         /* clear entry used by index  */
         if :: (0 == shm_idx_free) ->
               /* free the head frame */
               FREE_IDX(shm_idx_head)
            :: else ->
               assert( 0 == shm_idx[shm_idx_head].used );
         fi;

         assert( shm_idx_free > 0 );
         assert( 0 == shm_idx[shm_idx_head].used );

         /* clear overlapping entries (not enough free data space) */
         i = (shm_idx_head + shm_idx_free) % INDEX_CNT;
         do
           :: (shm_data_free < buflen) ->
              assert( i < INDEX_CNT );
              assert( i != shm_idx_head);
              assert( 0 != shm_idx[i].size );
              assert( 1 == shm_idx[i].used );
              FREE_IDX(i)
              i = (i+1) % INDEX_CNT
           :: else -> break
         od;

         assert( shm_data_free >= buflen );
         assert( 0 ==  shm_idx[shm_idx_head].size );

         /* copy buffer */
         if
           :: ( DATA_SIZE - shm_data_head >= buflen ) ->
              /* simple copy */
              MEMCPY( shm_data, shm_data_head, buf, 0, buflen, i);
           :: else ->
              /* wraparound copy */
#define TMP_END_CNT (DATA_SIZE-shm_data_head)
              MEMCPY( shm_data, shm_data_head, buf, 0,
                      TMP_END_CNT, i);
              MEMCPY( shm_data, 0, buf, TMP_END_CNT,
                      buflen - TMP_END_CNT, i);
#undef TMP_END_CNT
         fi;

         /* modify counts */
         shm_last_seq++;
         shm_idx[shm_idx_head].seq_num = shm_last_seq;
         shm_idx[shm_idx_head].size = buflen;
         shm_idx[shm_idx_head].offset = shm_data_head;
         shm_idx[shm_idx_head].used = 1;

         shm_data_head = (shm_data_head+buflen) % DATA_SIZE;
         shm_data_free = shm_data_free - buflen;
         shm_idx_head = (shm_idx_head + 1) % INDEX_CNT;
         shm_idx_free--;
         
       }
       assert(shm_last_seq > 0)
    :: else -> break
  od;
  printf("hello put\n")
}


/*
active proctype getter()
{

  byte chan_seq_num = 0;
  byte chan_idx = 0;
  byte data = 0;
  do
    :: (data < 99) ->
       assert(chan_seq_num <= shm_seq_num);
       atomic {
         if
           :: (chan_seq_num == shm_seq_num) ->
              assert(data == cbuf[chan_idx]);
           :: (chan_seq_num < shm_seq_num) ->
              assert(cbuf[chan_idx] >= data);
              data = cbuf[chan_idx];
              chan_idx = (chan_idx + 1)%10;
              chan_seq_num = shm_seq_num;
              printf("read %d\n", data);
         fi;
       }
    :: (data>=99) -> break;
  od;
  printf("hello get\n")
}
*/
