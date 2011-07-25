
#define INDEX_CNT 3
#define DATA_SIZE 5
#define MAX_MSG_SIZE 2

#define ALIGN 1 /* not really implemented */

#define SIZE_BITS 3
#define SEQ_NUM_BITS 5
#define N_MSGS (2*INDEX_CNT)

#define MEMCPY( dst, dstoff, src, srcoff, length, counter_var) \
  counter_var = 0;                                          \
  do :: (counter_var < length ) ->                          \
        dst[dstoff+counter_var] = src[srcoff+counter_var];  \
        counter_var++;                                      \
     :: else -> break;                                      \
  od


inline FREE_IDX(I){
  assert(1 == shm_idx[I].used );
  assert(shm_idx_free < INDEX_CNT);
  shm_data_free = shm_data_free + shm_idx[I].size;
  shm_idx_free++;
  shm_idx[I].size = 0;
  shm_idx[I].offset = 0;
  shm_idx[I].seq_num = 0;
  shm_idx[I].used = 0;
}

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
bit shm_mutex;


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

inline lock() {
  atomic{ shm_mutex == 0 -> shm_mutex = 1; }
}

inline unlock() {
  atomic{
    assert(shm_mutex == 1);
    shm_mutex = 0;
  }
}

inline wrlock() { lock() }
inline wrunlock(){ unlock() }
inline rdlock() { lock() }
inline rdunlock(){ unlock() }

inline rdlock_wait(seq_num){
  /* model the condition variable */
  atomic{ (shm_mutex == 0 && shm_last_seq > seq_num) -> shm_mutex = 1; }
}

init {
  run putter();
  run putter();
  run getter();
}

/* Validate the channel structure */
active proctype monitor ()  {
  byte j;
  select( j : 0 .. (INDEX_CNT-1));
  assert( j < INDEX_CNT );

  lock()

  /* check used flags */
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
  /* Check data validity */
  /* This PML model writes the sequence number as the first byte of every frame,
   * so we can just check that here. */
  if :: (shm_idx[j].used) ->
        assert( shm_data[ shm_idx[j].offset ] == shm_idx[j].seq_num);
     :: else -> skip;
  fi;
  unlock();
}



proctype putter()
{
  byte buf[MAX_MSG_SIZE];
  byte buflen = 0; 
  byte i;
  do
    :: (shm_last_seq < N_MSGS) ->
       
       /* generate some data */
       select( buflen : 1 .. MAX_MSG_SIZE);
       assert( buflen <= DATA_SIZE);

       wrlock();
         buf[0] = shm_last_seq + 1; /* that's the current sequence number */
         i = 1;
         do
           :: (i < buflen ) ->
              buf[i] = shm_last_seq - i;
              i++;
           :: else -> break
         od;

         
         /* ach_put() function */
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
         
       wrunlock();
       assert(shm_last_seq > 0)
    :: else -> break
  od;
  printf("hello put\n")
}


proctype getter()
{

  bit o_last = 0;
  bit o_wait = 0;
  bit o_copy = 0;
  unsigned seq_num : SEQ_NUM_BITS;
  unsigned next_index : SIZE_BITS;
  unsigned read_idx : SIZE_BITS;
  bit missed_frame;
  bit stale_frames;
  select(o_last : 0 .. 1);
  select(o_wait : 0 .. 1);
  select(o_copy : 0 .. 1);

  /* Set the last read frame. 
   * This may be a more conservative model, since we could be looking
   * into anywhere in the index array, not necessarily at something
   * that was ever written to. */
  rdlock();
  select( next_index : 0 .. (INDEX_CNT-1) );
  seq_num = shm_idx[(next_index + INDEX_CNT - 1)%INDEX_CNT].seq_num;
  rdunlock();
  
  /* rdlock() */
  if :: (o_wait) -> rdlock_wait(seq_num);
     :: else -> rdlock();
  fi;
  assert( seq_num <= shm_last_seq );
  /* get the data */
  if :: ( (seq_num == shm_last_seq && !o_copy) || 0 == shm_last_seq ) -> 
        assert( !o_wait );
        stale_frames = 1;
     :: else ->
        if :: (o_last ) ->
              /* Normal case, getting last frame.*/
              read_idx = LAST_INDEX_I;
           :: (!o_last && shm_idx[next_index].seq_num == seq_num+1) ->
              /* Normal case, getting the next frame. */
              read_idx = next_index;
           :: else ->
              /* Exceptional cases, have to figure out which frame is right. */
              if :: (seq_num == shm_last_seq) ->
                    /* copy last */
                    assert(o_copy);
                    read_idx = LAST_INDEX_I;
                 :: (seq_num != shm_last_seq) ->
                    /* copy oldest */
                    read_idx = OLDEST_INDEX_I;
                 :: else -> assert(false)
              fi;
        fi;
        assert( shm_idx[read_idx].seq_num > 0 );

        /* check for missed frames */
        if :: (shm_idx[read_idx].seq_num > seq_num + 1) -> missed_frame = 1;
           :: else -> skip;
        fi;
  fi ;

  
  /* validate */
  if :: (o_last) ->
        /* Last always gets last frame */
        if :: (!stale_frames) ->
              assert( shm_idx[read_idx].seq_num == shm_last_seq );
              assert( read_idx == LAST_INDEX_I );
           :: (stale_frames) ->
              assert( seq_num == shm_last_seq );
              assert( !o_wait );
           :: else -> assert(0);
        fi;
        /* Wait never gets stale frames. */
     :: (o_wait) ->
        assert(!stale_frames);
        assert( shm_idx[read_idx].seq_num > seq_num );
     :: (o_copy) -> 
        /* Copy get's something, unless the channel is empty and we didn't wait*/
        assert( shm_idx[read_idx].seq_num >= seq_num );
        assert(!stale_frames || (shm_last_seq == 0 && !o_wait));
     :: (stale_frames) ->
        assert(!missed_frame);
        assert(!o_wait);
        assert(shm_last_seq == seq_num);
     ::(shm_last_seq == seq_num) ->
        assert(stale_frames || o_copy);
     :: (missed_frame) ->
        assert(!stale_frames);
        assert(shm_last_seq > seq_num+1);
        assert(shm_idx[read_idx].seq_num > seq_num + 1);
        assert( (!o_last && read_idx == OLDEST_INDEX_I) ||
                (o_last && read_idx == LAST_INDEX_I) );
     :: (seq_num < shm_last_seq) ->
        /* if new data, we got it */
        assert(!stale_frames);
        assert(shm_idx[read_idx].seq_num > seq_num);
     :: else -> skip;
  fi;

  rdunlock();
  printf("hello get\n")
}
