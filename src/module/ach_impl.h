/*
 * This file replicates some of the structures already defined 
 * in ach_impl.h for userspace library - consider joining the two files
 *
 */

/** prefix to apply to channel names to get the shared memory file name */
/** prefix to apply to channel names got get the chardev file name */
#define ACH_CHAR_CHAN_NAME_PREFIX_PATH "/dev/"
#define ACH_CHAR_CHAN_NAME_PREFIX_NAME "ach-"

/** Number of times to retry a syscall on EINTR before giving up */
//#define ACH_INTR_RETRY 8

    /** magic number that appears the the beginning of our mmaped files.

        This is just to be used as a check.
    */
#define ACH_SHM_MAGIC_NUM 0xb07511f3

    /** A separator between different shm sections.

        This one comes after the header.  Should aid debugging by
        showing we don't overstep and bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_HEADER_NUM ((uint64_t)0x1A2A3A4A5A6A7A8ALLU)
    /** A separator between different shm sections.

        This ones comes after the index array.  Should aid debugging by
        showing we don't overstep and bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_INDEX_NUM ((uint64_t)0x1B2B3B4B5B6B7B8BLLU)

    /** A separator between different shm sections.

        This one comes after the data section (at the very end of the
        file).  Should aid debugging by showing we don't overstep and
        bounds.  64-bit for alignment.
    */
#define ACH_SHM_GUARD_DATA_NUM ((uint64_t)0x1C2C3C4C5C6C7C8CLLU)

    /** Header for shared memory area.
     *
     * There is no tail pointer here.  Every subscriber that opens the
     * channel must maintain its own tail pointer.
     */
typedef struct ach_header {
	uint32_t magic;		 /**< magic number of ach shm files */
	size_t len;		 /**< length of mmap'ed file */
	union {
		struct {
			size_t index_cnt;/**< number of entries in index */
			size_t data_size;/**< size of data bytes */
			size_t data_head;/**< offset to first open byte of data */
			size_t data_free;/**< number of free data bytes */
			size_t index_head;
					 /**< index into index array of first unused index entry */
			size_t index_free;
					 /**< number of unused index entries */
		};
		uint64_t reserved[16];
				    /**< Reserve to compatibly add future variables */
	};

#if 1
	struct {
		struct mutex mutex;
		wait_queue_head_t readq;
		int dirty;
	} sync;
#else
	struct {		/* anonymous structure */
		pthread_mutex_t mutex;	   /**< mutex for condition variables */
		pthread_cond_t cond;	   /**< condition variable */
		int dirty;
	} sync;			  /**< variables for synchronization */
#endif
	/* should force our alignment to 8-bytes... */
	uint64_t last_seq;	  /**< last sequence number written */
} ach_header_t;

    /** Entry in shared memory index array
     */
typedef struct {
	size_t size;	  /**< size of frame */
	size_t offset;	  /**< byte offset of entry from beginning of data array */
	uint64_t seq_num; /**< number of frame */
} ach_index_t;

/** Gets pointer to guard uint64 following the header */
#define ACH_SHM_GUARD_HEADER( shm ) ((uint64_t*)((ach_header_t*)(shm) + 1))

/** Gets the pointer to the index array in the shm block */
#define ACH_SHM_INDEX( shm ) ((ach_index_t*)(ACH_SHM_GUARD_HEADER(shm) + 1))

/**  gets pointer to the guard following the index section */
#define ACH_SHM_GUARD_INDEX( shm )                                      \
    ((uint64_t*)(ACH_SHM_INDEX(shm) + ((ach_header_t*)(shm))->index_cnt))

/** Gets the pointer to the data buffer in the shm block */
#define ACH_SHM_DATA( shm ) ( (uint8_t*)(ACH_SHM_GUARD_INDEX(shm) + 1) )

/** Gets the pointer to the guard following data buffer in the shm block */
#define ACH_SHM_GUARD_DATA( shm )                                       \
    ((uint64_t*)(ACH_SHM_DATA(shm) + ((ach_header_t*)(shm))->data_size))

/** Default number of index entries in a channel */
#define ACH_DEFAULT_FRAME_COUNT 16

/** Default nominal frame size for a channel */
#define ACH_DEFAULT_FRAME_SIZE 512
