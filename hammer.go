// https://github.com/varialus/hammer/blob/master/hammer.go

/*
Copyright (c) 2013, Aulus Egnatius Varialus <varialus@gmail.com>

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

// http://gitweb.dragonflybsd.org/dragonfly.git/blob/HEAD:/sys/vfs/hammer/hammer.h

///*
// * Copyright (c) 2007-2008 The DragonFly Project.  All rights reserved.
// * 
// * This code is derived from software contributed to The DragonFly Project
// * by Matthew Dillon <dillon@backplane.com>
// * 
// * Redistribution and use in source and binary forms, with or without
// * modification, are permitted provided that the following conditions
// * are met:
// * 
// * 1. Redistributions of source code must retain the above copyright
// *    notice, this list of conditions and the following disclaimer.
// * 2. Redistributions in binary form must reproduce the above copyright
// *    notice, this list of conditions and the following disclaimer in
// *    the documentation and/or other materials provided with the
// *    distribution.
// * 3. Neither the name of The DragonFly Project nor the names of its
// *    contributors may be used to endorse or promote products derived
// *    from this software without specific, prior written permission.
// * 
// * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
// * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
// * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
// * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
// * SUCH DAMAGE.
// */
///*
// * This header file contains structures used internally by the HAMMERFS
// * implementation.  See hammer_disk.h for on-disk structures.
// */
//

package hammer


import (
	"bytes"
	"encoding/binary"
	"github.com/varialus/hammer/dependencies/system"
	"github.com/varialus/hammer/dependencies/system/kernel"
	"unsafe"
)

//#include <sys/param.h>
//#include <sys/types.h>
//#ifdef _KERNEL
//#include <sys/kernel.h>
//#endif
//#include <sys/conf.h>
//#ifdef _KERNEL
//#include <sys/systm.h>
//#endif
//#include <sys/tree.h>
//#include <sys/malloc.h>
//#include <sys/mount.h>
//#include <sys/mountctl.h>
//#include <sys/vnode.h>
//#include <sys/proc.h>
//#include <sys/priv.h>
//#include <sys/stat.h>
//#include <sys/globaldata.h>
//#include <sys/lockf.h>
//#include <sys/buf.h>
//#include <sys/queue.h>
//#include <sys/ktr.h>
//#include <sys/limits.h>
//#include <vm/vm_extern.h>
//

//#include <sys/buf2.h>
//#ifdef _KERNEL
//#include <sys/signal2.h>
//#include <vm/vm_page2.h>
//#endif
//

//#include "hammer_disk.h"
//#include "hammer_mount.h"
//#include "hammer_ioctl.h"
//

//#if defined(_KERNEL) || defined(_KERNEL_STRUCTURES)
//

//MALLOC_DECLARE(M_HAMMER);
/* Manually Expanded Macro */
var M_HAMMER [1]system.Malloc_type
//

///*
// * Kernel trace
// */
//#if !defined(KTR_HAMMER)
//#define KTR_HAMMER	KTR_ALL
const	KTR_HAMMER	= system.KTR_ALL
//#endif
///* KTR_INFO_MASTER_EXTERN(hammer); */
//

///*
// * Misc structures
// */
//struct hammer_mount;
/* Already Defined */
//

///*
// * Key structure used for custom RB tree inode lookups.  This prototypes
// * the function hammer_ino_rb_tree_RB_LOOKUP_INFO(root, info).
// */
//typedef struct hammer_inode_info {

type hammer_inode_info struct {
//	int64_t		obj_id;		/* (key) object identifier */
	obj_id			int64		/* (key) object identifier */
//	hammer_tid_t	obj_asof;	/* (key) snapshot transid or 0 */
	obj_asof		hammer_tid_t	/* (key) snapshot transid or 0 */
//	u_int32_t	obj_localization; /* (key) pseudo-fs */
	obj_localization	uint32		/* (key) pseudo-fs */
//	union {
//		struct hammer_btree_leaf_elm *leaf;
//	} u;
	u			hammer_btree_elm
//} *hammer_inode_info_t;
}
type hammer_inode_info_t *hammer_inode_info
//

//typedef enum hammer_transaction_type {
type hammer_transaction_type int

const (
//	HAMMER_TRANS_RO,
	HAMMER_TRANS_RO hammer_transaction_type = iota
//	HAMMER_TRANS_STD,
	HAMMER_TRANS_STD
//	HAMMER_TRANS_FLS
	HAMMER_TRANS_FLS
//} hammer_transaction_type_t;
)
type hammer_transaction_type_t hammer_transaction_type
//

///*
// * HAMMER Transaction tracking
// */
//struct hammer_transaction {
type hammer_transaction struct {
//	hammer_transaction_type_t type;
	type_	hammer_transaction_type_t
//	struct hammer_mount *hmp;
	hmp		*hammer_mount
//	hammer_tid_t	tid;
	tid		hammer_tid_t
//	u_int64_t	time;
	time		uint64
//	u_int32_t	time32;
	time32		uint32
//	int		sync_lock_refs;
	sync_lock_refs	int
//	int		flags;
	flags		int
//	struct hammer_volume *rootvol;
//};
}
type rootvol * hammer_volume
//

//typedef struct hammer_transaction *hammer_transaction_t;
type hammer_transaction_t *hammer_transaction
//

//#define HAMMER_TRANSF_NEWINODE	0x0001
const	HAMMER_TRANSF_NEWINODE	= 0x0001
//#define HAMMER_TRANSF_DIDIO	0x0002
const	HAMMER_TRANSF_DIDIO	= 0x0002
//#define HAMMER_TRANSF_CRCDOM	0x0004	/* EDOM on CRC error, less critical */
const	HAMMER_TRANSF_CRCDOM	= 0x0004	/* EDOM on CRC error, less critical */
//

///*
// * HAMMER locks
// */
//struct hammer_lock {
type hammer_lock struct{
//	volatile u_int	refs;		/* active references */
	refs		uint		/* active references */
//	volatile u_int	lockval;	/* lock count and control bits */
	lockval		uint		/* lock count and control bits */
//	struct thread	*lowner;	/* owner if exclusively held */
	lowner		*system.Thread	/* owner if exclusively held */
//	struct thread	*rowner;	/* owner if exclusively held */
	rowner		*system.Thread		/* owner if exclusively held */
//};
}
//

//#define HAMMER_REFS_LOCKED	0x40000000	/* transition check */
const HAMMER_REFS_LOCKED	= 0x40000000	/* transition check */
//#define HAMMER_REFS_WANTED	0x20000000	/* transition check */
const HAMMER_REFS_WANTED	= 0x20000000	/* transition check */
//#define HAMMER_REFS_CHECK	0x10000000	/* transition check */
const HAMMER_REFS_CHECK		= 0x10000000	/* transition check */
//

//#define HAMMER_REFS_FLAGS	(HAMMER_REFS_LOCKED | \
//				 HAMMER_REFS_WANTED | \
//				 HAMMER_REFS_CHECK)
const	HAMMER_REFS_FLAGS	= (HAMMER_REFS_LOCKED | HAMMER_REFS_WANTED | HAMMER_REFS_CHECK)
//

//#define HAMMER_LOCKF_EXCLUSIVE	0x40000000
const HAMMER_LOCKF_EXCLUSIVE	= 0x40000000
//#define HAMMER_LOCKF_WANTED	0x20000000
const HAMMER_LOCKF_WANTED	= 0x20000000
//

//static __inline int
//hammer_notlocked(struct hammer_lock *lock)
//{
func hammer_notlocked(lock *hammer_lock) int {
//	return(lock->lockval == 0);
	if lock.lockval == 0 {
		return 1
	} else {
		return 0
	}
//}
}
//

//static __inline int
//hammer_islocked(struct hammer_lock *lock)
//{
func hammer_islocked(lock *hammer_lock) int {
//	return(lock->lockval != 0);
	if lock.lockval != 0 {
		return 1
	} else {
		return 0
	}
//}
}
//

///*
// * Returns the number of refs on the object.
// */
//static __inline int
//hammer_isactive(struct hammer_lock *lock)
//{
func hammer_isactive(lock *hammer_lock) int {
//	return(lock->refs & ~HAMMER_REFS_FLAGS);
	return int(lock.refs & ^uint(HAMMER_REFS_FLAGS))
//}
}
//

//static __inline int
//hammer_oneref(struct hammer_lock *lock)
//{
func hammer_oneref(lock *hammer_lock) int {
//	return((lock->refs & ~HAMMER_REFS_FLAGS) == 1);
	if (lock.refs & ^uint(HAMMER_REFS_FLAGS)) == 1 {
		return 1
	} else {
		return 0
	}
//}
}
//

//static __inline int
//hammer_norefs(struct hammer_lock *lock)
//{
func hammer_norefs(lock *hammer_lock) int {
//	return((lock->refs & ~HAMMER_REFS_FLAGS) == 0);
	if (lock.refs & ^uint(HAMMER_REFS_FLAGS)) == 0 {
		return 1
	} else {
		return 0
	}
//}
}
//

//static __inline int
//hammer_norefsorlock(struct hammer_lock *lock)
//{
func hammer_norefsorlock(lock *hammer_lock) int {
//	return(lock->refs == 0);
	if lock.refs != 0 {
		return 1
	} else {
		return 0
	}
//}
}
//

//static __inline int
//hammer_refsorlock(struct hammer_lock *lock)
//{
func hammer_refsorlock(lock *hammer_lock) int {
//	return(lock->refs != 0);
	if lock.refs != 0 {
		return 1
	} else {
		return 0
	}
//}
}
//

///*
// * Return if we specifically own the lock exclusively.
// */
//static __inline int
//hammer_lock_excl_owned(struct hammer_lock *lock, thread_t td)
//{
func hammer_lock_excl_owned(lock *hammer_lock, td system.Thread_t) int {
//	if ((lock->lockval & HAMMER_LOCKF_EXCLUSIVE) &&
	if ((lock.lockval & HAMMER_LOCKF_EXCLUSIVE) !=0 &&
//	    lock->lowner == td) {
	    lock.lowner == td) {
//		return(1);
		return 1
//	}
	}
//	return(0);
	return 0
//}
}
//

///*
// * Flush state, used by various structures
// */
//typedef enum hammer_inode_state {
type hammer_inode_state int
const(
//	HAMMER_FST_IDLE,
	HAMMER_FST_IDLE hammer_inode_state = iota
//	HAMMER_FST_SETUP,
	HAMMER_FST_SETUP
//	HAMMER_FST_FLUSH
	HAMMER_FST_FLUSH
)
//} hammer_inode_state_t;
type hammer_inode_state_t hammer_inode_state
//

//TAILQ_HEAD(hammer_record_list, hammer_record);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
/* Manually Expanded Macro */
type hammer_record_list struct {
	tqh_first *hammer_record	/* first element */
	tqh_last **hammer_record	/* addr of last next element */
	trace system.Qm_trace
}
//

///*
// * Pseudo-filesystem extended data tracking
// */
//struct hammer_pfs_rb_tree;
/* Already Defined */
//struct hammer_pseudofs_inmem;
//RB_HEAD(hammer_pfs_rb_tree, hammer_pseudofs_inmem);
/* Manually Expanded Macro */
//struct hammer_pfs_rb_tree {
type hammer_pfs_rb_tree struct {
//	struct hammer_pseudofs_inmem *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_pseudofs_inmem				/* root of the tree */
//	struct hammer_pfs_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_pfs_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPE2(hammer_pfs_rb_tree, hammer_pseudofs_inmem, rb_node,
//	      hammer_pfs_rb_compare, u_int32_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_pfs_rb_tree_scan_info {
type hammer_pfs_rb_tree_scan_info struct {
//	struct hammer_pfs_rb_tree_scan_info *link;
	link *hammer_pfs_rb_tree_scan_info
//	struct hammer_pseudofs_inmem	*node;
	node	*hammer_pseudofs_inmem
//}
}
//

//struct hammer_pseudofs_inmem {
type hammer_pseudofs_inmem struct {
//	RB_ENTRY(hammer_pseudofs_inmem)	rb_node;
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_pseudofs_inmem		/* left element */
		rbe_right *hammer_pseudofs_inmem	/* right element */
		rbe_parent *hammer_pseudofs_inmem	/* parent element */
		rbe_color int				/* node color */
	}
//	struct hammer_lock	lock;
	lock		hammer_lock
//	u_int32_t		localization;
	localization	uint32
//	hammer_tid_t		create_tid;
	create_tid	hammer_tid_t
//	int			flags;
	flags		int
//	udev_t			fsid_udev;
	fsid_udev	system.Udev_t
//	struct hammer_pseudofs_data pfsd;
	pfsd		hammer_pseudofs_data
//};
}
//

//typedef struct hammer_pseudofs_inmem *hammer_pseudofs_inmem_t;
type hammer_pseudofs_inmem_t *hammer_pseudofs_inmem
//

//#define HAMMER_PFSM_DELETED	0x0001
const	HAMMER_PFSM_DELETED	= 0x0001
//

///*
// * Cache object ids.  A fixed number of objid cache structures are
// * created to reserve object id's for newly created files in multiples
// * of 100,000, localized to a particular directory, and recycled as
// * needed.  This allows parallel create operations in different
// * directories to retain fairly localized object ids which in turn
// * improves reblocking performance and layout.
// */
//#define OBJID_CACHE_SIZE	2048
const	OBJID_CACHE_SIZE	= 2048
//#define OBJID_CACHE_BULK_BITS	10		/* 10 bits (1024)	*/
const	OBJID_CACHE_BULK_BITS	= 10		/* 10 bits (1024)	*/
//#define OBJID_CACHE_BULK	(32 * 32)	/* two level (1024)	*/
const	OBJID_CACHE_BULK	= (32 * 32)	/* two level (1024)	*/
//#define OBJID_CACHE_BULK_MASK	(OBJID_CACHE_BULK - 1)
const	OBJID_CACHE_BULK_MASK	= (OBJID_CACHE_BULK - 1)
//#define OBJID_CACHE_BULK_MASK64	((u_int64_t)(OBJID_CACHE_BULK - 1))
const	OBJID_CACHE_BULK_MASK64	= (uint64(OBJID_CACHE_BULK - 1))
//

//typedef struct hammer_objid_cache {
type hammer_objid_cache struct {
//	TAILQ_ENTRY(hammer_objid_cache) entry;
	entry struct {
		tqe_next *hammer_objid_cache	/* next element */
		tqe_prev **hammer_objid_cache	/* address of previous next element */
		trace system.Qm_trace
	}
//	struct hammer_inode		*dip;
	dip				*hammer_inode
//	hammer_tid_t			base_tid;
	base_tid			hammer_tid_t
//	int				count;
	count				int
//	u_int32_t			bm0;
	bm0				uint32
//	u_int32_t			bm1[32];
	bm1				[32]uint32
//} *hammer_objid_cache_t;
}
type hammer_objid_cache_t *hammer_objid_cache
//

///*
// * Associate an inode with a B-Tree node to cache search start positions
// */
//typedef struct hammer_node_cache {
type hammer_node_cache struct {
//	TAILQ_ENTRY(hammer_node_cache)	entry;
	/* Manually Expanded Macro */
	entry struct {
		tqe_next *hammer_node_cache	/* next element */
		tqe_prev **hammer_node_cache	/* address of previous next element */
		trace system.Qm_trace
	}
//	struct hammer_node		*node;
	node	*hammer_node
//	struct hammer_inode		*ip;
	ip	*hammer_inode
//} *hammer_node_cache_t;
}
type hammer_node_cache_t *hammer_node_cache
//
//TAILQ_HEAD(hammer_node_cache_list, hammer_node_cache);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
/* Manually Expanded Macro */
type hammer_node_cache_list struct {
	tqh_first *hammer_node_cache	/* first element */
	tqh_last **hammer_node_cache	/* addr of last next element */
	trace system.Qm_trace
}
//

///*
// * Live dedup cache
// */
//struct hammer_dedup_crc_rb_tree;
/* Already Defined */
//RB_HEAD(hammer_dedup_crc_rb_tree, hammer_dedup_cache);
/* Manually Expanded Macro */
//struct hammer_dedup_crc_rb_tree {
type hammer_dedup_crc_rb_tree struct {
//	struct hammer_dedup_cache *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_dedup_cache				/* root of the tree */
//	struct hammer_dedup_crc_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_dedup_crc_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPE2(hammer_dedup_crc_rb_tree, hammer_dedup_cache, crc_entry,
//		hammer_dedup_crc_rb_compare, hammer_crc_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_dedup_crc_rb_tree_scan_info {
type hammer_dedup_crc_rb_tree_scan_info struct {
//	struct hammer_dedup_crc_rb_tree_scan_info *link;
	link *hammer_dedup_crc_rb_tree_scan_info
//	struct hammer_inode	*node;
	node	*hammer_inode
//}
}
//

//struct hammer_dedup_off_rb_tree;
/* Already Defined */
//RB_HEAD(hammer_dedup_off_rb_tree, hammer_dedup_cache);
/* Manually Expanded Macro */
//struct hammer_dedup_off_rb_tree {
type hammer_dedup_off_rb_tree struct {
//	struct hammer_dedup_cache *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_dedup_cache				/* root of the tree */
//	struct hammer_dedup_off_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_dedup_off_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPE2(hammer_dedup_off_rb_tree, hammer_dedup_cache, off_entry,
//		hammer_dedup_off_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_dedup_off_rb_tree_scan_info {
type hammer_dedup_off_rb_tree_scan_info struct {
//	struct hammer_dedup_off_rb_tree_scan_info *link;
	link *hammer_dedup_off_rb_tree_scan_info
//	struct hammer_dedup_cache	*node;
	node	*hammer_dedup_cache
//}
}
//

//#define DEDUP_CACHE_SIZE	4096 /* XXX make it a dynamic tunable */
const	DEDUP_CACHE_SIZE	= 4096 /* XXX make it a dynamic tunable */
//

//typedef struct hammer_dedup_cache {
type hammer_dedup_cache struct {
//	RB_ENTRY(hammer_dedup_cache) crc_entry;
	/* Manually Expanded Macro */
	crc_entry struct {
		rbe_left *hammer_dedup_cache	/* left element */
		rbe_right *hammer_dedup_cache	/* right element */
		rbe_parent *hammer_dedup_cache	/* parent element */
		rbe_color int			/* node color */
	}
//	RB_ENTRY(hammer_dedup_cache) off_entry;
	/* Manually Expanded Macro */
	off_entry struct {
		rbe_left *hammer_dedup_cache	/* left element */
		rbe_right *hammer_dedup_cache	/* right element */
		rbe_parent *hammer_dedup_cache	/* parent element */
		rbe_color int			/* node color */
	}
//	TAILQ_ENTRY(hammer_dedup_cache) lru_entry;
	lru_entry struct {
		tqe_next *hammer_dedup_cache	/* next element */
		tqe_prev **hammer_dedup_cache	/* address of previous next element */
		trace system.Qm_trace
	}
//	struct hammer_mount *hmp;
	hmp *hammer_mount
//	int64_t obj_id;
	obj_id int64
//	u_int32_t localization;
	localization uint32
//	off_t file_offset;
	file_offset system.Off_t
//	int bytes;
	bytes int
//	hammer_off_t data_offset; 
	data_offset hammer_off_t
//	hammer_crc_t crc;
	crc hammer_crc_t
//} *hammer_dedup_cache_t;
}
type hammer_dedup_cache_t *hammer_dedup_cache
//

///*
// * Structure used to organize flush groups.  Flush groups must be
// * organized into chunks in order to avoid blowing out the UNDO FIFO.
// * Without this a 'sync' could end up flushing 50,000 inodes in a single
// * transaction.
// */
//struct hammer_fls_rb_tree;
/* Already Defined */
//RB_HEAD(hammer_fls_rb_tree, hammer_inode);
/* Manually Expanded Macro */
//struct hammer_fls_rb_tree {
type hammer_fls_rb_tree struct {
//	struct hammer_inode *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_inode				/* root of the tree */
//	struct hammer_fls_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_fls_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPE(hammer_fls_rb_tree, hammer_inode, rb_flsnode,
//	      hammer_ino_rb_compare);
/* Manually Expanded Macro */
/* First Expansion */
//_RB_PROTOTYPE(hammer_fls_rb_tree, hammer_inode, rb_flsnode, hammer_ino_rb_compare,)
/* Partial Second Expansion */
type hammer_fls_rb_tree_scan_info struct {
//	struct hammer_fls_rb_tree_scan_info *link;
	link *hammer_fls_rb_tree_scan_info
//	struct hammer_inode	*node;
	node *hammer_inode
//}
}
//

//struct hammer_flush_group {
type hammer_flush_group struct {
//	TAILQ_ENTRY(hammer_flush_group)	flush_entry;
	flush_entry struct {
		tqe_next *hammer_flush_group	/* next element */
		tqe_prev **hammer_flush_group	/* address of previous next element */
		trace system.Qm_trace
	}
//	struct hammer_fls_rb_tree	flush_tree;
	flush_tree hammer_fls_rb_tree
//	int				seq;		/* our seq no */
	seq		int				/* our seq no */
//	int				total_count;	/* record load */
	total_count	int				/* record load */
//	int				running;	/* group is running */
	running		int				/* group is running */
//	int				closed;
	closed		int
//	int				refs;
	refs		int
//};
}
//

//typedef struct hammer_flush_group *hammer_flush_group_t;
type hammer_flush_group_t *hammer_flush_group
//

//TAILQ_HEAD(hammer_flush_group_list, hammer_flush_group);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
/* Manually Expanded Macro */
type hammer_flush_group_list struct {
	tqh_first *hammer_flush_group	/* first element */
	tqh_last **hammer_flush_group	/* addr of last next element */
	trace system.Qm_trace
}
//

///*
// * Structure used to represent an inode in-memory.
// *
// * The record and data associated with an inode may be out of sync with
// * the disk (xDIRTY flags), or not even on the disk at all (ONDISK flag
// * clear).
// *
// * An inode may also hold a cache of unsynchronized records, used for
// * database and directories only.  Unsynchronized regular file data is
// * stored in the buffer cache.
// *
// * NOTE: A file which is created and destroyed within the initial
// * synchronization period can wind up not doing any disk I/O at all.
// *
// * Finally, an inode may cache numerous disk-referencing B-Tree cursors.
// */
//struct hammer_ino_rb_tree;
/* Already Defined */
//struct hammer_inode;
//RB_HEAD(hammer_ino_rb_tree, hammer_inode);
/* Manually Expanded Macro */
//struct hammer_ino_rb_tree {
type hammer_ino_rb_tree struct {
//	struct hammer_inode *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_inode				/* root of the tree */
//	struct hammer_ino_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_ino_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPEX(hammer_ino_rb_tree, INFO, hammer_inode, rb_node,
//	      hammer_ino_rb_compare, hammer_inode_info_t);
/* Manually Expanded Macro */
/* Partially Expanded */
type hammer_ino_rb_tree_scan_info struct {
//	struct hammer_ino_rb_tree_scan_info *link;
	link *hammer_ino_rb_tree_scan_info
//	struct hammer_inode	*node;
	node *hammer_inode
//}
}
//

//struct hammer_redo_rb_tree;
/* Already Defined */
//RB_HEAD(hammer_redo_rb_tree, hammer_inode);
/* Manually Expanded Macro */
//struct hammer_redo_rb_tree {
type hammer_redo_rb_tree struct {
//	struct hammer_inode *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_inode				/* root of the tree */
//	struct hammer_redo_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_redo_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPE2(hammer_redo_rb_tree, hammer_inode, rb_redonode,
//	      hammer_redo_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_redo_rb_tree_scan_info {
type hammer_redo_rb_tree_scan_info struct {
//	struct hammer_redo_rb_tree_scan_info *link;
	link *hammer_redo_rb_tree_scan_info
//	struct hammer_inode	*node;
	node	*hammer_inode
//}
}
//

//struct hammer_rec_rb_tree;
/* Already Defined */
//struct hammer_record;
/* Already Defined */
//RB_HEAD(hammer_rec_rb_tree, hammer_record);
/* Manually Expanded Macro */
//struct hammer_rec_rb_tree {
type hammer_rec_rb_tree struct {
//	struct hammer_record *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_record				/* root of the tree */
//	struct hammer_rec_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_rec_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_PROTOTYPEX(hammer_rec_rb_tree, INFO, hammer_record, rb_node,
//	      hammer_rec_rb_compare, hammer_btree_leaf_elm_t);
/* Manually Expanded Macro */
/* Partially Expanded and Partially Translated */
/* Third Expansion
//struct hammer_record *hammer_rec_rb_tree_RB_REMOVE(struct hammer_rec_rb_tree *, struct hammer_record *);
//struct hammer_record *hammer_rec_rb_tree_RB_INSERT(struct hammer_rec_rb_tree *, struct hammer_record *);
//struct hammer_record *hammer_rec_rb_tree_RB_FIND(struct hammer_rec_rb_tree *, struct hammer_record *);
//int hammer_rec_rb_tree_RB_SCAN(struct hammer_rec_rb_tree *, int (*)(struct hammer_record *, void *),
//			int (*)(struct hammer_record *, void *), void *);
//struct hammer_record *hammer_rec_rb_tree_RB_NEXT(struct hammer_record *);
//struct hammer_record *hammer_rec_rb_tree_RB_PREV(struct hammer_record *);
//struct hammer_record *hammer_rec_rb_tree_RB_MINMAX(struct hammer_rec_rb_tree *, int);
/* Fourth Expansion */
//struct hammer_rec_rb_tree_scan_info {
type hammer_rec_rb_tree_scan_info struct {
//	struct hammer_rec_rb_tree_scan_info *link;
	link *hammer_rec_rb_tree_scan_info
//	struct hammer_record	*node;
	node *hammer_record
//}
}
//RB_SCAN_INFO(hammer_rec_rb_tree, hammer_record)
/* Second Expansion */
//_RB_PROTOTYPE(hammer_rec_rb_tree, hammer_record, rb_node, hammer_rec_rb_compare,)
/* First Expansion */
//RB_PROTOTYPE(hammer_rec_rb_tree, hammer_record, rb_node, hammer_rec_rb_compare);
//struct hammer_record *hammer_rec_rb_tree_RB_LOOKUP_INFO (struct hammer_rec_rb_tree *, hammer_btree_leaf_elm_t)
//

//TAILQ_HEAD(hammer_node_list, hammer_node);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
/* Manually Expanded Macro */
type hammer_node_list struct {
	//tqh_first *hammer_node	/* first element */
	//tqh_last **hammer_node	/* addr of last next element */
	trace system.Qm_trace
}
//

//struct hammer_inode {
type hammer_inode struct {
//	RB_ENTRY(hammer_inode)	rb_node;
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_inode		/* left element */
		rbe_right *hammer_inode		/* right element */
		rbe_parent *hammer_inode		/* parent element */
		rbe_color int			/* node color */
	}
//	hammer_inode_state_t	flush_state;
	flush_state	hammer_inode_state_t
//	hammer_flush_group_t	flush_group;
	flush_group	hammer_flush_group_t
//	RB_ENTRY(hammer_inode)	rb_flsnode;	/* when on flush list */
	/* Manually Expanded Macro */
	rb_flsnode struct {
		rbe_left *hammer_inode		/* left element */
		rbe_right *hammer_inode		/* right element */
		rbe_parent *hammer_inode		/* parent element */
		rbe_color int			/* node color */
	}
//	RB_ENTRY(hammer_inode)	rb_redonode	/* when INODE_RDIRTY is set */
	/* Manually Expanded Macro */
	rb_redonode struct {
		rbe_left *hammer_inode		/* left element */
		rbe_right *hammer_inode		/* right element */
		rbe_parent *hammer_inode		/* parent element */
		rbe_color int			/* node color */
	}
//	struct hammer_record_list target_list;	/* target of dependant recs */
	target_list	hammer_record_list 	/* target of dependant recs */
//	int64_t			obj_id;		/* (key) object identifier */
	obj_id		int64			/* (key) object identifier */
//	hammer_tid_t		obj_asof;	/* (key) snapshot or 0 */
	obj_asof	hammer_tid_t		/* (key) snapshot or 0 */
//	u_int32_t		obj_localization; /* (key) pseudo-fs */
	obj_localization	uint32		 /* (key) pseudo-fs */
//	struct hammer_mount 	*hmp;
	hmp		*hammer_mount
//	hammer_objid_cache_t 	objid_cache;
	objid_cache hammer_objid_cache_t
//	int			flags;
	flags		int
//	int			error;		/* flush error */
	error		int			/* flush error */
//	int			cursor_ip_refs;	/* sanity */
	cursor_ip_refs	int			/* sanity */
//	int			cursor_exclreq_count;
	cursor_exclreq_count	int
//	int			rsv_recs;
	rsv_recs	int
//	struct vnode		*vp;
	vp		*system.Vnode
//	hammer_pseudofs_inmem_t	pfsm;
	pfsm		hammer_pseudofs_inmem_t
//	struct lockf		advlock;
	advlock		system.Lockf
//	struct hammer_lock	lock;		/* sync copy interlock */
	lock		hammer_lock		/* sync copy interlock */
//	off_t			trunc_off;
	trunc_off	system.Off_t
//	struct hammer_btree_leaf_elm ino_leaf;  /* in-memory cache */
	ino_leaf	hammer_btree_leaf_elm   /* in-memory cache */
//	struct hammer_inode_data ino_data;	/* in-memory cache */
	ino_data	hammer_inode_data
//	struct hammer_rec_rb_tree rec_tree;	/* in-memory cache */
	rec_tree	hammer_rec_rb_tree 	/* in-memory cache */
//	int			rec_generation;
	rec_generation	int
//	struct hammer_node_cache cache[4];	/* search initiate cache */
	cache		[4]hammer_node_cache 	/* search initiate cache */
//

//	/*
//	 * When a demark is created to synchronize an inode to
//	 * disk, certain fields are copied so the front-end VOPs
//	 * can continue to run in parallel with the synchronization
//	 * occuring in the background.
//	 */
//	int		sync_flags;		/* to-sync flags cache */
	sync_flags	int			/* to-sync flags cache */
//	off_t		sync_trunc_off;		/* to-sync truncation */
	sync_trunc_off	system.Off_t		/* to-sync truncation */
//	off_t		save_trunc_off;		/* write optimization */
	save_trunc_off	system.Off_t		/* write optimization */
//	struct hammer_btree_leaf_elm sync_ino_leaf; /* to-sync cache */
	sync_ino_leaf	hammer_btree_leaf_elm  /* to-sync cache */
//	struct hammer_inode_data sync_ino_data; /* to-sync cache */
	sync_ino_data	hammer_inode_data  /* to-sync cache */
//	size_t		redo_count;
	redo_count	uint
//

//	/*
//	 * Track the earliest offset in the UNDO/REDO FIFO containing
//	 * REDO records.  This is staged to the backend during flush
//	 * sequences.  While the inode is staged redo_fifo_next is used
//	 * to track the earliest offset for rotation into redo_fifo_start
//	 * on completion of the flush.
//	 */
//	hammer_off_t	redo_fifo_start;
	redo_fifo_start	hammer_off_t
//	hammer_off_t	redo_fifo_next;
	redo_fifo_next	hammer_off_t
//};
}
//

//typedef struct hammer_inode *hammer_inode_t;
//

//#define VTOI(vp)	((struct hammer_inode *)(vp)->v_data)
/* Expanding Macro */
//

///*
// * NOTE: DDIRTY does not include atime or mtime and does not include
// *	 write-append size changes.  SDIRTY handles write-append size
// *	 changes.
// *
// *	 REDO indicates that REDO logging is active, creating a definitive
// *	 stream of REDO records in the UNDO/REDO log for writes and
// *	 truncations, including boundary records when/if REDO is turned off.
// *	 REDO is typically enabled by fsync() and turned off if excessive
// *	 writes without an fsync() occurs.
// *
// *	 RDIRTY indicates that REDO records were laid down in the UNDO/REDO
// *	 FIFO (even if REDO is turned off some might still be active) and
// *	 still being tracked for this inode.  See hammer_redo.c
// */
//					/* (not including atime/mtime) */
//					/* (not including atime/mtime) */
//#define HAMMER_INODE_DDIRTY	0x0001	/* in-memory ino_data is dirty */
const HAMMER_INODE_DDIRTY	= 0x0001	/* in-memory ino_data is dirty */
//#define HAMMER_INODE_RSV_INODES	0x0002	/* hmp->rsv_inodes bumped */
const HAMMER_INODE_RSV_INODES	= 0x0002	/* hmp->rsv_inodes bumped */
//#define HAMMER_INODE_CONN_DOWN	0x0004	/* include in downward recursion */
const HAMMER_INODE_CONN_DOWN	= 0x0004	/* include in downward recursion */
//#define HAMMER_INODE_XDIRTY	0x0008	/* in-memory records */
const HAMMER_INODE_XDIRTY	= 0x0008	/* in-memory records */
//#define HAMMER_INODE_ONDISK	0x0010	/* inode is on-disk (else not yet) */
const HAMMER_INODE_ONDISK	= 0x0010	/* inode is on-disk (else not yet) */
//#define HAMMER_INODE_FLUSH	0x0020	/* flush on last ref */
const HAMMER_INODE_FLUSH	= 0x0020	/* flush on last ref */
//#define HAMMER_INODE_DELETED	0x0080	/* inode delete (backend) */
const HAMMER_INODE_DELETED	= 0x0080	/* inode delete (backend) */
//#define HAMMER_INODE_DELONDISK	0x0100	/* delete synchronized to disk */
const HAMMER_INODE_DELONDISK	= 0x0100	/* delete synchronized to disk */
//#define HAMMER_INODE_RO		0x0200	/* read-only (because of as-of) */
const HAMMER_INODE_RO		= 0x0200	/* read-only (because of as-of) */
//#define HAMMER_INODE_RECSW	0x0400	/* waiting on data record flush */
const HAMMER_INODE_RECSW	= 0x0400	/* waiting on data record flush */
//#define HAMMER_INODE_DONDISK	0x0800	/* data records may be on disk */
const HAMMER_INODE_DONDISK	= 0x0800	/* data records may be on disk */
//#define HAMMER_INODE_BUFS	0x1000	/* dirty high level bps present */
const HAMMER_INODE_BUFS		= 0x1000	/* dirty high level bps present */
//#define HAMMER_INODE_REFLUSH	0x2000	/* flush on dependancy / reflush */
const HAMMER_INODE_REFLUSH	= 0x2000	/* flush on dependancy / reflush */
//#define HAMMER_INODE_RECLAIM	0x4000	/* trying to reclaim */
const HAMMER_INODE_RECLAIM	= 0x4000	/* trying to reclaim */
//#define HAMMER_INODE_FLUSHW	0x8000	/* Someone waiting for flush */
const HAMMER_INODE_FLUSHW	= 0x8000	/* Someone waiting for flush */
//

//#define HAMMER_INODE_TRUNCATED	0x00010000
const HAMMER_INODE_TRUNCATED	= 0x00010000
//#define HAMMER_INODE_DELETING	0x00020000 /* inode delete request (frontend)*/
const HAMMER_INODE_DELETING	= 0x00020000 /* inode delete request (frontend)*/
//#define HAMMER_INODE_RESIGNAL	0x00040000 /* re-signal on re-flush */
const HAMMER_INODE_RESIGNAL	= 0x00040000 /* re-signal on re-flush */
//#define HAMMER_INODE_ATIME	0x00100000 /* in-memory atime modified */
const HAMMER_INODE_ATIME	= 0x00100000 /* in-memory atime modified */
//#define HAMMER_INODE_MTIME	0x00200000 /* in-memory mtime modified */
const HAMMER_INODE_MTIME	= 0x00200000 /* in-memory mtime modified */
//#define HAMMER_INODE_WOULDBLOCK 0x00400000 /* re-issue to new flush group */
const HAMMER_INODE_WOULDBLOCK 	= 0x00400000 /* re-issue to new flush group */
//#define HAMMER_INODE_DUMMY 	0x00800000 /* dummy inode covering bad file */
const HAMMER_INODE_DUMMY 	= 0x00800000 /* dummy inode covering bad file */
//#define HAMMER_INODE_SDIRTY	0x01000000 /* in-memory ino_data.size is dirty*/
const HAMMER_INODE_SDIRTY	= 0x01000000 /* in-memory ino_data.size is dirty*/
//#define HAMMER_INODE_REDO	0x02000000 /* REDO logging active */
const HAMMER_INODE_REDO		= 0x02000000 /* REDO logging active */
//#define HAMMER_INODE_RDIRTY	0x04000000 /* REDO records active in fifo */
const HAMMER_INODE_RDIRTY	= 0x04000000 /* REDO records active in fifo */
//#define HAMMER_INODE_SLAVEFLUSH	0x08000000 /* being flushed by slave */
const HAMMER_INODE_SLAVEFLUSH	= 0x08000000 /* being flushed by slave */
//

//#define HAMMER_INODE_MODMASK	(HAMMER_INODE_DDIRTY|HAMMER_INODE_SDIRTY|   \
//				 HAMMER_INODE_XDIRTY|HAMMER_INODE_BUFS|	    \
//				 HAMMER_INODE_ATIME|HAMMER_INODE_MTIME|     \
//				 HAMMER_INODE_TRUNCATED|HAMMER_INODE_DELETING)
const	HAMMER_INODE_MODMASK	= (HAMMER_INODE_DDIRTY|HAMMER_INODE_SDIRTY|HAMMER_INODE_XDIRTY|HAMMER_INODE_BUFS|HAMMER_INODE_ATIME|HAMMER_INODE_MTIME|HAMMER_INODE_TRUNCATED|HAMMER_INODE_DELETING)
//

//#define HAMMER_INODE_MODMASK_NOXDIRTY	\
//				(HAMMER_INODE_MODMASK & ~HAMMER_INODE_XDIRTY)
const	HAMMER_INODE_MODMASK_NOXDIRTY	= (HAMMER_INODE_MODMASK & ^HAMMER_INODE_XDIRTY)
//

//#define HAMMER_INODE_MODMASK_NOREDO	\
//				(HAMMER_INODE_DDIRTY|			    \
//				 HAMMER_INODE_XDIRTY|			    \
//				 HAMMER_INODE_TRUNCATED|HAMMER_INODE_DELETING)
const	HAMMER_INODE_MODMASK_NOREDO	= (HAMMER_INODE_DDIRTY|HAMMER_INODE_XDIRTY|HAMMER_INODE_TRUNCATED|HAMMER_INODE_DELETING)
//

//#define HAMMER_FLUSH_SIGNAL	0x0001
const	HAMMER_FLUSH_SIGNAL	= 0x0001
//#define HAMMER_FLUSH_RECURSION	0x0002
const	HAMMER_FLUSH_RECURSION	= 0x0002
//

///*
// * Used by the inode reclaim code to pipeline reclaims and avoid
// * blowing out kernel memory or letting the flusher get too far
// * behind.  The reclaim wakes up when count reaches 0 or the
// * timer expires.
// */
//struct hammer_reclaim {
type hammer_reclaim struct {
//	TAILQ_ENTRY(hammer_reclaim) entry;
	entry struct {
		tqe_next *hammer_reclaim	/* next element */
		tqe_prev **hammer_reclaim	/* address of previous next element */
		trace system.Qm_trace
	}
//	int	count;
	count	int
//};
}
//

///*
// * Track who is creating the greatest burden on the
// * inode cache.
// */
//struct hammer_inostats {
type hammer_inostats struct {
//	pid_t		pid;	/* track user process */
	pid		system.Pid_t	/* track user process */
//	int		ltick;	/* last tick */
	ltick		int	/* last tick */
//	int		count;	/* count (degenerates) */
	count		int	/* count (degenerates) */
//};
}
//
//#define HAMMER_INOSTATS_HSIZE	32
const	HAMMER_INOSTATS_HSIZE	= 32
//#define HAMMER_INOSTATS_HMASK	(HAMMER_INOSTATS_HSIZE - 1)
const	HAMMER_INOSTATS_HMASK	= (HAMMER_INOSTATS_HSIZE - 1)
//
///*
// * Structure used to represent an unsynchronized record in-memory.  These
// * records typically represent directory entries.  Only non-historical
// * records are kept in-memory.
// *
// * Records are organized as a per-inode RB-Tree.  If the inode is not
// * on disk then neither are any records and the in-memory record tree
// * represents the entire contents of the inode.  If the inode is on disk
// * then the on-disk B-Tree is scanned in parallel with the in-memory
// * RB-Tree to synthesize the current state of the file.
// *
// * Records are also used to enforce the ordering of directory create/delete
// * operations.  A new inode will not be flushed to disk unless its related
// * directory entry is also being flushed at the same time.  A directory entry
// * will not be removed unless its related inode is also being removed at the
// * same time.
// */
//typedef enum hammer_record_type {
type hammer_record_type int
const(
//	HAMMER_MEM_RECORD_GENERAL,	/* misc record */
	HAMMER_MEM_RECORD_GENERAL hammer_record_type = iota	/* misc record */
//	HAMMER_MEM_RECORD_INODE,	/* inode record */
	HAMMER_MEM_RECORD_INODE		/* inode record */
//	HAMMER_MEM_RECORD_ADD,		/* positive memory cache record */
	HAMMER_MEM_RECORD_ADD		/* positive memory cache record */
//	HAMMER_MEM_RECORD_DEL,		/* negative delete-on-disk record */
	HAMMER_MEM_RECORD_DEL		/* negative delete-on-disk record */
//	HAMMER_MEM_RECORD_DATA		/* bulk-data record w/on-disk ref */
	HAMMER_MEM_RECORD_DATA		/* bulk-data record w/on-disk ref */
)
//} hammer_record_type_t;
type hammer_record_type_t hammer_record_type
//

//struct hammer_record {
type hammer_record struct {
//	RB_ENTRY(hammer_record)		rb_node;
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_record		/* left element */
		rbe_right *hammer_record		/* right element */
		rbe_parent *hammer_record		/* parent element */
		rbe_color int			/* node color */
	}
//	TAILQ_ENTRY(hammer_record)	target_entry;
	/* Manually Expanded Macro */
	target_entry struct {
		tqe_next *hammer_record	/* next element */
		tqe_prev **hammer_record	/* address of previous next element */
		trace system.Qm_trace
	}
//	hammer_inode_state_t		flush_state;
	flush_state			hammer_inode_state_t
//	hammer_flush_group_t		flush_group;
	flush_group			hammer_flush_group_t
//	hammer_record_type_t		type;
	type_				hammer_record_type_t
//	struct hammer_lock		lock;
	lock				hammer_lock
//	struct hammer_reserve		*resv;
	resv				*hammer_reserve
//	struct hammer_inode		*ip;
	ip				*hammer_inode
//	struct hammer_inode		*target_ip;
	target_ip			*hammer_inode
//	struct hammer_btree_leaf_elm	leaf;
	leaf	 			hammer_btree_leaf_elm
//	union hammer_data_ondisk	*data;
	data				*hammer_data_ondisk
//	int				flags;
	flags				int
//	int				gflags;
	gflags				int
//	hammer_off_t			zone2_offset;	/* direct-write only */
	zone2_offset			hammer_off_t	/* direct-write only */
//};
}
//

//typedef struct hammer_record *hammer_record_t;
type	hammer_record_t		*hammer_record
//

///*
// * Record flags.  Note that FE can only be set by the frontend if the
// * record has not been interlocked by the backend w/ BE.
// */
//#define HAMMER_RECF_ALLOCDATA		0x0001
const HAMMER_RECF_ALLOCDATA		= 0x0001
//#define HAMMER_RECF_ONRBTREE		0x0002
const HAMMER_RECF_ONRBTREE		= 0x0002
//#define HAMMER_RECF_DELETED_FE		0x0004	/* deleted (frontend) */
const HAMMER_RECF_DELETED_FE		= 0x0004	/* deleted (frontend) */
//#define HAMMER_RECF_DELETED_BE		0x0008	/* deleted (backend) */
const HAMMER_RECF_DELETED_BE		= 0x0008	/* deleted (backend) */
//#define HAMMER_RECF_COMMITTED		0x0010	/* committed to the B-Tree */
const HAMMER_RECF_COMMITTED		= 0x0010	/* committed to the B-Tree */
//#define HAMMER_RECF_INTERLOCK_BE	0x0020	/* backend interlock */
const HAMMER_RECF_INTERLOCK_BE		= 0x0020	/* backend interlock */
//#define HAMMER_RECF_WANTED		0x0040	/* wanted by the frontend */
const HAMMER_RECF_WANTED		= 0x0040	/* wanted by the frontend */
//#define HAMMER_RECF_DEDUPED		0x0080	/* will be live-dedup'ed */
const HAMMER_RECF_DEDUPED		= 0x0080	/* will be live-dedup'ed */
//#define HAMMER_RECF_CONVERT_DELETE 	0x0100	/* special case */
const HAMMER_RECF_CONVERT_DELETE 	= 0x0100	/* special case */
//#define HAMMER_RECF_REDO		0x1000	/* REDO was laid down */
const HAMMER_RECF_REDO			= 0x1000	/* REDO was laid down */
//

///*
// * These flags must be separate to deal with SMP races
// */
//#define HAMMER_RECG_DIRECT_IO		0x0001	/* related direct I/O running*/
const HAMMER_RECG_DIRECT_IO		= 0x0001	/* related direct I/O running*/
//#define HAMMER_RECG_DIRECT_WAIT		0x0002	/* related direct I/O running*/
const HAMMER_RECG_DIRECT_WAIT		= 0x0002	/* related direct I/O running*/
//#define HAMMER_RECG_DIRECT_INVAL	0x0004	/* buffer alias invalidation */
const HAMMER_RECG_DIRECT_INVAL		= 0x0004	/* buffer alias invalidation */
///*
// * hammer_create_at_cursor() and hammer_delete_at_cursor() flags.
// */
//#define HAMMER_CREATE_MODE_UMIRROR	0x0001
const HAMMER_CREATE_MODE_UMIRROR	= 0x0001
//#define HAMMER_CREATE_MODE_SYS		0x0002
const HAMMER_CREATE_MODE_SYS		= 0x0002
//

//#define HAMMER_DELETE_ADJUST		0x0001
const HAMMER_DELETE_ADJUST		= 0x0001
//#define HAMMER_DELETE_DESTROY		0x0002
const HAMMER_DELETE_DESTROY		= 0x0002
//

///*
// * In-memory structures representing on-disk structures.
// */
//struct hammer_volume;
/* Already Defined */
//struct hammer_buffer;
/* Already Defined */
//struct hammer_node;
/* Already Defined */
//struct hammer_undo;
/* Already Defined */
//struct hammer_reserve;
/* Already Defined */
//

//RB_HEAD(hammer_vol_rb_tree, hammer_volume);
/* Manually Expanded Macro */
//struct hammer_vol_rb_tree {
type hammer_vol_rb_tree struct {
//	struct hammer_volume *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_volume 				/* root of the tree */
//	struct hammer_vol_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_vol_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_HEAD(hammer_buf_rb_tree, hammer_buffer);
/* Manually Expanded Macro */
//struct hammer_buf_rb_tree {
type hammer_buf_rb_tree struct {
//	struct hammer_buffer *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_buffer 				/* root of the tree */
//	struct hammer_buf_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_buf_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_HEAD(hammer_nod_rb_tree, hammer_node);
/* Manually Expanded Macro */
//struct hammer_nod_rb_tree {
type hammer_nod_rb_tree struct {
//	struct hammer_node *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_node 				/* root of the tree */
//	struct hammer_nod_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_nod_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_HEAD(hammer_und_rb_tree, hammer_undo);
/* Manually Expanded Macro */
//struct hammer_und_rb_tree {
type hammer_und_rb_tree struct {
//	struct hammer_undo *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_undo 				/* root of the tree */
//	struct hammer_und_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_und_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_HEAD(hammer_res_rb_tree, hammer_reserve);
/* Manually Expanded Macro */
//struct hammer_res_rb_tree {
type hammer_res_rb_tree struct {
//	struct hammer_reserve *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_reserve 				/* root of the tree */
//	struct hammer_res_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_res_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//RB_HEAD(hammer_mod_rb_tree, hammer_io);
/* Manually Expanded Macro */
//struct hammer_mod_rb_tree {
type hammer_mod_rb_tree struct {
//	struct hammer_io *rbh_root; 		     /* root of the tree */
	rbh_root *hammer_io 				/* root of the tree */
//	struct hammer_mod_rb_tree_scan_info *rbh_inprog; /* scans in progress */
	rbh_inprog *hammer_mod_rb_tree_scan_info	/* scans in progress */
//	struct spinlock rbh_spin;
	rbh_spin system.Spinlock
//}
}
//

//RB_PROTOTYPE2(hammer_vol_rb_tree, hammer_volume, rb_node,
//	      hammer_vol_rb_compare, int32_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_vol_rb_tree_scan_info {
type hammer_vol_rb_tree_scan_info struct {
//	struct hammer_vol_rb_tree_scan_info *link;
	link *hammer_vol_rb_tree_scan_info
//	struct hammer_volume	*node;
	node	*hammer_volume
//}
}
//RB_PROTOTYPE2(hammer_buf_rb_tree, hammer_buffer, rb_node,
//	      hammer_buf_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_buf_rb_tree_scan_info {
type hammer_buf_rb_tree_scan_info struct {
//	struct hammer_buf_rb_tree_scan_info *link;
	link *hammer_buf_rb_tree_scan_info
//	struct hammer_buffer	*node;
	node	*hammer_buffer
//}
}
//RB_PROTOTYPE2(hammer_nod_rb_tree, hammer_node, rb_node,
//	      hammer_nod_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_nod_rb_tree_scan_info {
type hammer_nod_rb_tree_scan_info struct {
//	struct hammer_nod_rb_tree_scan_info *link;
	link *hammer_nod_rb_tree_scan_info
//	struct hammer_io	*node;
	node	*hammer_io
//}
}
//RB_PROTOTYPE2(hammer_und_rb_tree, hammer_undo, rb_node,
//	      hammer_und_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_und_rb_tree_scan_info {
type hammer_und_rb_tree_scan_info struct {
//	struct hammer_und_rb_tree_scan_info *link;
	link *hammer_und_rb_tree_scan_info
//	struct hammer_undo	*node;
	node	*hammer_undo
//}
}
//RB_PROTOTYPE2(hammer_res_rb_tree, hammer_reserve, rb_node,
//	      hammer_res_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded */
//struct hammer_res_rb_tree_scan_info {
type hammer_res_rb_tree_scan_info struct {
//	struct hammer_res_rb_tree_scan_info *link;
	link *hammer_res_rb_tree_scan_info
//	struct hammer_reserve	*node;
	node	*hammer_reserve
//}
}
//RB_PROTOTYPE2(hammer_mod_rb_tree, hammer_io, rb_node,
//	      hammer_mod_rb_compare, hammer_off_t);
/* Manually Expanded Macro */
/* Partially Expanded And Partially Translated */
//struct hammer_io *hammer_mod_rb_tree_RB_REMOVE(struct hammer_mod_rb_tree *, struct hammer_io *);
//struct hammer_io *hammer_mod_rb_tree_RB_INSERT(struct hammer_mod_rb_tree *, struct hammer_io *);
//struct hammer_io *hammer_mod_rb_tree_RB_FIND(struct hammer_mod_rb_tree *, struct hammer_io *);
//int hammer_mod_rb_tree_RB_SCAN(struct hammer_mod_rb_tree *, int (*)(struct hammer_io *, void *),
//			int (*)(struct hammer_io *, void *), void *);
//struct hammer_io *hammer_mod_rb_tree_RB_NEXT(struct hammer_io *);
//struct hammer_io *hammer_mod_rb_tree_RB_PREV(struct hammer_io *);
//struct hammer_io *hammer_mod_rb_tree_RB_MINMAX(struct hammer_mod_rb_tree *, int);
//struct hammer_mod_rb_tree_scan_info {
type hammer_mod_rb_tree_scan_info struct {
//	struct hammer_mod_rb_tree_scan_info *link;
	link *hammer_mod_rb_tree_scan_info
//	struct hammer_io	*node;
	node	*hammer_io
//}
}
//struct hammer_io *hammer_mod_rb_tree_RB_LOOKUP(struct hammer_mod_rb_tree *, hammer_off_t)
//

///*
// * IO management - embedded at the head of various in-memory structures
// *
// * VOLUME	- hammer_volume containing meta-data
// * META_BUFFER	- hammer_buffer containing meta-data
// * DATA_BUFFER	- hammer_buffer containing pure-data
// *
// * Dirty volume headers and dirty meta-data buffers are locked until the
// * flusher can sequence them out.  Dirty pure-data buffers can be written.
// * Clean buffers can be passively released.
// */
//typedef enum hammer_io_type {
type hammer_io_type int
const(
//	HAMMER_STRUCTURE_VOLUME,
	HAMMER_STRUCTURE_VOLUME hammer_io_type = iota
//	HAMMER_STRUCTURE_META_BUFFER,
	HAMMER_STRUCTURE_META_BUFFER
//	HAMMER_STRUCTURE_UNDO_BUFFER,
	HAMMER_STRUCTURE_UNDO_BUFFER
//	HAMMER_STRUCTURE_DATA_BUFFER,
	HAMMER_STRUCTURE_DATA_BUFFER
//	HAMMER_STRUCTURE_DUMMY
	HAMMER_STRUCTURE_DUMMY
//} hammer_io_type_t;
)
type hammer_io_type_t hammer_io_type
//

//union hammer_io_structure;
/* Already Defined */
//struct hammer_io;
/* Already Defined */
//

//struct worklist {
type worklist struct {
//	LIST_ENTRY(worklist) node;
	/* Manually Expanded Macro */
	node struct {
		le_next *worklist	/* next element */
		le_prev **worklist	/* address of previous next element */
	}
//};
}
//

//TAILQ_HEAD(hammer_io_list, hammer_io);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
/* Manually Expanded Macro */
type hammer_io_list struct {
	tqh_first *hammer_io	/* first element */
	tqh_last **hammer_io	/* addr of last next element */
	trace system.Qm_trace
}
//typedef struct hammer_io_list *hammer_io_list_t;
type hammer_io_list_t *hammer_io_list
//

//struct hammer_io {
type hammer_io struct {
//	struct worklist		worklist;
	worklist_		worklist
//	struct hammer_lock	lock;
	lock			hammer_lock
//	enum hammer_io_type	type;
	type_			*hammer_io_type
//	struct hammer_mount	*hmp;
	hmp			*hammer_mount
//	struct hammer_volume	*volume;
	volume			*hammer_volume
//	RB_ENTRY(hammer_io)	rb_node;     /* if modified */
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_io		/* left element */
		rbe_right *hammer_io		/* right element */
		rbe_parent *hammer_io		/* parent element */
		rbe_color int			/* node color */
	}
//	TAILQ_ENTRY(hammer_io)	iorun_entry; /* iorun_list */
	/* Manually Expanded Macro */
	iorun_entry struct {		/* iorun_list */
		tqe_next *hammer_io	/* next element */
		tqe_prev **hammer_io	/* address of previous next element */
		trace system.Qm_trace
	}
//	struct hammer_mod_rb_tree *mod_root;
	mod_root 		*hammer_mod_rb_tree
//	struct buf		*bp;
	bp			*system.Buf
//	int64_t			offset;	   /* zone-2 offset */
	offset			int64	   /* zone-2 offset */
//	int			bytes;	   /* buffer cache buffer size */
	bytes			int	   /* buffer cache buffer size */
//	int			modify_refs;
	modify_refs		int
//

//	/*
//	 * These can be modified at any time by the backend while holding
//	 * io_token, due to bio_done and hammer_io_complete() callbacks.
//	 */
//	u_int		running : 1;	/* bp write IO in progress */
	running		bool		/* bp write IO in progress */
//	u_int		waiting : 1;	/* someone is waiting on us */
	waiting 	bool		/* someone is waiting on us */
//	u_int		ioerror : 1;	/* abort on io-error */
	ioerror		bool		/* abort on io-error */
//	u_int		unusedA : 29;
	unusedA		[29]bool
//

//	/*
//	 * These can only be modified by the frontend while holding
//	 * fs_token, or by the backend while holding the io interlocked
//	 * with no references (which will block the frontend when it
//	 * tries to reference it).
//	 *
//	 * WARNING! SMP RACES will create havoc if the callbacks ever tried
//	 *	    to modify any of these outside the above restrictions.
//	 */
//	u_int		modified : 1;	/* bp's data was modified */
	modified	uint	/* bp's data was modified */
//	u_int		released : 1;	/* bp released (w/ B_LOCKED set) */
	released	uint	/* bp released (w/ B_LOCKED set) */
//	u_int		validated : 1;	/* ondisk has been validated */
	validated	uint	/* ondisk has been validated */
//	u_int		waitdep : 1;	/* flush waits for dependancies */
	waitdep		uint	/* flush waits for dependancies */
//	u_int		recovered : 1;	/* has recovery ref */
	recovered	uint	/* has recovery ref */
//	u_int		waitmod : 1;	/* waiting for modify_refs */
	waitmod		uint	/* waiting for modify_refs */
//	u_int		reclaim : 1;	/* reclaim requested */
	reclaim		uint	/* reclaim requested */
//	u_int		gencrc : 1;	/* crc needs to be generated */
	gencrc		uint
//	u_int		unusedB : 24;
	unusedB		uint
//};
}
//

//typedef struct hammer_io *hammer_io_t;
type hammer_io_t *hammer_io
//

//#define HAMMER_CLUSTER_SIZE	(64 * 1024)
const	HAMMER_CLUSTER_SIZE	= (64 * 1024)
//#if HAMMER_CLUSTER_SIZE > MAXBSIZE
//#undef  HAMMER_CLUSTER_SIZE
//#define HAMMER_CLUSTER_SIZE	MAXBSIZE
/* Max byte size is normally 65536. Set it below if it's smaller. It must be a power of two. */
//const	HAMMER_CLUSTER_SIZE	= system.MAXBSIZE
//#endif
//#define HAMMER_CLUSTER_BUFS	(HAMMER_CLUSTER_SIZE / HAMMER_BUFSIZE)
const	HAMMER_CLUSTER_BUFS	= (HAMMER_CLUSTER_SIZE / HAMMER_BUFSIZE)
//

///*
// * In-memory volume representing on-disk buffer
// */
//struct hammer_volume {
type hammer_volume struct {
//	struct hammer_io io;
	io hammer_io
//	RB_ENTRY(hammer_volume) rb_node;
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_volume		/* left element */
		rbe_right *hammer_volume	/* right element */
		rbe_parent *hammer_volume	/* parent element */
		rbe_color int			/* node color */
	}
//	struct hammer_volume_ondisk *ondisk;
	ondisk *hammer_volume_ondisk
//	int32_t	vol_no;
	vol_no int32
//	int64_t nblocks;	/* note: special calculation for statfs */
	nblocks int64		/* note: special calculation for statfs */
//	int64_t buffer_base;	/* base offset of buffer 0 */
	buffer_base int64 	/* base offset of buffer 0 */
//	hammer_off_t maxbuf_off; /* Maximum buffer offset (zone-2) */
	maxbuf_off hammer_off_t  /* Maximum buffer offset (zone-2) */
//	hammer_off_t maxraw_off; /* Maximum raw offset for device */
	maxraw_off hammer_off_t  /* Maximum raw offset for device */
//	char	*vol_name;
	vol_name string
//	struct vnode *devvp;
	devvp *system.Vnode
//	int	vol_flags;
	vol_flags int
//};
}
//

//typedef struct hammer_volume *hammer_volume_t;
type hammer_volume_t *hammer_volume
//

///*
// * In-memory buffer (other then volume, super-cluster, or cluster),
// * representing an on-disk buffer.
// */
//struct hammer_buffer {
type hammer_buffer struct {
//	struct hammer_io io;
	io hammer_io
//	RB_ENTRY(hammer_buffer) rb_node;
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_buffer		/* left element */
		rbe_right *hammer_buffer	/* right element */
		rbe_parent *hammer_buffer	/* parent element */
		rbe_color int			/* node color */
	}
//	void *ondisk;
	ondisk *interface{}
//	hammer_off_t zoneX_offset;
	zoneX_offset hammer_off_t
//	hammer_off_t zone2_offset;
	zone2_offset hammer_off_t
//	struct hammer_reserve *resv;
	resv *hammer_reserve
//	struct hammer_node_list clist;
	clist hammer_node_list
//};
}
//

//typedef struct hammer_buffer *hammer_buffer_t;
type hammer_buffer_t *hammer_buffer
//

///*
// * In-memory B-Tree node, representing an on-disk B-Tree node.
// *
// * This is a hang-on structure which is backed by a hammer_buffer,
// * indexed by a hammer_cluster, and used for fine-grained locking of
// * B-Tree nodes in order to properly control lock ordering.  A hammer_buffer
// * can contain multiple nodes representing wildly disassociated portions
// * of the B-Tree so locking cannot be done on a buffer-by-buffer basis.
// *
// * This structure uses a cluster-relative index to reduce the number
// * of layers required to access it, and also because all on-disk B-Tree
// * references are cluster-relative offsets.
// */
//struct hammer_node {
type hammer_node struct {
//	struct hammer_lock	lock;		/* node-by-node lock */
	lock			hammer_lock	/* node-by-node lock */
//	TAILQ_ENTRY(hammer_node) entry;		/* per-buffer linkage */
	entry struct {				/* per-buffer linkage */
		tqe_next *hammer_node		/* next element */
		tqe_prev **hammer_node		/* address of previous next element */
		trace system.Qm_trace
	}
//	RB_ENTRY(hammer_node)	rb_node;	/* per-cluster linkage */
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_node		/* left element */
		rbe_right *hammer_node		/* right element */
		rbe_parent *hammer_node		/* parent element */
		rbe_color int			/* node color */
	}
//	hammer_off_t		node_offset;	/* full offset spec */
	node_offset	hammer_off_t		/* full offset spec */
//	struct hammer_mount	*hmp;
	hmp			*hammer_mount
//	struct hammer_buffer	*buffer;	/* backing buffer */
	buffer			*hammer_buffer	/* backing buffer */
//	hammer_node_ondisk_t	ondisk;		/* ptr to on-disk structure */
	ondisk		hammer_node_ondisk_t	/* ptr to on-disk structure */
//	TAILQ_HEAD(, hammer_cursor) cursor_list;  /* deadlock recovery */
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	cursor_list struct {
		tqh_first *hammer_cursor	/* first element */
		tqh_last **hammer_cursor	/* addr of last next element */
		trace system.Qm_trace
	} 

//	struct hammer_node_cache_list cache_list; /* passive caches */
	cache_list	hammer_node_cache_list	/* passive caches */
//	int			flags;
	flags 			int
//	int			cursor_exclreq_count;
	cursor_exclreq_count 	int
//};
}
//

//#define HAMMER_NODE_DELETED	0x0001
const HAMMER_NODE_DELETED	= 0x0001
//#define HAMMER_NODE_FLUSH	0x0002
const HAMMER_NODE_FLUSH		= 0x0002
//#define HAMMER_NODE_CRCGOOD	0x0004
const HAMMER_NODE_CRCGOOD	= 0x0004
//#define HAMMER_NODE_NEEDSCRC	0x0008
const HAMMER_NODE_NEEDSCRC	= 0x0008
//#define HAMMER_NODE_NEEDSMIRROR	0x0010
const HAMMER_NODE_NEEDSMIRROR	= 0x0010
//#define HAMMER_NODE_CRCBAD	0x0020
const HAMMER_NODE_CRCBAD	= 0x0020
//#define HAMMER_NODE_NONLINEAR	0x0040		/* linear heuristic */
const HAMMER_NODE_NONLINEAR	= 0x0040	/* linear heuristic */
//

//#define HAMMER_NODE_CRCANY	(HAMMER_NODE_CRCGOOD | HAMMER_NODE_CRCBAD)
const HAMMER_NODE_CRCANY	= (HAMMER_NODE_CRCGOOD | HAMMER_NODE_CRCBAD)
//

//typedef struct hammer_node	*hammer_node_t;
type hammer_node_t		*hammer_node
//

///*
// * List of locked nodes.  This structure is used to lock potentially large
// * numbers of nodes as an aid for complex B-Tree operations.
// */
//struct hammer_node_lock;
/* Already Defined */
//TAILQ_HEAD(hammer_node_lock_list, hammer_node_lock);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
/* Manually Expanded Macro */
type hammer_node_lock_list struct {
	tqh_first *hammer_node_lock	/* first element */
	tqh_last **hammer_node_lock	/* addr of last next element */
	trace system.Qm_trace
}
//

//struct hammer_node_lock {
type hammer_node_lock struct {
//	TAILQ_ENTRY(hammer_node_lock) entry;
	/* Manually Expanded Macro */
	entry struct {
		tqe_next *hammer_node_lock	/* next element */
		tqe_prev **hammer_node_lock	/* address of previous next element */
		trace system.Qm_trace
	}
//	struct hammer_node_lock_list  list;
	list 	hammer_node_lock_list
//	struct hammer_node_lock	      *parent;
	parent	*hammer_node_lock
//	hammer_node_t	node;
	node	hammer_node_t
//	hammer_node_ondisk_t copy;	/* copy of on-disk data */
	copy	hammer_node_ondisk_t	/* copy of on-disk data */
//	int		index;		/* index of this node in parent */
	index		int
//	int		count;		/* count children */
	count		int		/* count children */
//	int		flags;
	flags		int
//};
}
//

//typedef struct hammer_node_lock *hammer_node_lock_t;
type hammer_node_lock_t *hammer_node_lock
//

//#define HAMMER_NODE_LOCK_UPDATED	0x0001
const HAMMER_NODE_LOCK_UPDATED		= 0x0001
//#define HAMMER_NODE_LOCK_LCACHE		0x0002
const HAMMER_NODE_LOCK_LCACHE		= 0x0002
//

///*
// * Common I/O management structure - embedded in in-memory structures
// * which are backed by filesystem buffers.
// */
//union hammer_io_structure {
type hammer_io_structure []byte
//	struct hammer_io	io;
func (h *hammer_io_structure) io(index int) (io *hammer_io) {
	binary.Read(bytes.NewReader(*h), binary.LittleEndian, io)
	return io
}
//	struct hammer_volume	volume;
func (h *hammer_io_structure) volume(index int) (volume *hammer_volume) {
	binary.Read(bytes.NewReader(*h), binary.LittleEndian, volume)
	return volume
}
//	struct hammer_buffer	buffer;
func (h *hammer_io_structure) buffer(index int) (buffer *hammer_buffer) {
	binary.Read(bytes.NewReader(*h), binary.LittleEndian, buffer)
	return buffer
}

func (h *hammer_io_structure) write(hammer_io_structure interface{}) {
	binary.Write(bytes.NewBuffer(*h), binary.LittleEndian, hammer_io_structure)
//};
}
//

//typedef union hammer_io_structure *hammer_io_structure_t;
type hammer_io_structure_t *hammer_io_structure
//

///*
// * The reserve structure prevents the blockmap from allocating
// * out of a reserved bigblock.  Such reservations are used by
// * the direct-write mechanism.
// *
// * The structure is also used to hold off on reallocations of
// * big blocks from the freemap until flush dependancies have
// * been dealt with.
// */
//struct hammer_reserve {
type hammer_reserve struct {
//	RB_ENTRY(hammer_reserve) rb_node;
	/* Manually Expanded Macro */
	rbnode struct {
		rbe_left *hammer_reserve		/* left element */
		rbe_right *hammer_reserve		/* right element */
		rbe_parent *hammer_reserve	/* parent element */
		rbe_color int			/* node color */
	}
//	TAILQ_ENTRY(hammer_reserve) delay_entry;
	/* Manually Expanded Macro */
	delay_entry struct {
		tqe_next *hammer_reserve	/* next element */
		tqe_prev **hammer_reserve	/* address of previous next element */
		trace system.Qm_trace
	}
//	int		flush_group;
	flush_group	int
//	int		flags;
	flags		int
//	int		refs;
	refs		int
//	int		zone;
	zone		int
//	int		append_off;
	append_off	int
//	int32_t		bytes_free;
	bytes_free	int32
//	hammer_off_t	zone_offset;
	zone_offset	hammer_off_t
//};
}
//

//typedef struct hammer_reserve *hammer_reserve_t;
type hammer_reserve_t *hammer_reserve
//

//#define HAMMER_RESF_ONDELAY	0x0001
const HAMMER_RESF_ONDELAY	= 0x0001
//#define HAMMER_RESF_LAYER2FREE	0x0002
const HAMMER_RESF_LAYER2FREE	= 0x0002
//

//#include "hammer_cursor.h"

//

///*
// * The undo structure tracks recent undos to avoid laying down duplicate
// * undos within a flush group, saving us a significant amount of overhead.
// *
// * This is strictly a heuristic.
// */
//#define HAMMER_MAX_UNDOS		1024
const HAMMER_MAX_UNDOS			= 1024
//#define HAMMER_MAX_FLUSHERS		4
const HAMMER_MAX_FLUSHERS		= 4
//

//struct hammer_undo {
type hammer_undo struct {
//	RB_ENTRY(hammer_undo)	rb_node;
	/* Manually Expanded Macro */
	rb_node struct {
		rbe_left *hammer_undo	/* left element */
		rbe_right *hammer_undo	/* right element */
		rbe_parent *hammer_undo	/* parent element */
		rbe_color int		/* node color */
	}
//	TAILQ_ENTRY(hammer_undo) lru_entry;
	/* Manually Expanded Macro */
	lru_entry struct {
		tqe_next *hammer_undo	/* next element */
		tqe_prev **hammer_undo	/* address of previous next element */
		trace system.Qm_trace
	}
//	hammer_off_t		offset;
	offset			hammer_off_t
//	int			bytes;
	bytes			int
//};
}
//

//typedef struct hammer_undo *hammer_undo_t;
type hammer_undo_t *hammer_undo
//

//struct hammer_flusher_info;
type hammer_flusher_info struct {}
//TAILQ_HEAD(hammer_flusher_info_list, hammer_flusher_info);
/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
type hammer_flusher_info_list struct {
	tqh_first *hammer_flusher_info	/* first element */
	tqh_last **hammer_flusher_info	/* addr of last next element */
	trace system.Qm_trace
}
//

//struct hammer_flusher {
type hammer_flusher struct {
//	int		signal;		/* flusher thread sequencer */
	signal		int		/* flusher thread sequencer */
//	int		done;		/* last completed flush group */
	done		int		/* last completed flush group */
//	int		next;		/* next unallocated flg seqno */
	next		int		/* next unallocated flg seqno */
//	int		group_lock;	/* lock sequencing of the next flush */
	group_lock	int		/* lock sequencing of the next flush */
//	int		exiting;	/* request master exit */
	exiting		int		/* request master exit */
//	thread_t	td;		/* master flusher thread */
	td		system.Thread_t	/* master flusher thread */
//	hammer_tid_t	tid;		/* last flushed transaction id */
	tid		hammer_tid_t		/* last flushed transaction id */
//	int		finalize_want;		/* serialize finalization */
	finalize_want	int			/* serialize finalization */
//	struct hammer_lock finalize_lock;	/* serialize finalization */
	finalize_lock	hammer_lock		/* serialize finalization */
//	struct hammer_transaction trans;	/* shared transaction */
	trans		hammer_transaction	/* shared transaction */
//	struct hammer_flusher_info_list run_list;
	run_list	hammer_flusher_info_list
//	struct hammer_flusher_info_list ready_list;
	ready_list	hammer_flusher_info_list
//};
}
//

//#define HAMMER_FLUSH_UNDOS_RELAXED	0
const HAMMER_FLUSH_UNDOS_RELAXED	= 0
//#define HAMMER_FLUSH_UNDOS_FORCED	1
const HAMMER_FLUSH_UNDOS_FORCED		= 1
//#define HAMMER_FLUSH_UNDOS_AUTO		2
const HAMMER_FLUSH_UNDOS_AUTO		= 2
///*
// * Internal hammer mount data structure
// */
//struct hammer_mount {
type hammer_mount struct {
//	struct mount *mp;
	mp *system.Mount
//	/*struct vnode *rootvp;*/
	/*struct vnode *rootvp;*/
//	struct hammer_ino_rb_tree rb_inos_root;
	rb_inos_root hammer_ino_rb_tree
//	struct hammer_redo_rb_tree rb_redo_root;
	rb_redo_root hammer_redo_rb_tree
//	struct hammer_vol_rb_tree rb_vols_root;
	rb_vols_root hammer_vol_rb_tree
//	struct hammer_nod_rb_tree rb_nods_root;
	rb_nods_root hammer_nod_rb_tree
//	struct hammer_und_rb_tree rb_undo_root;
	rb_undo_root hammer_und_rb_tree
//	struct hammer_res_rb_tree rb_resv_root;
	rb_resv_root hammer_res_rb_tree
//	struct hammer_buf_rb_tree rb_bufs_root;
	rb_bufs_root hammer_buf_rb_tree
//	struct hammer_pfs_rb_tree rb_pfsm_root;
	rb_pfsm_root hammer_pfs_rb_tree
//

//	struct hammer_dedup_crc_rb_tree rb_dedup_crc_root;
	rb_dedup_crc_root hammer_dedup_crc_rb_tree
//	struct hammer_dedup_off_rb_tree rb_dedup_off_root;
	rb_dedup_off_root hammer_dedup_off_rb_tree
//

//	struct hammer_volume *rootvol;
	rootvol			*hammer_volume
//	struct hammer_base_elm root_btree_beg;
	root_btree_beg		hammer_base_elm
//	struct hammer_base_elm root_btree_end;
	root_btree_end		hammer_base_elm
//
//	struct malloc_type	*m_misc;
	m_misc			*system.Malloc_type
//	struct malloc_type	*m_inodes;
	m_inodes		*system.Malloc_type
//

//	int	flags;		/* HAMMER_MOUNT_xxx flags */
	flags		int	/* HAMMER_MOUNT_xxx flags */
//	int	hflags;
	hflags		int
//	int	ronly;
	ronly		int
//	int	nvolumes;
	nvolumes	int
//	int	volume_iterator;
	volume_iterator	int
//	int	master_id;	/* -1 or 0-15 - clustering and mirroring */
	master_id	int	/* -1 or 0-15 - clustering and mirroring */
//	int	version;	/* hammer filesystem version to use */
	version		int	/* hammer filesystem version to use */
//	int	rsv_inodes;	/* reserved space due to dirty inodes */
	rsv_inodes	int	/* reserved space due to dirty inodes */
//	int64_t	rsv_databytes;	/* reserved space due to record data */
	rsv_databytes	int64	/* reserved space due to record data */
//	int	rsv_recs;	/* reserved space due to dirty records */
	rsv_recs	int	/* reserved space due to dirty records */
//	int	rsv_fromdelay;	/* bigblocks reserved due to flush delay */
	rsv_fromdelay	int	/* bigblocks reserved due to flush delay */
//	int	undo_rec_limit;	/* based on size of undo area */
	undo_rec_limit	int	/* based on size of undo area */
//	int	last_newrecords;
	last_newrecords	int
//	int	count_newrecords;
	count_newrecords	int
//

//	int	volume_to_remove; /* volume that is currently being removed */
	volume_to_remove	int	/* volume that is currently being removed */
//

//	int	count_inodes;	/* total number of inodes */
	count_inodes	int	/* total number of inodes */
//	int	count_iqueued;	/* inodes queued to flusher */
	count_iqueued	int	/* inodes queued to flusher */
//	int	count_reclaims; /* inodes pending reclaim by flusher */
	count_reclaims	int	/* inodes pending reclaim by flusher */
//

//	struct hammer_flusher flusher;
	flusher hammer_flusher
//

//	u_int	check_interrupt;
	check_interrupt		uint
//	u_int	check_yield;
	check_yield		uint
//	uuid_t	fsid;
	fsid			system.Uuid_t
//	struct hammer_mod_rb_tree volu_root;	/* dirty undo buffers */
	volu_root	hammer_mod_rb_tree 	/* dirty undo buffers */
//	struct hammer_mod_rb_tree undo_root;	/* dirty undo buffers */
	undo_root	hammer_mod_rb_tree 	/* dirty undo buffers */
//	struct hammer_mod_rb_tree data_root;	/* dirty data buffers */
	data_root	hammer_mod_rb_tree 	/* dirty data buffers */
//	struct hammer_mod_rb_tree meta_root;	/* dirty meta bufs    */
	meta_root	hammer_mod_rb_tree 	/* dirty meta bufs    */
//	struct hammer_mod_rb_tree lose_root;	/* loose buffers      */
	lose_root	hammer_mod_rb_tree 	/* loose buffers      */
//	long	locked_dirty_space;		/* meta/volu count    */
	locked_dirty_space	int32		/* meta/volu count    */
//	long	io_running_space;		/* io_token */
	io_running_space	int32		/* io_token */
//	int	unused01;
	unused01		int
//	int	objid_cache_count;
	objid_cache_count	int
//	int	dedup_cache_count;
	dedup_cache_count	int
//	int	error;				/* critical I/O error */
	error			int		/* critical I/O error */
//	struct krate	krate;			/* rate limited kprintf */
	krate	 		system.Krate	/* rate limited kprintf */
//	hammer_tid_t	asof;			/* snapshot mount */
	asof			hammer_tid_t	/* snapshot mount */
//	hammer_tid_t	next_tid;
	next_tid		hammer_tid_t
//	hammer_tid_t	flush_tid1;		/* flusher tid sequencing */
	flush_tid1		hammer_tid_t	/* flusher tid sequencing */
//	hammer_tid_t	flush_tid2;		/* flusher tid sequencing */
	flush_tid2		hammer_tid_t	/* flusher tid sequencing */
//	int64_t copy_stat_freebigblocks;	/* number of free bigblocks */
	copy_stat_freebigblocks	int64 		/* number of free bigblocks */
//	u_int32_t	undo_seqno;		/* UNDO/REDO FIFO seqno */
	undo_seqno		uint32		/* UNDO/REDO FIFO seqno */
//	u_int32_t	recover_stage2_seqno;	/* REDO recovery seqno */
	recover_stage2_seqno	uint32		/* REDO recovery seqno */
//	hammer_off_t	recover_stage2_offset;	/* REDO recovery offset */
	recover_stage2_offset	hammer_off_t	/* REDO recovery offset */
//

//	struct netexport export;
	export		system.Netexport
//	struct hammer_lock sync_lock;
	sync_lock	hammer_lock
//	struct hammer_lock free_lock;
	free_lock	hammer_lock
//	struct hammer_lock undo_lock;
	undo_lock	hammer_lock
//	struct hammer_lock blkmap_lock;
	blkmap_lock	hammer_lock
//	struct hammer_lock snapshot_lock;
	snapshot_lock	hammer_lock
//	struct hammer_lock volume_lock;
	volume_lock	hammer_lock
//	struct hammer_blockmap  blockmap[HAMMER_MAX_ZONES];
	blockmap	[HAMMER_MAX_ZONES]hammer_blockmap
//	struct hammer_undo	undos[HAMMER_MAX_UNDOS];
	undos		[HAMMER_MAX_UNDOS]hammer_undo
//	int			undo_alloc;
	undo_alloc	int
//	TAILQ_HEAD(, hammer_undo)  undo_lru_list;
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	undo_lru_list struct {
		tqh_first *hammer_undo	/* first element */
		tqh_last **hammer_undo	/* addr of last next element */
		trace system.Qm_trace}
//	TAILQ_HEAD(, hammer_reserve) delay_list;
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	delay_list struct {
		tqh_first *hammer_reserve	/* first element */
		tqh_last **hammer_reserve	/* addr of last next element */
		trace system.Qm_trace
	}
//	struct hammer_flush_group_list	flush_group_list;
	flush_group_list hammer_flush_group_list
//	hammer_flush_group_t	fill_flush_group;
	fill_flush_group	hammer_flush_group_t
//	hammer_flush_group_t	next_flush_group;
	next_flush_group	hammer_flush_group_t
//	TAILQ_HEAD(, hammer_objid_cache) objid_cache_list;
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	objid_cache_list struct {
		tqh_first *hammer_objid_cache	/* first element */
		tqh_last **hammer_objid_cache	/* addr of last next element */
		trace system.Qm_trace
	}
//	TAILQ_HEAD(, hammer_dedup_cache) dedup_lru_list;
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	dedup_lru_list struct {
		tqh_first *hammer_dedup_cache	/* first element */
		tqh_last **hammer_dedup_cache	/* addr of last next element */
		trace system.Qm_trace
	}
//	hammer_dedup_cache_t	dedup_free_cache;
	dedup_free_cache	hammer_dedup_cache_t
//	TAILQ_HEAD(, hammer_reclaim) reclaim_list;
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	reclaim_list struct {
		tqh_first *hammer_reclaim	/* first element */
		tqh_last **hammer_reclaim	/* addr of last next element */
		trace system.Qm_trace
	}
//	TAILQ_HEAD(, hammer_io) iorun_list;
	/* TAILQ_HEAD defined in hammer/dependencies/system/queue.go */
	/* Manually Expanded Macro */
	iorun_list struct {
		tqh_first *hammer_io	/* first element */
		tqh_last **hammer_io	/* addr of last next element */
		trace system.Qm_trace
	}
//

//	struct lwkt_token	fs_token;	/* high level */
	fs_token	system.Lwkt_token	/* high level */
//	struct lwkt_token	io_token;	/* low level (IO callback) */
	io_token	system.Lwkt_token	/* low level (IO callback) */
//

//	struct hammer_inostats	inostats[HAMMER_INOSTATS_HSIZE];
	inostats [HAMMER_INOSTATS_HSIZE]hammer_inostats
//};
}
//

//typedef struct hammer_mount	*hammer_mount_t;
type hammer_mount_t *hammer_mount
//

//#define HAMMER_MOUNT_CRITICAL_ERROR	0x0001
const HAMMER_MOUNT_CRITICAL_ERROR	= 0x0001
//#define HAMMER_MOUNT_FLUSH_RECOVERY	0x0002
const HAMMER_MOUNT_FLUSH_RECOVERY	= 0x0002
//#define HAMMER_MOUNT_REDO_SYNC		0x0004
const HAMMER_MOUNT_REDO_SYNC		= 0x0004
//#define HAMMER_MOUNT_REDO_RECOVERY_REQ	0x0008
const HAMMER_MOUNT_REDO_RECOVERY_REQ	= 0x0008
//#define HAMMER_MOUNT_REDO_RECOVERY_RUN	0x0010
const HAMMER_MOUNT_REDO_RECOVERY_RUN	= 0x0010
//

//struct hammer_sync_info {
type hammer_sync_info struct {
//	int error;
	error int
//	int waitfor;
	waitfor int
//};
}
//

///*
// * Minium buffer cache bufs required to rebalance the B-Tree.
// * This is because we must hold the children and the children's children
// * locked.  Even this might not be enough if things are horribly out
// * of balance.
// */
//#define HAMMER_REBALANCE_MIN_BUFS	\
//	(HAMMER_BTREE_LEAF_ELMS * HAMMER_BTREE_LEAF_ELMS)
const HAMMER_REBALANCE_MIN_BUFS = (HAMMER_BTREE_LEAF_ELMS * HAMMER_BTREE_LEAF_ELMS)
//

//

//#endif
/* no matching ifdef */
//

///*
// * checkspace slop (8MB chunks), higher numbers are more conservative.
// */
//#define HAMMER_CHKSPC_REBLOCK	25
const HAMMER_CHKSPC_REBLOCK	= 25
//#define HAMMER_CHKSPC_MIRROR	20
const HAMMER_CHKSPC_MIRROR	= 20
//#define HAMMER_CHKSPC_WRITE	20
const HAMMER_CHKSPC_WRITE	= 20
//#define HAMMER_CHKSPC_CREATE	20
const HAMMER_CHKSPC_CREATE	= 20
//#define HAMMER_CHKSPC_REMOVE	10
const HAMMER_CHKSPC_REMOVE	= 10
//#define HAMMER_CHKSPC_EMERGENCY	0
const HAMMER_CHKSPC_EMERGENCY	= 0
//

//#if defined(_KERNEL)
/* ends at #endif */
//

//extern struct vop_ops hammer_vnode_vops;
var hammer_vnode_vops vop_ops
//extern struct vop_ops hammer_spec_vops;
var hammer_spec_vops vop_ops
//extern struct vop_ops hammer_fifo_vops;
var hammer_fifo_vops vop_ops
//extern struct bio_ops hammer_bioops;
var hammer_bioops bio_ops
//

//extern int hammer_debug_io;
var hammer_debug_io int
//extern int hammer_debug_general;
var hammer_debug_general int
//extern int hammer_debug_debug;
var hammer_debug_debug int
//extern int hammer_debug_inode;
var hammer_debug_inode int
//extern int hammer_debug_locks;
var hammer_debug_locks int
//extern int hammer_debug_btree;
var hammer_debug_btree int
//extern int hammer_debug_tid;
var hammer_debug_tid int
//extern int hammer_debug_recover;
var hammer_debug_recover int
//extern int hammer_debug_recover_faults;
var hammer_debug_recover_faults int
//extern int hammer_debug_critical;
var hammer_debug_critical int
//extern int hammer_cluster_enable;
var hammer_cluster_enable int
//extern int hammer_live_dedup;
var hammer_live_dedup int
//extern int hammer_tdmux_ticks;
var hammer_tdmux_ticks int
//extern int hammer_count_fsyncs;
var hammer_count_fsyncs int
//extern int hammer_count_inodes;
var hammer_count_inodes int
//extern int hammer_count_iqueued;
var hammer_count_iqueued int
//extern int hammer_count_reclaims;
var hammer_count_reclaims int
//extern int hammer_count_records;
var hammer_count_records int
//extern int hammer_count_record_datas;
var hammer_count_record_datas int
//extern int hammer_count_volumes;
var hammer_count_volumes int
//extern int hammer_count_buffers;
var hammer_count_buffers int
//extern int hammer_count_nodes;
var hammer_count_nodes int
//extern int64_t hammer_count_extra_space_used;
var hammer_count_extra_space_used int64
//extern int64_t hammer_stats_btree_lookups;
var hammer_stats_btree_lookups int64
//extern int64_t hammer_stats_btree_searches;
var hammer_stats_btree_searches int64
//extern int64_t hammer_stats_btree_inserts;
var hammer_stats_btree_inserts int64
//extern int64_t hammer_stats_btree_deletes;
var hammer_stats_btree_deletes int64
//extern int64_t hammer_stats_btree_elements;
var hammer_stats_btree_elements int64
//extern int64_t hammer_stats_btree_splits;
var hammer_stats_btree_splits int64
//extern int64_t hammer_stats_btree_iterations;
var hammer_stats_btree_iterations int64
//extern int64_t hammer_stats_btree_root_iterations;
var hammer_stats_btree_root_iterations int64
//extern int64_t hammer_stats_record_iterations;
var hammer_stats_record_iterations int64
//extern int64_t hammer_stats_file_read;
var hammer_stats_file_read int64
//extern int64_t hammer_stats_file_write;
var hammer_stats_file_write int64
//extern int64_t hammer_stats_file_iopsr;
var hammer_stats_file_iopsr int64
//extern int64_t hammer_stats_file_iopsw;
var hammer_stats_file_iopsw int64
//extern int64_t hammer_stats_disk_read;
var hammer_stats_disk_read int64
//extern int64_t hammer_stats_disk_write;
var hammer_stats_disk_write int64
//extern int64_t hammer_stats_inode_flushes;
var hammer_stats_inode_flushes int64
//extern int64_t hammer_stats_commits;
var hammer_stats_commits int64
//extern int64_t hammer_stats_undo;
var hammer_stats_undo int64
//extern int64_t hammer_stats_redo;
var hammer_stats_redo int64
//extern long hammer_count_dirtybufspace;
var hammer_count_dirtybufspace int32
//extern int hammer_count_refedbufs;
var hammer_count_refedbufs int
//extern int hammer_count_reservations;
var hammer_count_reservations int
//extern long hammer_count_io_running_read;
var hammer_count_io_running_read int32
//extern long hammer_count_io_running_write;
var hammer_count_io_running_write int32
//extern int hammer_count_io_locked;
var hammer_count_io_locked int
//extern long hammer_limit_dirtybufspace;
var hammer_limit_dirtybufspace int32
//extern int hammer_limit_recs;
var hammer_limit_recs int
//extern int hammer_limit_inode_recs;
var hammer_limit_inode_recs int
//extern int hammer_limit_reclaims;
var hammer_limit_reclaims int
//extern int hammer_live_dedup_cache_size;
var hammer_live_dedup_cache_size int
//extern int hammer_limit_redo;
var hammer_limit_redo int
//extern int hammer_bio_count;
var hammer_bio_count int
//extern int hammer_verify_zone;
var hammer_verify_zone int
//extern int hammer_verify_data;
var hammer_verify_data int
//extern int hammer_write_mode;
var hammer_write_mode int
//extern int hammer_double_buffer;
var hammer_double_buffer int
//extern int hammer_btree_full_undo;
var hammer_btree_full_undo int
//extern int hammer_yield_check;
var hammer_yield_check int
//extern int hammer_fsync_mode;
var hammer_fsync_mode int
//extern int hammer_autoflush;
var hammer_autoflush int
//extern int64_t hammer_contention_count;
var hammer_contention_count int64
//

//extern int64_t hammer_live_dedup_vnode_bcmps;
var hammer_live_dedup_vnode_bcmps int64
//extern int64_t hammer_live_dedup_device_bcmps;
var hammer_live_dedup_device_bcmps int64
//extern int64_t hammer_live_dedup_findblk_failures;
var hammer_live_dedup_findblk_failures int64
//extern int64_t hammer_live_dedup_bmap_saves;
var hammer_live_dedup_bmap_saves int64
//

//void	hammer_critical_error(hammer_mount_t hmp, hammer_inode_t ip,
//			int error, const char *msg);
//int	hammer_vop_inactive(struct vop_inactive_args *);
//int	hammer_vop_reclaim(struct vop_reclaim_args *);
//int	hammer_get_vnode(struct hammer_inode *ip, struct vnode **vpp);
//struct hammer_inode *hammer_get_inode(hammer_transaction_t trans,
//			hammer_inode_t dip, int64_t obj_id,
//			hammer_tid_t asof, u_int32_t localization,
//			int flags, int *errorp);
//struct hammer_inode *hammer_get_dummy_inode(hammer_transaction_t trans,
//			hammer_inode_t dip, int64_t obj_id,
//			hammer_tid_t asof, u_int32_t localization,
//			int flags, int *errorp);
//struct hammer_inode *hammer_find_inode(hammer_transaction_t trans,
//			int64_t obj_id, hammer_tid_t asof,
//			u_int32_t localization);
//void	hammer_scan_inode_snapshots(hammer_mount_t hmp,
//			hammer_inode_info_t iinfo,
//			int (*callback)(hammer_inode_t ip, void *data),
//			void *data);
//void	hammer_put_inode(struct hammer_inode *ip);
//void	hammer_put_inode_ref(struct hammer_inode *ip);
//void	hammer_inode_waitreclaims(hammer_transaction_t trans);
//

//int	hammer_unload_volume(hammer_volume_t volume, void *data __unused);
//int	hammer_adjust_volume_mode(hammer_volume_t volume, void *data __unused);
//

//int	hammer_unload_buffer(hammer_buffer_t buffer, void *data);
//int	hammer_install_volume(hammer_mount_t hmp, const char *volname,
//			struct vnode *devvp);
//int	hammer_mountcheck_volumes(hammer_mount_t hmp);
//

//int	hammer_mem_add(hammer_record_t record);
//int	hammer_ip_lookup(hammer_cursor_t cursor);
//int	hammer_ip_first(hammer_cursor_t cursor);
//int	hammer_ip_next(hammer_cursor_t cursor);
//int	hammer_ip_resolve_data(hammer_cursor_t cursor);
//int	hammer_ip_delete_record(hammer_cursor_t cursor, hammer_inode_t ip,
//			hammer_tid_t tid);
//int	hammer_create_at_cursor(hammer_cursor_t cursor,
//			hammer_btree_leaf_elm_t leaf, void *udata, int mode);
//int	hammer_delete_at_cursor(hammer_cursor_t cursor, int delete_flags,
//			hammer_tid_t delete_tid, u_int32_t delete_ts,
//			int track, int64_t *stat_bytes);
//int	hammer_ip_check_directory_empty(hammer_transaction_t trans,
//			hammer_inode_t ip);
//int	hammer_sync_hmp(hammer_mount_t hmp, int waitfor);
//int	hammer_queue_inodes_flusher(hammer_mount_t hmp, int waitfor);
//

//hammer_record_t
//	hammer_alloc_mem_record(hammer_inode_t ip, int data_len);
//void	hammer_flush_record_done(hammer_record_t record, int error);
//void	hammer_wait_mem_record_ident(hammer_record_t record, const char *ident);
//void	hammer_rel_mem_record(hammer_record_t record);
//

//int	hammer_cursor_up(hammer_cursor_t cursor);
//int	hammer_cursor_up_locked(hammer_cursor_t cursor);
//int	hammer_cursor_down(hammer_cursor_t cursor);
//int	hammer_cursor_upgrade(hammer_cursor_t cursor);
//int	hammer_cursor_upgrade_node(hammer_cursor_t cursor);
//void	hammer_cursor_downgrade(hammer_cursor_t cursor);
//int	hammer_cursor_upgrade2(hammer_cursor_t c1, hammer_cursor_t c2);
//void	hammer_cursor_downgrade2(hammer_cursor_t c1, hammer_cursor_t c2);
//int	hammer_cursor_seek(hammer_cursor_t cursor, hammer_node_t node,
//			int index);
//void	hammer_lock_ex_ident(struct hammer_lock *lock, const char *ident);
//int	hammer_lock_ex_try(struct hammer_lock *lock);
//void	hammer_lock_sh(struct hammer_lock *lock);
//int	hammer_lock_sh_try(struct hammer_lock *lock);
//int	hammer_lock_upgrade(struct hammer_lock *lock, int shcount);
//void	hammer_lock_downgrade(struct hammer_lock *lock, int shcount);
//int	hammer_lock_status(struct hammer_lock *lock);
//void	hammer_unlock(struct hammer_lock *lock);
//void	hammer_ref(struct hammer_lock *lock);
//int	hammer_ref_interlock(struct hammer_lock *lock);
//int	hammer_ref_interlock_true(struct hammer_lock *lock);
//void	hammer_ref_interlock_done(struct hammer_lock *lock);
//void	hammer_rel(struct hammer_lock *lock);
//int	hammer_rel_interlock(struct hammer_lock *lock, int locked);
//void	hammer_rel_interlock_done(struct hammer_lock *lock, int orig_locked);
//int	hammer_get_interlock(struct hammer_lock *lock);
//int	hammer_try_interlock_norefs(struct hammer_lock *lock);
//void	hammer_put_interlock(struct hammer_lock *lock, int error);
//

//void	hammer_sync_lock_ex(hammer_transaction_t trans);
//void	hammer_sync_lock_sh(hammer_transaction_t trans);
//int	hammer_sync_lock_sh_try(hammer_transaction_t trans);
//void	hammer_sync_unlock(hammer_transaction_t trans);
//

//u_int32_t hammer_to_unix_xid(uuid_t *uuid);
//void hammer_guid_to_uuid(uuid_t *uuid, u_int32_t guid);
//void	hammer_time_to_timespec(u_int64_t xtime, struct timespec *ts);
//u_int64_t hammer_timespec_to_time(struct timespec *ts);
//int	hammer_str_to_tid(const char *str, int *ispfsp,
//			hammer_tid_t *tidp, u_int32_t *localizationp);
//int	hammer_is_atatext(const char *name, int len);
//hammer_tid_t hammer_alloc_objid(hammer_mount_t hmp, hammer_inode_t dip,
//			int64_t namekey);
//void hammer_clear_objid(hammer_inode_t dip);
//void hammer_destroy_objid_cache(hammer_mount_t hmp);
//

//int hammer_dedup_crc_rb_compare(hammer_dedup_cache_t dc1,
//			hammer_dedup_cache_t dc2);
//int hammer_dedup_off_rb_compare(hammer_dedup_cache_t dc1,
//			hammer_dedup_cache_t dc2);
//hammer_dedup_cache_t hammer_dedup_cache_add(hammer_inode_t ip,
//			hammer_btree_leaf_elm_t leaf);
//hammer_dedup_cache_t hammer_dedup_cache_lookup(hammer_mount_t hmp,
//			hammer_crc_t crc);
//void hammer_dedup_cache_inval(hammer_mount_t hmp, hammer_off_t base_offset);
//void hammer_destroy_dedup_cache(hammer_mount_t hmp);
//void hammer_dump_dedup_cache(hammer_mount_t hmp);
//int hammer_dedup_validate(hammer_dedup_cache_t dcp, int zone, int bytes,
//			void *data);
//
//int hammer_enter_undo_history(hammer_mount_t hmp, hammer_off_t offset,
//			int bytes);
//void hammer_clear_undo_history(hammer_mount_t hmp);
//enum vtype hammer_get_vnode_type(u_int8_t obj_type);
//int hammer_get_dtype(u_int8_t obj_type);
//u_int8_t hammer_get_obj_type(enum vtype vtype);
//int64_t hammer_directory_namekey(hammer_inode_t dip, const void *name, int len,
//			u_int32_t *max_iterationsp);
//int	hammer_nohistory(hammer_inode_t ip);
//

//int	hammer_init_cursor(hammer_transaction_t trans, hammer_cursor_t cursor,
//			hammer_node_cache_t cache, hammer_inode_t ip);
//void	hammer_normalize_cursor(hammer_cursor_t cursor);
//void	hammer_done_cursor(hammer_cursor_t cursor);
//int	hammer_recover_cursor(hammer_cursor_t cursor);
//void	hammer_unlock_cursor(hammer_cursor_t cursor);
//int	hammer_lock_cursor(hammer_cursor_t cursor);
//hammer_cursor_t	hammer_push_cursor(hammer_cursor_t ocursor);
//void	hammer_pop_cursor(hammer_cursor_t ocursor, hammer_cursor_t ncursor);
//

//void	hammer_cursor_replaced_node(hammer_node_t onode, hammer_node_t nnode);
//void	hammer_cursor_removed_node(hammer_node_t onode, hammer_node_t parent,
//			int index);
//void	hammer_cursor_split_node(hammer_node_t onode, hammer_node_t nnode,
//			int index);
//void	hammer_cursor_moved_element(hammer_node_t oparent, int pindex,
//			hammer_node_t onode, int oindex,
//			hammer_node_t nnode, int nindex);
//void	hammer_cursor_parent_changed(hammer_node_t node, hammer_node_t oparent,
//			hammer_node_t nparent, int nindex);
//void	hammer_cursor_inserted_element(hammer_node_t node, int index);
//void	hammer_cursor_deleted_element(hammer_node_t node, int index);
//void	hammer_cursor_invalidate_cache(hammer_cursor_t cursor);
//

//int	hammer_btree_lookup(hammer_cursor_t cursor);
//int	hammer_btree_first(hammer_cursor_t cursor);
//int	hammer_btree_last(hammer_cursor_t cursor);
//int	hammer_btree_extract(hammer_cursor_t cursor, int flags);
//int	hammer_btree_iterate(hammer_cursor_t cursor);
//int	hammer_btree_iterate_reverse(hammer_cursor_t cursor);
//int	hammer_btree_insert(hammer_cursor_t cursor,
//			    hammer_btree_leaf_elm_t elm, int *doprop);
//int	hammer_btree_delete(hammer_cursor_t cursor);
//void	hammer_btree_do_propagation(hammer_cursor_t cursor,
//			    hammer_pseudofs_inmem_t pfsm,
//			    hammer_btree_leaf_elm_t leaf);
//int	hammer_btree_cmp(hammer_base_elm_t key1, hammer_base_elm_t key2);
//int	hammer_btree_chkts(hammer_tid_t ts, hammer_base_elm_t key);
//int	hammer_btree_correct_rhb(hammer_cursor_t cursor, hammer_tid_t tid);
//int	hammer_btree_correct_lhb(hammer_cursor_t cursor, hammer_tid_t tid);
//

//int	btree_set_parent(hammer_transaction_t trans, hammer_node_t node,
//                        hammer_btree_elm_t elm);
//void	hammer_node_lock_init(hammer_node_lock_t parent, hammer_node_t node);
//void	hammer_btree_lcache_init(hammer_mount_t hmp, hammer_node_lock_t lcache,
//			int depth);
//void	hammer_btree_lcache_free(hammer_mount_t hmp, hammer_node_lock_t lcache);
//int	hammer_btree_lock_children(hammer_cursor_t cursor, int depth,
//			hammer_node_lock_t parent,
//			hammer_node_lock_t lcache);
//void	hammer_btree_lock_copy(hammer_cursor_t cursor,
//			hammer_node_lock_t parent);
//int	hammer_btree_sync_copy(hammer_cursor_t cursor,
//			hammer_node_lock_t parent);
//void	hammer_btree_unlock_children(hammer_mount_t hmp,
//			hammer_node_lock_t parent,
//			hammer_node_lock_t lcache);
//int	hammer_btree_search_node(hammer_base_elm_t elm, hammer_node_ondisk_t node);
//hammer_node_t hammer_btree_get_parent(hammer_transaction_t trans,
//			hammer_node_t node, int *parent_indexp,
//			int *errorp, int try_exclusive);
//

//void	hammer_print_btree_node(hammer_node_ondisk_t ondisk);
//void	hammer_print_btree_elm(hammer_btree_elm_t elm, u_int8_t type, int i);
//

//void	*hammer_bread(struct hammer_mount *hmp, hammer_off_t off,
//			int *errorp, struct hammer_buffer **bufferp);
//void	*hammer_bnew(struct hammer_mount *hmp, hammer_off_t off,
//			int *errorp, struct hammer_buffer **bufferp);
//void	*hammer_bread_ext(struct hammer_mount *hmp, hammer_off_t off, int bytes,
//			int *errorp, struct hammer_buffer **bufferp);
//void	*hammer_bnew_ext(struct hammer_mount *hmp, hammer_off_t off, int bytes,
//			int *errorp, struct hammer_buffer **bufferp);
//

//hammer_volume_t hammer_get_root_volume(hammer_mount_t hmp, int *errorp);
//

//hammer_volume_t	hammer_get_volume(hammer_mount_t hmp,
//			int32_t vol_no, int *errorp);
//hammer_buffer_t	hammer_get_buffer(hammer_mount_t hmp, hammer_off_t buf_offset,
//			int bytes, int isnew, int *errorp);
//void		hammer_sync_buffers(hammer_mount_t hmp,
//			hammer_off_t base_offset, int bytes);
//int		hammer_del_buffers(hammer_mount_t hmp,
//			hammer_off_t base_offset,
//			hammer_off_t zone2_offset, int bytes,
//			int report_conflicts);
//

//int		hammer_ref_volume(hammer_volume_t volume);
//int		hammer_ref_buffer(hammer_buffer_t buffer);
//void		hammer_flush_buffer_nodes(hammer_buffer_t buffer);
//

//void		hammer_rel_volume(hammer_volume_t volume, int locked);
//void		hammer_rel_buffer(hammer_buffer_t buffer, int locked);
//

//int		hammer_vfs_export(struct mount *mp, int op,
//			const struct export_args *export);
//hammer_node_t	hammer_get_node(hammer_transaction_t trans,
//			hammer_off_t node_offset, int isnew, int *errorp);
//void		hammer_ref_node(hammer_node_t node);
//hammer_node_t	hammer_ref_node_safe(hammer_transaction_t trans,
//			hammer_node_cache_t cache, int *errorp);
//void		hammer_rel_node(hammer_node_t node);
//void		hammer_delete_node(hammer_transaction_t trans,
//			hammer_node_t node);
//void		hammer_cache_node(hammer_node_cache_t cache,
//			hammer_node_t node);
//void		hammer_uncache_node(hammer_node_cache_t cache);
//void		hammer_flush_node(hammer_node_t node, int locked);
//

//void hammer_dup_buffer(struct hammer_buffer **bufferp,
//			struct hammer_buffer *buffer);
//hammer_node_t hammer_alloc_btree(hammer_transaction_t trans,
//			hammer_off_t hint, int *errorp);
//void *hammer_alloc_data(hammer_transaction_t trans, int32_t data_len,
//			u_int16_t rec_type, hammer_off_t *data_offsetp,
//			struct hammer_buffer **data_bufferp,
//			hammer_off_t hint, int *errorp);
//

//int hammer_generate_undo(hammer_transaction_t trans,
//			hammer_off_t zone1_offset, void *base, int len);
//int hammer_generate_redo(hammer_transaction_t trans, hammer_inode_t ip,
//			hammer_off_t file_offset, u_int32_t flags,
//			void *base, int len);
//void hammer_generate_redo_sync(hammer_transaction_t trans);
//void hammer_redo_fifo_start_flush(hammer_inode_t ip);
//void hammer_redo_fifo_end_flush(hammer_inode_t ip);
//

//void hammer_format_undo(void *base, u_int32_t seqno);
//int hammer_upgrade_undo_4(hammer_transaction_t trans);
//

//void hammer_put_volume(struct hammer_volume *volume, int flush);
//void hammer_put_buffer(struct hammer_buffer *buffer, int flush);
//

//hammer_off_t hammer_freemap_alloc(hammer_transaction_t trans,
//			hammer_off_t owner, int *errorp);
//void hammer_freemap_free(hammer_transaction_t trans, hammer_off_t phys_offset,
//			hammer_off_t owner, int *errorp);
//int _hammer_checkspace(hammer_mount_t hmp, int slop, int64_t *resp);
//hammer_off_t hammer_blockmap_alloc(hammer_transaction_t trans, int zone,
//			int bytes, hammer_off_t hint, int *errorp);
//hammer_reserve_t hammer_blockmap_reserve(hammer_mount_t hmp, int zone,
//			int bytes, hammer_off_t *zone_offp, int *errorp);
//hammer_reserve_t hammer_blockmap_reserve_dedup(hammer_mount_t hmp, int zone,
//			int bytes, hammer_off_t zone_offset, int *errorp);
//void hammer_blockmap_reserve_complete(hammer_mount_t hmp,
//			hammer_reserve_t resv);
//void hammer_reserve_clrdelay(hammer_mount_t hmp, hammer_reserve_t resv);
//void hammer_blockmap_free(hammer_transaction_t trans,
//			hammer_off_t bmap_off, int bytes);
//int hammer_blockmap_dedup(hammer_transaction_t trans,
//			hammer_off_t bmap_off, int bytes);
//int hammer_blockmap_finalize(hammer_transaction_t trans,
//			hammer_reserve_t resv,
//			hammer_off_t bmap_off, int bytes);
//int hammer_blockmap_getfree(hammer_mount_t hmp, hammer_off_t bmap_off,
//			int *curp, int *errorp);
//hammer_off_t hammer_blockmap_lookup(hammer_mount_t hmp, hammer_off_t bmap_off,
//			int *errorp);
//hammer_off_t hammer_undo_lookup(hammer_mount_t hmp, hammer_off_t bmap_off,
//			int *errorp);
//int64_t hammer_undo_used(hammer_transaction_t trans);
//int64_t hammer_undo_space(hammer_transaction_t trans);
//int64_t hammer_undo_max(hammer_mount_t hmp);
//int hammer_undo_reclaim(hammer_io_t io);
//

//void hammer_start_transaction(struct hammer_transaction *trans,
//			      struct hammer_mount *hmp);
//void hammer_simple_transaction(struct hammer_transaction *trans,
//			      struct hammer_mount *hmp);
//void hammer_start_transaction_fls(struct hammer_transaction *trans,
//			          struct hammer_mount *hmp);
//void hammer_done_transaction(struct hammer_transaction *trans);
//hammer_tid_t hammer_alloc_tid(hammer_mount_t hmp, int count);
//

//void hammer_modify_inode(hammer_transaction_t trans, hammer_inode_t ip, int flags);
//void hammer_flush_inode(hammer_inode_t ip, int flags);
//void hammer_flush_inode_done(hammer_inode_t ip, int error);
//void hammer_wait_inode(hammer_inode_t ip);
//

//int  hammer_create_inode(struct hammer_transaction *trans, struct vattr *vap,
//			struct ucred *cred, struct hammer_inode *dip,
//			const char *name, int namelen,
//			hammer_pseudofs_inmem_t pfsm,
//			struct hammer_inode **ipp);
//void hammer_rel_inode(hammer_inode_t ip, int flush);
//int hammer_reload_inode(hammer_inode_t ip, void *arg __unused);
//int hammer_ino_rb_compare(hammer_inode_t ip1, hammer_inode_t ip2);
//int hammer_redo_rb_compare(hammer_inode_t ip1, hammer_inode_t ip2);
//int hammer_destroy_inode_callback(hammer_inode_t ip, void *data __unused);
//
//int hammer_sync_inode(hammer_transaction_t trans, hammer_inode_t ip);
//void hammer_test_inode(hammer_inode_t dip);
//void hammer_inode_unloadable_check(hammer_inode_t ip, int getvp);
//int hammer_update_atime_quick(hammer_inode_t ip);
//

//int  hammer_ip_add_directory(struct hammer_transaction *trans,
//			hammer_inode_t dip, const char *name, int bytes,
//			hammer_inode_t nip);
//int  hammer_ip_del_directory(struct hammer_transaction *trans,
//			hammer_cursor_t cursor, hammer_inode_t dip,
//			hammer_inode_t ip);
//void hammer_ip_replace_bulk(hammer_mount_t hmp, hammer_record_t record);
//hammer_record_t hammer_ip_add_bulk(hammer_inode_t ip, off_t file_offset,
//			void *data, int bytes, int *errorp);
//int  hammer_ip_frontend_trunc(struct hammer_inode *ip, off_t file_size);
//int  hammer_ip_add_record(struct hammer_transaction *trans,
//			hammer_record_t record);
//int  hammer_ip_delete_range(hammer_cursor_t cursor, hammer_inode_t ip,
//			int64_t ran_beg, int64_t ran_end, int truncating);
//int  hammer_ip_delete_clean(hammer_cursor_t cursor, hammer_inode_t ip,
//			int *countp);
//int  hammer_ip_sync_data(hammer_cursor_t cursor, hammer_inode_t ip,
//			int64_t offset, void *data, int bytes);
//int  hammer_ip_sync_record(hammer_transaction_t trans, hammer_record_t rec);
//int  hammer_ip_sync_record_cursor(hammer_cursor_t cursor, hammer_record_t rec);
//hammer_pseudofs_inmem_t  hammer_load_pseudofs(hammer_transaction_t trans,
//			u_int32_t localization, int *errorp);
//int  hammer_mkroot_pseudofs(hammer_transaction_t trans, struct ucred *cred,
//			hammer_pseudofs_inmem_t pfsm);
//int  hammer_save_pseudofs(hammer_transaction_t trans,
//			hammer_pseudofs_inmem_t pfsm);
//int  hammer_unload_pseudofs(hammer_transaction_t trans, u_int32_t localization);
//void hammer_rel_pseudofs(hammer_mount_t hmp, hammer_pseudofs_inmem_t pfsm);
//int hammer_ioctl(hammer_inode_t ip, u_long com, caddr_t data, int fflag,
//			struct ucred *cred);
//

//void hammer_io_init(hammer_io_t io, hammer_volume_t volume,
//			enum hammer_io_type type);
//int hammer_io_read(struct vnode *devvp, struct hammer_io *io, int limit);
//void hammer_io_advance(struct hammer_io *io);
//int hammer_io_new(struct vnode *devvp, struct hammer_io *io);
//int hammer_io_inval(hammer_volume_t volume, hammer_off_t zone2_offset);
//struct buf *hammer_io_release(struct hammer_io *io, int flush);
//void hammer_io_flush(struct hammer_io *io, int reclaim);
//void hammer_io_wait(struct hammer_io *io);
//void hammer_io_waitdep(struct hammer_io *io);
//void hammer_io_wait_all(hammer_mount_t hmp, const char *ident, int doflush);
//int hammer_io_direct_read(hammer_mount_t hmp, struct bio *bio,
//			hammer_btree_leaf_elm_t leaf);
//int hammer_io_indirect_read(hammer_mount_t hmp, struct bio *bio,
//			hammer_btree_leaf_elm_t leaf);
//int hammer_io_direct_write(hammer_mount_t hmp, struct bio *bio,
//			hammer_record_t record);
//void hammer_io_direct_wait(hammer_record_t record);
//void hammer_io_direct_uncache(hammer_mount_t hmp, hammer_btree_leaf_elm_t leaf);
//void hammer_io_write_interlock(hammer_io_t io);
//void hammer_io_done_interlock(hammer_io_t io);
//void hammer_io_clear_modify(struct hammer_io *io, int inval);
//void hammer_io_clear_modlist(struct hammer_io *io);
//void hammer_io_flush_sync(hammer_mount_t hmp);
//void hammer_io_clear_error(struct hammer_io *io);
//void hammer_io_clear_error_noassert(struct hammer_io *io);
//void hammer_io_notmeta(hammer_buffer_t buffer);
//void hammer_io_limit_backlog(hammer_mount_t hmp);
//
//void hammer_modify_volume(hammer_transaction_t trans, hammer_volume_t volume,
//			void *base, int len);
//void hammer_modify_buffer(hammer_transaction_t trans, hammer_buffer_t buffer,
//			void *base, int len);
//void hammer_modify_volume_done(hammer_volume_t volume);
//void hammer_modify_buffer_done(hammer_buffer_t buffer);
//

//int hammer_ioc_reblock(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_reblock *reblock);
//int hammer_ioc_rebalance(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_rebalance *rebal);
//int hammer_ioc_prune(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_prune *prune);
//int hammer_ioc_mirror_read(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_mirror_rw *mirror);
//int hammer_ioc_mirror_write(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_mirror_rw *mirror);
//int hammer_ioc_set_pseudofs(hammer_transaction_t trans, hammer_inode_t ip,
//			struct ucred *cred, struct hammer_ioc_pseudofs_rw *pfs);
//int hammer_ioc_get_pseudofs(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_pseudofs_rw *pfs);
//int hammer_ioc_destroy_pseudofs(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_pseudofs_rw *pfs);
//int hammer_ioc_downgrade_pseudofs(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_pseudofs_rw *pfs);
//int hammer_ioc_upgrade_pseudofs(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_pseudofs_rw *pfs);
//int hammer_ioc_wait_pseudofs(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_pseudofs_rw *pfs);
//int hammer_ioc_volume_add(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_volume *ioc);
//int hammer_ioc_volume_del(hammer_transaction_t trans, hammer_inode_t ip,
//                        struct hammer_ioc_volume *ioc);
//int hammer_ioc_volume_list(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_volume_list *ioc);
//int hammer_ioc_dedup(hammer_transaction_t trans, hammer_inode_t ip,
//			struct hammer_ioc_dedup *dedup);
//

//int hammer_signal_check(hammer_mount_t hmp);
//

//void hammer_flusher_create(hammer_mount_t hmp);
//void hammer_flusher_destroy(hammer_mount_t hmp);
//void hammer_flusher_sync(hammer_mount_t hmp);
//int  hammer_flusher_async(hammer_mount_t hmp, hammer_flush_group_t flg);
//int  hammer_flusher_async_one(hammer_mount_t hmp);
//int hammer_flusher_running(hammer_mount_t hmp);
//void hammer_flusher_wait(hammer_mount_t hmp, int seq);
//void hammer_flusher_wait_next(hammer_mount_t hmp);
//int  hammer_flusher_meta_limit(hammer_mount_t hmp);
//int  hammer_flusher_meta_halflimit(hammer_mount_t hmp);
//int  hammer_flusher_undo_exhausted(hammer_transaction_t trans, int quarter);
//void hammer_flusher_clean_loose_ios(hammer_mount_t hmp);
//void hammer_flusher_finalize(hammer_transaction_t trans, int final);
//int  hammer_flusher_haswork(hammer_mount_t hmp);
//void hammer_flusher_flush_undos(hammer_mount_t hmp, int already_flushed);
//

//int hammer_recover_stage1(hammer_mount_t hmp, hammer_volume_t rootvol);
//int hammer_recover_stage2(hammer_mount_t hmp, hammer_volume_t rootvol);
//void hammer_recover_flush_buffers(hammer_mount_t hmp,
//			hammer_volume_t root_volume, int final);
//

//void hammer_crc_set_blockmap(hammer_blockmap_t blockmap);
//void hammer_crc_set_volume(hammer_volume_ondisk_t ondisk);
//void hammer_crc_set_leaf(void *data, hammer_btree_leaf_elm_t leaf);
//

//int hammer_crc_test_blockmap(hammer_blockmap_t blockmap);
//int hammer_crc_test_volume(hammer_volume_ondisk_t ondisk);
//int hammer_crc_test_btree(hammer_node_ondisk_t ondisk);
//int hammer_crc_test_leaf(void *data, hammer_btree_leaf_elm_t leaf);
//void hkprintf(const char *ctl, ...) __printflike(1, 2);
//udev_t hammer_fsid_to_udev(uuid_t *uuid);
//

//

//int hammer_blocksize(int64_t file_offset);
//int hammer_blockoff(int64_t file_offset);
//int64_t hammer_blockdemarc(int64_t file_offset1, int64_t file_offset2);
//

///*
// * Shortcut for _hammer_checkspace(), used all over the code.
// */
//static __inline int
//hammer_checkspace(hammer_mount_t hmp, int slop)
//{
func hammer_checkspace(hmp hammer_mount_t, slop int) int {
	if KERNEL_DEFINED == true {
//	return(_hammer_checkspace(hmp, slop, NULL));
		return hammer_checkspace_(hmp, slop, nil)
	} else {
	return 0
	}
//}
}
//

//#endif
/* starts at #if defined(_KERNEL) */
//

//#ifdef _KERNEL
//static __inline void
//hammer_wait_mem_record(hammer_record_t record)
//{
func hammer_wait_mem_record(record hammer_record_t) {
	if KERNEL_DEFINED == true {
//	hammer_wait_mem_record_ident(record, "hmmwai");
		hammer_wait_mem_record_ident(record, "hmmwai")
	}
//}
}
//

//static __inline void
//hammer_lock_ex(struct hammer_lock *lock)
//{
func hammer_lock_ex(lock *hammer_lock) {
	if KERNEL_DEFINED == true {
//	hammer_lock_ex_ident(lock, "hmrlck");
		hammer_lock_ex_ident(lock, "hmrlck")
	}
//}
}
//

///*
// * Indicate that a B-Tree node is being modified.
// */
//static __inline void
//hammer_modify_node_noundo(hammer_transaction_t trans, hammer_node_t node)
//{
func hammer_modify_node_noundo(trans hammer_transaction_t, node hammer_node_t) {
	if KERNEL_DEFINED == true {
//	KKASSERT((node->flags & HAMMER_NODE_CRCBAD) == 0);
		kernel.KKASSERT((node.flags & HAMMER_NODE_CRCBAD) == 0)
//	hammer_modify_buffer(trans, node->buffer, NULL, 0);
		hammer_modify_buffer(trans, node.buffer, nil, 0)
	}
//}
}
//

//static __inline void
//hammer_modify_node_all(hammer_transaction_t trans, struct hammer_node *node)
//{
func hammer_modify_node_all(trans hammer_transaction_t, node *hammer_node) {
//	KKASSERT((node->flags & HAMMER_NODE_CRCBAD) == 0);
	if KERNEL_DEFINED == true {
		kernel.KKASSERT((node.flags & HAMMER_NODE_CRCBAD) == 0)
//	hammer_modify_buffer(trans, node->buffer,
		hammer_modify_buffer(trans, node.buffer,
//			     node->ondisk, sizeof(*node->ondisk));
			node.ondisk, int(unsafe.Sizeof(*node.ondisk)))
	}
//}
}
//

//static __inline void
//hammer_modify_node(hammer_transaction_t trans, hammer_node_t node,
func hammer_modify_node(trans hammer_transaction_t, node hammer_node_t,
//		   void *base, int len)
//{
		base interface{}, len int) {
	if KERNEL_DEFINED == true {
//	hammer_crc_t *crcptr;
		var crcptr *hammer_crc_t
//

//	KKASSERT((char *)base >= (char *)node->ondisk &&
		kernel.KKASSERT(unsafe.Sizeof(base) >= unsafe.Sizeof(node.ondisk) &&
//		 (char *)base + len <=
			int(unsafe.Sizeof(base)) + len <=
//		    (char *)node->ondisk + sizeof(*node->ondisk));
			int(unsafe.Sizeof(node.ondisk) + unsafe.Sizeof(*node.ondisk)))
//	KKASSERT((node->flags & HAMMER_NODE_CRCBAD) == 0);
		kernel.KKASSERT((node.flags & HAMMER_NODE_CRCBAD) == 0)
//
//	if (hammer_btree_full_undo) {
		if hammer_btree_full_undo != 0 {
//		hammer_modify_node_all(trans, node);
			hammer_modify_node_all(trans, node)
//	} else {
		} else {
//		hammer_modify_buffer(trans, node->buffer, base, len);
			hammer_modify_buffer(trans, node.buffer, base, len)
//		crcptr = &node->ondisk->crc;
			crcptr = &node.ondisk.crc
//		hammer_modify_buffer(trans, node->buffer,
			hammer_modify_buffer(trans, node.buffer,
//				     crcptr, sizeof(hammer_crc_t));
				crcptr, int(unsafe.Sizeof(new(hammer_crc_t))))
//		--node->buffer->io.modify_refs;	/* only want one ref */
			node.buffer.io.modify_refs--	/* only want one ref */
//	}
		}
	}
//}
}
//

/*
 * Indicate that the specified modifications have been completed.
 *
 * Do not try to generate the crc here, it's very expensive to do and a
 * sequence of insertions or deletions can result in many calls to this
 * function on the same node.
 */
//static __inline void
//hammer_modify_node_done(hammer_node_t node)
//{
func hammer_modify_node_done(node hammer_node_t) {
	if KERNEL_DEFINED == true {
//	node->flags |= HAMMER_NODE_CRCGOOD;
		node.flags |= HAMMER_NODE_NEEDSCRC
//	if ((node->flags & HAMMER_NODE_NEEDSCRC) == 0) {
		if node.flags & HAMMER_NODE_NEEDSCRC == 0 {
//		node->flags |= HAMMER_NODE_NEEDSCRC;
			node.flags |= HAMMER_NODE_NEEDSCRC
//		node->buffer->io.gencrc = 1;
			node.buffer.io.gencrc = 1
//		hammer_ref_node(node);
			hammer_ref_node(node)
//	}
		}
//	hammer_modify_buffer_done(node->buffer);
		hammer_modify_buffer_done(node.buffer)
	}
//}
}
//#endif

//

//#define hammer_modify_volume_field(trans, vol, field)		\
//	hammer_modify_volume(trans, vol, &(vol)->ondisk->field,	\
//			     sizeof((vol)->ondisk->field))
/* Manually Expanded Macro */
//

//#define hammer_modify_node_field(trans, node, field)		\
//	hammer_modify_node(trans, node, &(node)->ondisk->field,	\
//			     sizeof((node)->ondisk->field))
/* Manually Expanded Macro */
//

//#ifdef _KERNEL

/*
 * The HAMMER_INODE_CAP_DIR_LOCAL_INO capability is set on newly
 * created directories for HAMMER version 2 or greater and causes
 * directory entries to be placed the inode localization zone in
 * the B-Tree instead of the misc zone.
 *
 * This greatly improves localization between directory entries and
 * inodes
 */
//static __inline u_int32_t
//hammer_dir_localization(hammer_inode_t dip)
//{
func hammer_dir_localization(dip hammer_inode) uint32 {
	if KERNEL_DEFINED == true {
//	if (dip->ino_data.cap_flags & HAMMER_INODE_CAP_DIR_LOCAL_INO)
		if (dip.ino_data.cap_flags & HAMMER_INODE_CAP_DIR_LOCAL_INO) != 0 {
//		return(HAMMER_LOCALIZE_INODE);
			return HAMMER_LOCALIZE_INODE
//	else
		} else {
//		return(HAMMER_LOCALIZE_MISC);
			return HAMMER_LOCALIZE_MISC
		}
	} else {
		return 0
	}
//}
}
//#endif

