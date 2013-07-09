// https://github.com/varialus/hammer/blob/master/hammer_btree.go

/*
Copyright (c) 2013, Aulus Egnatius Varialus <varialus@gmail.com>

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

// http://gitweb.dragonflybsd.org/dragonfly.git/blob/HEAD:/sys/vfs/hammer/hammer_btree.h
// http://gitweb.dragonflybsd.org/dragonfly.git/blob/HEAD:/sys/vfs/hammer/hammer_btree.c

/*
 * Copyright (c) 2007-2008 The DragonFly Project.  All rights reserved.
 * 
 * This code is derived from software contributed to The DragonFly Project
 * by Matthew Dillon <dillon@backplane.com>
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of The DragonFly Project nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific, prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

package hammer

import (
	"bytes"
	"encoding/binary"
	"fmt"
	"github.com/varialus/hammer/dependencies/system"
	"github.com/varialus/hammer/dependencies/system/kernel"
	"github.com/varialus/hammer/dependencies/system/platform"
	"github.com/varialus/hammer/dependencies/library"
	"unsafe"
)

/*
 * HAMMER B-Tree index
 *
 * HAMMER implements a modified B+Tree.   B+Trees store records only
 * at their leaves and HAMMER's modification is to adjust the internal
 * elements so there is a boundary element on each side instead of sub-tree
 * pointers.
 *
 * We just call our modified B+Tree a 'B-Tree' in HAMMER documentation to
 * reduce confusion.
 *
 * A B-Tree internal node looks like this:
 *
 *	B N N N N N N B   <-- boundary and internal elements
 *       S S S S S S S    <-- subtree pointers
 *
 * A B-Tree leaf node looks like this:
 *
 *	L L L L L L L L   <-- leaf elemenets
 *			      (there is also a previous and next-leaf pointer)
 *
 * The recursion radix of an internal node is reduced by 1 relative to
 * a normal B-Tree in order to accomodate the right-hand boundary.
 *
 * The big benefit to using a B-Tree with built-in bounds information is
 * that it makes it possible to cache pointers into the middle of the tree
 * and not have to start searches, insertions, OR deletions at the root node.
 * The boundary elements allow searches to progress in a definitive direction
 * from any point in the tree without revisting nodes.  It is also possible
 * to terminate searches early and make minor adjustments to the boundaries
 * (within the confines of the parent's boundaries) on the fly.  This greatly
 * improves the efficiency of many operations.
 *
 * HAMMER B-Trees are per-cluster.  The global multi-cluster B-Tree is
 * constructed by allowing internal nodes to link to the roots of other
 * clusters.  Fields in the cluster header then reference back to its
 * parent and use the cluster generation number to detect stale linkages.
 *
 * The B-Tree balancing code can operate within a cluster or across the
 * filesystem's ENTIRE B-Tree super-structure.  A cluster's B-Tree root
 * can be a leaf node in the worse case.  A cluster is guarenteed to have
 * sufficient free space to hold a single completely full leaf in the
 * degenerate case.
 *
 * All of the structures below are on-disk structures.
 */

/*
 * Common base for all B-Tree element types (40 bytes)
 *
 * obj_type is set to the object type the record represents if an inode,
 * directory entry, or an inter-cluster reference.  A cluster range is
 * special in that the B-Tree nodes represent a range within the B-Tree
 * inclusive of rec_type field, so obj_type must be used to detect the
 * cluster range entries.
 *
 * btype is only used by the elements making up an internal or leaf B-Tree
 * node and applies to the node rather then to the key.  This means that
 * btype must be assigned/reassigned after any update to the base_elm making
 * up a B-Tree element.
 */

//struct hammer_base_elm {
type hammer_base_elm struct{
//	int64_t	obj_id;		/* 00 object record is associated with */
	obj_id int64		/* 00 object record is associated with */
//	int64_t key;		/* 08 indexing key (offset or namekey) */
	key int64		/* 08 indexing key (offset or namekey) */
//

//	hammer_tid_t create_tid; /* 10 transaction id for record creation */
	create_tid hammer_tid_t  /* 10 transaction id for record creation */
//	hammer_tid_t delete_tid; /* 18 transaction id for record update/del */
	delete_tid hammer_tid_t  /* 18 transaction id for record update/del */
//

//	u_int16_t rec_type;	/* 20 _RECTYPE_ */
	rec_type uint16		/* 20 _RECTYPE_ */
//	u_int8_t obj_type;	/* 22 _OBJTYPE_ (restricted) */
	obj_type uint8		/* 22 _OBJTYPE_ (restricted) */
//	u_int8_t btype;		/* 23 B-Tree element type */
	btype uint8		/* 23 B-Tree element type */
//	u_int32_t localization;	/* 24 B-Tree localization parameter */
	localization uint32	/* 24 B-Tree localization parameter * /
//				/* 28 */
				/* 28 */
//};
}
//

//typedef struct hammer_btree_internal_elm *hammer_btree_internal_elm_t;
type hammer_base_elm_t *hammer_base_elm
//

/*
 * Localization has sorting priority over the obj_id and is used to
 * localize inodes for very fast directory scans.
 *
 * Localization can also be used to create pseudo-filesystems within
 * a HAMMER filesystem.  Pseudo-filesystems would be suitable
 * replication targets.
 */
//#define HAMMER_LOCALIZE_RESERVED00	0x00000000
const HAMMER_LOCALIZE_RESERVED00	= 0x00000000
//#define HAMMER_LOCALIZE_INODE		0x00000001
const HAMMER_LOCALIZE_INODE		= 0x00000001
//#define HAMMER_LOCALIZE_MISC		0x00000002
const HAMMER_LOCALIZE_MISC		= 0x00000002
//#define HAMMER_LOCALIZE_RESERVED03	0x00000003
const HAMMER_LOCALIZE_RESERVED03	= 0x00000003
//#define HAMMER_LOCALIZE_MASK		0x0000FFFF
const HAMMER_LOCALIZE_MASK		= 0x0000FFFF
//#define HAMMER_LOCALIZE_PSEUDOFS_MASK	0xFFFF0000
const HAMMER_LOCALIZE_PSEUDOFS_MASK	= 0xFFFF0000
//#define HAMMER_LOCALIZE_PSEUDOFS_INC	0x00010000
const HAMMER_LOCALIZE_PSEUDOFS_INC	= 0x00010000
//

//#define HAMMER_MIN_LOCALIZATION		0x00000000U
const HAMMER_MIN_LOCALIZATION		= 0x00000000
//#define HAMMER_MAX_LOCALIZATION		0x0000FFFFU
const HAMMER_MAX_LOCALIZATION		= 0x0000FFFF
//#define HAMMER_DEF_LOCALIZATION		0x00000000U
const HAMMER_DEF_LOCALIZATION		= 0x00000000
//

/*
 * Internal element (40 + 24 = 64 bytes).
 *
 * An internal element contains the left-hand boundary, right-hand boundary,
 * and a recursion to another B-Tree node.
 */
//struct hammer_btree_internal_elm {
type hammer_btree_internal_elm struct {
//	struct hammer_base_elm base;
	base hammer_base_elm
//	hammer_tid_t	mirror_tid;		/* mirroring support */
	mirror_tid hammer_tid_t			/* mirroring support */
//	hammer_off_t	subtree_offset;
	subtree_offset hammer_off_t
//	int32_t		unused02;
	unused02 int32
//	int32_t		unused03;
	unused03 int32
//};
}
//

//typedef struct hammer_btree_internal_elm *hammer_btree_internal_elm_t;
type hammer_btree_internal_elm_t hammer_btree_internal_elm
//

//type hammer_btree_internal_elm_t *hammer_btree_internal_elm

/*
 * Leaf B-Tree element (40 + 24 = 64 bytes).
 *
 * NOTE: create_ts/delete_ts are not used by any core algorithms, they are
 *       used only by userland to present nominal real-time date strings
 *	 to the user.
 */
//struct hammer_btree_leaf_elm {
type hammer_btree_leaf_elm struct {
//	struct hammer_base_elm base;
	base hammer_base_elm
//	u_int32_t	create_ts;
	create_ts uint32
//	u_int32_t	delete_ts;
	delete_ts uint32
//	hammer_off_t	data_offset;
	data_offset hammer_off_t
//	int32_t		data_len;
	data_len int32
//	hammer_crc_t	data_crc;
	data_crc hammer_crc_t
//};
}
//

//typedef struct hammer_btree_leaf_elm *hammer_btree_leaf_elm_t;
type hammer_btree_leaf_elm_t *hammer_btree_leaf_elm
//

/*
 * Rollup btree leaf element types - 64 byte structure
 */
//union hammer_btree_elm {
type hammer_btree_elm [64]byte
//	struct hammer_base_elm		base;
func (h *hammer_btree_elm) base(index int) (base *hammer_base_elm) {
	binary.Read(bytes.NewReader(h[:]), binary.LittleEndian, base)
	return base
}
//	struct hammer_btree_leaf_elm	leaf;
func (h *hammer_btree_elm) leaf(index int) (leaf *hammer_btree_leaf_elm) {
	binary.Read(bytes.NewReader(h[:]), binary.LittleEndian, leaf)
	return leaf
}
//	struct hammer_btree_internal_elm internal;
func (h *hammer_btree_elm) internal(index int) (internal *hammer_btree_internal_elm) {
	binary.Read(bytes.NewReader(h[:]), binary.LittleEndian, internal)
	return internal
}
func (h *hammer_btree_elm) write(elm interface{}) {
	binary.Write(bytes.NewBuffer(h[:]), binary.LittleEndian, elm)
//};
}
//

//typedef union hammer_btree_elm *hammer_btree_elm_t;
type hammer_btree_elm_t *hammer_btree_elm
//

/*
 * B-Tree node (normal or meta)	(64x64 = 4K structure)
 *
 * Each node contains 63 elements.  The last element for an internal node
 * is the right-boundary so internal nodes have one fewer logical elements
 * then leaf nodes.
 *
 * 'count' always refers to the number of elements and is non-inclusive of
 * tHAMMER_BTREE_LEAF_ELMShe right-hand boundary for an internal node.
 *
 * The use of a fairly large radix is designed to reduce the number of
 * discrete disk accesses required to locate something.  Keep in mind
 * that nodes are allocated out of 16K hammer buffers so supported values
 * are (256-1), (128-1), (64-1), (32-1), or (16-1).
 *
 * NOTE: The node head for an internal does not contain the subtype
 * (The B-Tree node type for the nodes referenced by its elements). 
 * Instead, each element specifies the subtype (elm->base.subtype).
 * This allows us to maintain an unbalanced B-Tree and to easily identify
 * special inter-cluster link elements.
 *
 * NOTE: FUTURE EXPANSION: The reserved fields in hammer_node_ondisk are
 * reserved for left/right leaf linkage fields, flags, and other future
 * features.
 */
//#define HAMMER_BTREE_LEAF_ELMS	63
const HAMMER_BTREE_LEAF_ELMS		= 63
//#define HAMMER_BTREE_INT_ELMS	(HAMMER_BTREE_LEAF_ELMS - 1)
const HAMMER_BTREE_INT_ELMS		= (HAMMER_BTREE_LEAF_ELMS - 1)
//

/*
 * It is safe to combine two adjacent nodes if the total number of elements
 * is less then or equal to the *_FILL constant.
 */
//#define HAMMER_BTREE_LEAF_FILL	(HAMMER_BTREE_LEAF_ELMS - 3)
const HAMMER_BTREE_LEAF_FILL	= (HAMMER_BTREE_LEAF_ELMS - 3)
//#define HAMMER_BTREE_INT_FILL	(HAMMER_BTREE_INT_ELMS - 3)
const HAMMER_BTREE_INT_FILL	= (HAMMER_BTREE_INT_ELMS - 3)
//

//#define HAMMER_BTREE_TYPE_INTERNAL	((u_int8_t)'I')
const HAMMER_BTREE_TYPE_INTERNAL	= uint8('I')
//#define HAMMER_BTREE_TYPE_LEAF		((u_int8_t)'L')
const HAMMER_BTREE_TYPE_LEAF		= uint8('L')
//#define HAMMER_BTREE_TYPE_RECORD	((u_int8_t)'R')
const HAMMER_BTREE_TYPE_RECORD		= uint8('R')
//#define HAMMER_BTREE_TYPE_DELETED	((u_int8_t)'D')
const HAMMER_BTREE_TYPE_DELETED		= uint8('D')
//

//struct hammer_node_ondisk {
type hammer_node_ondisk struct {
	/*
	 * B-Tree node header (64 bytes)
	 */
//	hammer_crc_t	crc;		/* MUST BE FIRST FIELD OF STRUCTURE */
	crc 		hammer_crc_t	/* MUST BE FIRST FIELD OF STRUCTURE */
//	u_int32_t	signature;
	signature	uint32
//	hammer_off_t	parent;		/* 0 if at root of cluster */
	parent		hammer_off_t	/* 0 if at root of cluster */
//	int32_t		count;
	count		int32
//	u_int8_t	type;
	_type		uint8
//	u_int8_t	reserved01;
	reserved01	uint8
//	u_int16_t	reserved02;
	reserved02	uint16
//	hammer_off_t	reserved03;	/* future link_left */
	reserved03	hammer_off_t	/* future link_left */
//	hammer_off_t	reserved04;	/* future link_right */
	reserved04	hammer_off_t	/* future link_right */
//	hammer_off_t	reserved05;
	reserved05	hammer_off_t
//	hammer_off_t	reserved06;
	reserved06	hammer_off_t
//	hammer_tid_t	mirror_tid;	/* mirroring support (aggregator) */
	mirror_tid	hammer_tid_t	/* mirroring support (aggregator) */
//

	/*
	 * Element array.  Internal nodes have one less logical element
	 * (meaning: the same number of physical elements) in order to
	 * accomodate the right-hand boundary.  The left-hand boundary
	 * is integrated into the first element.  Leaf nodes have no
	 * boundary elements.
	 */
//	union hammer_btree_elm elms[HAMMER_BTREE_LEAF_ELMS];
	elms []hammer_btree_elm
//};
}
//

//#define HAMMER_BTREE_SIGNATURE_GOOD		0xB3A49586
const HAMMER_BTREE_SIGNATURE_GOOD		= 0xB3A49586
//#define HAMMER_BTREE_SIGNATURE_DESTROYED	0x4A3B2C1D
const HAMMER_BTREE_SIGNATURE_DESTROYED		= 0x4A3B2C1D
//#define HAMMER_BTREE_CRCSIZE	\
//	(sizeof(struct hammer_node_ondisk) - sizeof(hammer_crc_t))
func HAMMER_BTREE_CRCSIZE() uintptr {
	return unsafe.Sizeof(*new(hammer_node_ondisk)) - unsafe.Sizeof(*new(*hammer_crc_t))
}
//

//typedef struct hammer_node_ondisk *hammer_node_ondisk_t;
type hammer_node_ondisk_t *hammer_node_ondisk
//}
//

/*
 * HAMMER B-Tree index
 *
 * HAMMER implements a modified B+Tree.  In documentation this will
 * simply be refered to as the HAMMER B-Tree.  Basically a HAMMER B-Tree
 * looks like a B+Tree (A B-Tree which stores its records only at the leafs
 * of the tree), but adds two additional boundary elements which describe
 * the left-most and right-most element a node is able to represent.  In
 * otherwords, we have boundary elements at the two ends of a B-Tree node
 * instead of sub-tree pointers.
 *
 * A B-Tree internal node looks like this:
 *
 *	B N N N N N N B   <-- boundary and internal elements
 *       S S S S S S S    <-- subtree pointers
 *
 * A B-Tree leaf node basically looks like this:
 *
 *	L L L L L L L L   <-- leaf elemenets
 *
 * The radix for an internal node is 1 less then a leaf but we get a
 * number of significant benefits for our troubles.
 *
 * The big benefit to using a B-Tree containing boundary information
 * is that it is possible to cache pointers into the middle of the tree
 * and not have to start searches, insertions, OR deletions at the root
 * node.   In particular, searches are able to progress in a definitive
 * direction from any point in the tree without revisting nodes.  This
 * greatly improves the efficiency of many operations, most especially
 * record appends.
 *
 * B-Trees also make the stacking of trees fairly straightforward.
 *
 * INSERTIONS:  A search performed with the intention of doing
 * an insert will guarantee that the terminal leaf node is not full by
 * splitting full nodes.  Splits occur top-down during the dive down the
 * B-Tree.
 *
 * DELETIONS: A deletion makes no attempt to proactively balance the
 * tree and will recursively remove nodes that become empty.  If a
 * deadlock occurs a deletion may not be able to remove an empty leaf.
 * Deletions never allow internal nodes to become empty (that would blow
 * up the boundaries).
 */
//#include "hammer.h"

//#include <sys/buf.h>

//#include <sys/buf2.h>

//

//static int btree_search(hammer_cursor_t cursor, int flags);

//static int btree_split_internal(hammer_cursor_t cursor);

//static int btree_split_leaf(hammer_cursor_t cursor);

//static int btree_remove(hammer_cursor_t cursor);

//static int btree_node_is_full(hammer_node_ondisk_t node);

//static int hammer_btree_mirror_propagate(hammer_cursor_t cursor,	

//			hammer_tid_t mirror_tid);

//static void hammer_make_separator(hammer_base_elm_t key1,

//			hammer_base_elm_t key2, hammer_base_elm_t dest);

//static void hammer_cursor_mirror_filter(hammer_cursor_t cursor);

//

/*
 * Iterate records after a search.  The cursor is iterated forwards past
 * the current record until a record matching the key-range requirements
 * is found.  ENOENT is returned if the iteration goes past the ending
 * key. 
 *
 * The iteration is inclusive of key_beg and can be inclusive or exclusive
 * of key_end depending on whether HAMMER_CURSOR_END_INCLUSIVE is set.
 *
 * When doing an as-of search (cursor.asof != 0), key_beg.create_tid
 * may be modified by B-Tree functions.
 *
 * cursor.key_beg may or may not be modified by this function during
 * the iteration.  XXX future - in case of an inverted lock we may have
 * to reinitiate the lookup and set key_beg to properly pick up where we
 * left off.
 *
 * If HAMMER_CURSOR_ITERATE_CHECK is set it is possible that the cursor
 * was reverse indexed due to being moved to a parent while unlocked,
 * and something else might have inserted an element outside the iteration
 * range.  When this case occurs the iterator just keeps iterating until
 * it gets back into the iteration range (instead of asserting).
 *
 * NOTE!  EDEADLK *CANNOT* be returned by this procedure.
 */
//int
//hammer_btree_iterate(hammer_cursor_t cursor)
//{
func hammer_btree_iterate(cursor *hammer_cursor) int {
	var base_elm hammer_base_elm
//	hammer_node_ondisk_t node;
	var node hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_mount_t hmp;
	var hmp hammer_mount_t
//	int error = 0;
	error := 0
//	int r;
	var r int
//	int s;
	var s int
//

	/*
	 * Skip past the current record
	 */
//	hmp = cursor->trans->hmp;
	hmp = cursor.trans.hmp
//	node = cursor->node->ondisk;
	node = cursor.node.ondisk
//	if (node == NULL)
	if node == nil {
//		return(ENOENT);
		return system.ENOENT
	}
//	if (cursor->index < node->count && 
	if (int64(cursor.index) < int64(node.count)) &&
//	    (cursor->flags & HAMMER_CURSOR_ATEDISK)) {
			((cursor.flags & HAMMER_CURSOR_ATEDISK) != 0) {
//		++cursor->index;
		cursor.index++
//	}
	}
//

	/*
	 * HAMMER can wind up being cpu-bound.
	 */
	hmp.check_yield++
//	if (++hmp->check_yield > hammer_yield_check) {
	if int64(hmp.check_yield) > int64(hammer_yield_check) {
//		hmp->check_yield = 0;
		hmp.check_yield = 0;
//		lwkt_user_yield();
		kernel.Lwkt_user_yield();
//	}
	}
//

//

	/*
	 * Loop until an element is found or we are done.
	 */
//	for (;;) {
	for {
		/*
		 * We iterate up the tree and then index over one element
		 * while we are at the last element in the current node.
		 *
		 * If we are at the root of the filesystem, cursor_up
		 * returns ENOENT.
		 *
		 * XXX this could be optimized by storing the information in
		 * the parent reference.
		 *
		 * XXX we can lose the node lock temporarily, this could mess
		 * up our scan.
		 */
//		++hammer_stats_btree_iterations;
		hammer_stats_btree_iterations++
//		hammer_flusher_clean_loose_ios(hmp);
		hammer_flusher_clean_loose_ios(hmp)
//

//		if (cursor->index == node->count) {
		if int64(cursor.index) == int64(node.count) {
//			if (hammer_debug_btree) {
			if hammer_debug_btree != 0 {
				var zeroed_hammer_node_t hammer_node_t
				var arg_for_kprintf int64
				if cursor.parent != zeroed_hammer_node_t {
					arg_for_kprintf = int64(cursor.parent.node_offset)
				} else {
					arg_for_kprintf = int64(-1)
				}
//				kprintf("BRACKETU %016llx[%d] -> %016llx[%d] (td=%p)\n",
				kernel.Kprintf("BRACKETU %016llx[%d] . %016llx[%d] (td=%p)\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index,
					cursor.index,
//					(long long)(cursor->parent ? cursor->parent->node_offset : -1),
					arg_for_kprintf,
//					cursor->parent_index,
					cursor.parent_index,
//					curthread);
					platform.Curthread);
			}
//			KKASSERT(cursor->parent == NULL || cursor->parent->ondisk->elms[cursor->parent_index].internal.subtree_offset == cursor->node->node_offset);
			kernel.KKASSERT((cursor.parent == nil) || (cursor.parent.ondisk.elms[cursor.parent_index].internal(0).subtree_offset == hammer_off_t(cursor.node.node_offset)))
//			error = hammer_cursor_up(cursor);
			error = hammer_cursor_up(cursor)
//			if (error)
			if error != 0 {
//				break;
				break
			}
//			/* reload stale pointer */
			/* reload stale pointer */
//			node = cursor->node->ondisk;
			node = cursor.node.ondisk
//			KKASSERT(cursor->index != node->count);
			kernel.KKASSERT(int64(cursor.index) != int64(node.count))
//

			/*
			 * If we are reblocking we want to return internal
			 * nodes.  Note that the internal node will be
			 * returned multiple times, on each upward recursion
			 * from its children.  The caller selects which
			 * revisit it cares about (usually first or last only).
			 */
//			if (cursor->flags & HAMMER_CURSOR_REBLOCKING) {
			if (cursor.flags & HAMMER_CURSOR_REBLOCKING) != 0 {
//				cursor->flags |= HAMMER_CURSOR_ATEDISK;
				cursor.flags |= HAMMER_CURSOR_ATEDISK
//				return(0);
				return 0
//			}
			}
//			++cursor->index;
			cursor.index++
//			continue;
			continue
//		}
		}
//

		/*
		 * Check internal or leaf element.  Determine if the record
		 * at the cursor has gone beyond the end of our range.
		 *
		 * We recurse down through internal nodes.
		 */
//		if (node->type == HAMMER_BTREE_TYPE_INTERNAL) {
		if (node._type == HAMMER_BTREE_TYPE_INTERNAL) {
//			elm = &node->elms[cursor->index];
			elm = node.elms[cursor.index]
//

//			r = hammer_btree_cmp(&cursor->key_end, &elm[0].base);
			r = hammer_btree_cmp(&cursor.key_end, hammer_base_elm_t(elm.base(0)))
//			s = hammer_btree_cmp(&cursor->key_beg, &elm[1].base);
			s = hammer_btree_cmp(&cursor.key_beg, hammer_base_elm_t(elm.base(1)))
//			if (hammer_debug_btree) {
			if hammer_debug_btree != 0 {
//				kprintf("BRACKETL %016llx[%d] %016llx %02x %016llx lo=%02x %d (td=%p)\n",
				kernel.Kprintf("BRACKETL %016llx[%d] %016llx %02x %016llx lo=%02x %d (td=%p)\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index,
					cursor.index,
//					(long long)elm[0].internal.base.obj_id,
					int64(elm.internal(0).base.obj_id),
//					elm[0].internal.base.rec_type,
					elm.internal(0).base.rec_type,
//					(long long)elm[0].internal.base.key,
					int64(elm.internal(0).base.key),
//					elm[0].internal.base.localization,
					elm.internal(0).base.localization,
//					r,
					r,
//					curthread
//				);
					platform.Curthread)
//				kprintf("BRACKETR %016llx[%d] %016llx %02x %016llx lo=%02x %d\n",
				kernel.Kprintf("BRACKETR %016llx[%d] %016llx %02x %016llx lo=%02x %d\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index + 1,
					cursor.index + 1,
//					(long long)elm[1].internal.base.obj_id,
					int64(elm.internal(1).base.obj_id),
//					elm[1].internal.base.rec_type,
					elm.internal(1).base.rec_type,
//					(long long)elm[1].internal.base.key,
					int64(elm.internal(1).base.key),
//					elm[1].internal.base.localization,
					elm.internal(1).base.localization,
//					s
//				);
					s)
//			}
			}
//

//			if (r < 0) {
			if (r < 0) {
//				error = ENOENT;
				error = system.ENOENT
//				break;
				break
//			}
			}
//			if (r == 0 && (cursor->flags &
			if (r == 0 && (cursor.flags &
//				       HAMMER_CURSOR_END_INCLUSIVE) == 0) {
				       HAMMER_CURSOR_END_INCLUSIVE) == 0) {
//				error = ENOENT;
				error = system.ENOENT
//				break;
				break
//			}
			}
//

			/*
			 * Better not be zero
			 */
//			KKASSERT(elm->internal.subtree_offset != 0);
			kernel.KKASSERT(elm.internal(0).subtree_offset != 0)
//

//			if (s <= 0) {
			if (s <= 0) {
				/*
				 * If running the mirror filter see if we
				 * can skip one or more entire sub-trees.
				 * If we can we return the internal node
				 * and the caller processes the skipped
				 * range (see mirror_read).
				 */
//				if (cursor->flags &
				if (cursor.flags &
//				    HAMMER_CURSOR_MIRROR_FILTERED) {
				    HAMMER_CURSOR_MIRROR_FILTERED) != 0 {
//					if (elm->internal.mirror_tid <
					if (elm.internal(0).mirror_tid <
//					    cursor->cmirror->mirror_tid) {
					    hammer_tid_t(cursor.cmirror.mirror_tid)) {
//						hammer_cursor_mirror_filter(cursor);
						hammer_cursor_mirror_filter(cursor)
//						return(0);
						return 0
//					}
					}
//				}
				}
//			} else {
			} else {
				/*
				 * Normally it would be impossible for the
				 * cursor to have gotten back-indexed,
				 * but it can happen if a node is deleted
				 * and the cursor is moved to its parent
				 * internal node.  ITERATE_CHECK will be set.
				 */
//				KKASSERT(cursor->flags &
				kernel.KKASSERT(cursor.flags &
//					 HAMMER_CURSOR_ITERATE_CHECK);
					 HAMMER_CURSOR_ITERATE_CHECK);
//				kprintf("hammer_btree_iterate: "
//					"DEBUG: Caught parent seek "
//					"in internal iteration\n");
				kernel.Kprintf("hammer_btree_iterate: DEBUG: Caught parent seek in internal iteration\n")
//			}
			}
//

//			error = hammer_cursor_down(cursor);
			error = hammer_cursor_down(cursor)
//			if (error)
			if error != 0 {
//				break;
				break
			}
//			KKASSERT(cursor->index == 0);
			kernel.KKASSERT(cursor.index == 0)
//			/* reload stale pointer */
			/* reload stale pointer */
//			node = cursor->node->ondisk;
			node = cursor.node.ondisk
//			continue;
			continue
//		} else {
		} else {
//			elm = &node->elms[cursor->index];
			elm = node.elms[cursor.index]
//			r = hammer_btree_cmp(&cursor->key_end, &elm->base);
			r = hammer_btree_cmp(&cursor.key_end, hammer_base_elm_t(elm.base(0)));
//			if (hammer_debug_btree) {
			if hammer_debug_btree != 0 {
				var arg_for_kprintf uint8
				if elm.leaf(0).base.btype != 0 {
					arg_for_kprintf = elm.leaf(0).base.btype
				} else {
					arg_for_kprintf = uint8('?')
				}
//				kprintf("ELEMENT  %016llx:%d %c %016llx %02x %016llx lo=%02x %d\n",
				kernel.Kprintf("ELEMENT  %016llx:%d %c %016llx %02x %016llx lo=%02x %d\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index,
					cursor.index,
//					(elm[0].leaf.base.btype ?
//					 elm[0].leaf.base.btype : '?'),
					arg_for_kprintf,
//					(long long)elm[0].leaf.base.obj_id,
					int64(elm.leaf(0).base.obj_id),
//					elm[0].leaf.base.rec_type,
					elm.leaf(0).base.rec_type,
//					(long long)elm[0].leaf.base.key,
					int64(elm.leaf(0).base.key),
//					elm.leaf.base.localization,
					elm.leaf(0).base.localization,
//					r
//				);
					r)
//			}
			}
//			if (r < 0) {
			if (r < 0) {
//				error = ENOENT;
				error = system.ENOENT
//				break;
				break
//			}
			}
//

			/*
			 * We support both end-inclusive and
			 * end-exclusive searches.
			 */
//			if (r == 0 &&
			if (r == 0 &&
//			   (cursor->flags & HAMMER_CURSOR_END_INCLUSIVE) == 0) {
			   (cursor.flags & HAMMER_CURSOR_END_INCLUSIVE) == 0) {
//				error = ENOENT;
				error = system.ENOENT
//				break;
				break
//			}
			}
//

			/*
			 * If ITERATE_CHECK is set an unlocked cursor may
			 * have been moved to a parent and the iterate can
			 * happen upon elements that are not in the requested
			 * range.
			 */
//			if (cursor->flags & HAMMER_CURSOR_ITERATE_CHECK) {
			if (cursor.flags & HAMMER_CURSOR_ITERATE_CHECK) != 0 {
//				s = hammer_btree_cmp(&cursor->key_beg,
				s = hammer_btree_cmp(&cursor.key_beg,
//						     &elm->base);
					hammer_base_elm_t(elm.base(0)))
//				if (s > 0) {
				if (s > 0) {
//					kprintf("hammer_btree_iterate: "
//						"DEBUG: Caught parent seek "
//						"in leaf iteration\n");
					kernel.Kprintf("hammer_btree_iterate: DEBUG: Caught parent seek in leaf iteration\n")
//					++cursor->index;
					cursor.index++
//					continue;
					continue
//				}
				}
//			}
			}
//			cursor->flags &= ~HAMMER_CURSOR_ITERATE_CHECK;
			cursor.flags &= ^HAMMER_CURSOR_ITERATE_CHECK
//

			/*
			 * Return the element
			 */
//			switch(elm->leaf.base.btype) {
			switch(elm.leaf(0).base.btype) {
//			case HAMMER_BTREE_TYPE_RECORD:
			case HAMMER_BTREE_TYPE_RECORD:
				base_elm = *elm.base(0)
//				if ((cursor->flags & HAMMER_CURSOR_ASOF) &&
				if (((cursor.flags & HAMMER_CURSOR_ASOF) != 0) &&
//				    hammer_btree_chkts(cursor->asof, &elm->base)) {
						(hammer_btree_chkts(cursor.asof, base_elm) !=0)) {
//					++cursor->index;
					cursor.index++
//					continue;
					continue
//				}
				}
//				error = 0;
				error = 0
//				break;
				break
//			default:
			default:
//				error = EINVAL;
				error = system.EINVAL
//				break;
				break
//			}
			}
//			if (error)
			if error != 0 {
//				break;
				break
			}
//		}
		}
		/*
		 * node pointer invalid after loop
		 */
//

		/*
		 * Return entry
		 */
//		if (hammer_debug_btree) {
		if hammer_debug_btree != 0 {
//			int i = cursor->index;
			i := cursor.index
//			hammer_btree_elm_t elm = &cursor->node->ondisk->elms[i];
			elm := cursor.node.ondisk.elms[i]
//			kprintf("ITERATE  %p:%d %016llx %02x %016llx lo=%02x\n",
			kernel.Kprintf("ITERATE  %p:%d %016llx %02x %016llx lo=%02x\n",
//				cursor->node, i,
				cursor.node, i,
//				(long long)elm->internal.base.obj_id,
				int64(elm.internal(0).base.obj_id),
//				elm->internal.base.rec_type,
				elm.internal(0).base.rec_type,
//				(long long)elm->internal.base.key,
				int64(elm.internal(0).base.key),
//				elm->internal.base.localization
//			);
				elm.internal(0).base.localization)
//		}
		}
//		return(0);
		return 0
//	}
	}
//	return(error);
	return error
//}
}
//

/*
 * We hit an internal element that we could skip as part of a mirroring
 * scan.  Calculate the entire range being skipped.
 *
 * It is important to include any gaps between the parent's left_bound
 * and the node's left_bound, and same goes for the right side.
 */
//static void
//hammer_cursor_mirror_filter(hammer_cursor_t cursor)
//{
func hammer_cursor_mirror_filter(cursor *hammer_cursor) {
//	struct hammer_cmirror *cmirror;
	var cmirror *hammer_cmirror
//	hammer_node_ondisk_t ondisk;
	var ondisk hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//

//	ondisk = cursor->node->ondisk;
	ondisk = cursor.node.ondisk
//	cmirror = cursor->cmirror;
	cmirror = cursor.cmirror
//

	/*
	 * Calculate the skipped range
	 */
//	elm = &ondisk->elms[cursor->index];
	elm = ondisk.elms[cursor.index]
//	if (cursor->index == 0)
	if (cursor.index == 0) {
//		cmirror->skip_beg = *cursor->left_bound;
		cmirror.skip_beg = *cursor.left_bound
//	else
	} else {
//		cmirror->skip_beg = elm->internal.base;
		cmirror.skip_beg = elm.internal(0).base
	}
//	while (cursor->index < ondisk->count) {
	for ; int64(cursor.index) < int64(ondisk.count); {
//		if (elm->internal.mirror_tid >= cmirror->mirror_tid)
		if (elm.internal(0).mirror_tid >= cmirror.mirror_tid) {
//			break;
			break
		}
//		++cursor->index;
		cursor.index++
//		++elm;
		elm[cursor.index]++
//	}
	}
//	if (cursor->index == ondisk->count)
	if (int64(cursor.index) == int64(ondisk.count)) {
//		cmirror->skip_end = *cursor->right_bound;
		cmirror.skip_end = *cursor.right_bound
//	else
	} else {
//		cmirror->skip_end = elm->internal.base;
		cmirror.skip_end = elm.internal(0).base
	}
//

	/*
	 * clip the returned result.
	 */
//	if (hammer_btree_cmp(&cmirror->skip_beg, &cursor->key_beg) < 0)
	if (hammer_btree_cmp(&cmirror.skip_beg, &cursor.key_beg) < 0) {
//		cmirror->skip_beg = cursor->key_beg;
		cmirror.skip_beg = cursor.key_beg
	}
//	if (hammer_btree_cmp(&cmirror->skip_end, &cursor->key_end) > 0)
	if (hammer_btree_cmp(&cmirror.skip_end, &cursor.key_end) > 0) {
//		cmirror->skip_end = cursor->key_end;
		cmirror.skip_end = cursor.key_end
	}
//}
}
//

/*
 * Iterate in the reverse direction.  This is used by the pruning code to
 * avoid overlapping records.
 */
//int
//hammer_btree_iterate_reverse(hammer_cursor_t cursor)
//{
func hammer_btree_iterate_reverse(cursor *hammer_cursor) int {
//	hammer_node_ondisk_t node;
	var node hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_mount_t hmp;
	var hmp hammer_mount_t
//	int error = 0;
	error := 0
//	int r;
	var r int
//	int s;
	var s int
//

//	/* mirror filtering not supported for reverse iteration */
	/* mirror filtering not supported for reverse iteration */
//	KKASSERT ((cursor->flags & HAMMER_CURSOR_MIRROR_FILTERED) == 0);
	kernel.KKASSERT((cursor.flags & HAMMER_CURSOR_MIRROR_FILTERED) == 0)
//

	/*
	 * Skip past the current record.  For various reasons the cursor
	 * may end up set to -1 or set to point at the end of the current
	 * node.  These cases must be addressed.
	 */
//	node = cursor->node->ondisk;
	node = cursor.node.ondisk
//	if (node == NULL)
	if node == nil {
//		return(ENOENT);
		return system.ENOENT
	}
//	if (cursor->index != -1 && 
	if (cursor.index != -1 && 
//	    (cursor->flags & HAMMER_CURSOR_ATEDISK)) {
	    ((cursor.flags & HAMMER_CURSOR_ATEDISK) != 0)) {
//		--cursor->index;
		cursor.index--
//	}
	}
//	if (cursor->index == cursor->node->ondisk->count)
	if int64(cursor.index) == int64(cursor.node.ondisk.count) {
//		--cursor->index;
		cursor.index--
	}
//

	/*
	 * HAMMER can wind up being cpu-bound.
	 */
//	hmp = cursor->trans->hmp;
	hmp = cursor.trans.hmp
	hmp.check_yield++
//	if (++hmp->check_yield > hammer_yield_check) {
	if int64(hmp.check_yield) > int64(hammer_yield_check) {
//		hmp->check_yield = 0;
		hmp.check_yield = 0
//		lwkt_user_yield();
		kernel.Lwkt_user_yield()
//	}
	}
//

	/*
	 * Loop until an element is found or we are done.
	 */
//	for (;;) {
	for {
//		++hammer_stats_btree_iterations;
		hammer_stats_btree_iterations++
//		hammer_flusher_clean_loose_ios(hmp);
		hammer_flusher_clean_loose_ios(hmp)
//

		/*
		 * We iterate up the tree and then index over one element
		 * while we are at the last element in the current node.
		 */
//		if (cursor->index == -1) {
		if (cursor.index == -1) {
//			error = hammer_cursor_up(cursor);
			error = hammer_cursor_up(cursor);
//			if (error) {
			if error != 0 {
//				cursor->index = 0; /* sanity */
				cursor.index = 0 /* sanity */
//				break;
				break
//			}
			}
//			/* reload stale pointer */
			/* reload stale pointer */
//			node = cursor->node->ondisk;
			node = cursor.node.ondisk
//			KKASSERT(cursor->index != node->count);
			kernel.KKASSERT(int64(cursor.index) != int64(node.count))
//			--cursor->index;
			cursor.index--
//			continue;
			continue
//		}
		}
//

		/*
		 * Check internal or leaf element.  Determine if the record
		 * at the cursor has gone beyond the end of our range.
		 *
		 * We recurse down through internal nodes. 
		 */
//		KKASSERT(cursor->index != node->count);
		kernel.KKASSERT(int64(cursor.index) != int64(node.count));
//		if (node->type == HAMMER_BTREE_TYPE_INTERNAL) {
		if (node._type == HAMMER_BTREE_TYPE_INTERNAL) {
//			elm = &node->elms[cursor->index];
			elm = node.elms[cursor.index]
//			r = hammer_btree_cmp(&cursor->key_end, &elm[0].base);
			r = hammer_btree_cmp(&cursor.key_end, hammer_base_elm_t(elm.base(0)))
//			s = hammer_btree_cmp(&cursor->key_beg, &elm[1].base);
			s = hammer_btree_cmp(&cursor.key_beg, hammer_base_elm_t(elm.base(1)))
//			if (hammer_debug_btree) {
			if hammer_debug_btree != 0 {
//				kprintf("BRACKETL %016llx[%d] %016llx %02x %016llx lo=%02x %d\n",
				kernel.Kprintf("BRACKETL %016llx[%d] %016llx %02x %016llx lo=%02x %d\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index,
					cursor.index,
//					(long long)elm[0].internal.base.obj_id,
					int64(elm.internal(0).base.obj_id),
//					elm[0].internal.base.rec_type,
					elm.internal(0).base.rec_type,
//					(long long)elm[0].internal.base.key,
					int64(elm.internal(0).base.key),
//					elm[0].internal.base.localization,
					elm.internal(0).base.localization,
//					r
//				);
					r)
//				kprintf("BRACKETR %016llx[%d] %016llx %02x %016llx lo=%02x %d\n",
				kernel.Kprintf("BRACKETR %016llx[%d] %016llx %02x %016llx lo=%02x %d\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index + 1,
					cursor.index + 1,
//					(long long)elm[1].internal.base.obj_id,
					int64(elm.internal(1).base.obj_id),
//					elm[1].internal.base.rec_type,
					elm.internal(1).base.rec_type,
//					(long long)elm[1].internal.base.key,
					int64(elm.internal(1).base.key),
//					elm[1].internal.base.localization,
					elm.internal(1).base.localization,
//					s
//				);
					s)
//			}
			}
//

//			if (s >= 0) {
			if s >= 0 {
//				error = ENOENT;
				error = system.ENOENT
//				break;
				break
//			}
			}
//

			/*
			 * It shouldn't be possible to be seeked past key_end,
			 * even if the cursor got moved to a parent.
			 */
//			KKASSERT(r >= 0);
			kernel.KKASSERT(r >= 0)
//

			/*
			 * Better not be zero
			 */
//			KKASSERT(elm->internal.subtree_offset != 0);
			kernel.KKASSERT(elm.internal(0).subtree_offset != 0)
//

//			error = hammer_cursor_down(cursor);
			error = hammer_cursor_down(cursor)
//			if (error)
			if error != 0 {
//				break;
				break
			}
//			KKASSERT(cursor->index == 0);
			kernel.KKASSERT(cursor.index == 0)
//			/* reload stale pointer */
			/* reload stale pointer */
//			node = cursor->node->ondisk;
			node = cursor.node.ondisk
//

//			/* this can assign -1 if the leaf was empty */
			/* this can assign -1 if the leaf was empty */
//			cursor->index = node->count - 1;
			cursor.index = int(node.count - 1)
//			continue;
			continue
//		} else {
		} else {
//			elm = &node->elms[cursor->index];
			elm = node.elms[cursor.index]
//			s = hammer_btree_cmp(&cursor->key_beg, &elm->base);
			s = hammer_btree_cmp(&cursor.key_beg, hammer_base_elm_t(elm.base(0)))
//			if (hammer_debug_btree) {
			if hammer_debug_btree != 0 {
				var arg_for_kprintf uint8
				if elm.leaf(0).base.btype != 0 {
					arg_for_kprintf = elm.leaf(0).base.btype
				} else {
					arg_for_kprintf = uint8('?')
				}
//				kprintf("ELEMENT  %016llx:%d %c %016llx %02x %016llx lo=%02x %d\n",
				kernel.Kprintf("ELEMENT  %016llx:%d %c %016llx %02x %016llx lo=%02x %d\n",
//					(long long)cursor->node->node_offset,
					int64(cursor.node.node_offset),
//					cursor->index,
					cursor.index,
//					(elm[0].leaf.base.btype ?
//					 elm[0].leaf.base.btype : '?'),
					arg_for_kprintf,
//					(long long)elm[0].leaf.base.obj_id,
					int64(elm.leaf(0).base.obj_id),
//					elm[0].leaf.base.rec_type,
					elm.leaf(0).base.rec_type,
//					(long long)elm[0].leaf.base.key,
					int64(elm.leaf(0).base.key),
//					elm[0].leaf.base.localization,
					elm.leaf(0).base.localization,
//					s
//				);
					s)
//			}
			}
//			if (s > 0) {
			if s > 0 {
//				error = ENOENT;
				error = system.ENOENT
//				break;
				break
//			}
			}
//

			/*
			 * It shouldn't be possible to be seeked past key_end,
			 * even if the cursor got moved to a parent.
			 */
//			cursor->flags &= ~HAMMER_CURSOR_ITERATE_CHECK;
			cursor.flags &= ^HAMMER_CURSOR_ITERATE_CHECK
//

			/*
			 * Return the element
			 */
//			switch(elm->leaf.base.btype) {
			switch(elm.leaf(0).base.btype) {
//			case HAMMER_BTREE_TYPE_RECORD:
			case HAMMER_BTREE_TYPE_RECORD:
//				if ((cursor->flags & HAMMER_CURSOR_ASOF) &&
				if (((cursor.flags & HAMMER_CURSOR_ASOF) != 0) &&
//				    hammer_btree_chkts(cursor->asof, &elm->base)) {
				    (hammer_btree_chkts(cursor.asof, *elm.base(0)) != 0)) {
//					--cursor->index;
					cursor.index--
//					continue;
					continue
//				}
				}
//				error = 0;
				error = 0
//				break;
				break
//			default:
			default:
//				error = EINVAL;
				error = system.EINVAL
//				break;
				break
//			}
			}
//			if (error)
			if error != 0 {
//				break;
				break
			}
//		}
		}
		/*
		 * node pointer invalid after loop
		 */
//

		/*
		 * Return entry
		 */
//		if (hammer_debug_btree) {
		if hammer_debug_btree != 0 {
//			int i = cursor->index;
			i := cursor.index
//			hammer_btree_elm_t elm = &cursor->node->ondisk->elms[i];
			elm := cursor.node.ondisk.elms[i];
//			kprintf("ITERATE  %p:%d %016llx %02x %016llx lo=%02x\n",
			kernel.Kprintf("ITERATE  %p:%d %016llx %02x %016llx lo=%02x\n",
//				cursor->node, i,
				cursor.node, i,
//				(long long)elm->internal.base.obj_id,
				int64(elm.internal(0).base.obj_id),
//				elm->internal.base.rec_type,
				elm.internal(0).base.rec_type,
//				(long long)elm->internal.base.key,
				int64(elm.internal(0).base.key),
//				elm->internal.base.localization
//			);
				elm.internal(0).base.localization)
//		}
		}
//		return(0);
		return 0
//	}
	}
//	return(error);
	return error
//}
}
//

/*
 * Lookup cursor.key_beg.  0 is returned on success, ENOENT if the entry
 * could not be found, EDEADLK if inserting and a retry is needed, and a
 * fatal error otherwise.  When retrying, the caller must terminate the
 * cursor and reinitialize it.  EDEADLK cannot be returned if not inserting.
 * 
 * The cursor is suitably positioned for a deletion on success, and suitably
 * positioned for an insertion on ENOENT if HAMMER_CURSOR_INSERT was
 * specified.
 *
 * The cursor may begin anywhere, the search will traverse the tree in
 * either direction to locate the requested element.
 *
 * Most of the logic implementing historical searches is handled here.  We
 * do an initial lookup with create_tid set to the asof TID.  Due to the
 * way records are laid out, a backwards iteration may be required if
 * ENOENT is returned to locate the historical record.  Here's the
 * problem:
 *
 * create_tid:    10      15       20
 *		     LEAF1   LEAF2
 * records:         (11)        (18)
 *
 * Lets say we want to do a lookup AS-OF timestamp 17.  We will traverse
 * LEAF2 but the only record in LEAF2 has a create_tid of 18, which is
 * not visible and thus causes ENOENT to be returned.  We really need
 * to check record 11 in LEAF1.  If it also fails then the search fails
 * (e.g. it might represent the range 11-16 and thus still not match our
 * AS-OF timestamp of 17).  Note that LEAF1 could be empty, requiring
 * further iterations.
 *
 * If this case occurs btree_search() will set HAMMER_CURSOR_CREATE_CHECK
 * and the cursor.create_check TID if an iteration might be needed.
 * In the above example create_check would be set to 14.
 */
//int
//hammer_btree_lookup(hammer_cursor_t cursor)
//{
func hammer_btree_lookup(cursor *hammer_cursor) int {
//	int error;
	var error int
//

//	cursor->flags &= ~HAMMER_CURSOR_ITERATE_CHECK;
	cursor.flags &= ^HAMMER_CURSOR_ITERATE_CHECK;
//	KKASSERT ((cursor->flags & HAMMER_CURSOR_INSERT) == 0 ||
	kernel.KKASSERT ((cursor.flags & HAMMER_CURSOR_INSERT) == 0 ||
//		  cursor->trans->sync_lock_refs > 0);
		  cursor.trans.sync_lock_refs > 0)
//	++hammer_stats_btree_lookups;
	hammer_stats_btree_lookups++
//	if (cursor->flags & HAMMER_CURSOR_ASOF) {
	if (cursor.flags & HAMMER_CURSOR_ASOF) != 0 {
//		KKASSERT((cursor->flags & HAMMER_CURSOR_INSERT) == 0);
		kernel.KKASSERT((cursor.flags & HAMMER_CURSOR_INSERT) == 0)
//		cursor->key_beg.create_tid = cursor->asof;
		cursor.key_beg.create_tid = cursor.asof
//		for (;;) {
		for {
//			cursor->flags &= ~HAMMER_CURSOR_CREATE_CHECK;
			cursor.flags &= ^HAMMER_CURSOR_CREATE_CHECK
//			error = btree_search(cursor, 0);
			error = btree_search(cursor, 0)
//			if (error != ENOENT ||
			if (error != system.ENOENT ||
//			    (cursor->flags & HAMMER_CURSOR_CREATE_CHECK) == 0) {
			    (cursor.flags & HAMMER_CURSOR_CREATE_CHECK) == 0) {
				/*
				 * Stop if no error.
				 * Stop if error other then ENOENT.
				 * Stop if ENOENT and not special case.
				 */
//				break;
				break
//			}
			}
//			if (hammer_debug_btree) {
			if hammer_debug_btree != 0 {
//				kprintf("CREATE_CHECK %016llx\n",
				kernel.Kprintf("CREATE_CHECK %016llx\n",
//					(long long)cursor->create_check);
					int64(cursor.create_check))
//			}
			}
//			cursor->key_beg.create_tid = cursor->create_check;
			cursor.key_beg.create_tid = cursor.create_check
//			/* loop */
			/* loop */
//		}
		}
//	} else {
	} else {
//		error = btree_search(cursor, 0);
		error = btree_search(cursor, 0)
//	}
	}
//	if (error == 0)
	if (error == 0) {
//		error = hammer_btree_extract(cursor, cursor->flags);
		error = hammer_btree_extract(cursor, cursor.flags)
	}
//	return(error);
	return error
//}
}
//

/*
 * Execute the logic required to start an iteration.  The first record
 * located within the specified range is returned and iteration control
 * flags are adjusted for successive hammer_btree_iterate() calls.
 *
 * Set ATEDISK so a low-level caller can call btree_first/btree_iterate
 * in a loop without worrying about it.  Higher-level merged searches will
 * adjust the flag appropriately.
 */
//int
//hammer_btree_first(hammer_cursor_t cursor)
//{
func hammer_btree_first(cursor *hammer_cursor) int {
//	int error;
	var error int
//

//	error = hammer_btree_lookup(cursor);
	error = hammer_btree_lookup(cursor)
//	if (error == ENOENT) {
	if (error == system.ENOENT) {
//		cursor->flags &= ~HAMMER_CURSOR_ATEDISK;
		cursor.flags &= ^HAMMER_CURSOR_ATEDISK
//		error = hammer_btree_iterate(&cursor);
		error = hammer_btree_iterate(cursor)
//	}
	}
//	cursor->flags |= HAMMER_CURSOR_ATEDISK;
	cursor.flags |= HAMMER_CURSOR_ATEDISK
//	return(error);
	return error
//}
}
//

/*
 * Similarly but for an iteration in the reverse direction.
 *
 * Set ATEDISK when iterating backwards to skip the current entry,
 * which after an ENOENT lookup will be pointing beyond our end point.
 *
 * Set ATEDISK so a low-level caller can call btree_last/btree_iterate_reverse
 * in a loop without worrying about it.  Higher-level merged searches will
 * adjust the flag appropriately.
 */
//int
//hammer_btree_last(hammer_cursor_t cursor)
//{
func hammer_btree_last(cursor *hammer_cursor) int {
//	struct hammer_base_elm save;
	var save hammer_base_elm
//	int error;
	var error int
//

//	save = cursor->key_beg;
	save = cursor.key_beg
//	cursor->key_beg = cursor->key_end;
	cursor.key_beg = cursor.key_end
//	error = hammer_btree_lookup(cursor);
	error = hammer_btree_lookup(cursor)
//	cursor->key_beg = save;
	cursor.key_beg = save
//	if (error == ENOENT ||
	if (error == system.ENOENT ||
//	    (cursor->flags & HAMMER_CURSOR_END_INCLUSIVE) == 0) {
	    (cursor.flags & HAMMER_CURSOR_END_INCLUSIVE) == 0) {
//		cursor->flags |= HAMMER_CURSOR_ATEDISK;
		cursor.flags |= HAMMER_CURSOR_ATEDISK
//		error = hammer_btree_iterate_reverse(cursor);
		error = hammer_btree_iterate_reverse(cursor)
//	}
	}
//	cursor->flags |= HAMMER_CURSOR_ATEDISK;
	cursor.flags |= HAMMER_CURSOR_ATEDISK
//	return(error);
	return error
//}
}
//

/*
 * Extract the record and/or data associated with the cursor's current
 * position.  Any prior record or data stored in the cursor is replaced.
 * The cursor must be positioned at a leaf node.
 *
 * NOTE: All extractions occur at the leaf of the B-Tree.
 */
//int
//hammer_btree_extract(hammer_cursor_t cursor, int flags)
//{
func hammer_btree_extract(cursor *hammer_cursor, flags int) int {
//	hammer_node_ondisk_t node;
	var node hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_off_t data_off;
	var data_off hammer_off_t
//	hammer_mount_t hmp;
	var hmp hammer_mount_t
//	int32_t data_len;
	var data_len int32
//	int error;
	var error int
//

	/*
	 * The case where the data reference resolves to the same buffer
	 * as the record reference must be handled.
	 */
//	node = cursor->node->ondisk;
	node = cursor.node.ondisk
//	elm = &node->elms[cursor->index];
	elm = node.elms[cursor.index]
//	cursor->data = NULL;
	cursor.data = nil
//	hmp = cursor->node->hmp;
	hmp = cursor.node.hmp
//

	/*
	 * There is nothing to extract for an internal element.
	 */
//	if (node->type == HAMMER_BTREE_TYPE_INTERNAL)
	if (node._type == HAMMER_BTREE_TYPE_INTERNAL) {
//		return(EINVAL);
		return(system.EINVAL)
	}
//

	/*
	 * Only record types have data.
	 */
//	KKASSERT(node->type == HAMMER_BTREE_TYPE_LEAF);
	kernel.KKASSERT(node._type == HAMMER_BTREE_TYPE_LEAF)
//	cursor->leaf = &elm->leaf;
	cursor.leaf = elm.leaf(0)
//

//	if ((flags & HAMMER_CURSOR_GET_DATA) == 0)
	if ((flags & HAMMER_CURSOR_GET_DATA) == 0) {
//		return(0);
		return 0
	}
//	if (elm->leaf.base.btype != HAMMER_BTREE_TYPE_RECORD)
	if (elm.leaf(0).base.btype != HAMMER_BTREE_TYPE_RECORD) {
//		return(0);
		return 0
	}
//	data_off = elm->leaf.data_offset;
	data_off = elm.leaf(0).data_offset
//	data_len = elm->leaf.data_len;
	data_len = elm.leaf(0).data_len
//	if (data_off == 0)
	if data_off == 0 {
//		return(0);
		return 0
	}
//

	/*
	 * Load the data
	 */
//	KKASSERT(data_len >= 0 && data_len <= HAMMER_XBUFSIZE);
	kernel.KKASSERT(data_len >= 0 && data_len <= HAMMER_XBUFSIZE)
//	cursor->data = hammer_bread_ext(hmp, data_off, data_len,
	int32_cast_as_int := int(error)
	cursor.data = hammer_bread_ext(hmp, data_off, int(data_len),
//					&error, &cursor->data_buffer);
					&int32_cast_as_int,
					&cursor.data_buffer)
//

	/*
	 * Mark the data buffer as not being meta-data if it isn't
	 * meta-data (sometimes bulk data is accessed via a volume
	 * block device).
	 */
//	if (error == 0) {
	if error == 0 {
//		switch(elm->leaf.base.rec_type) {
		switch(elm.leaf(0).base.rec_type) {
//		case HAMMER_RECTYPE_DATA:
		case HAMMER_RECTYPE_DATA:
//		case HAMMER_RECTYPE_DB:
		case HAMMER_RECTYPE_DB:
//			if ((data_off & HAMMER_ZONE_LARGE_DATA) == 0)
			if ((data_off & HAMMER_ZONE_LARGE_DATA) == 0) {
//				break;
				break
			}
//			if (hammer_double_buffer == 0 ||
			if ((hammer_double_buffer == 0) ||
//			    (cursor->flags & HAMMER_CURSOR_NOSWAPCACHE)) {
					((cursor.flags & HAMMER_CURSOR_NOSWAPCACHE) != 0)) {
//				hammer_io_notmeta(cursor->data_buffer);
				hammer_io_notmeta(cursor.data_buffer)
//			}
			}
//			break;
			break
//		default:
		default:
//			break;
			break
//		}
		}
//	}
	}
//

	/*
	 * Deal with CRC errors on the extracted data.
	 */
//	if (error == 0 &&
	if ((error == 0) &&
//	    hammer_crc_test_leaf(cursor->data, &elm->leaf) == 0) {
			(hammer_crc_test_leaf(cursor.data, elm.leaf(0)) == 0)) {
//		kprintf("CRC DATA @ %016llx/%d FAILED\n",
		kernel.Kprintf("CRC DATA @ %016llx/%d FAILED\n",
//			(long long)elm->leaf.data_offset, elm->leaf.data_len);
			int64(elm.leaf(0).data_offset), elm.leaf(0).data_len)
//		if (hammer_debug_critical)
		if hammer_debug_critical != 0 {
//			Debugger("CRC FAILED: DATA");
			platform.Debugger("CRC FAILED: DATA")
		}
//		if (cursor->trans->flags & HAMMER_TRANSF_CRCDOM)
		if (cursor.trans.flags & HAMMER_TRANSF_CRCDOM) != 0 {
//			error = EDOM;	/* less critical (mirroring) */
			error = system.EDOM	/* less critical (mirroring) */
//		else
		} else {
//			error = EIO;	/* critical */
			error = system.EIO	/* critical */
		}
//	}
	}
//	return(error);
	return error
//}
}
//

//

/*
 * Insert a leaf element into the B-Tree at the current cursor position.
 * The cursor is positioned such that the element at and beyond the cursor
 * are shifted to make room for the new record.
 *
 * The caller must call hammer_btree_lookup() with the HAMMER_CURSOR_INSERT
 * flag set and that call must return ENOENT before this function can be
 * called.
 *
 * The caller may depend on the cursor's exclusive lock after return to
 * interlock frontend visibility (see HAMMER_RECF_CONVERT_DELETE).
 *
 * ENOSPC is returned if there is no room to insert a new record.
 */
//int
//hammer_btree_insert(hammer_cursor_t cursor, hammer_btree_leaf_elm_t elm,
//		    int *doprop)
//{
func hammer_btree_insert(cursor hammer_cursor, elm hammer_btree_leaf_elm, doprop *int) int {
//	hammer_node_ondisk_t node;
	var node hammer_node_ondisk_t
//	int i;
	var i int
//	int error;
	var error int
//

//	*doprop = 0;
	*doprop = 0
//	if ((error = hammer_cursor_upgrade_node(cursor)) != 0)
	error = hammer_cursor_upgrade_node(&cursor)
	if error != 0 {
//		return(error);
		return error
	}
//	++hammer_stats_btree_inserts;
	hammer_stats_btree_inserts++
//

	/*
	 * Insert the element at the leaf node and update the count in the
	 * parent.  It is possible for parent to be NULL, indicating that
	 * the filesystem's ROOT B-Tree node is a leaf itself, which is
	 * possible.  The root inode can never be deleted so the leaf should
	 * never be empty.
	 *
	 * Remember that the right-hand boundary is not included in the
	 * count.
	 */
//	hammer_modify_node_all(cursor->trans, cursor->node);
	hammer_modify_node_all(cursor.trans, cursor.node)
//	node = cursor->node->ondisk;
	node = cursor.node.ondisk
//	i = cursor->index;
	i = cursor.index
//	KKASSERT(elm->base.btype != 0);
	kernel.KKASSERT(elm.base.btype != 0)
//	KKASSERT(node->type == HAMMER_BTREE_TYPE_LEAF);
	kernel.KKASSERT(node._type == HAMMER_BTREE_TYPE_LEAF)
//	KKASSERT(node->count < HAMMER_BTREE_LEAF_ELMS);
	kernel.KKASSERT(node.count < HAMMER_BTREE_LEAF_ELMS)
//	if (i != node->count) {
	if (int64(i) != int64(node.count)) {
//		bcopy(&node->elms[i], &node->elms[i+1],
		library.Bcopy(node.elms[i], &node.elms[i+1],
//		      (node->count - i) * sizeof(*elm));
			int((int64(node.count) - int64(i)) * int64(unsafe.Sizeof(elm))));
//	}
	}
//	node->elms[i].leaf = *elm;
	node.elms[i].write(&elm)
//	++node->count;
	node.count++
//	hammer_cursor_inserted_element(cursor->node, i);
	hammer_cursor_inserted_element(cursor.node, i)
//

	/*
	 * Update the leaf node's aggregate mirror_tid for mirroring
	 * support.
	 */
//	if (node->mirror_tid < elm->base.delete_tid) {
	if (node.mirror_tid < elm.base.delete_tid) {
//		node->mirror_tid = elm->base.delete_tid;
		node.mirror_tid = elm.base.delete_tid
//		*doprop = 1;
		*doprop = 1
//	}
	}
//	if (node->mirror_tid < elm->base.create_tid) {
	if (node.mirror_tid < elm.base.create_tid) {
//		node->mirror_tid = elm->base.create_tid;
		node.mirror_tid = elm.base.create_tid
//		*doprop = 1;
		*doprop = 1
//	}
	}
//	hammer_modify_node_done(cursor->node);
	hammer_modify_node_done(cursor.node)
//

	/*
	 * Debugging sanity checks.
	 */
//	KKASSERT(hammer_btree_cmp(cursor->left_bound, &elm->base) <= 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound, &elm.base) <= 0)
//	KKASSERT(hammer_btree_cmp(cursor->right_bound, &elm->base) > 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound, &elm.base) > 0)
//	if (i) {
	if i != 0 {
//		KKASSERT(hammer_btree_cmp(&node->elms[i-1].leaf.base, &elm->base) < 0);
		kernel.KKASSERT(hammer_btree_cmp(hammer_base_elm_t(&node.elms[i-1].leaf(0).base), &elm.base) < 0)
//	}
	}
//	if (i != node->count - 1)
	if int64(i) != (int64(node.count) - int64(1)) {
//		KKASSERT(hammer_btree_cmp(&node->elms[i+1].leaf.base, &elm->base) > 0);
		kernel.KKASSERT(hammer_btree_cmp(hammer_base_elm_t(&node.elms[i+1].leaf(0).base), &elm.base) > 0)
	}
//

//	return(0);
	return 0
//}
}
//

/*
 * Delete a record from the B-Tree at the current cursor position.
 * The cursor is positioned such that the current element is the one
 * to be deleted.
 *
 * On return the cursor will be positioned after the deleted element and
 * MAY point to an internal node.  It will be suitable for the continuation
 * of an iteration but not for an insertion or deletion.
 *
 * Deletions will attempt to partially rebalance the B-Tree in an upward
 * direction, but will terminate rather then deadlock.  Empty internal nodes
 * are never allowed by a deletion which deadlocks may end up giving us an
 * empty leaf.  The pruner will clean up and rebalance the tree.
 *
 * This function can return EDEADLK, requiring the caller to retry the
 * operation after clearing the deadlock.
 */
//int
//hammer_btree_delete(hammer_cursor_t cursor)
//{
func hammer_btree_delete(cursor *hammer_cursor) int {
//	hammer_node_ondisk_t ondisk;
	var ondisk hammer_node_ondisk_t
//	hammer_node_t node;
	var node hammer_node_t
//	hammer_node_t parent __debugvar;
	var parent hammer_node_t
//	int error;
	var error int
//	int i;
	var i int
//

//	KKASSERT (cursor->trans->sync_lock_refs > 0);
	kernel.KKASSERT(cursor.trans.sync_lock_refs > 0)
//	if ((error = hammer_cursor_upgrade(cursor)) != 0)
	error = hammer_cursor_upgrade(cursor)
	if error != 0 {
//		return(error);
		return error
	}
//	++hammer_stats_btree_deletes;
	hammer_stats_btree_deletes++
//

	/*
	 * Delete the element from the leaf node. 
	 *
	 * Remember that leaf nodes do not have boundaries.
	 */
//	node = cursor->node;
	node = cursor.node
//	ondisk = node->ondisk;
	ondisk = node.ondisk
//	i = cursor->index;
	i = cursor.index
//

//	KKASSERT(ondisk->type == HAMMER_BTREE_TYPE_LEAF);
	kernel.KKASSERT(ondisk._type == HAMMER_BTREE_TYPE_LEAF)
//	KKASSERT(i >= 0 && i < ondisk->count);
	kernel.KKASSERT(i >= 0 && int64(i) < int64(ondisk.count))
//	hammer_modify_node_all(cursor->trans, node);
	hammer_modify_node_all(cursor.trans, node)
//	if (i + 1 != ondisk->count) {
	if (int64(i + 1) != int64(ondisk.count)) {
//		bcopy(&ondisk->elms[i+1], &ondisk->elms[i],
		library.Bcopy(&ondisk.elms[i+1], &ondisk.elms[i],
//		      (ondisk->count - i - 1) * sizeof(ondisk->elms[0]));
		      int((int64(ondisk.count) - int64(i) - 1) * int64(unsafe.Sizeof(ondisk.elms[0]))))
//	}
	}
//	--ondisk->count;
	ondisk.count--
//	hammer_modify_node_done(node);
	hammer_modify_node_done(node)
//	hammer_cursor_deleted_element(node, i);
	hammer_cursor_deleted_element(node, i)
//

	/*
	 * Validate local parent
	 */
//	if (ondisk->parent) {
	if ondisk.parent != 0 {
//		parent = cursor->parent;
		parent = cursor.parent
//

//		KKASSERT(parent != NULL);
		kernel.KKASSERT(parent != nil)
//		KKASSERT(parent->node_offset == ondisk->parent);
		kernel.KKASSERT(parent.node_offset == ondisk.parent)
//	}
	}
//

	/*
	 * If the leaf becomes empty it must be detached from the parent,
	 * potentially recursing through to the filesystem root.
	 *
	 * This may reposition the cursor at one of the parent's of the
	 * current node.
	 *
	 * Ignore deadlock errors, that simply means that btree_remove
	 * was unable to recurse and had to leave us with an empty leaf. 
	 */
//	KKASSERT(cursor->index <= ondisk->count);
	kernel.KKASSERT(int64(cursor.index) <= int64(ondisk.count))
//	if (ondisk->count == 0) {
	if ondisk.count == 0 {
//		error = btree_remove(cursor);
		error = btree_remove(cursor)
//		if (error == EDEADLK)
		if (error == system.EDEADLK) {
//			error = 0;
			error = 0
		}
//	} else {
	} else {
//		error = 0;
		error = 0
//	}
	}
//	KKASSERT(cursor->parent == NULL ||
	kernel.KKASSERT((cursor.parent == nil) ||
//		 cursor->parent_index < cursor->parent->ondisk->count);
		(int64(cursor.parent_index) < int64(cursor.parent.ondisk.count)))
//	return(error);
	return error
//}
}
//

/*
 * PRIMAY B-TREE SEARCH SUPPORT PROCEDURE
 *
 * Search the filesystem B-Tree for cursor.key_beg, return the matching node.
 *
 * The search can begin ANYWHERE in the B-Tree.  As a first step the search
 * iterates up the tree as necessary to properly position itself prior to
 * actually doing the sarch.
 * 
 * INSERTIONS: The search will split full nodes and leaves on its way down
 * and guarentee that the leaf it ends up on is not full.  If we run out
 * of space the search continues to the leaf (to position the cursor for
 * the spike), but ENOSPC is returned.
 *
 * The search is only guarenteed to end up on a leaf if an error code of 0
 * is returned, or if inserting and an error code of ENOENT is returned.
 * Otherwise it can stop at an internal node.  On success a search returns
 * a leaf node.
 *
 * COMPLEXITY WARNING!  This is the core B-Tree search code for the entire
 * filesystem, and it is not simple code.  Please note the following facts:
 *
 * - Internal node recursions have a boundary on the left AND right.  The
 *   right boundary is non-inclusive.  The create_tid is a generic part
 *   of the key for internal nodes.
 *
 * - Leaf nodes contain terminal elements only now.
 *
 * - Filesystem lookups typically set HAMMER_CURSOR_ASOF, indicating a
 *   historical search.  ASOF and INSERT are mutually exclusive.  When
 *   doing an as-of lookup btree_search() checks for a right-edge boundary
 *   case.  If while recursing down the left-edge differs from the key
 *   by ONLY its create_tid, HAMMER_CURSOR_CREATE_CHECK is set along
 *   with cursor.create_check.  This is used by btree_lookup() to iterate.
 *   The iteration backwards because as-of searches can wind up going
 *   down the wrong branch of the B-Tree.
 */
//static 
//int
//btree_search(hammer_cursor_t cursor, int flags)
//{
func btree_search(cursor hammer_cursor_t, flags int) (result int) {
//	hammer_node_ondisk_t node;
	var node hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	int error;
	var error int
//	int enospc = 0;
	enospc := 0
//	int i;
	var i int
//	int r;
	var r int
//	int s;
	var s int
//

//	flags |= cursor->flags;
	flags |= cursor.flags
//	++hammer_stats_btree_searches;
	hammer_stats_btree_searches++
//

//	if (hammer_debug_btree) {
	if hammer_debug_btree != 0 {
//		kprintf("SEARCH   %016llx[%d] %016llx %02x key=%016llx cre=%016llx lo=%02x (td = %p)\n",
		kernel.Kprintf("SEARCH   %016llx[%d] %016llx %02x key=%016llx cre=%016llx lo=%02x (td = %p)\n",
//			(long long)cursor->node->node_offset,
			int64(cursor.node.node_offset),
//			cursor->index,
			cursor.index,
//			(long long)cursor->key_beg.obj_id,
			int64(cursor.key_beg.obj_id),
//			cursor->key_beg.rec_type,
			cursor.key_beg.rec_type,
//			(long long)cursor->key_beg.key,
			int64(cursor.key_beg.key),
//			(long long)cursor->key_beg.create_tid,
			int64(cursor.key_beg.create_tid),
//			cursor->key_beg.localization, 
			cursor.key_beg.localization, 
//			curthread
//		);
			platform.Curthread)
//		if (cursor->parent)
		if cursor.parent != (*new(hammer_cursor_t)).parent {
//		    kprintf("SEARCHP %016llx[%d] (%016llx/%016llx %016llx/%016llx) (%p/%p %p/%p)\n",
			kernel.Kprintf("SEARCHP %016llx[%d] (%016llx/%016llx %016llx/%016llx) (%p/%p %p/%p)\n",
//			(long long)cursor->parent->node_offset,
				int64(cursor.parent.node_offset),
//			cursor->parent_index,
				cursor.parent_index,
//			(long long)cursor->left_bound->obj_id,
				int64(cursor.left_bound.obj_id),
//			(long long)cursor->parent->ondisk->elms[cursor->parent_index].internal.base.obj_id,
				int64(cursor.parent.ondisk.elms[cursor.parent_index].internal(0).base.obj_id),
//			(long long)cursor->right_bound->obj_id,
				int64(cursor.right_bound.obj_id),
//			(long long)cursor->parent->ondisk->elms[cursor->parent_index+1].internal.base.obj_id,
				int64(cursor.parent.ondisk.elms[cursor.parent_index+1].internal(0).base.obj_id),
//			cursor->left_bound,
				cursor.left_bound,
//			&cursor->parent->ondisk->elms[cursor->parent_index],
				&cursor.parent.ondisk.elms[cursor.parent_index],
//			cursor->right_bound,
				cursor.right_bound,
//			&cursor->parent->ondisk->elms[cursor->parent_index+1]
//		    );
				&cursor.parent.ondisk.elms[cursor.parent_index+1])
		}
//	}
	}
//

	/*
	 * Move our cursor up the tree until we find a node whos range covers
	 * the key we are trying to locate.
	 *
	 * The left bound is inclusive, the right bound is non-inclusive.
	 * It is ok to cursor up too far.
	 */
//	for (;;) {
	for {
//		r = hammer_btree_cmp(&cursor->key_beg, cursor->left_bound);
		r = hammer_btree_cmp(&cursor.key_beg, cursor.left_bound)
//		s = hammer_btree_cmp(&cursor->key_beg, cursor->right_bound);
		s = hammer_btree_cmp(&cursor.key_beg, cursor.right_bound)
//		if (r >= 0 && s < 0)
		if ((r >= 0) && (s < 0)) {
//			break;
			break
		}
//		KKASSERT(cursor->parent);
		kernel.KKASSERT(cursor.parent)
//		++hammer_stats_btree_iterations;
		hammer_stats_btree_iterations++
//		error = hammer_cursor_up(cursor);
		error = hammer_cursor_up(cursor)
//		if (error)
		if error != 0 {
//			goto done;
			goto done
		}
//	}
	}
//

	/*
	 * The delete-checks below are based on node, not parent.  Set the
	 * initial delete-check based on the parent.
	 */
//	if (r == 1) {
	if (r == 1) {
//		KKASSERT(cursor->left_bound->create_tid != 1);
		kernel.KKASSERT(cursor.left_bound.create_tid != 1)
//		cursor->create_check = cursor->left_bound->create_tid - 1;
		cursor.create_check = hammer_tid_t(cursor.left_bound.create_tid - 1)
//		cursor->flags |= HAMMER_CURSOR_CREATE_CHECK;
		cursor.flags |= HAMMER_CURSOR_CREATE_CHECK
//	}
	}
//

	/*
	 * We better have ended up with a node somewhere.
	 */
//	KKASSERT(cursor->node != NULL);
	kernel.KKASSERT(cursor.node != nil)
//

	/*
	 * If we are inserting we can't start at a full node if the parent
	 * is also full (because there is no way to split the node),
	 * continue running up the tree until the requirement is satisfied
	 * or we hit the root of the filesystem.
	 *
	 * (If inserting we aren't doing an as-of search so we don't have
	 *  to worry about create_check).
	 */
//	while ((flags & HAMMER_CURSOR_INSERT) && enospc == 0) {
	for ; (((flags & HAMMER_CURSOR_INSERT) != 0) && (enospc == 0)) ; {
//		if (cursor->node->ondisk->type == HAMMER_BTREE_TYPE_INTERNAL) {
		if (cursor.node.ondisk._type == HAMMER_BTREE_TYPE_INTERNAL) {
//			if (btree_node_is_full(cursor->node->ondisk) == 0)
			if (btree_node_is_full(cursor.node.ondisk) == 0) {
//				break;
				break
			}
//		} else {
		} else {
//			if (btree_node_is_full(cursor->node->ondisk) ==0)
			if (btree_node_is_full(cursor.node.ondisk) == 0) {
//				break;
				break
			}
//		}
		}
//		if (cursor->node->ondisk->parent == 0 ||
		if (cursor.node.ondisk.parent == 0 ||
//		    cursor->parent->ondisk->count != HAMMER_BTREE_INT_ELMS) {
				cursor.parent.ondisk.count != HAMMER_BTREE_INT_ELMS) {
//			break;
			break
//		}
		}
//		++hammer_stats_btree_iterations;
		hammer_stats_btree_iterations++
//		error = hammer_cursor_up(cursor);
		error = hammer_cursor_up(cursor)
//		/* node may have become stale */
		/* node may have become stale */
//		if (error)
		if error != 0 {
//			goto done;
			goto done
		}
//	}
	}
//

	/*
	 * Push down through internal nodes to locate the requested key.
	 */
//	node = cursor->node->ondisk;
	node = cursor.node.ondisk
//	while (node->type == HAMMER_BTREE_TYPE_INTERNAL) {
	for ; node._type == HAMMER_BTREE_TYPE_INTERNAL ; {
		/*
		 * Scan the node to find the subtree index to push down into.
		 * We go one-past, then back-up.
		 *
		 * We must proactively remove deleted elements which may
		 * have been left over from a deadlocked btree_remove().
		 *
		 * The left and right boundaries are included in the loop
		 * in order to detect edge cases.
		 *
		 * If the separator only differs by create_tid (r == 1)
		 * and we are doing an as-of search, we may end up going
		 * down a branch to the left of the one containing the
		 * desired key.  This requires numerous special cases.
		 */
//		++hammer_stats_btree_iterations;
		hammer_stats_btree_iterations++
//		if (hammer_debug_btree) {
		if hammer_debug_btree != 0 {
//			kprintf("SEARCH-I %016llx count=%d\n",
			kernel.Kprintf("SEARCH-I %016llx count=%d\n",
//				(long long)cursor->node->node_offset,
				int64(cursor.node.node_offset),
//				node->count);
				node.count)
//		}
		}
//

		/*
		 * Try to shortcut the search before dropping into the
		 * linear loop.  Locate the first node where r <= 1.
		 */
//		i = hammer_btree_search_node(&cursor->key_beg, node);
		i = hammer_btree_search_node(&cursor.key_beg, node)
//		while (i <= node->count) {
		for ; int64(i) <= int64(node.count); {
//			++hammer_stats_btree_elements;
			hammer_stats_btree_elements++
//			elm = &node->elms[i];
			elm = node.elms[i]
//			r = hammer_btree_cmp(&cursor->key_beg, &elm->base);
			r = hammer_btree_cmp(&cursor.key_beg, elm.base(0))
//			if (hammer_debug_btree > 2) {
			if hammer_debug_btree > 2 {
//				kprintf(" IELM %p %d r=%d\n",
				kernel.Kprintf(" IELM %p %d r=%d\n",
//					&node->elms[i], i, r);
					&node.elms[i], i, r)
//			}
			}
//			if (r < 0)
			if r < 0 {
//				break;
				break
			}
//			if (r == 1) {
			if r == 1 {
//				KKASSERT(elm->base.create_tid != 1);
				kernel.KKASSERT(elm.base(0).create_tid != 1)
//				cursor->create_check = elm->base.create_tid - 1;
				cursor.create_check = hammer_tid_t(elm.base(0).create_tid - 1)
//				cursor->flags |= HAMMER_CURSOR_CREATE_CHECK;
				cursor.flags |= HAMMER_CURSOR_CREATE_CHECK
//			}
			}
//			++i;
			i++
//		}
		}
//		if (hammer_debug_btree) {
		if hammer_debug_btree != 0 {
//			kprintf("SEARCH-I preI=%d/%d r=%d\n",
			kernel.Kprintf("SEARCH-I preI=%d/%d r=%d\n",
//				i, node->count, r);
				i, node.count, r)
//		}
		}
//

		/*
		 * These cases occur when the parent's idea of the boundary
		 * is wider then the child's idea of the boundary, and
		 * require special handling.  If not inserting we can
		 * terminate the search early for these cases but the
		 * child's boundaries cannot be unconditionally modified.
		 */
//		if (i == 0) {
		if i == 0 {
			/*
			 * If i == 0 the search terminated to the LEFT of the
			 * left_boundary but to the RIGHT of the parent's left
			 * boundary.
			 */
//			u_int8_t save;
			var save uint8
//

//			elm = &node->elms[0];
			elm = node.elms[0]
//

			/*
			 * If we aren't inserting we can stop here.
			 */
//			if ((flags & (HAMMER_CURSOR_INSERT |
			if ((flags & (HAMMER_CURSOR_INSERT |
//				      HAMMER_CURSOR_PRUNING)) == 0) {
					HAMMER_CURSOR_PRUNING)) == 0) {
//				cursor->index = 0;
				cursor.index = 0
//				return(ENOENT);
				return system.ENOENT
//			}
			}
//

			/*
			 * Correct a left-hand boundary mismatch.
			 *
			 * We can only do this if we can upgrade the lock,
			 * and synchronized as a background cursor (i.e.
			 * inserting or pruning).
			 *
			 * WARNING: We can only do this if inserting, i.e.
			 * we are running on the backend.
			 */
//			if ((error = hammer_cursor_upgrade(cursor)) != 0)
			error = hammer_cursor_upgrade(cursor)
			if error != 0 {
//				return(error);
				return error
			}
//			KKASSERT(cursor->flags & HAMMER_CURSOR_BACKEND);
			kernel.KKASSERT(cursor.flags & HAMMER_CURSOR_BACKEND)
//			hammer_modify_node_field(cursor->trans, cursor->node,
			hammer_modify_node_field(cursor.trans, cursor.node,
//						 elms[0]);
				"elms[0]")
//			save = node->elms[0].base.btype;
			save = node.elms[0].base(0).btype
//			node->elms[0].base = *cursor->left_bound;
			node.elms[0].write(cursor.left_bound)
//			node->elms[0].base.btype = save;
			temp_elm := node.elms[0].base(0)
			temp_elm.btype = save
			node.elms[0].write(temp_elm)
//			hammer_modify_node_done(cursor->node);
			hammer_modify_node_done(cursor.node)
//		} else if (i == node->count + 1) {
		} else if (int64(i) == int64(node.count) + 1) {
			/*
			 * If i == node.count + 1 the search terminated to
			 * the RIGHT of the right boundary but to the LEFT
			 * of the parent's right boundary.  If we aren't
			 * inserting we can stop here.
			 *
			 * Note that the last element in this case is
			 * elms[i-2] prior to adjustments to 'i'.
			 */
//			--i;
			i--
//			if ((flags & (HAMMER_CURSOR_INSERT |
			if ((flags & (HAMMER_CURSOR_INSERT |
//				      HAMMER_CURSOR_PRUNING)) == 0) {
					HAMMER_CURSOR_PRUNING)) == 0) {
//				cursor->index = i;
				cursor.index = i
//				return (ENOENT);
				return system.ENOENT
//			}
			}
//

			/*
			 * Correct a right-hand boundary mismatch.
			 * (actual push-down record is i-2 prior to
			 * adjustments to i).
			 *
			 * We can only do this if we can upgrade the lock,
			 * and synchronized as a background cursor (i.e.
			 * inserting or pruning).
			 *
			 * WARNING: We can only do this if inserting, i.e.
			 * we are running on the backend.
			 */
//			if ((error = hammer_cursor_upgrade(cursor)) != 0)
			error = hammer_cursor_upgrade(cursor)
			if (error != 0) {
//				return(error);
				return error
			}
//			elm = &node->elms[i];
			elm = node.elms[i]
//			KKASSERT(cursor->flags & HAMMER_CURSOR_BACKEND);
			kernel.KKASSERT(cursor.flags & HAMMER_CURSOR_BACKEND)
//			hammer_modify_node(cursor->trans, cursor->node,
			hammer_modify_node(cursor.trans,
//					   &elm->base, sizeof(elm->base));
				cursor.node, elm.base(0), int(unsafe.Sizeof(elm.base(0))))
//			elm->base = *cursor->right_bound;
			elm.write(cursor.right_bound)
//			hammer_modify_node_done(cursor->node);
			hammer_modify_node_done(cursor.node)
//			--i;
			i--
//		} else {
		} else {
			/*
			 * The push-down index is now i - 1.  If we had
			 * terminated on the right boundary this will point
			 * us at the last element.
			 */
//			--i;
			i--
//		}
		}
//		cursor->index = i;
		cursor.index = i
//		elm = &node->elms[i];
		elm = node.elms[i]
//

//		if (hammer_debug_btree) {
		if hammer_debug_btree != 0 {
//			kprintf("RESULT-I %016llx[%d] %016llx %02x "
//				"key=%016llx cre=%016llx lo=%02x\n",
			kernel.Kprintf("RESULT-I %016llx[%d] %016llx %02x key=%016llx cre=%016llx lo=%02x\n",
//				(long long)cursor->node->node_offset,
				int64(cursor.node.node_offset),
//				i,
				i,
//				(long long)elm->internal.base.obj_id,
				int64(elm.internal(0).base.obj_id),
//				elm->internal.base.rec_type,
				elm.internal(0).base.rec_type,
//				(long long)elm->internal.base.key,
				int64(elm.internal(0).base.key),
//				(long long)elm->internal.base.create_tid,
				int64(elm.internal(0).base.create_tid),
//				elm->internal.base.localization
//			);
				elm.internal(0).base.localization)
//		}
		}
//

		/*
		 * We better have a valid subtree offset.
		 */
//		KKASSERT(elm->internal.subtree_offset != 0);
		kernel.KKASSERT(elm.internal(0).subtree_offset != 0)
//

		/*
		 * Handle insertion and deletion requirements.
		 *
		 * If inserting split full nodes.  The split code will
		 * adjust cursor.node and cursor.index if the current
		 * index winds up in the new node.
		 *
		 * If inserting and a left or right edge case was detected,
		 * we cannot correct the left or right boundary and must
		 * prepend and append an empty leaf node in order to make
		 * the boundary correction.
		 *
		 * If we run out of space we set enospc and continue on
		 * to a leaf to provide the spike code with a good point
		 * of entry.
		 */
//		if ((flags & HAMMER_CURSOR_INSERT) && enospc == 0) {
		if (((flags & HAMMER_CURSOR_INSERT) != 0) && (enospc == 0)) {
//			if (btree_node_is_full(node)) {
			if (btree_node_is_full(node) != 0) {
//				error = btree_split_internal(cursor);
				error = btree_split_internal(cursor)
//				if (error) {
				if error != 0 {
//					if (error != ENOSPC)
					if error != system.ENOSPC {
//						goto done;
						goto done
					}
//					enospc = 1;
					enospc = 1
//				}
				}
				/*
				 * reload stale pointers
				 */
//				i = cursor->index;
				i = cursor.index
//				node = cursor->node->ondisk;
				node = cursor.node.ondisk
//			}
			}
//		}
		}
//

		/*
		 * Push down (push into new node, existing node becomes
		 * the parent) and continue the search.
		 */
//		error = hammer_cursor_down(cursor);
		error = hammer_cursor_down(cursor)
//		/* node may have become stale */
		/* node may have become stale */
//		if (error)
		if error != 0 {
//			goto done;
			goto done
		}
//		node = cursor->node->ondisk;
		node = cursor.node.ondisk
//	}
	}
//

	/*
	 * We are at a leaf, do a linear search of the key array.
	 *
	 * On success the index is set to the matching element and 0
	 * is returned.
	 *
	 * On failure the index is set to the insertion point and ENOENT
	 * is returned.
	 *
	 * Boundaries are not stored in leaf nodes, so the index can wind
	 * up to the left of element 0 (index == 0) or past the end of
	 * the array (index == node.count).  It is also possible that the
	 * leaf might be empty.
	 */
//	++hammer_stats_btree_iterations;
	hammer_stats_btree_iterations++
//	KKASSERT (node->type == HAMMER_BTREE_TYPE_LEAF);
	kernel.KKASSERT(node._type == HAMMER_BTREE_TYPE_LEAF)
//	KKASSERT(node->count <= HAMMER_BTREE_LEAF_ELMS);
	kernel.KKASSERT(node.count <= HAMMER_BTREE_LEAF_ELMS)
//	if (hammer_debug_btree) {
	if hammer_debug_btree != 0 {
//		kprintf("SEARCH-L %016llx count=%d\n",
		kernel.Kprintf("SEARCH-L %016llx count=%d\n",
//			(long long)cursor->node->node_offset,
			int64(cursor.node.node_offset),
//			node->count);
			node.count)
//	}
	}
//

	/*
	 * Try to shortcut the search before dropping into the
	 * linear loop.  Locate the first node where r <= 1.
	 */
//	i = hammer_btree_search_node(&cursor->key_beg, node);
	i = hammer_btree_search_node(&cursor.key_beg, node)
//	while (i < node->count) {
	for ; int64(i) < int64(node.count); {
//		++hammer_stats_btree_elements;
		hammer_stats_btree_elements++
//		elm = &node->elms[i];
		elm = node.elms[i]
//

//		r = hammer_btree_cmp(&cursor->key_beg, &elm->leaf.base);
		r = hammer_btree_cmp(&cursor.key_beg, &elm.leaf(0).base)
//

//		if (hammer_debug_btree > 1)
		if hammer_debug_btree > 1 {
//			kprintf("  ELM %p %d r=%d\n", &node->elms[i], i, r);
			kernel.Kprintf("  ELM %p %d r=%d\n", node.elms[i], i, r)
		}
//

		/*
		 * We are at a record element.  Stop if we've flipped past
		 * key_beg, not counting the create_tid test.  Allow the
		 * r == 1 case (key_beg > element but differs only by its
		 * create_tid) to fall through to the AS-OF check.
		 */
//		KKASSERT (elm->leaf.base.btype == HAMMER_BTREE_TYPE_RECORD);
		kernel.KKASSERT(elm.leaf(0).base.btype == HAMMER_BTREE_TYPE_RECORD)
//

//		if (r < 0)
		if r < 0 {
//			goto failed;
			goto failed
		}
//		if (r > 1) {
		if r > 1 {
//			++i;
			i++
//			continue;
			continue
//		}
		}
//

		/*
		 * Check our as-of timestamp against the element.
		 */
//		if (flags & HAMMER_CURSOR_ASOF) {
		if (flags & HAMMER_CURSOR_ASOF) > 0 {
//			if (hammer_btree_chkts(cursor->asof,
			node_elms_i_base_ptr := *node.elms[i].base(0)
			if hammer_btree_chkts(cursor.asof,
//					       &node->elms[i].base) != 0) {
					node_elms_i_base_ptr) != 0 {
//				++i;
				i++
//				continue;
				continue
//			}
			}
//			/* success */
			/* success */
//		} else {
		} else {
//			if (r > 0) {	/* can only be +1 */
			if (r > 0) {	/* can only be +1 */
//				++i;
				i++
//				continue;
				continue
//			}
			}
//			/* success */
			/* success */
//		}
		}
//		cursor->index = i;
		cursor.index = i
//		error = 0;
		error = 0
//		if (hammer_debug_btree) {
		if hammer_debug_btree != 0 {
//			kprintf("RESULT-L %016llx[%d] (SUCCESS)\n",
			kernel.Kprintf("RESULT-L %016llx[%d] (SUCCESS)\n",
//				(long long)cursor->node->node_offset, i);
				int64(cursor.node.node_offset), i)
//		}
		}
//		goto done;
		goto done
//	}
	}
//

	/*
	 * The search of the leaf node failed.  i is the insertion point.
	 */
//failed:
failed:
//	if (hammer_debug_btree) {
	if hammer_debug_btree != 0 {
//		kprintf("RESULT-L %016llx[%d] (FAILED)\n",
		kernel.Kprintf("RESULT-L %016llx[%d] (FAILED)\n",
//			(long long)cursor->node->node_offset, i);
			int64(cursor.node.node_offset), i);
//	}
	}
//

	/*
	 * No exact match was found, i is now at the insertion point.
	 *
	 * If inserting split a full leaf before returning.  This
	 * may have the side effect of adjusting cursor.node and
	 * cursor.index.
	 */
//	cursor->index = i;
	cursor.index = i
//	if ((flags & HAMMER_CURSOR_INSERT) && enospc == 0 &&
	if (((flags & HAMMER_CURSOR_INSERT) != 0) && (enospc == 0) &&
//	     btree_node_is_full(node)) {
			(btree_node_is_full(node) != 0)) {
//		error = btree_split_leaf(cursor);
		error = btree_split_leaf(cursor)
//		if (error) {
		if error != 0 {
//			if (error != ENOSPC)
			if error != system.ENOSPC {
//				goto done;
				goto done
			}
//			enospc = 1;
			enospc = 1
//		}
		}
		/*
		 * reload stale pointers
		 */
		/* NOT USED
		i = cursor.index
		node = cursor.node.internal
		*/
//	}
	}
//

	/*
	 * We reached a leaf but did not find the key we were looking for.
	 * If this is an insert we will be properly positioned for an insert
	 * (ENOENT) or spike (ENOSPC) operation.
	 */
//	error = enospc ? ENOSPC : ENOENT;
	if enospc != 0 {
		error = system.ENOSPC
	} else {
		error = system.ENOENT
	}
//done:
done:
//	return(error);
	return error 
//}
}
//

/*
 * Heuristical search for the first element whos comparison is <= 1.  May
 * return an index whos compare result is > 1 but may only return an index
 * whos compare result is <= 1 if it is the first element with that result.
 */
//int
//hammer_btree_search_node(hammer_base_elm_t elm, hammer_node_ondisk_t node)
//{
func hammer_btree_search_node(elm hammer_base_elm_t, node hammer_node_ondisk_t) int {
//	int b;
	var b int
//	int s;
	var s int
//	int i;
	var i int
//	int r;
	var r int
//

	/*
	 * Don't bother if the node does not have very many elements
	 */
//	b = 0;
	b = 0
//	s = node->count;
	s = int(node.count)
//	while (s - b > 4) {
	for ; s - b > 4; {
//		i = b + (s - b) / 2;
		i = b + (s - b) / 2
//		++hammer_stats_btree_elements;
		hammer_stats_btree_elements++
//		r = hammer_btree_cmp(elm, &node->elms[i].leaf.base);
		r = hammer_btree_cmp(elm, &node.elms[i].leaf(0).base)
//		if (r <= 1) {
		if (r <= 1) {
//			s = i;
			s = i
//		} else {
		} else {
//			b = i;
			b = i
//		}
		}
//	}
	}
//	return(b);
	return b
//}
}
//

//

/************************************************************************
 *			   SPLITTING AND MERGING 			*
 ************************************************************************
 *
 * These routines do all the dirty work required to split and merge nodes.
 */
//

/*
 * Split an internal node into two nodes and move the separator at the split
 * point to the parent.
 *
 * (cursor.node, cursor.index) indicates the element the caller intends
 * to push into.  We will adjust node and index if that element winds
 * up in the split node.
 *
 * If we are at the root of the filesystem a new root must be created with
 * two elements, one pointing to the original root and one pointing to the
 * newly allocated split node.
 */
//static
//int
//btree_split_internal(hammer_cursor_t cursor)
//{
func btree_split_internal(cursor hammer_cursor_t) int {
//	hammer_node_ondisk_t ondisk;
	var ondisk hammer_node_ondisk_t
//	hammer_node_t node;
	var node hammer_node_t
//	hammer_node_t parent;
	var parent hammer_node_t
//	hammer_node_t new_node;
	var new_node hammer_node_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_btree_elm_t parent_elm;
	var parent_elm hammer_btree_elm_t
//	struct hammer_node_lock lockroot;
	var lockroot hammer_node_lock
//	hammer_mount_t hmp = cursor->trans->hmp;
	hmp := cursor.trans.hmp
//	int parent_index;
	var parent_index int
//	int made_root;
	var made_root int
//	int split;
	var split int
//	int error;
	var error int
//	int i;
	var i int
//	const int esize = sizeof(*elm);
	var esize int
	esize = int(unsafe.Sizeof(elm))
	var elmAsInternal hammer_btree_internal_elm
	var empty_hammer_node_t hammer_node_t
//

//	hammer_node_lock_init(&lockroot, cursor->node);
	hammer_node_lock_init(&lockroot, cursor.node);
//	error = hammer_btree_lock_children(cursor, 1, &lockroot, NULL);
	error = hammer_btree_lock_children(cursor, 1, &lockroot, nil)
//	if (error)
	if error != 0 {
//		goto done;
		goto done
	}
//	if ((error = hammer_cursor_upgrade(cursor)) != 0)
	if error = hammer_cursor_upgrade(cursor); error != 0 {
//		goto done;
		goto done
	}
//	++hammer_stats_btree_splits;
	hammer_stats_btree_splits++
//

	/* 
	 * Calculate the split point.  If the insertion point is at the
	 * end of the leaf we adjust the split point significantly to the
	 * right to try to optimize node fill and flag it.  If we hit
	 * that same leaf again our heuristic failed and we don't try
	 * to optimize node fill (it could lead to a degenerate case).
	 */
//	node = cursor->node;
	node = cursor.node
//	ondisk = node->ondisk;
	ondisk = node.ondisk
//	KKASSERT(ondisk->count > 4);
	kernel.KKASSERT(ondisk.count > 4)
//	if (cursor->index == ondisk->count &&
	if (int64(cursor.index) == int64(ondisk.count) &&
//	    (node->flags & HAMMER_NODE_NONLINEAR) == 0) {
			(node.flags & HAMMER_NODE_NONLINEAR) == 0) {
//		split = (ondisk->count + 1) * 3 / 4;
		split = int((ondisk.count + 1) * 3 / 4)
//		node->flags |= HAMMER_NODE_NONLINEAR;
		node.flags |= HAMMER_NODE_NONLINEAR
//	} else {
	} else {
		/*
		 * We are splitting but elms[split] will be promoted to
		 * the parent, leaving the right hand node with one less
		 * element.  If the insertion point will be on the
		 * left-hand side adjust the split point to give the
		 * right hand side one additional node.
		 */
//		split = (ondisk->count + 1) / 2;
		split = int((ondisk.count + 1) / 2)
//		if (cursor->index <= split)
		if (cursor.index <= split) {
//			--split;
			split--
		}
//	}
	}
//

	/*
	 * If we are at the root of the filesystem, create a new root node
	 * with 1 element and split normally.  Avoid making major
	 * modifications until we know the whole operation will work.
	 */
//	if (ondisk->parent == 0) {
	if ondisk.parent == 0 {
//		parent = hammer_alloc_btree(cursor->trans, 0, &error);
		parent = hammer_alloc_btree(cursor.trans, 0, &error)
//		if (parent == NULL)
		if (parent == nil) {
//			goto done;
			goto done
		}
//		hammer_lock_ex(&parent->lock);
		hammer_lock_ex(&parent.lock)
//		hammer_modify_node_noundo(cursor->trans, parent);
		hammer_modify_node_noundo(cursor.trans, parent)
//		ondisk = parent->ondisk;
		ondisk = parent.ondisk
//		ondisk->count = 1;
		ondisk.count = 1
//		ondisk->parent = 0;
		ondisk.parent = 0
//		ondisk->mirror_tid = node->ondisk->mirror_tid;
		ondisk.mirror_tid = node.ondisk.mirror_tid
//		ondisk->type = HAMMER_BTREE_TYPE_INTERNAL;
		ondisk._type = HAMMER_BTREE_TYPE_INTERNAL
		ondisk.elms[0].write(hmp.root_btree_beg)
//		ondisk->elms[0].base = hmp->root_btree_beg;
		elmAsBase := ondisk.elms[0].base(0)
		elmAsBase.btype = node.ondisk._type
		ondisk.elms[0].write(elmAsBase)
//		ondisk->elms[0].base.btype = node->ondisk->type;
		elmAsInternal := ondisk.elms[0].internal(0)
		elmAsInternal.subtree_offset = node.node_offset
//		ondisk->elms[0].internal.subtree_offset = node->node_offset;
		ondisk.elms[0].write(elmAsInternal)
//		ondisk->elms[1].base = hmp->root_btree_end;
		ondisk.elms[1].write(hmp.root_btree_end)
//		hammer_modify_node_done(parent);
		hammer_modify_node_done(parent)
//		/* ondisk->elms[1].base.btype - not used */
		/* ondisk.elms[1].base.btype - not used */
//		made_root = 1;
		made_root = 1
//		parent_index = 0;	/* index of current node in parent */
		parent_index = 0	/* index of current node in parent */
//	} else {
	} else {
//		made_root = 0;
		made_root = 0
//		parent = cursor->parent;
		parent = cursor.parent
//		parent_index = cursor->parent_index;
		parent_index = cursor.parent_index
//	}
	}
//

	/*
	 * Split node into new_node at the split point.
	 *
	 *  B O O O P N N B	<-- P = node.elms[split] (index 4)
	 *   0 1 2 3 4 5 6	<-- subtree indices
	 *
	 *       x x P x x
	 *        s S S s  
	 *         /   \
	 *  B O O O B    B N N B	<--- inner boundary points are 'P'
	 *   0 1 2 3      4 5 6  
	 */
//	new_node = hammer_alloc_btree(cursor->trans, 0, &error);
	new_node = hammer_alloc_btree(cursor.trans, 0, &error)
//	if (new_node == NULL) {
	if new_node == nil {
//		if (made_root) {
		if made_root != 0 {
//			hammer_unlock(&parent->lock);
			hammer_unlock(&parent.lock)
//			hammer_delete_node(cursor->trans, parent);
			hammer_delete_node(cursor.trans, parent)
//			hammer_rel_node(parent);
			hammer_rel_node(parent)
//		}
		}
//		goto done;
		goto done
//	}
	}
//	hammer_lock_ex(&new_node->lock);
	hammer_lock_ex(&new_node.lock)
//

	/*
	 * Create the new node.  P becomes the left-hand boundary in the
	 * new node.  Copy the right-hand boundary as well.
	 *
	 * elm is the new separator.
	 */
//	hammer_modify_node_noundo(cursor->trans, new_node);
	hammer_modify_node_noundo(cursor.trans, new_node)
//	hammer_modify_node_all(cursor->trans, node);
	hammer_modify_node_all(cursor.trans, node)
//	ondisk = node->ondisk;
	ondisk = node.ondisk
//	elm = &ondisk->elms[split];
	elm = ondisk.elms[split]
//	bcopy(elm, &new_node->ondisk->elms[0],
	library.Bcopy(elm, &new_node.ondisk.elms[0],
//	      (ondisk->count - split + 1) * esize);
	      int((int64(ondisk.count) - int64(split) + 1) * int64(esize)))
//	new_node->ondisk->count = ondisk->count - split;
	new_node.ondisk.count = int32(int64(ondisk.count) - int64(split))
//	new_node->ondisk->parent = parent->node_offset;
	new_node.ondisk.parent = parent.node_offset
//	new_node->ondisk->type = HAMMER_BTREE_TYPE_INTERNAL;
	new_node.ondisk._type = HAMMER_BTREE_TYPE_INTERNAL
//	new_node->ondisk->mirror_tid = ondisk->mirror_tid;
	new_node.ondisk.mirror_tid = ondisk.mirror_tid
//	KKASSERT(ondisk->type == new_node->ondisk->type);
	kernel.KKASSERT(ondisk._type == new_node.ondisk._type)
//	hammer_cursor_split_node(node, new_node, split);
	hammer_cursor_split_node(node, new_node, split)
//

	/*
	 * Cleanup the original node.  Elm (P) becomes the new boundary,
	 * its subtree_offset was moved to the new node.  If we had created
	 * a new root its parent pointer may have changed.
	 */
	elmAsInternal = *elm.internal(0)
//	elm->internal.subtree_offset = 0;
	elmAsInternal.subtree_offset = 0
//	ondisk->count = split;
	ondisk.count = int32(split)
//

	/*
	 * Insert the separator into the parent, fixup the parent's
	 * reference to the original node, and reference the new node.
	 * The separator is P.
	 *
	 * Remember that base.count does not include the right-hand boundary.
	 */
//	hammer_modify_node_all(cursor->trans, parent);
	hammer_modify_node_all(cursor.trans, parent)
//	ondisk = parent->ondisk;
	ondisk = parent.ondisk
//	KKASSERT(ondisk->count != HAMMER_BTREE_INT_ELMS);
	kernel.KKASSERT(ondisk.count != HAMMER_BTREE_INT_ELMS)
//	parent_elm = &ondisk->elms[parent_index+1];
	parent_elm = &ondisk.elms[parent_index+1]
//	bcopy(parent_elm, parent_elm + 1,
	library.Bcopy(parent_elm, parent_elm/* + 1*/,
//	      (ondisk->count - parent_index) * esize);
		int((int64(ondisk.count) - int64(parent_index)) * int64(esize)))
//	parent_elm->internal.base = elm->base;	/* separator P */
	elmAsInternal = *(*parent_elm).internal(0)
	elmAsInternal.base = *elm.base(0)	/* separator P */
//	parent_elm->internal.base.btype = new_node->ondisk->type;
	elmAsInternal.base.btype = new_node.ondisk._type
//	parent_elm->internal.subtree_offset = new_node->node_offset;
	elmAsInternal.subtree_offset = new_node.node_offset
//	parent_elm->internal.mirror_tid = new_node->ondisk->mirror_tid;
	elmAsInternal.mirror_tid = new_node.ondisk.mirror_tid
	(*parent_elm).write(elmAsInternal)
//	++ondisk->count;
	ondisk.count++
//	hammer_modify_node_done(parent);
	hammer_modify_node_done(parent)
//	hammer_cursor_inserted_element(parent, parent_index + 1);
	hammer_cursor_inserted_element(parent, parent_index + 1)
//

	/*
	 * The children of new_node need their parent pointer set to new_node.
	 * The children have already been locked by
	 * hammer_btree_lock_children().
	 */
//	for (i = 0; i < new_node->ondisk->count; ++i) {
	for i = 0; int64(i) < int64(new_node.ondisk.count); i++ {
//		elm = &new_node->ondisk->elms[i];
		elm = new_node.ondisk.elms[i]
//		error = btree_set_parent(cursor->trans, new_node, elm);
		error = btree_set_parent(cursor.trans, new_node, &elm)
//		if (error) {
		if error != 0 {
//			panic("btree_split_internal: btree-fixup problem");
			panic("btree_split_internal: btree-fixup problem")
//		}
		}
//	}
	}
//	hammer_modify_node_done(new_node);
	hammer_modify_node_done(new_node)
//

	/*
	 * The filesystem's root B-Tree pointer may have to be updated.
	 */
//	if (made_root) {
	if made_root != 0 {
//		hammer_volume_t volume;
		var volume hammer_volume_t
//

//		volume = hammer_get_root_volume(hmp, &error);
		volume = hammer_get_root_volume(hmp, &error)
//		KKASSERT(error == 0);
		kernel.KKASSERT(error == 0)
//

//		hammer_modify_volume_field(cursor->trans, volume,
		hammer_modify_volume_field(cursor.trans, volume,
//					   vol0_btree_root);
			"vol0_btree_root")
//		volume->ondisk->vol0_btree_root = parent->node_offset;
		volume.ondisk.vol0_btree_root = parent.node_offset
//		hammer_modify_volume_done(volume);
		hammer_modify_volume_done(volume)
//		node->ondisk->parent = parent->node_offset;
		node.ondisk.parent = parent.node_offset
//		if (cursor->parent) {
		if cursor.parent != empty_hammer_node_t {
//			hammer_unlock(&cursor->parent->lock);
			hammer_unlock(&cursor.parent.lock)
//			hammer_rel_node(cursor->parent);
			hammer_rel_node(cursor.parent)
//		}
		}
//		cursor->parent = parent;	/* lock'd and ref'd */
		cursor.parent = parent	/* lock'd and ref'd */
//		hammer_rel_volume(volume, 0);
		hammer_rel_volume(volume, 0)
//	}
	}
//	hammer_modify_node_done(node);
	hammer_modify_node_done(node)
//

	/*
	 * Ok, now adjust the cursor depending on which element the original
	 * index was pointing at.  If we are >= the split point the push node
	 * is now in the new node.
	 *
	 * NOTE: If we are at the split point itself we cannot stay with the
	 * original node because the push index will point at the right-hand
	 * boundary, which is illegal.
	 *
	 * NOTE: The cursor's parent or parent_index must be adjusted for
	 * the case where a new parent (new root) was created, and the case
	 * where the cursor is now pointing at the split node.
	 */
//	if (cursor->index >= split) {
	if (cursor.index >= split) {
//		cursor->parent_index = parent_index + 1;
		cursor.parent_index = parent_index + 1
//		cursor->index -= split;
		cursor.index -= split
//		hammer_unlock(&cursor->node->lock);
		hammer_unlock(&cursor.node.lock)
//		hammer_rel_node(cursor->node);
		hammer_rel_node(cursor.node)
//		cursor->node = new_node;	/* locked and ref'd */
		cursor.node = new_node;	/* locked and ref'd */
//	} else {
	} else {
//		cursor->parent_index = parent_index;
		cursor.parent_index = parent_index
//		hammer_unlock(&new_node->lock);
		hammer_unlock(&new_node.lock)
//		hammer_rel_node(new_node);
		hammer_rel_node(new_node)
//	}
	}
//

	/*
	 * Fixup left and right bounds
	 */
//	parent_elm = &parent->ondisk->elms[cursor->parent_index];
	parent_elm = &parent.ondisk.elms[cursor.parent_index]
//	cursor->left_bound = &parent_elm[0].internal.base;
	cursor.left_bound = &(*parent_elm).internal(0).base
//	cursor->right_bound = &parent_elm[1].internal.base;
	cursor.right_bound = &(*parent_elm).internal(1).base
//	KKASSERT(hammer_btree_cmp(cursor->left_bound,
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound,
//		 &cursor->node->ondisk->elms[0].internal.base) <= 0);
		 &cursor.node.ondisk.elms[0].internal(0).base) <= 0)
//	KKASSERT(hammer_btree_cmp(cursor->right_bound,
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound,
//		 &cursor->node->ondisk->elms[cursor->node->ondisk->count].internal.base) >= 0);
		 &cursor.node.ondisk.elms[cursor.node.ondisk.count].internal(0).base) >= 0)
//

//done:
done:
//	hammer_btree_unlock_children(cursor->trans->hmp, &lockroot, NULL);
	hammer_btree_unlock_children(cursor.trans.hmp, &lockroot, nil)
//	hammer_cursor_downgrade(cursor);
	hammer_cursor_downgrade(cursor)
//	return (error);
	return error
//}
}
//

/*
 * Same as the above, but splits a full leaf node.
 *
 * This function
 */
//static
//int
//btree_split_leaf(hammer_cursor_t cursor)
//{
func btree_split_leaf(cursor hammer_cursor_t) int {
//	hammer_node_ondisk_t ondisk;
	var ondisk hammer_node_ondisk_t
//	hammer_node_t parent;
	var parent hammer_node_t
//	hammer_node_t leaf;
	var leaf hammer_node_t
//	hammer_mount_t hmp;
	var hmp hammer_mount_t
//	hammer_node_t new_leaf;
	var new_leaf hammer_node_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_btree_elm_t parent_elm;
	var parent_elm hammer_btree_elm
//	hammer_base_elm_t mid_boundary;
	var mid_boundary hammer_base_elm
//	int parent_index;
	var parent_index int
//	int made_root;
	var made_root int
//	int split;
	var split int
//	int error;
	var error int
//	const size_t esize = sizeof(*elm);
	var esize uint = uint(unsafe.Sizeof(elm))
	var elmAsInternal hammer_btree_internal_elm
	var empty_hammer_node_t hammer_node_t
	var elm_base_ptr hammer_base_elm_t
//

	error = hammer_cursor_upgrade(cursor)
//	if ((error = hammer_cursor_upgrade(cursor)) != 0)
	if error != 0 {
//		return(error);
		return error
	}
//	++hammer_stats_btree_splits;
	hammer_stats_btree_splits++
//

//	KKASSERT(hammer_btree_cmp(cursor->left_bound,
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound,
//		 &cursor->node->ondisk->elms[0].leaf.base) <= 0);
		&cursor.node.ondisk.elms[0].leaf(0).base) <= 0);
//	KKASSERT(hammer_btree_cmp(cursor->right_bound,
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound,
//		 &cursor->node->ondisk->elms[cursor->node->ondisk->count-1].leaf.base) > 0);
		&cursor.node.ondisk.elms[cursor.node.ondisk.count-1].leaf(0).base) > 0);
//

	/* 
	 * Calculate the split point.  If the insertion point is at the
	 * end of the leaf we adjust the split point significantly to the
	 * right to try to optimize node fill and flag it.  If we hit
	 * that same leaf again our heuristic failed and we don't try
	 * to optimize node fill (it could lead to a degenerate case).
	 *
	 * Spikes are made up of two leaf elements which cannot be
	 * safely split.
	 */
//	leaf = cursor->node;
	leaf = cursor.node
//	ondisk = leaf->ondisk;
	ondisk = leaf.ondisk
//	KKASSERT(ondisk->count > 4);
	kernel.KKASSERT(ondisk.count > 4)
//	if (cursor->index == ondisk->count &&
	if (int64(cursor.index) == int64(ondisk.count) &&
//	    (leaf->flags & HAMMER_NODE_NONLINEAR) == 0) {
			(leaf.flags & HAMMER_NODE_NONLINEAR) == 0) {
//		split = (ondisk->count + 1) * 3 / 4;
		split = int((ondisk.count + 1) * 3 / 4)
//		leaf->flags |= HAMMER_NODE_NONLINEAR;
		leaf.flags |= HAMMER_NODE_NONLINEAR
//	} else {
	} else {
//		split = (ondisk->count + 1) / 2;
		split = int((ondisk.count + 1) / 2)
//	}
	}
//

/*#if 0
	/*
	 * If the insertion point is at the split point shift the
	 * split point left so we don't have to worry about
	 * /
	if (cursor.index == split)
		split--
#endif*/
//	KKASSERT(split > 0 && split < ondisk->count);
	kernel.KKASSERT(split > 0 && int64(split) < int64(ondisk.count))
//

//	error = 0;
	error = 0
//	hmp = leaf->hmp;
	hmp = leaf.hmp
//

//	elm = &ondisk->elms[split];
	elm = ondisk.elms[split]
//

//	KKASSERT(hammer_btree_cmp(cursor->left_bound, &elm[-1].leaf.base) <= 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound, &elm.leaf(-1).base) <= 0)
//	KKASSERT(hammer_btree_cmp(cursor->left_bound, &elm->leaf.base) <= 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound, &elm.leaf(0).base) <= 0)
//	KKASSERT(hammer_btree_cmp(cursor->right_bound, &elm->leaf.base) > 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound, &elm.leaf(0).base) > 0)
//	KKASSERT(hammer_btree_cmp(cursor->right_bound, &elm[1].leaf.base) > 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound, &elm.leaf(1).base) > 0)
//

	/*
	 * If we are at the root of the tree, create a new root node with
	 * 1 element and split normally.  Avoid making major modifications
	 * until we know the whole operation will work.
	 */
//	if (ondisk->parent == 0) {
	if (ondisk.parent == 0) {
//		parent = hammer_alloc_btree(cursor->trans, 0, &error);
		parent = hammer_alloc_btree(cursor.trans, 0, &error)
//		if (parent == NULL)
		if (parent == nil) {
//			goto done;
			goto done
		}
//		hammer_lock_ex(&parent->lock);
		hammer_lock_ex(&parent.lock)
//		hammer_modify_node_noundo(cursor->trans, parent);
		hammer_modify_node_noundo(cursor.trans, parent)
//		ondisk = parent->ondisk;
		ondisk = parent.ondisk
//		ondisk->count = 1;
		ondisk.count = 1
//		ondisk->parent = 0;
		ondisk.parent = 0
//		ondisk->mirror_tid = leaf->ondisk->mirror_tid;
		ondisk.mirror_tid = leaf.ondisk.mirror_tid
//		ondisk->type = HAMMER_BTREE_TYPE_INTERNAL;
		ondisk._type = HAMMER_BTREE_TYPE_INTERNAL
		ondisk.elms[0].write(hmp.root_btree_beg)
		elmAsBase := ondisk.elms[0].base(0)
//		ondisk->elms[0].base = hmp->root_btree_beg;
		elmAsBase.btype = leaf.ondisk._type
//		ondisk->elms[0].base.btype = leaf->ondisk->type;
		ondisk.elms[0].write(elmAsBase)
//		ondisk->elms[0].internal.subtree_offset = leaf->node_offset;
		elmAsInternal = *ondisk.elms[0].internal(0)
		elmAsInternal.subtree_offset = leaf.node_offset
//		ondisk->elms[1].base = hmp->root_btree_end;
		elmAsBase = ondisk.elms[1].base(1)
		ondisk.elms[1].write(hmp.root_btree_end)
//		/* ondisk->elms[1].base.btype = not used */
		/* ondisk.elms[1].base.btype = not used */
//		hammer_modify_node_done(parent);
		hammer_modify_node_done(parent)
//		made_root = 1;
		made_root = 1
//		parent_index = 0;	/* insertion point in parent */
		parent_index = 0	/* insertion point in parent */
//	} else {
	} else {
//		made_root = 0;
		made_root = 0
//		parent = cursor->parent;
		parent = cursor.parent
//		parent_index = cursor->parent_index;
		parent_index = cursor.parent_index
//	}
	}
//

	/*
	 * Split leaf into new_leaf at the split point.  Select a separator
	 * value in-between the two leafs but with a bent towards the right
	 * leaf since comparisons use an 'elm >= separator' inequality.
	 *
	 *  L L L L L L L L
	 *
	 *       x x P x x
	 *        s S S s  
	 *         /   \
	 *  L L L L     L L L L
	 */
//	new_leaf = hammer_alloc_btree(cursor->trans, 0, &error);
	new_leaf = hammer_alloc_btree(cursor.trans, 0, &error)
//	if (new_leaf == NULL) {
	if (new_leaf == nil) {
//		if (made_root) {
		if made_root != 0 {
//			hammer_unlock(&parent->lock);
			hammer_unlock(&parent.lock)
//			hammer_delete_node(cursor->trans, parent);
			hammer_delete_node(cursor.trans, parent)
//			hammer_rel_node(parent);
			hammer_rel_node(parent)
//		}
		}
//		goto done;
		goto done
//	}
	}
//	hammer_lock_ex(&new_leaf->lock);
	hammer_lock_ex(&new_leaf.lock)
//

	/*
	 * Create the new node and copy the leaf elements from the split 
	 * point on to the new node.
	 */
//	hammer_modify_node_all(cursor->trans, leaf);
	hammer_modify_node_all(cursor.trans, leaf)
//	hammer_modify_node_noundo(cursor->trans, new_leaf);
	hammer_modify_node_noundo(cursor.trans, new_leaf)
//	ondisk = leaf->ondisk;
	ondisk = leaf.ondisk
//	elm = &ondisk->elms[split];
	elm = ondisk.elms[split]
//	bcopy(elm, &new_leaf->ondisk->elms[0], (ondisk->count - split) * esize);
	library.Bcopy(elm, &new_leaf.ondisk.elms[0], int((int64(ondisk.count) - int64(split)) * int64(esize)))
//	new_leaf->ondisk->count = ondisk->count - split;
	new_leaf.ondisk.count = int32(int64(ondisk.count) - int64(split))
//	new_leaf->ondisk->parent = parent->node_offset;
	new_leaf.ondisk.parent = parent.node_offset
//	new_leaf->ondisk->type = HAMMER_BTREE_TYPE_LEAF;
	new_leaf.ondisk._type = HAMMER_BTREE_TYPE_LEAF
//	new_leaf->ondisk->mirror_tid = ondisk->mirror_tid;
	new_leaf.ondisk.mirror_tid = ondisk.mirror_tid
//	KKASSERT(ondisk->type == new_leaf->ondisk->type);
	kernel.KKASSERT(ondisk._type == new_leaf.ondisk._type)
//	hammer_modify_node_done(new_leaf);
	hammer_modify_node_done(new_leaf)
//	hammer_cursor_split_node(leaf, new_leaf, split);
	hammer_cursor_split_node(leaf, new_leaf, int(split))
//

	/*
	 * Cleanup the original node.  Because this is a leaf node and
	 * leaf nodes do not have a right-hand boundary, there
	 * aren't any special edge cases to clean up.  We just fixup the
	 * count.
	 */
//	ondisk->count = split;
	ondisk.count = int32(split)
//

	/*
	 * Insert the separator into the parent, fixup the parent's
	 * reference to the original node, and reference the new node.
	 * The separator is P.
	 *
	 * Remember that base.count does not include the right-hand boundary.
	 * We are copying parent_index+1 to parent_index+2, not +0 to +1.
	 */
//	hammer_modify_node_all(cursor->trans, parent);
	hammer_modify_node_all(cursor.trans, parent)
//	ondisk = parent->ondisk;
	ondisk = parent.ondisk
//	KKASSERT(split != 0);
	kernel.KKASSERT(split != 0)
//	KKASSERT(ondisk->count != HAMMER_BTREE_INT_ELMS);
	kernel.KKASSERT(ondisk.count != HAMMER_BTREE_INT_ELMS)
//	parent_elm = &ondisk->elms[parent_index+1];
	parent_elm = ondisk.elms[parent_index+1]
//	bcopy(parent_elm, parent_elm + 1,
	library.Bcopy(parent_elm, parent_elm/* + 1*/,
//	      (ondisk->count - parent_index) * esize);
	      int((int64(ondisk.count) - int64(parent_index)) * int64(esize)))
//

//	hammer_make_separator(&elm[-1].base, &elm[0].base, &parent_elm->base);
	elm_base_ptr = hammer_base_elm_t(elm.base(-1))
	hammer_make_separator(elm_base_ptr, *elm.base(0), *parent_elm.base(0))
//	parent_elm->internal.base.btype = new_leaf->ondisk->type;
	elmAsInternal = *parent_elm.internal(0)
	elmAsInternal.base.btype = new_leaf.ondisk._type
//	parent_elm->internal.subtree_offset = new_leaf->node_offset;
	elmAsInternal.subtree_offset = new_leaf.node_offset
//	parent_elm->internal.mirror_tid = new_leaf->ondisk->mirror_tid;
	elmAsInternal.mirror_tid = new_leaf.ondisk.mirror_tid
//	mid_boundary = &parent_elm->base;
	mid_boundary = *parent_elm.base(0)
//	++ondisk->count;
	ondisk.count++
//	hammer_modify_node_done(parent);
	hammer_modify_node_done(parent)
//	hammer_cursor_inserted_element(parent, parent_index + 1);
	hammer_cursor_inserted_element(parent, parent_index + 1)
//

	/*
	 * The filesystem's root B-Tree pointer may have to be updated.
	 */
//	if (made_root) {
	if made_root != 0 {
//		hammer_volume_t volume;
		var volume hammer_volume_t
//

//		volume = hammer_get_root_volume(hmp, &error);
		volume = hammer_get_root_volume(hmp, &error)
//		KKASSERT(error == 0);
		kernel.KKASSERT(error == 0)
//

//		hammer_modify_volume_field(cursor->trans, volume,
		hammer_modify_volume_field(cursor.trans, volume,
//					   vol0_btree_root);
			"vol0_btree_root")
//		volume->ondisk->vol0_btree_root = parent->node_offset;
		volume.ondisk.vol0_btree_root = parent.node_offset
//		hammer_modify_volume_done(volume);
		hammer_modify_volume_done(volume)
//		leaf->ondisk->parent = parent->node_offset;
		leaf.ondisk.parent = parent.node_offset
//		if (cursor->parent) {
		if cursor.parent != empty_hammer_node_t {
//			hammer_unlock(&cursor->parent->lock);
			hammer_unlock(&cursor.parent.lock)
//			hammer_rel_node(cursor->parent);
			hammer_rel_node(cursor.parent)
//		}
		}
//		cursor->parent = parent;	/* lock'd and ref'd */
		cursor.parent = parent	/* lock'd and ref'd */
//		hammer_rel_volume(volume, 0);
		hammer_rel_volume(volume, 0)
//	}
	}
//	hammer_modify_node_done(leaf);
	hammer_modify_node_done(leaf)
//

	/*
	 * Ok, now adjust the cursor depending on which element the original
	 * index was pointing at.  If we are >= the split point the push node
	 * is now in the new node.
	 *
	 * NOTE: If we are at the split point itself we need to select the
	 * old or new node based on where key_beg's insertion point will be.
	 * If we pick the wrong side the inserted element will wind up in
	 * the wrong leaf node and outside that node's bounds.
	 */
//	if (cursor->index > split ||
	if (cursor.index > split ||
//	    (cursor->index == split &&
			(cursor.index == split &&
//	     hammer_btree_cmp(&cursor->key_beg, mid_boundary) >= 0)) {
			hammer_btree_cmp(&cursor.key_beg, &mid_boundary) >= 0)) {
//		cursor->parent_index = parent_index + 1;
		cursor.parent_index = parent_index + 1
//		cursor->index -= split;
		cursor.index -= split
//		hammer_unlock(&cursor->node->lock);
		hammer_unlock(&cursor.node.lock)
//		hammer_rel_node(cursor->node);
		hammer_rel_node(cursor.node)
//		cursor->node = new_leaf;
		cursor.node = new_leaf
//	} else {
	} else {
//		cursor->parent_index = parent_index;
		cursor.parent_index = parent_index
//		hammer_unlock(&new_leaf->lock);
		hammer_unlock(&new_leaf.lock)
//		hammer_rel_node(new_leaf);
		hammer_rel_node(new_leaf)
//	}
	}
//

	/*
	 * Fixup left and right bounds
	 */
//	parent_elm = &parent->ondisk->elms[cursor->parent_index];
	elm = parent.ondisk.elms[cursor.parent_index]
//	cursor->left_bound = &parent_elm[0].internal.base;
	cursor.left_bound = &elm.internal(0).base
//	cursor->right_bound = &parent_elm[1].internal.base;
	cursor.right_bound = &elm.internal(1).base
//

	/*
	 * Assert that the bounds are correct.
	 */
//	KKASSERT(hammer_btree_cmp(cursor->left_bound,
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound,
//		 &cursor->node->ondisk->elms[0].leaf.base) <= 0);
		 &cursor.node.ondisk.elms[0].leaf(0).base) <= 0)
//	KKASSERT(hammer_btree_cmp(cursor->right_bound,
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound,
//		 &cursor->node->ondisk->elms[cursor->node->ondisk->count-1].leaf.base) > 0);
		 &cursor.node.ondisk.elms[cursor.node.ondisk.count-1].leaf(0).base) > 0)
//	KKASSERT(hammer_btree_cmp(cursor->left_bound, &cursor->key_beg) <= 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.left_bound, &cursor.key_beg) <= 0)
//	KKASSERT(hammer_btree_cmp(cursor->right_bound, &cursor->key_beg) > 0);
	kernel.KKASSERT(hammer_btree_cmp(cursor.right_bound, &cursor.key_beg) > 0)
//

//done:
done:
//	hammer_cursor_downgrade(cursor);
	hammer_cursor_downgrade(cursor)
//	return (error);
	return error
//}
}
//

/*#if 0

/*
 * Recursively correct the right-hand boundary's create_tid to (tid) as
 * long as the rest of the key matches.  We have to recurse upward in
 * the tree as well as down the left side of each parent's right node.
 *
 * Return EDEADLK if we were only partially successful, forcing the caller
 * to try again.  The original cursor is not modified.  This routine can
 * also fail with EDEADLK if it is forced to throw away a portion of its
 * record history.
 *
 * The caller must pass a downgraded cursor to us (otherwise we can't dup it).
 * /
struct hammer_rhb {
	TAILQ_ENTRY(hammer_rhb) entry;
	hammer_node_t	node;
	int		index;
};

TAILQ_HEAD(hammer_rhb_list, hammer_rhb);

int
hammer_btree_correct_rhb(cursor hammer_cursor_t, hammer_tid_t tid)
{
	struct hammer_mount *hmp;
	struct hammer_rhb_list rhb_list;
	hammer_base_elm_t elm;
	hammer_node_t orig_node;
	struct hammer_rhb *rhb;
	int orig_index;
	var error int

	TAILQ_INIT(&rhb_list);
	hmp = cursor.trans.hmp;

	/*
	 * Save our position so we can restore it on return.  This also
	 * gives us a stable 'elm'.
	 * /
	orig_node = cursor.node;
	hammer_ref_node(orig_node);
	hammer_lock_sh(&orig_node.lock);
	orig_index = cursor.index;
	elm = &orig_node.ondisk.elms[orig_index].base;

	/*
	 * Now build a list of parents going up, allocating a rhb
	 * structure for each one.
	 * /
	while (cursor.parent) {
		/*
		 * Stop if we no longer have any right-bounds to fix up
		 * /
		if (elm.obj_id != cursor.right_bound.obj_id ||
		    elm.rec_type != cursor.right_bound.rec_type ||
		    elm.key != cursor.right_bound.key) {
			break;
		}

		/*
		 * Stop if the right-hand bound's create_tid does not
		 * need to be corrected.
		 * /
		if (cursor.right_bound.create_tid >= tid)
			break;

		rhb = kmalloc(sizeof(*rhb), hmp.m_misc, M_WAITOK|M_ZERO);
		rhb.node = cursor.parent;
		rhb.index = cursor.parent_index;
		hammer_ref_node(rhb.node);
		hammer_lock_sh(&rhb.node.lock);
		TAILQ_INSERT_HEAD(&rhb_list, rhb, entry);

		hammer_cursor_up(cursor);
	}

	/*
	 * now safely adjust the right hand bound for each rhb.  This may
	 * also require taking the right side of the tree and iterating down
	 * ITS left side.
	 * /
	error = 0;
	while (error == 0 && (rhb = TAILQ_FIRST(&rhb_list)) != NULL) {
		error = hammer_cursor_seek(cursor, rhb.node, rhb.index);
		if error != 0 {
			break;
		TAILQ_REMOVE(&rhb_list, rhb, entry);
		hammer_unlock(&rhb.node.lock);
		hammer_rel_node(rhb.node);
		kfree(rhb, hmp.m_misc);

		switch (cursor.node.ondisk._type) {
		case HAMMER_BTREE_TYPE_INTERNAL:
			/*
			 * Right-boundary for parent at internal node
			 * is one element to the right of the element whos
			 * right boundary needs adjusting.  We must then
			 * traverse down the left side correcting any left
			 * bounds (which may now be too far to the left).
			 * /
			cursor.index++
			error = hammer_btree_correct_lhb(cursor, tid);
			break;
		default:
			panic("hammer_btree_correct_rhb(): Bad node type");
			error = EINVAL;
			break;
		}
	}

	/*
	 * Cleanup
	 * /
	while ((rhb = TAILQ_FIRST(&rhb_list)) != NULL) {
		TAILQ_REMOVE(&rhb_list, rhb, entry);
		hammer_unlock(&rhb.node.lock);
		hammer_rel_node(rhb.node);
		kfree(rhb, hmp.m_misc);
	}
	error = hammer_cursor_seek(cursor, orig_node, orig_index);
	hammer_unlock(&orig_node.lock);
	hammer_rel_node(orig_node);
	return error
}

/*
 * Similar to rhb (in fact, rhb calls lhb), but corrects the left hand
 * bound going downward starting at the current cursor position.
 *
 * This function does not restore the cursor after use.
 * /
int
hammer_btree_correct_lhb(cursor hammer_cursor_t, hammer_tid_t tid)
{
	struct hammer_rhb_list rhb_list;
	hammer_base_elm_t elm;
	hammer_base_elm_t cmp;
	struct hammer_rhb *rhb;
	struct hammer_mount *hmp;
	var error int

	TAILQ_INIT(&rhb_list);
	hmp = cursor.trans.hmp;

	cmp = cursor.node.ondisk.elms[cursor.index].base;

	/*
	 * Record the node and traverse down the left-hand side for all
	 * matching records needing a boundary correction.
	 * /
	error = 0;
	for (;;) {
		rhb = kmalloc(sizeof(*rhb), hmp.m_misc, M_WAITOK|M_ZERO);
		rhb.node = cursor.node;
		rhb.index = cursor.index;
		hammer_ref_node(rhb.node);
		hammer_lock_sh(&rhb.node.lock);
		TAILQ_INSERT_HEAD(&rhb_list, rhb, entry);

		if (cursor.node.ondisk._type == HAMMER_BTREE_TYPE_INTERNAL) {
			/*
			 * Nothing to traverse down if we are at the right
			 * boundary of an internal node.
			 * /
			if (cursor.index == cursor.node.ondisk.count)
				break;
		} else {
			elm = cursor.node.ondisk.elms[cursor.index].base;
			if (elm.btype == HAMMER_BTREE_TYPE_RECORD)
				break;
			panic("Illegal leaf record type %02x", elm.btype);
		}
		error = hammer_cursor_down(cursor);
		if error != 0 {
			break;

		elm = cursor.node.ondisk.elms[cursor.index].base;
		if (elm.obj_id != cmp.obj_id ||
		    elm.rec_type != cmp.rec_type ||
		    elm.key != cmp.key) {
			break;
		}
		if (elm.create_tid >= tid)
			break;

	}

	/*
	 * Now we can safely adjust the left-hand boundary from the bottom-up.
	 * The last element we remove from the list is the caller's right hand
	 * boundary, which must also be adjusted.
	 * /
	while (error == 0 && (rhb = TAILQ_FIRST(&rhb_list)) != NULL) {
		error = hammer_cursor_seek(cursor, rhb.node, rhb.index);
		if error != 0 {
			break;
		TAILQ_REMOVE(&rhb_list, rhb, entry);
		hammer_unlock(&rhb.node.lock);
		hammer_rel_node(rhb.node);
		kfree(rhb, hmp.m_misc);

		elm = cursor.node.ondisk.elms[cursor.index].base;
		if (cursor.node.ondisk._type == HAMMER_BTREE_TYPE_INTERNAL) {
			hammer_modify_node(cursor.trans, cursor.node,
					   &elm.create_tid,
					   sizeof(elm.create_tid));
			elm.create_tid = tid;
			hammer_modify_node_done(cursor.node);
		} else {
			panic("hammer_btree_correct_lhb(): Bad element type");
		}
	}

	/*
	 * Cleanup
	 * /
	while ((rhb = TAILQ_FIRST(&rhb_list)) != NULL) {
		TAILQ_REMOVE(&rhb_list, rhb, entry);
		hammer_unlock(&rhb.node.lock);
		hammer_rel_node(rhb.node);
		kfree(rhb, hmp.m_misc);
	}
	return error
}

#endif*/
//

/*
 * Attempt to remove the locked, empty or want-to-be-empty B-Tree node at
 * (cursor.node).  Returns 0 on success, EDEADLK if we could not complete
 * the operation due to a deadlock, or some other error.
 *
 * This routine is initially called with an empty leaf and may be
 * recursively called with single-element internal nodes.
 *
 * It should also be noted that when removing empty leaves we must be sure
 * to test and update mirror_tid because another thread may have deadlocked
 * against us (or someone) trying to propagate it up and cannot retry once
 * the node has been deleted.
 *
 * On return the cursor may end up pointing to an internal node, suitable
 * for further iteration but not for an immediate insertion or deletion.
 */
//static int
//btree_remove(hammer_cursor_t cursor)
//{
func btree_remove(cursor hammer_cursor_t) int {
//	hammer_node_ondisk_t ondisk;
	var ondisk hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_node_t node;
	var node hammer_node_t
//	hammer_node_t parent;
	var parent hammer_node_t
//	const int esize = sizeof(*elm);
	esize := unsafe.Sizeof(elm)
//	int error;
	var error int
	var elmAsInternal hammer_btree_internal_elm
//

//	node = cursor->node;
	node = cursor.node
//

	/*
	 * When deleting the root of the filesystem convert it to
	 * an empty leaf node.  Internal nodes cannot be empty.
	 */
//	ondisk = node->ondisk;
	ondisk = node.ondisk
//	if (ondisk->parent == 0) {
	if (ondisk.parent == 0) {
//		KKASSERT(cursor->parent == NULL);
		kernel.KKASSERT(cursor.parent == nil)
//		hammer_modify_node_all(cursor->trans, node);
		hammer_modify_node_all(cursor.trans, node)
//		KKASSERT(ondisk == node->ondisk);
		kernel.KKASSERT(ondisk == node.ondisk)
//		ondisk->type = HAMMER_BTREE_TYPE_LEAF;
		ondisk._type = HAMMER_BTREE_TYPE_LEAF
//		ondisk->count = 0;
		ondisk.count = 0
//		hammer_modify_node_done(node);
		hammer_modify_node_done(node)
//		cursor->index = 0;
		cursor.index = 0
//		return(0);
		return 0
//	}
	}
//

//	parent = cursor->parent;
	parent = cursor.parent
//

	/*
	 * Attempt to remove the parent's reference to the child.  If the
	 * parent would become empty we have to recurse.  If we fail we 
	 * leave the parent pointing to an empty leaf node.
	 *
	 * We have to recurse successfully before we can delete the internal
	 * node as it is illegal to have empty internal nodes.  Even though
	 * the operation may be aborted we must still fixup any unlocked
	 * cursors as if we had deleted the element prior to recursing
	 * (by calling hammer_cursor_deleted_element()) so those cursors
	 * are properly forced up the chain by the recursion.
	 */
//	if (parent->ondisk->count == 1) {
	if parent.ondisk.count == 1 {
		/*
		 * This special cursor_up_locked() call leaves the original
		 * node exclusively locked and referenced, leaves the
		 * original parent locked (as the new node), and locks the
		 * new parent.  It can return EDEADLK.
		 *
		 * We cannot call hammer_cursor_removed_node() until we are
		 * actually able to remove the node.  If we did then tracked
		 * cursors in the middle of iterations could be repointed
		 * to a parent node.  If this occurs they could end up
		 * scanning newly inserted records into the node (that could
		 * not be deleted) when they push down again.
		 *
		 * Due to the way the recursion works the final parent is left
		 * in cursor.parent after the recursion returns.  Each
		 * layer on the way back up is thus able to call
		 * hammer_cursor_removed_node() and 'jump' the node up to
		 * the (same) final parent.
		 *
		 * NOTE!  The local variable 'parent' is invalid after we
		 *	  call hammer_cursor_up_locked().
		 */
//		error = hammer_cursor_up_locked(cursor);
		error = hammer_cursor_up_locked(cursor)
//		parent = NULL;
		parent = nil
//

//		if (error == 0) {
		if (error == 0) {
//			hammer_cursor_deleted_element(cursor->node, 0);
			hammer_cursor_deleted_element(cursor.node, 0)
//			error = btree_remove(cursor);
			error = btree_remove(cursor)
//			if (error == 0) {
			if (error == 0) {
//				KKASSERT(node != cursor->node);
				kernel.KKASSERT(node != cursor.node)
//				hammer_cursor_removed_node(
				hammer_cursor_removed_node(
//					node, cursor->node,
					node, cursor.node,
//					cursor->index);
					cursor.index)
//				hammer_modify_node_all(cursor->trans, node);
				hammer_modify_node_all(cursor.trans, node)
//				ondisk = node->ondisk;
				ondisk = node.ondisk
//				ondisk->type = HAMMER_BTREE_TYPE_DELETED;
				ondisk._type = HAMMER_BTREE_TYPE_DELETED
//				ondisk->count = 0;
				ondisk.count = 0
//				hammer_modify_node_done(node);
				hammer_modify_node_done(node)
//				hammer_flush_node(node, 0);
				hammer_flush_node(node, 0)
//				hammer_delete_node(cursor->trans, node);
				hammer_delete_node(cursor.trans, node)
//			} else {
			} else {
				/*
				 * Defer parent removal because we could not
				 * get the lock, just let the leaf remain
				 * empty.
				 */
				/**/
//			}
			}
//			hammer_unlock(&node->lock);
			hammer_unlock(&node.lock)
//			hammer_rel_node(node);
			hammer_rel_node(node)
//		} else {
		} else {
			/*
			 * Defer parent removal because we could not
			 * get the lock, just let the leaf remain
			 * empty.
			 */
			/**/
//		}
		}
//	} else {
	} else {
//		KKASSERT(parent->ondisk->count > 1);
		kernel.KKASSERT(parent.ondisk.count > 1)
//

//		hammer_modify_node_all(cursor->trans, parent);
		hammer_modify_node_all(cursor.trans, parent)
//		ondisk = parent->ondisk;
		ondisk = parent.ondisk
//		KKASSERT(ondisk->type == HAMMER_BTREE_TYPE_INTERNAL);
		kernel.KKASSERT(ondisk._type == HAMMER_BTREE_TYPE_INTERNAL)
//

//		elm = &ondisk->elms[cursor->parent_index];
		elm = ondisk.elms[cursor.parent_index]
//		KKASSERT(elm->internal.subtree_offset == node->node_offset);
		kernel.KKASSERT(elm.internal(0).subtree_offset == node.node_offset)
//		KKASSERT(ondisk->count > 0);
		kernel.KKASSERT(ondisk.count > 0)
//

		/*
		 * We must retain the highest mirror_tid.  The deleted
		 * range is now encompassed by the element to the left.
		 * If we are already at the left edge the new left edge
		 * inherits mirror_tid.
		 *
		 * Note that bounds of the parent to our parent may create
		 * a gap to the left of our left-most node or to the right
		 * of our right-most node.  The gap is silently included
		 * in the mirror_tid's area of effect from the point of view
		 * of the scan.
		 */
//		if (cursor->parent_index) {
		if cursor.parent_index != 0 {
//			if (elm[-1].internal.mirror_tid <
			if (elm.internal(-1).mirror_tid <
//			    elm[0].internal.mirror_tid) {
					elm.internal(0).mirror_tid) {
//				elm[-1].internal.mirror_tid =
				elmAsInternal = *elm.internal(-1)
//				    elm[0].internal.mirror_tid;
				elmAsInternal.mirror_tid = elm.internal(0).mirror_tid
				elm.write(elmAsInternal)
//			}
			}
//		} else {
		} else {
//			if (elm[1].internal.mirror_tid <
			if (elm.internal(1).mirror_tid <
//			    elm[0].internal.mirror_tid) {
					elm.internal(0).mirror_tid) {
//				elm[1].internal.mirror_tid =
				elmAsInternal = *elm.internal(1)
//				    elm[0].internal.mirror_tid;
				elmAsInternal.mirror_tid = elm.internal(0).mirror_tid
				elm.write(elmAsInternal)
//			}
			}
//		}
		}
//

		/*
		 * Delete the subtree reference in the parent.  Include
		 * boundary element at end.
		 */
//		bcopy(&elm[1], &elm[0],
		library.Bcopy(&elm[1], &elm[0],
//		      (ondisk->count - cursor->parent_index) * esize);
		      int((int64(ondisk.count) - int64(cursor.parent_index)) * int64(esize)))
//		--ondisk->count;
		ondisk.count--
//		hammer_modify_node_done(parent);
		hammer_modify_node_done(parent)
//		hammer_cursor_removed_node(node, parent, cursor->parent_index);
		hammer_cursor_removed_node(node, parent, cursor.parent_index)
//		hammer_cursor_deleted_element(parent, cursor->parent_index);
		hammer_cursor_deleted_element(parent, cursor.parent_index)
//		hammer_flush_node(node, 0);
		hammer_flush_node(node, 0)
//		hammer_delete_node(cursor->trans, node);
		hammer_delete_node(cursor.trans, node)
//

		/*
		 * cursor.node is invalid, cursor up to make the cursor
		 * valid again.  We have to flag the condition in case
		 * another thread wiggles an insertion in during an
		 * iteration.
		 */
//		cursor->flags |= HAMMER_CURSOR_ITERATE_CHECK;
		cursor.flags |= HAMMER_CURSOR_ITERATE_CHECK;
//		error = hammer_cursor_up(cursor);
		error = hammer_cursor_up(cursor);
//	}
	}
//	return (error);
	return error
//}
}
//

/*
 * Propagate cursor.trans.tid up the B-Tree starting at the current
 * cursor position using pseudofs info gleaned from the passed inode.
 *
 * The passed inode has no relationship to the cursor position other
 * then being in the same pseudofs as the insertion or deletion we
 * are propagating the mirror_tid for.
 *
 * WARNING!  Because we push and pop the passed cursor, it may be
 *	     modified by other B-Tree operations while it is unlocked
 *	     and things like the node & leaf pointers, and indexes might
 *	     change.
 */
//void
//hammer_btree_do_propagation(hammer_cursor_t cursor,
func hammer_btree_do_propagation(cursor hammer_cursor_t,
//			    hammer_pseudofs_inmem_t pfsm,
		pfsm hammer_pseudofs_inmem_t,
//			    hammer_btree_leaf_elm_t leaf)
//{
		leaf hammer_btree_leaf_elm_t) {
//	hammer_cursor_t ncursor;
	var ncursor hammer_cursor_t
//	hammer_tid_t mirror_tid;
	var mirror_tid hammer_tid_t
//	int error __debugvar;
	var error int
//

	/*
	 * We do not propagate a mirror_tid if the filesystem was mounted
	 * in no-mirror mode.
	 */
//	if (cursor->trans->hmp->master_id < 0)
	if (cursor.trans.hmp.master_id < 0) {
//		return;
		return
	}
//

	/*
	 * This is a bit of a hack because we cannot deadlock or return
	 * EDEADLK here.  The related operation has already completed and
	 * we must propagate the mirror_tid now regardless.
	 *
	 * Generate a new cursor which inherits the original's locks and
	 * unlock the original.  Use the new cursor to propagate the
	 * mirror_tid.  Then clean up the new cursor and reacquire locks
	 * on the original.
	 *
	 * hammer_dup_cursor() cannot dup locks.  The dup inherits the
	 * original's locks and the original is tracked and must be
	 * re-locked.
	 */
//	mirror_tid = cursor->node->ondisk->mirror_tid;
	mirror_tid = cursor.node.ondisk.mirror_tid
//	KKASSERT(mirror_tid != 0);
	kernel.KKASSERT(mirror_tid != 0)
//	ncursor = hammer_push_cursor(cursor);
	ncursor = hammer_push_cursor(cursor)
//	error = hammer_btree_mirror_propagate(ncursor, mirror_tid);
	error = hammer_btree_mirror_propagate(ncursor, mirror_tid)
//	KKASSERT(error == 0);
	kernel.KKASSERT(error == 0)
//	hammer_pop_cursor(cursor, ncursor);
	hammer_pop_cursor(cursor, ncursor)
//	/* WARNING: cursor's leaf pointer may change after pop */
	/* WARNING: cursor's leaf pointer may change after pop */
//}
}
//

//

/*
 * Propagate a mirror TID update upwards through the B-Tree to the root.
 *
 * A locked internal node must be passed in.  The node will remain locked
 * on return.
 *
 * This function syncs mirror_tid at the specified internal node's element,
 * adjusts the node's aggregation mirror_tid, and then recurses upwards.
 */
//static int
//hammer_btree_mirror_propagate(hammer_cursor_t cursor, hammer_tid_t mirror_tid)
//{
func hammer_btree_mirror_propagate(cursor hammer_cursor_t, mirror_tid hammer_tid_t) int {
//	hammer_btree_internal_elm_t elm;
	var elm hammer_btree_internal_elm
//	hammer_node_t node;
	var node hammer_node_t
//	int error;
	var error int
//

//	for (;;) {
	for {
//		error = hammer_cursor_up(cursor);
		error = hammer_cursor_up(cursor)
//		if (error == 0)
		if (error == 0) {
//			error = hammer_cursor_upgrade(cursor);
			error = hammer_cursor_upgrade(cursor)
		}
//

		/*
		 * We can ignore HAMMER_CURSOR_ITERATE_CHECK, the
		 * cursor will still be properly positioned for
		 * mirror propagation, just not for iterations.
		 */
//		while (error == EDEADLK) {
		for ; (error == system.EDEADLK); {
//			hammer_recover_cursor(cursor);
			hammer_recover_cursor(cursor)
//			error = hammer_cursor_upgrade(cursor);
			error = hammer_cursor_upgrade(cursor)
//		}
		}
//		if (error)
		if error != 0 {
//			break;
			break
		}
//

		/*
		 * If the cursor deadlocked it could end up at a leaf
		 * after we lost the lock.
		 */
//		node = cursor->node;
		node = cursor.node
//		if (node->ondisk->type != HAMMER_BTREE_TYPE_INTERNAL)
		if (node.ondisk._type != HAMMER_BTREE_TYPE_INTERNAL) {
//			continue;
			continue
		}
//

		/*
		 * Adjust the node's element
		 */
//		elm = &node->ondisk->elms[cursor->index].internal;
		elm = *node.ondisk.elms[cursor.index].internal(0)
//		if (elm->mirror_tid >= mirror_tid)
		if (elm.mirror_tid >= mirror_tid) {
//			break;
			break
		}
//		hammer_modify_node(cursor->trans, node, &elm->mirror_tid,
		hammer_modify_node(cursor.trans, node, &elm.mirror_tid,
//				   sizeof(elm->mirror_tid));
			int(unsafe.Sizeof(elm.mirror_tid)))
//		elm->mirror_tid = mirror_tid;
		elm.mirror_tid = mirror_tid
//		hammer_modify_node_done(node);
		hammer_modify_node_done(node)
//		if (hammer_debug_general & 0x0002) {
		if (hammer_debug_general & 0x0002) != 0 {
//			kprintf("mirror_propagate: propagate "
//				"%016llx @%016llx:%d\n",
			kernel.Kprintf("mirror_propagate: propagate %016llx @%016llx:%d\n",
//				(long long)mirror_tid,
				int64(mirror_tid),
//				(long long)node->node_offset,
				int64(node.node_offset),
//				cursor->index);
				cursor.index)
//		}
		}
//

//

		/*
		 * Adjust the node's mirror_tid aggregator
		 */
//		if (node->ondisk->mirror_tid >= mirror_tid)
		if (node.ondisk.mirror_tid >= mirror_tid) {
//			return(0);
			return 0
		}
//		hammer_modify_node_field(cursor->trans, node, mirror_tid);
		hammer_modify_node_field(cursor.trans, node, mirror_tid)
//		node->ondisk->mirror_tid = mirror_tid;
		node.ondisk.mirror_tid = mirror_tid
//		hammer_modify_node_done(node);
		hammer_modify_node_done(node)
//		if (hammer_debug_general & 0x0002) {
		if (hammer_debug_general & 0x0002) != 0 {
//			kprintf("mirror_propagate: propagate "
//				"%016llx @%016llx\n",
			kernel.Kprintf("mirror_propagate: propagate %016llx @%016llx\n",
//				(long long)mirror_tid,
				int64(mirror_tid),
//				(long long)node->node_offset);
				int64(node.node_offset))
//		}
		}
//	}
	}
//	if (error == ENOENT)
	if error == system.ENOENT {
//		error = 0;
		error = 0;
	}
//	return(error);
	return error
//}
}
//

//hammer_node_t
//hammer_btree_get_parent(hammer_transaction_t trans, hammer_node_t node,
func hammer_btree_get_parent(trans hammer_transaction_t, node hammer_node_t,
//			int *parent_indexp, int *errorp, int try_exclusive)
//{
			parent_indexp *int, errorp *int, try_exclusive int) hammer_node_t {
//	hammer_node_t parent;
	var parent hammer_node_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	int i;
	var i int
//

	/*
	 * Get the node
	 */
//	parent = hammer_get_node(trans, node->ondisk->parent, 0, errorp);
	parent = hammer_get_node(trans, node.ondisk.parent, 0, errorp)
//	if (*errorp) {
	if *errorp != 0 {
//		KKASSERT(parent == NULL);
		kernel.KKASSERT(parent == nil)
//		return(NULL);
		return nil
//	}
	}
//	KKASSERT ((parent->flags & HAMMER_NODE_DELETED) == 0);
	kernel.KKASSERT ((parent.flags & HAMMER_NODE_DELETED) == 0)
//

	/*
	 * Lock the node
	 */
//	if (try_exclusive) {
	if try_exclusive != 0 {
//		if (hammer_lock_ex_try(&parent->lock)) {
		if hammer_lock_ex_try(&parent.lock) != 0 {
//			hammer_rel_node(parent);
			hammer_rel_node(parent)
//			*errorp = EDEADLK;
			*errorp = system.EDEADLK
//			return(NULL);
			return nil
//		}
		}
//	} else {
	} else {
//		hammer_lock_sh(&parent->lock);
		hammer_lock_sh(&parent.lock)
//	}
	}
//

	/*
	 * Figure out which element in the parent is pointing to the
	 * child.
	 */
//	if (node->ondisk->count) {
	if node.ondisk.count != 0 {
//		i = hammer_btree_search_node(&node->ondisk->elms[0].base,
		i = hammer_btree_search_node(node.ondisk.elms[0].base(0),
//					     parent->ondisk);
						parent.ondisk)
//	} else {
	} else {
//		i = 0;
		i = 0
//	}
	}
//	while (i < parent->ondisk->count) {
	for ; int64(i) < int64(parent.ondisk.count); {
//		elm = &parent->ondisk->elms[i];
		elm = parent.ondisk.elms[i]
//		if (elm->internal.subtree_offset == node->node_offset)
		if (elm.internal(0).subtree_offset == node.node_offset) {
//			break;
			break
		}
//		++i;
		i++
//	}
	}
//	if (i == parent->ondisk->count) {
	if (int64(i) == int64(parent.ondisk.count)) {
//		hammer_unlock(&parent->lock);
		hammer_unlock(&parent.lock)
//		panic("Bad B-Tree link: parent %p node %p", parent, node);
		panic(fmt.Sprintf("Bad B-Tree link: parent %p node %p", parent, node))
//	}
	}
//	*parent_indexp = i;
	*parent_indexp = i
//	KKASSERT(*errorp == 0);
	kernel.KKASSERT(*errorp == 0)
//	return(parent);
	return parent
//}
}
//

/*
 * The element (elm) has been moved to a new internal node (node).
 *
 * If the element represents a pointer to an internal node that node's
 * parent must be adjusted to the element's new location.
 *
 * XXX deadlock potential here with our exclusive locks
 */
//int
//btree_set_parent(hammer_transaction_t trans, hammer_node_t node,
func btree_set_parent(trans hammer_transaction_t, node hammer_node_t,
//		 hammer_btree_elm_t elm)
//{
		elm *hammer_btree_elm) int {
//	hammer_node_t child;
	var child hammer_node_t
//	int error;
	var error int
//

//	error = 0;
	error = 0
//

//	switch(elm->base.btype) {
	switch(elm.base(0).btype) {
//	case HAMMER_BTREE_TYPE_INTERNAL:
	case HAMMER_BTREE_TYPE_INTERNAL:
//	case HAMMER_BTREE_TYPE_LEAF:
	case HAMMER_BTREE_TYPE_LEAF:
//		child = hammer_get_node(trans, elm->internal.subtree_offset,
		child = hammer_get_node(trans, elm.internal(0).subtree_offset,
//					0, &error);
					0, &error)
//		if (error == 0) {
		if error == 0 {
//			hammer_modify_node_field(trans, child, parent);
			hammer_modify_node_field(trans, child, "parent")
//			child->ondisk->parent = node->node_offset;
			child.ondisk.parent = node.node_offset
//			hammer_modify_node_done(child);
			hammer_modify_node_done(child)
//			hammer_rel_node(child);
			hammer_rel_node(child)
//		}
		}
//		break;
		break
//	default:
	default:
//		break;
		break
//	}
	}
//	return(error);
	return error
//}
}
//

/*
 * Initialize the root of a recursive B-Tree node lock list structure.
 */
//void
//hammer_node_lock_init(hammer_node_lock_t parent, hammer_node_t node)
//{
func hammer_node_lock_init(parent hammer_node_lock_t, node hammer_node_t) {
//	TAILQ_INIT(&parent->list);
	system.TAILQ_INIT(&parent.list)
//	parent->parent = NULL;
	parent.parent = nil
//	parent->node = node;
	parent.node = node
//	parent->index = -1;
	parent.index = -1
//	parent->count = node->ondisk->count;
	parent.count = int(node.ondisk.count)
//	parent->copy = NULL;
	parent.copy = nil
//	parent->flags = 0;
	parent.flags = 0
//}
}
//

/*
 * Initialize a cache of hammer_node_lock's including space allocated
 * for node copies.
 *
 * This is used by the rebalancing code to preallocate the copy space
 * for ~4096 B-Tree nodes (16MB of data) prior to acquiring any HAMMER
 * locks, otherwise we can blow out the pageout daemon's emergency
 * reserve and deadlock it.
 *
 * NOTE: HAMMER_NODE_LOCK_LCACHE is not set on items cached in the lcache.
 *	 The flag is set when the item is pulled off the cache for use.
 */
//void
//hammer_btree_lcache_init(hammer_mount_t hmp, hammer_node_lock_t lcache,
//			 int depth)
//{
func hammer_btree_lcache_init(hmp hammer_mount_t, lcache hammer_node_lock_t, depth int) {
//	hammer_node_lock_t item;
	var item hammer_node_lock_t
//	int count;
	var count int
//

//	for (count = 1; depth; --depth)
	for count = 1; depth != 0; depth-- {
//		count *= HAMMER_BTREE_LEAF_ELMS;
		count *= HAMMER_BTREE_LEAF_ELMS
	}
//	bzero(lcache, sizeof(*lcache));
	library.Bzero(lcache, unsafe.Sizeof(lcache))
//	TAILQ_INIT(&lcache->list);
	system.TAILQ_INIT(&lcache.list)
//	while (count) {
	for ; count != 0; {
//		item = kmalloc(sizeof(*item), hmp->m_misc, M_WAITOK|M_ZERO);
		item = new(hammer_node_lock)
//		item->copy = kmalloc(sizeof(*item->copy),
//				     hmp->m_misc, M_WAITOK);
		item.copy = new(hammer_node_ondisk)

//		TAILQ_INIT(&item->list);
		system.TAILQ_INIT(&item.list)
//		TAILQ_INSERT_TAIL(&lcache->list, item, entry);
		system.TAILQ_INSERT_TAIL(&lcache.list, item, "entry")
//		--count;
		count--
//	}
	}
//}
}
//

//void
//hammer_btree_lcache_free(hammer_mount_t hmp, hammer_node_lock_t lcache)
//{
func hammer_btree_lcache_free(hmp hammer_mount_t, lcache hammer_node_lock_t) {
//	hammer_node_lock_t item;
	var item hammer_node_lock_t
	var empty_hammer_node_lock_t hammer_node_lock_t
//

//	while ((item = TAILQ_FIRST(&lcache->list)) != NULL) {
	for ;item != empty_hammer_node_lock_t /*NULL*/; {
//		TAILQ_REMOVE(&lcache->list, item, entry);
		system.TAILQ_REMOVE(&lcache.list, item, "entry")
//		KKASSERT(item->copy);
		kernel.KKASSERT(item.copy)
//		KKASSERT(TAILQ_EMPTY(&item->list));
		kernel.KKASSERT(system.TAILQ_EMPTY(&item.list))
//		kfree(item->copy, hmp->m_misc);
		item.copy = nil
		hmp.m_misc = nil
//		kfree(item, hmp->m_misc);
		item = nil
		hmp.m_misc = nil
//	}
	}
//	KKASSERT(lcache->copy == NULL);
	kernel.KKASSERT(lcache.copy == nil)
//}
}
//

/*
 * Exclusively lock all the children of node.  This is used by the split
 * code to prevent anyone from accessing the children of a cursor node
 * while we fix-up its parent offset.
 *
 * If we don't lock the children we can really mess up cursors which block
 * trying to cursor-up into our node.
 *
 * On failure EDEADLK (or some other error) is returned.  If a deadlock
 * error is returned the cursor is adjusted to block on termination.
 *
 * The caller is responsible for managing parent.node, the root's node
 * is usually aliased from a cursor.
 */
//int
//hammer_btree_lock_children(hammer_cursor_t cursor, int depth,
//			   hammer_node_lock_t parent,
//			   hammer_node_lock_t lcache)
//{
func hammer_btree_lock_children(cursor hammer_cursor_t, depth int, parent hammer_node_lock_t, lcache hammer_node_lock_t) int {
//	hammer_node_t node;
	var node hammer_node_t
//	hammer_node_lock_t item;
	var item hammer_node_lock_t
//	hammer_node_ondisk_t ondisk;
	var ondisk hammer_node_ondisk_t
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	hammer_node_t child;
	var child hammer_node_t
//	struct hammer_mount *hmp;
	var hmp *hammer_mount
//	int error;
	var error int
//	int i;
	var i int
	var empty_hammer_node_t hammer_node_t
	var empty_hammer_node_lock_t hammer_node_lock_t
//

//	node = parent->node;
	node = parent.node
//	ondisk = node->ondisk;
	ondisk = node.ondisk
//	error = 0;
	error = 0
//	hmp = cursor->trans->hmp;
	hmp = cursor.trans.hmp
//

	/*
	 * We really do not want to block on I/O with exclusive locks held,
	 * pre-get the children before trying to lock the mess.  This is
	 * only done one-level deep for now.
	 */
//	for (i = 0; i < ondisk->count; ++i) {
	for i = 0; int64(i) < int64(ondisk.count); i++ {
//		++hammer_stats_btree_elements;
		hammer_stats_btree_elements++
//		elm = &ondisk->elms[i];
		elm = ondisk.elms[i]
//		if (elm->base.btype != HAMMER_BTREE_TYPE_LEAF &&
		if (elm.base(0).btype != HAMMER_BTREE_TYPE_LEAF &&
//		    elm->base.btype != HAMMER_BTREE_TYPE_INTERNAL) {
		    elm.base(0).btype != HAMMER_BTREE_TYPE_INTERNAL) {
//			continue;
			continue
//		}
		}
//		child = hammer_get_node(cursor->trans,
		child = hammer_get_node(cursor.trans,
//					elm->internal.subtree_offset,
			elm.internal(0).subtree_offset,
//					0, &error);
			0, &error)		
//		if (child)
		if child != empty_hammer_node_t {
//			hammer_rel_node(child);
			hammer_rel_node(child)
		}
//	}
	}
//

	/*
	 * Do it for real
	 */
//	for (i = 0; error == 0 && i < ondisk->count; ++i) {
	for i = 0; error == 0 && int64(i) < int64(ondisk.count); i++ {
//		++hammer_stats_btree_elements;
		hammer_stats_btree_elements++
//		elm = &ondisk->elms[i];
		elm = ondisk.elms[i]
//

//		switch(elm->base.btype) {
		switch(elm.base(0).btype) {
//		case HAMMER_BTREE_TYPE_INTERNAL:
		case HAMMER_BTREE_TYPE_INTERNAL:
//		case HAMMER_BTREE_TYPE_LEAF:
		case HAMMER_BTREE_TYPE_LEAF:
//			KKASSERT(elm->internal.subtree_offset != 0);
			kernel.KKASSERT(elm.internal(0).subtree_offset != 0)
//			child = hammer_get_node(cursor->trans,
			child = hammer_get_node(cursor.trans,
//						elm->internal.subtree_offset,
				elm.internal(0).subtree_offset,
//						0, &error);
				0, &error)
//			break;
			break
//		default:
		default:
//			child = NULL;
			child = nil
//			break;
			break
//		}
		}
//		if (child) {
		if child != empty_hammer_node_t {
//			if (hammer_lock_ex_try(&child->lock) != 0) {
			if (hammer_lock_ex_try(&child.lock) != 0) {
//				if (cursor->deadlk_node == NULL) {
				if (cursor.deadlk_node == nil) {
//					cursor->deadlk_node = child;
					cursor.deadlk_node = child
//					hammer_ref_node(cursor->deadlk_node);
					hammer_ref_node(cursor.deadlk_node)
//				}
				}
//				error = EDEADLK;
				error = system.EDEADLK
//				hammer_rel_node(child);
				hammer_rel_node(child)
//			} else {
			} else {
//				if (lcache) {
				if lcache != empty_hammer_node_lock_t {
//					item = TAILQ_FIRST(&lcache->list);
					item = lcache
//					KKASSERT(item != NULL);
					kernel.KKASSERT(item != nil)
//					item->flags |= HAMMER_NODE_LOCK_LCACHE;
					item.flags |= HAMMER_NODE_LOCK_LCACHE
//					TAILQ_REMOVE(&lcache->list,
					system.TAILQ_REMOVE(&lcache.list,
//						     item, entry);
						     item, "entry")
//				} else {
				} else {
//					item = kmalloc(sizeof(*item),
//						       hmp->m_misc,
//						       M_WAITOK|M_ZERO);
					item = new(hammer_node_lock)
//					TAILQ_INIT(&item->list);
					system.TAILQ_INIT(&item.list)
//				}
				}
//

//				TAILQ_INSERT_TAIL(&parent->list, item, entry);
				system.TAILQ_INSERT_TAIL(&parent.list, item, "entry")
//				item->parent = parent;
				item.parent = parent
//				item->node = child;
				item.node = child
//				item->index = i;
				item.index = i
//				item->count = child->ondisk->count;
				item.count = int(child.ondisk.count)
//

				/*
				 * Recurse (used by the rebalancing code)
				 */
//				if (depth > 1 && elm->base.btype == HAMMER_BTREE_TYPE_INTERNAL) {
				if (depth > 1 && elm.base(0).btype == HAMMER_BTREE_TYPE_INTERNAL) {
//					error = hammer_btree_lock_children(
					error = hammer_btree_lock_children(
//							cursor,
						cursor,
//							depth - 1,
						depth - 1,
//							item,
						item,
//							lcache);
						lcache)
//				}
				}
//			}
			}
//		}
		}
//	}
	}
//	if (error)
	if error != 0 {
//		hammer_btree_unlock_children(hmp, parent, lcache);
		hammer_btree_unlock_children(hmp, parent, lcache)
	}
//	return(error);
	return error
//}
}
//

/*
 * Create an in-memory copy of all B-Tree nodes listed, recursively,
 * including the parent.
 */
//void
//hammer_btree_lock_copy(hammer_cursor_t cursor, hammer_node_lock_t parent)
//{
func hammer_btree_lock_copy(cursor hammer_cursor_t, parent hammer_node_lock_t) {
//	hammer_mount_t hmp = cursor->trans->hmp;
	//hmp := cursor.trans.hmp
//	hammer_node_lock_t item;
	var item hammer_node_lock_t
//

//	if (parent->copy == NULL) {
	if parent.copy == nil {
//		KKASSERT((parent->flags & HAMMER_NODE_LOCK_LCACHE) == 0);
		kernel.KKASSERT((parent.flags & HAMMER_NODE_LOCK_LCACHE) == 0)
//		parent->copy = kmalloc(sizeof(*parent->copy),
//				       hmp->m_misc, M_WAITOK);
		parent.copy = new(hammer_node_ondisk)
//	}
	}
//	KKASSERT((parent->flags & HAMMER_NODE_LOCK_UPDATED) == 0);
	kernel.KKASSERT((parent.flags & HAMMER_NODE_LOCK_UPDATED) == 0)
//	*parent->copy = *parent->node->ondisk;
	*parent.copy = *parent.node.ondisk
//	TAILQ_FOREACH(item, &parent->list, entry) {
//		hammer_btree_lock_copy(cursor, item);
//	}
	system.TAILQ_FOREACH(item, &parent.list, "entry", hammer_btree_lock_copy, cursor, item)
//}
}
//

/*
 * Recursively sync modified copies to the media.
 */
//int
//hammer_btree_sync_copy(hammer_cursor_t cursor, hammer_node_lock_t parent)
//{
func hammer_btree_sync_copy(cursor hammer_cursor_t, parent hammer_node_lock_t) int {
//	hammer_node_lock_t item;
	var item hammer_node_lock_t
//	int count = 0;
	count := 0
//

//	if (parent->flags & HAMMER_NODE_LOCK_UPDATED) {
	if (parent.flags & HAMMER_NODE_LOCK_UPDATED) != 0 {
//		++count;
		count++
//		hammer_modify_node_all(cursor->trans, parent->node);
		hammer_modify_node_all(cursor.trans, parent.node)
//		*parent->node->ondisk = *parent->copy;
		*parent.node.ondisk = *parent.copy
//		hammer_modify_node_done(parent->node);
		hammer_modify_node_done(parent.node)
//		if (parent->copy->type == HAMMER_BTREE_TYPE_DELETED) {
		if (parent.copy._type == HAMMER_BTREE_TYPE_DELETED) {
//			hammer_flush_node(parent->node, 0);
			hammer_flush_node(parent.node, 0)
//			hammer_delete_node(cursor->trans, parent->node);
			hammer_delete_node(cursor.trans, parent.node)
//		}
		}
//	}
	}
//	TAILQ_FOREACH(item, &parent->list, entry) {
//		count += hammer_btree_sync_copy(cursor, item);
//	}
	system.TAILQ_FOREACH(item, &parent.list, "entry", func() {count += hammer_btree_sync_copy(cursor, item)})
//	return(count);
	return count
//}
}
//

/*
 * Release previously obtained node locks.  The caller is responsible for
 * cleaning up parent.node itself (its usually just aliased from a cursor),
 * but this function will take care of the copies.
 *
 * NOTE: The root node is not placed in the lcache and node.copy is not
 *	 deallocated when lcache != NULL.
 */
//void
//hammer_btree_unlock_children(hammer_mount_t hmp, hammer_node_lock_t parent,
func hammer_btree_unlock_children(hmp hammer_mount_t, parent hammer_node_lock_t,
//			     hammer_node_lock_t lcache)
//{
		lcache hammer_node_lock_t) {
//	hammer_node_lock_t item;
	var item hammer_node_lock_t
//	hammer_node_ondisk_t copy;
	var copy hammer_node_ondisk_t
	var empty_hammer_node_lock_t hammer_node_lock_t
	var empty_hammer_node_ondisk_t hammer_node_ondisk_t
//

//	while ((item = TAILQ_FIRST(&parent->list)) != NULL) {
	for ; item != nil; {
//		TAILQ_REMOVE(&parent->list, item, entry);
		system.TAILQ_REMOVE(&parent.list, item, "entry")
//		hammer_btree_unlock_children(hmp, item, lcache);
		hammer_btree_unlock_children(hmp, item, lcache)
//		hammer_unlock(&item->node->lock);
		hammer_unlock(&item.node.lock)
//		hammer_rel_node(item->node);
		hammer_rel_node(item.node)
//		if (lcache) {
		if lcache != empty_hammer_node_lock_t {
			/*
			 * NOTE: When placing the item back in the lcache
			 *	 the flag is cleared by the bzero().
			 *	 Remaining fields are cleared as a safety
			 *	 measure.
			 */
//			KKASSERT(item->flags & HAMMER_NODE_LOCK_LCACHE);
			kernel.KKASSERT(item.flags & HAMMER_NODE_LOCK_LCACHE)
//			KKASSERT(TAILQ_EMPTY(&item->list));
			kernel.KKASSERT(system.TAILQ_EMPTY(&item.list))
//			copy = item->copy;
			copy = item.copy
//			bzero(item, sizeof(*item));
			library.Bzero(item, unsafe.Sizeof(item))
//			TAILQ_INIT(&item->list);
			system.TAILQ_INIT(&item.list)
//			item->copy = copy;
			item.copy = copy
//			if (copy)
			if copy != empty_hammer_node_ondisk_t {
//				bzero(copy, sizeof(*copy));
				library.Bzero(copy, unsafe.Sizeof(copy))
			}
//			TAILQ_INSERT_TAIL(&lcache->list, item, entry);
			system.TAILQ_INSERT_TAIL(&lcache.list, item, "entry")
//		} else {
		} else {
//			kfree(item, hmp->m_misc);
			item = nil
			hmp.m_misc = nil
//		}
		}
//	}
	}
//	if (parent->copy && (parent->flags & HAMMER_NODE_LOCK_LCACHE) == 0) {
	if ((parent.copy != nil) && ((parent.flags & HAMMER_NODE_LOCK_LCACHE) == 0)) {
//		kfree(parent->copy, hmp->m_misc);
		parent.copy = nil
		hmp.m_misc = nil
//		parent->copy = NULL;	/* safety */
		parent.copy = nil	/* safety */
//	}
	}
//}
}
//

/************************************************************************
 *			   MISCELLANIOUS SUPPORT 			*
 ************************************************************************/
//

/*
 * Compare two B-Tree elements, return -N, 0, or +N (e.g. similar to strcmp).
 *
 * Note that for this particular function a return value of -1, 0, or +1
 * can denote a match if create_tid is otherwise discounted.  A create_tid
 * of zero is considered to be 'infinity' in comparisons.
 *
 * See also hammer_rec_rb_compare() and hammer_rec_cmp() in hammer_object.c.
 */
//int
//hammer_btree_cmp(hammer_base_elm_t key1, hammer_base_elm_t key2)
//{
func hammer_btree_cmp(key1 hammer_base_elm_t, key2 hammer_base_elm_t) int {
//	if (key1->localization < key2->localization)
	if (key1.localization < key2.localization) {
//		return(-5);
		return -5
	}
//	if (key1->localization > key2->localization)
	if (key1.localization > key2.localization) {
//		return(5);
		return 5
	}
//

//	if (key1->obj_id < key2->obj_id)
	if (key1.obj_id < key2.obj_id) {
//		return(-4);
		return -4
	}
//	if (key1->obj_id > key2->obj_id)
	if (key1.obj_id > key2.obj_id) {
//		return(4);
		return 4
	}
//

//	if (key1->rec_type < key2->rec_type)
	if (key1.rec_type < key2.rec_type) {
//		return(-3);
		return -3
	}
//	if (key1->rec_type > key2->rec_type)
	if (key1.rec_type > key2.rec_type) {
//		return(3);
		return 3
	}
//

//	if (key1->key < key2->key)
	if (key1.key < key2.key) {
//		return(-2);
		return -2
	}
//	if (key1->key > key2->key)
	if (key1.key > key2.key) {
//		return(2);
		return 2
	}
//

	/*
	 * A create_tid of zero indicates a record which is undeletable
	 * and must be considered to have a value of positive infinity.
	 */
//	if (key1->create_tid == 0) {
	if (key1.create_tid == 0) {
//		if (key2->create_tid == 0)
		if (key2.create_tid == 0) {
//			return(0);
			return 0
		}
//		return(1);
		return 1
//	}
	}
//	if (key2->create_tid == 0)
	if (key2.create_tid == 0) {
//		return(-1);
		return -1
	}
//	if (key1->create_tid < key2->create_tid)
	if (key1.create_tid < key2.create_tid) {
//		return(-1);
		return -1
	}
//	if (key1->create_tid > key2->create_tid)
	if (key1.create_tid > key2.create_tid) {
//		return(1);
		return 1
	}
//	return(0);
	return 0
//}
}
//

/*
 * Test a timestamp against an element to determine whether the
 * element is visible.  A timestamp of 0 means 'infinity'.
 */
//int
//hammer_btree_chkts(hammer_tid_t asof, hammer_base_elm_t base)
//{
func hammer_btree_chkts(asof hammer_tid_t, base hammer_base_elm) int {
	var empty_hammer_tid_t hammer_tid_t
//	if (asof == 0) {
	if (asof == 0) {
//		if (base->delete_tid)
		if base.delete_tid != 0 {
//			return(1);
			return(1)
		}
//		return(0);
		return 0
//	}
	}
//	if (asof < base->create_tid)
	if (asof < base.create_tid) {
//		return(-1);
		return(-1)
	}
//	if (base->delete_tid && asof >= base->delete_tid)
	if ((base.delete_tid != empty_hammer_tid_t) && (asof >= base.delete_tid)) {
//		return(1);
		return(1)
	}
//	return(0);
	return 0
//}
}
//

/*
 * Create a separator half way inbetween key1 and key2.  For fields just
 * one unit apart, the separator will match key2.  key1 is on the left-hand
 * side and key2 is on the right-hand side.
 *
 * key2 must be >= the separator.  It is ok for the separator to match key2.
 *
 * NOTE: Even if key1 does not match key2, the separator may wind up matching
 * key2.
 *
 * NOTE: It might be beneficial to just scrap this whole mess and just
 * set the separator to key2.
 */
//#define MAKE_SEPARATOR(key1, key2, dest, field)	\
//	dest->field = key1->field + ((key2->field - key1->field + 1) >> 1);
func MAKE_SEPARATOR(key1 hammer_base_elm_t, key2 hammer_base_elm_t, dest hammer_base_elm_t, field string) {}
//

//static void
//hammer_make_separator(hammer_base_elm_t key1, hammer_base_elm_t key2,
func hammer_make_separator(key1 hammer_base_elm_t, key2 hammer_base_elm,
//		      hammer_base_elm_t dest)
//{
		      dest hammer_base_elm) {
//	bzero(dest, sizeof(*dest));
	library.Bzero(dest, unsafe.Sizeof(dest))
//

//	dest->rec_type = key2->rec_type;
	dest.rec_type = key2.rec_type
//	dest->key = key2->key;
	dest.key = key2.key
//	dest->obj_id = key2->obj_id;
	dest.obj_id = key2.obj_id
//	dest->create_tid = key2->create_tid;
	dest.create_tid = key2.create_tid
//

//	MAKE_SEPARATOR(key1, key2, dest, localization);
	MAKE_SEPARATOR(key1, &key2, &dest, "localization");
//	if (key1->localization == key2->localization) {
	if (key1.localization == key2.localization) {
//		MAKE_SEPARATOR(key1, key2, dest, obj_id);
		MAKE_SEPARATOR(key1, &key2, &dest, "obj_id")
//		if (key1->obj_id == key2->obj_id) {
		if (key1.obj_id == key2.obj_id) {
//			MAKE_SEPARATOR(key1, key2, dest, rec_type);
			MAKE_SEPARATOR(key1, &key2, &dest, "rec_type")
//			if (key1->rec_type == key2->rec_type) {
			if (key1.rec_type == key2.rec_type) {
//				MAKE_SEPARATOR(key1, key2, dest, key);
				MAKE_SEPARATOR(key1, &key2, &dest, "key")
				/*
				 * Don't bother creating a separator for
				 * create_tid, which also conveniently avoids
				 * having to handle the create_tid == 0
				 * (infinity) case.  Just leave create_tid
				 * set to key2.
				 *
				 * Worst case, dest matches key2 exactly,
				 * which is acceptable.
				 */
//			}
			}
//		}
		}
//	}
	}
//}
}
//

//#undef MAKE_SEPARATOR

//

/*
 * Return whether a generic internal or leaf node is full
 */
//static int
//btree_node_is_full(hammer_node_ondisk_t node)
//{
func btree_node_is_full(node hammer_node_ondisk_t) int {
//	switch(node->type) {
	switch(node._type) {
//	case HAMMER_BTREE_TYPE_INTERNAL:
	case HAMMER_BTREE_TYPE_INTERNAL:
//		if (node->count == HAMMER_BTREE_INT_ELMS)
		if (node.count == HAMMER_BTREE_INT_ELMS) {
//			return(1);
			return(1)
		}
//		break;
		break
//	case HAMMER_BTREE_TYPE_LEAF:
	case HAMMER_BTREE_TYPE_LEAF:
//		if (node->count == HAMMER_BTREE_LEAF_ELMS)
		if (node.count == HAMMER_BTREE_LEAF_ELMS) {
//			return(1);
			return(1)
		}
//		break;
		break
//	default:
	default:
//		panic("illegal btree subtype");
		panic("illegal btree subtype")
//	}
	}
//	return(0);
	return 0
//}
}
//

//#if 0
//static int
//btree_max_elements(u_int8_t type)
//{
//	if (type == HAMMER_BTREE_TYPE_LEAF)
//		return(HAMMER_BTREE_LEAF_ELMS);
//	if (type == HAMMER_BTREE_TYPE_INTERNAL)
//		return(HAMMER_BTREE_INT_ELMS);
//	panic("btree_max_elements: bad type %d", type);
//}
//#endif
//

//void
//hammer_print_btree_node(hammer_node_ondisk_t ondisk)
//{
func hammer_print_btree_node(ondisk hammer_node_ondisk_t) {
//	hammer_btree_elm_t elm;
	var elm hammer_btree_elm
//	int i;
	var i int
//

//	kprintf("node %p count=%d parent=%016llx type=%c\n",
	kernel.Kprintf("node %p count=%d parent=%016llx type=%c\n",
//		ondisk, ondisk->count,
		ondisk, ondisk.count,
//		(long long)ondisk->parent, ondisk->type);
		int64(ondisk.parent), ondisk._type)
//

	/*
	 * Dump both boundary elements if an internal node
	 */
//	if (ondisk->type == HAMMER_BTREE_TYPE_INTERNAL) {
	if (ondisk._type == HAMMER_BTREE_TYPE_INTERNAL) {
//		for (i = 0; i <= ondisk->count; ++i) {
		for i = 0; int64(i) <= int64(ondisk.count); i++ {
//			elm = &ondisk->elms[i];
			elm = ondisk.elms[i]
//			hammer_print_btree_elm(elm, ondisk->type, i);
			hammer_print_btree_elm(elm, ondisk._type, i)
//		}
		}
//	} else {
	} else {
//		for (i = 0; i < ondisk->count; ++i) {
		for i = 0; int64(i) < int64(ondisk.count); i++ {
//			elm = &ondisk->elms[i];
			elm = ondisk.elms[i]
//			hammer_print_btree_elm(elm, ondisk->type, i);
			hammer_print_btree_elm(elm, ondisk._type, i)
//		}
		}
//	}
	}
//}
}
//

//void
//hammer_print_btree_elm(hammer_btree_elm_t elm, u_int8_t type, int i)
//{
func hammer_print_btree_elm(elm hammer_btree_elm, _type uint8, i int) {
//	kprintf("  %2d", i);
	kernel.Kprintf("  %2d", i)
//	kprintf("\tobj_id       = %016llx\n", (long long)elm->base.obj_id);
	kernel.Kprintf("\tobj_id       = %016llx\n", int64(elm.base(0).obj_id))
//	kprintf("\tkey          = %016llx\n", (long long)elm->base.key);
	kernel.Kprintf("\tkey          = %016llx\n", int64(elm.base(0).key))
//	kprintf("\tcreate_tid   = %016llx\n", (long long)elm->base.create_tid);
	kernel.Kprintf("\tcreate_tid   = %016llx\n", int64(elm.base(0).create_tid))
//	kprintf("\tdelete_tid   = %016llx\n", (long long)elm->base.delete_tid);
	kernel.Kprintf("\tdelete_tid   = %016llx\n", int64(elm.base(0).delete_tid))
//	kprintf("\trec_type     = %04x\n", elm->base.rec_type);
	kernel.Kprintf("\trec_type     = %04x\n", elm.base(0).rec_type)
//	kprintf("\tobj_type     = %02x\n", elm->base.obj_type);
	kernel.Kprintf("\tobj_type     = %02x\n", elm.base(0).obj_type)
	var arg_for_kprint uint8
	if elm.base(0).btype != 0 {
		arg_for_kprint = elm.base(0).btype
	} else {
		arg_for_kprint = uint8('?')
	}
//	kprintf("\tbtype 	= %02x (%c)\n",
	kernel.Kprintf("\tbtype 	= %02x (%c)\n",
//		elm->base.btype,
		elm.base(0).btype,
//		(elm->base.btype ? elm->base.btype : '?'));
		arg_for_kprint)
//	kprintf("\tlocalization	= %02x\n", elm->base.localization);
	kernel.Kprintf("\tlocalization	= %02x\n", elm.base(0).localization)
//

//	switch(type) {
	switch(_type) {
//	case HAMMER_BTREE_TYPE_INTERNAL:
	case HAMMER_BTREE_TYPE_INTERNAL:
//		kprintf("\tsubtree_off  = %016llx\n",
		kernel.Kprintf("\tsubtree_off  = %016llx\n",
//			(long long)elm->internal.subtree_offset);
			int64(elm.internal(0).subtree_offset));
//		break;
		break
//	case HAMMER_BTREE_TYPE_RECORD:
	case HAMMER_BTREE_TYPE_RECORD:
//		kprintf("\tdata_offset  = %016llx\n",
		kernel.Kprintf("\tdata_offset  = %016llx\n",
//			(long long)elm->leaf.data_offset);
			int64(elm.leaf(0).data_offset))
//		kprintf("\tdata_len     = %08x\n", elm->leaf.data_len);
		kernel.Kprintf("\tdata_len     = %08x\n", elm.leaf(0).data_len)
//		kprintf("\tdata_crc     = %08x\n", elm->leaf.data_crc);
		kernel.Kprintf("\tdata_crc     = %08x\n", elm.leaf(0).data_crc)
//		break;
		break
//	}
	}
//}
}
