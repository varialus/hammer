// https://github.com/varialus/hammer/blob/master/hammer_test.go

/*
Copyright (c) 2013, Aulus Egnatius Varialus <varialus@gmail.com>

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/

package hammer

import (
	"testing"
)

// Smoke Test
// Letter after Test has to be a capital regardless of whether the function exported or not.
func TestBtree_search(t *testing.T) {
	hammer_cursor := new(hammer_cursor)
	hammer_cursor.left_bound = new(hammer_base_elm)
	hammer_cursor.right_bound = new(hammer_base_elm)
	hammer_cursor.left_bound.localization = 0
	hammer_cursor.right_bound.localization = 1
	hammer_cursor.node = new(hammer_node)
	hammer_cursor.node.ondisk = new(hammer_node_ondisk)
	if result := btree_search(hammer_cursor, 0); result != 2 {
		t.Errorf("Failed dummy test hammer.TestBtree_search()")
	}
}
