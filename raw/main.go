// b main.go:41
// run ~/go/src/hammerdisk.raw /mnt

package main

import (
	"flag"
	"fmt"
	"github.com/hanwen/go-fuse/fuse"
	"github.com/varialus/bsd/lib/libstand"
	"github.com/varialus/bsd/sys/sys"
	"github.com/varialus/bsd/sys/vfs/hammer"
	"github.com/varialus/bsd/temporary_translation_utilities"
	"os"
	"path"
	"unsafe"
)

var hfs libstand.Hfs

type HammerFs struct {
	//fuse.RawFileSystem
	device string
}

func (me *HammerFs) GetAttr(*fuse.GetAttrIn, *fuse.AttrOut) (fuse.Status) {
	new_vop_getattr_args := new(sys.Vop_getattr_args)
	vop_getattr_args := new_vop_getattr_args
	new_vnode := new(sys.Vnode)
	vop_getattr_args.A_vp = new_vnode
	pointer := new(int)
	vop_getattr_args.A_vp.V_data = unsafe.Pointer(pointer)
	result := hammer.Hammer_vop_getattr(vop_getattr_args)
	//fmt.Println("vop_getattr_args ==", vop_getattr_args)
	//fmt.Println("result ==", result)
	temporary_translation_utilities.Use_vars_so_compiler_does_not_complain(result)
	//fmt.Println("fuse.ENOENT ==", fuse.ENOENT)
	return fuse.ENOENT
}

func main() {
	debug := flag.Bool("debug", false, "print debugging messages.")
	flag.Parse()
	if flag.NArg() < 2 {
		_, program_name := path.Split(os.Args[0])
		fmt.Fprintf(os.Stderr, "usage: %s DEVICE MOUNTPOINT\n", program_name)
		os.Exit(2)
	}

	device := flag.Arg(0)
	mountpoint := flag.Arg(1)
	//fd, err := os.Open(device)
	//if err != nil {
	//	fmt.Printf("unable to open %v: %v\n", device, err)
	//	os.Exit(sys.EINVAL)
	//}
	//hfs.Fd = fd
	fs := NewHammerFileSystem(device)
	server, err := fuse.NewServer(fuse.RawFileSystem(fs), mountpoint, nil)
	if err != nil {
		fmt.Printf("Mount fail: %v\n", err)
		os.Exit(1)
	}
	server.SetDebug(*debug)
	fmt.Println("Mounted!")
	server.Serve()
}

func NewHammerFileSystem(device string) *HammerFs {
	fs := new(HammerFs)
	fs.device = device
	return fs
}

func (fs *HammerFs) String() string {
	return fmt.Sprint(fs)
}

func (fs *HammerFs) Init(*fuse.Server) {
	conf := new(sys.Vfsconf)
	hammer.Hammer_vfs_init(conf)
}

//func (fs *HammerFs) String() string {
//	return os.Args[0]
//}

func (fs *HammerFs) SetDebug(dbg bool) {
}

func (fs *HammerFs) StatFs(header *fuse.InHeader, out *fuse.StatfsOut) fuse.Status {
	return fuse.ENOSYS
}

func (fs *HammerFs) Lookup(header *fuse.InHeader, name string, out *fuse.EntryOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Forget(nodeID, nlookup uint64) {
}

//func (fs *HammerFs) GetAttr(input *fuse.GetAttrIn, out *fuse.AttrOut) (code fuse.Status) {
//	return fuse.ENOSYS
//}

func (fs *HammerFs) Open(input *fuse.OpenIn, out *fuse.OpenOut) (status fuse.Status) {
	return fuse.OK
}

func (fs *HammerFs) SetAttr(input *fuse.SetAttrIn, out *fuse.AttrOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Readlink(header *fuse.InHeader) (out []byte, code fuse.Status) {
	return nil, fuse.ENOSYS
}

func (fs *HammerFs) Mknod(input *fuse.MknodIn, name string, out *fuse.EntryOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Mkdir(input *fuse.MkdirIn, name string, out *fuse.EntryOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Unlink(header *fuse.InHeader, name string) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Rmdir(header *fuse.InHeader, name string) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Symlink(header *fuse.InHeader, pointedTo string, linkName string, out *fuse.EntryOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Rename(input *fuse.RenameIn, oldName string, newName string) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Link(input *fuse.LinkIn, name string, out *fuse.EntryOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) GetXAttrSize(header *fuse.InHeader, attr string) (size int, code fuse.Status) {
	return 0, fuse.ENOSYS
}

func (fs *HammerFs) GetXAttrData(header *fuse.InHeader, attr string) (data []byte, code fuse.Status) {
	return nil, fuse.ENOSYS
}

func (fs *HammerFs) SetXAttr(input *fuse.SetXAttrIn, attr string, data []byte) fuse.Status {
	return fuse.ENOSYS
}

func (fs *HammerFs) ListXAttr(header *fuse.InHeader) (data []byte, code fuse.Status) {
	return nil, fuse.ENOSYS
}

func (fs *HammerFs) RemoveXAttr(header *fuse.InHeader, attr string) fuse.Status {
	return fuse.ENOSYS
}

func (fs *HammerFs) Access(input *fuse.AccessIn) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Create(input *fuse.CreateIn, name string, out *fuse.CreateOut) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) OpenDir(input *fuse.OpenIn, out *fuse.OpenOut) (status fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Read(input *fuse.ReadIn, buf []byte) (fuse.ReadResult, fuse.Status) {
	return nil, fuse.ENOSYS
}

func (fs *HammerFs) Release(input *fuse.ReleaseIn) {
}

func (fs *HammerFs) Write(input *fuse.WriteIn, data []byte) (written uint32, code fuse.Status) {
	return 0, fuse.ENOSYS
}

func (fs *HammerFs) Flush(input *fuse.FlushIn) fuse.Status {
	return fuse.OK
}

func (fs *HammerFs) Fsync(input *fuse.FsyncIn) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) ReadDir(input *fuse.ReadIn, l *fuse.DirEntryList) fuse.Status {
	return fuse.ENOSYS
}

func (fs *HammerFs) ReadDirPlus(input *fuse.ReadIn, l *fuse.DirEntryList) fuse.Status {
	return fuse.ENOSYS
}

func (fs *HammerFs) ReleaseDir(input *fuse.ReleaseIn) {
}

func (fs *HammerFs) FsyncDir(input *fuse.FsyncIn) (code fuse.Status) {
	return fuse.ENOSYS
}

func (fs *HammerFs) Fallocate(in *fuse.FallocateIn) (code fuse.Status) {
	return fuse.ENOSYS
}
