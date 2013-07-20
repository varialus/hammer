package main

import (
	"flag"
	"log"

	"fmt"
	"github.com/hanwen/go-fuse/fuse"
//	"github.com/hanwen/go-fuse/fuse/nodefs"
	//"github.com/hanwen/go-fuse/fuse/pathfs"
	"github.com/varialus/bsd/sys/sys"
	"github.com/varialus/bsd/sys/vfs/hammer2"
)
//
type Hammer2Fs struct {
	fuse.RawFileSystem
}
//
//func (me *HammerFs) GetAttr(name string, context *fuse.Context) (*fuse.Attr, fuse.Status) {
//func (me *HammerFs) GetAttr(vop_getattr_args *sys.Vop_getattr_args) (*fuse.Attr, fuse.Status) {
//func (me *Hammer2Fs) GetAttr(name string, context *fuse.Context) (*fuse.Attr, fuse.Status) {
	//var vop_getattr_args *sys.Vop_getattr_args
	//result := hammer2.Hammer_vop_getattr(vop_getattr_args)
//	switch name {
//	case "file.txt":
//		return &fuse.Attr{
//			Mode: fuse.S_IFREG | 0644, Size: uint64(len(name)),
//		}, fuse.OK
//	case "":
//		return &fuse.Attr{
//			Mode: fuse.S_IFDIR | 0755,
//		}, fuse.OK
//	}
	//fmt.Println("vop_getattr_args ==", vop_getattr_args)
	//fmt.Println("result ==", result)
//	return nil, fuse.ENOENT
//	return nil, fuse.ENOENT
//}
//
//func (me *Hammer2Fs) GetAttr(name string, context *fuse.Context) (*fuse.Attr, fuse.Status) {
//func (me *Hammer2Fs) GetAttr(vop_getattr_args *sys.Vop_getattr_args) int {
func (me *Hammer2Fs) GetAttr(name string, context *fuse.Context) (*fuse.Attr, fuse.Status) {
	var vop_getattr_args *sys.Vop_getattr_args
	result := hammer2.Hammer2_vop_getattr(vop_getattr_args)
//	vop_getattr_args = new(hammer_something.Vop_getattr_args)
//	hammer2_vnops.Hammer2_vop_getattr(vop_getattr_args)
//	switch name {
//	case "file.txt":
//		return &fuse.Attr{
//			Mode: fuse.S_IFREG | 0644, Size: uint64(len(name)),
//		}, fuse.OK
//	case "":
//		return &fuse.Attr{
//			Mode: fuse.S_IFDIR | 0755,
//		}, fuse.OK
//	}
	fmt.Println("vop_getattr_args ==", vop_getattr_args)
	fmt.Println("result ==", result)
//	return nil, fuse.ENOENT
	return nil, fuse.ENOENT
}
//
//func (me *HelloFs) OpenDir(name string, context *fuse.Context) (c []fuse.DirEntry, code fuse.Status) {
//	if name == "" {
//		c = []fuse.DirEntry{{Name: "file.txt", Mode: fuse.S_IFREG}}
//		return c, fuse.OK
//	}
//	return nil, fuse.ENOENT
//}
//
//func (me *HelloFs) Open(name string, flags uint32, context *fuse.Context) (file nodefs.File, code fuse.Status) {
//	if name != "file.txt" {
//		return nil, fuse.ENOENT
//	}
//	if flags&fuse.O_ANYWRITE != 0 {
//		return nil, fuse.EPERM
//	}
//	return nodefs.NewDataFile([]byte(name)), fuse.OK
//}
//
func main() {
	flag.Parse()
	if len(flag.Args()) < 1 {
		log.Fatal("Usage:\n  hello MOUNTPOINT")
	}
	//nfs := pathfs.NewPathNodeFs(&Hammer2Fs{RawFileSystem: pathfs.NewDefaultFileSystem()}, nil)
	//fmt.Println("nfs ==", nfs)
//	server, _, err := nodefs.MountFileSystem(flag.Arg(0), nfs, nil)
//	if err != nil {
//		log.Fatal("Mount fail: %v\n", err)
//	}
//	server.Serve()
}
