HAMMER
======

This repository contains an attempt at porting [a Go translation][1] of [HAMMER file system][2] to [go-fuse][3].

Status
------

**Under Development**

 - hammer/bazil.org/fuse/serve/main.go

Setup
-----

 1. [Install and configure Go][4]
 2. export GOPATH=$HOME/go
 3. go get -d github.com/varialus/hammer
 4. cd ~/go/src/github.com/varialus/hammer/
 5. go test ./...

[1]:https://github.com/varialus/bsd
[2]:http://en.wikipedia.org/wiki/HAMMER
[3]:https://github.com/hanwen/go-fuse
[4]:http://golang.org/doc/install
