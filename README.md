HAMMER
======

This repository contains a yet as incomplete attempt to translate the [HAMMER file system][1] and necessary dependencies to the [Go programming language][2].

Status
------

**Rough Translation Complete**

 - hammer_btree.go

Setup
-----

 1. [Install and configure Go][3]
 2. export GOPATH=$HOME/go
 3. go get -d github.com/varialus/hammer
 4. cd ~/go/src/github.com/varialus/hammer/
 5. go test

[1]:http://en.wikipedia.org/wiki/HAMMER
[2]:http://en.wikipedia.org/wiki/Go_%28programming_language%29
[3]:http://golang.org/doc/install
