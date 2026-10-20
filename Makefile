# surface-touch — touch stack for the Microsoft Surface Laptop Studio 2

NAME     := surface-touch
VERSION  := $(shell cat VERSION)
MODULE   := surface-thc-hid

DESTDIR  ?=
PREFIX   ?= /usr/local

.PHONY: all clean

all:
	$(MAKE) -C src

clean:
	$(MAKE) -C src clean
