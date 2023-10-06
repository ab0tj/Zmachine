#!/bin/bash

rm -f "$1"
z80asm zbootsect.asm -I ../include/ -lzbootsect.lst -ozbootsect.bin
mkfs.cpm -f 8mb-sd-p1 -b zbootsect.bin -b CPMLDR.COM -L ZMACHINE -t "$1"
cpmcp -f 8mb-sd-p1 "$1" imgfiles/* 0:
