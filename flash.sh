#! /bin/bash -e
make
jflasharm -openprj$1 -opengcc/infrar.bin,0x08000000 -auto -startapp -exit
