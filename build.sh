#! /bin/bash -e

make ID=$1 TIME=$2 clean
make ID=$1 TIME=$2
cp gcc/infrar_$1_$2.bin bin
