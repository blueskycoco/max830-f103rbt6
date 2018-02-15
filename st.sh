#! /bin/bash -e
make clean
make
openocd -f openocd.cfg -c "flash_image"
