#! /bin/bash
make
openocd -f openocd.cfg -c "flash_image"
