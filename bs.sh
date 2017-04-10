#!/bin/bash
_now=$(date +%Y-%m-%d-%H-%M-%S)
_file=/Users/zhaocongyuan/Desktop/BatterySorterLog_$_now.txt
#screen /dev/cu.usbserial-AK0599AF 9600 > $_file
script -a -t 0 $_file screen /dev/cu.usbserial-AK0599AF 9600