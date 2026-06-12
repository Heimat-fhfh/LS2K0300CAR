#!/bin/bash

cd build
make -j4

scp -O main root@192.168.1.136:/home/root/loongCarMAX/main
ssh root@192.168.1.136 'cd /home/root/loongCarMAX && ./main'
ssh root@192.168.1.136 'killall -9 main'