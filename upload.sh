#!/bin/bash

cd build
make -j4
scp -O main root@192.168.1.136:/home/root
ssh root@192.168.1.136 'cd /home/root && ./main'
ssh root@192.168.1.136 'pkill -f main'