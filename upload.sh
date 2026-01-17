#!/bin/bash

cd build
make -j4
ssh root@192.168.1.136 'cd /home/root/loongCarMAX && rm -rf web'
scp -rO /home/fhfh/Work/LS2K0300CAR/web root@192.168.1.136:/home/root/loongCarMAX/web
scp -O main root@192.168.1.136:/home/root/loongCarMAX/main
ssh root@192.168.1.136 'cd /home/root/loongCarMAX && ./main'
ssh root@192.168.1.136 'killall -9 main'