#!/bin/bash

cd build
make -j4
ssh root@192.168.1.136 'cd /home/root/loongCarMAX && rm -rf web'
scp -rO /home/fhfh/Work/LS2K0300CAR/web root@192.168.1.136:/home/root/loongCarMAX/web
tar -C /home/fhfh/Work/LS2K0300CAR/yolo -cf - upload \
| ssh root@192.168.1.136 "tar -C /home/root/loongCarMAX/yolo -xf -"
scp -O main root@192.168.1.136:/home/root/loongCarMAX/main
ssh root@192.168.1.136 'cd /home/root/loongCarMAX && ./main'
ssh root@192.168.1.136 'killall -9 main'