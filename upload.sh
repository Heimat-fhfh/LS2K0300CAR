#!/bin/bash

export IP="192.168.1.8"

cd build
make -j4
ssh root@${IP} 'cd /home/root/loongCarMAX && rm -rf web'
scp -rO /home/fhfh/Work/LS2K0300CAR/web root@${IP}:/home/root/loongCarMAX/web
tar -C /home/fhfh/Work/LS2K0300CAR/yolo -cf - upload \
| ssh root@${IP} "tar -C /home/root/loongCarMAX/yolo -xf -"
scp -O main root@${IP}:/home/root/loongCarMAX/main
ssh root@${IP} 'cd /home/root/loongCarMAX && ./main'
ssh root@${IP} 'killall -9 main'