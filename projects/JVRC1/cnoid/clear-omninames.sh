#!/bin/bash

sudo /etc/init.d/omniorb4-nameserver stop
sudo rm -f /var/log/omniorb-nameserver.log 2> /dev/null
sudo rm -f /var/lib/omniorb/* 2> /dev/null
sudo rm -f /tmp/rtcmanager.ref 2> /dev/null
sudo /etc/init.d/omniorb4-nameserver start
for i in {0..9}
do
    [ -f /var/log/omniorb-nameserver.log ] && break
    sleep 1
done
tail /var/log/omniorb-nameserver.log
