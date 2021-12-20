#!/bin/bash
#
# $Id$
# Purpose: Start ntripclient

# change these 3 according to your needs
User='PUrobot2'
Password='PUrobot2'
Port='10000'
Server='108.59.49.226'
Serial='/dev/ttyUSB0'
Stream='RTCM3_MAX'
mode=3

DateStart=`date -u '+%s'`
SleepMin=10     # Wait min sec for next reconnect try
SleepMax=10000  # Wait max sec for next reconnect try
(while true; do
  ./ntripclient -s $Server -r $Port -u $User -p $Password -D $Serial -B 38400 -m $Stream -M $mode
  if test $? -eq 0; then DateStart=`date -u '+%s'`; fi
  DateCurrent=`date -u '+%s'`
  SleepTime=`echo $DateStart $DateCurrent | awk '{printf("%d",($2-$1)*0.02)}'`
  if test $SleepTime -lt $SleepMin; then SleepTime=$SleepMin; fi
  if test $SleepTime -gt $SleepMax; then SleepTime=$SleepMax; fi
  # Sleep 2 percent of outage time before next reconnect try
  sleep $SleepTime
done)
