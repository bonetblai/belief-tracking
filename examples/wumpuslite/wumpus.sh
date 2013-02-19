#!/bin/bash

trials=$1
dim=$2
pits=$3
wumpus=$4
seed=$5

random=""
if [ "$seed" != "" ]; then
  random="-r $seed"
fi

#cd /home-users/nlipo/blai/Journal/wumpuslite
#export PATH=/home-users/nlipo/blai/jre1.7.0_11/bin:$PATH:/home-users/nlipo/blai/minisat
java WorldApplication -t $trials -d $dim -p $pits -w $wumpus $random

exit 0

