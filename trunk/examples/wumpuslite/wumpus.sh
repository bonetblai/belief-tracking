#!/bin/bash

# If trials should be checked for exploration, the flag "-c" should
# be passed to WorldApplication and the minisat SAT solved should
# be available in the path.

trials=$1
dim=$2
pits=$3
wumpus=$4
seed=$5

random=""
if [ "$seed" != "" ]; then
  random="-r $seed"
fi

java WorldApplication -t $trials -d $dim -p $pits -w $wumpus $random

exit 0

