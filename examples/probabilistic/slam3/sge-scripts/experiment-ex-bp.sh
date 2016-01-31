#!/bin/bash

if [ $# -ne 7 ]; then
  echo "usage: experiment-ex-bp.sh [ore-slam-peaked|ore-slam-non-peaked|aisle-slam] [mm|opt] <ncols> <nrows> <nparticles> <executions> <maxtime>"
  exit 1
fi

slam_type=$1
type=$2
nc=$3
nr=$4
np=$5
executions=$6
maxtime=$7

root=$HOME/software/github/belief-tracking/examples/probabilistic/slam3
$root/slam --${slam_type} --seed $RANDOM -c $nc -r $nr --pa .9 --po .9 --read-execution $executions \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=$maxtime))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=jt(updates=HUGIN))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=1,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=2,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=3,inverse-check=true))"

