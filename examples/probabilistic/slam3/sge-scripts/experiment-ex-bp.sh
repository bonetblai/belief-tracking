#!/bin/bash

if [ $# -ne 9 ]; then
  echo "usage: experiment-ex-bp.sh [ore-slam-peaked|ore-slam-non-peaked|aisle-slam] [mm|opt] <ncols> <nrows> <nparticles> <po> <kappa> <maxtime> <executions>"
  exit 1
fi

slam_type=$1
shift 1
type=$1
nc=$2
nr=$3
np=$4
po=$5
kappa=$6
maxtime=$7
executions=$8

root=$HOME/software/github/belief-tracking/examples/probabilistic/slam3
$root/slam --${slam_type} --seed $RANDOM -c $nc -r $nr --pa .9 --po $po --kappa $kappa --read-execution $executions \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=$maxtime))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=jt(updates=HUGIN))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=1,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=2,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=3,inverse-check=true))"

