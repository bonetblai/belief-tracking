#!/bin/bash

if [ $# -ne 8 ]; then
  echo "usage: experiment-ex-ac0.sh [ore-slam-peaked|ore-slam-non-peaked|aisle-slam] [mm|opt] <ncols> <nrows> <nparticles> <kappa> <executions> <level>"
  exit 1
fi

slam_type=$1
type=$2
nc=$3
nr=$4
np=$5
kappa=$6
executions=$7
level=$8

root=$HOME/software/github/belief-tracking/examples/probabilistic/slam3
$root/slam --${slam_type} --seed $RANDOM -c $nc -r $nr --pa .9 --po .9 --kappa $kappa --read-execution $executions \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=$level,inverse-check=true,lazy=true))" \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=$level,inverse-check=true,lazy=false,simple=false))" \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=iterated-ac3(level=$level,inverse-check=true,lazy=false,simple=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=jt(updates=HUGIN))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))"

