#!/bin/bash

if [ $# -ne 7 ]; then
  echo "usage: experiment.sh [ore-slam-peaked|ore-slam-non-peaked|aisle-slam] [mm|opt] <ncols> <nrows> <ntrials> <policy-parameter> <po>"
  exit 1
fi

slam_type=$1
shift 1
type=$1
nc=$2
nr=$3
nt=$4
p=$5
po=$6

./slam --${slam_type} --seed $RANDOM -c $nc -r $nr -p 1 $p --pa .9 --po $po --kappa .1 -t $nt \
    --tracker="${type}-rbpf(nparticles=64,force-resampling=true,sus=false,num-sampling-attempts=2,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=128,force-resampling=true,sus=false,num-sampling-attempts=2,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=256,force-resampling=true,sus=false,num-sampling-attempts=2,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=512,force-resampling=true,sus=false,num-sampling-attempts=2,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=64,force-resampling=true,sus=false,num-sampling-attempts=2,inference=iterated-ac3(level=0,inverse-check=true))"

