#!/bin/bash

if [ $# -ne 8 ]; then
  echo "usage: experiment.sh [ore-slam-peaked|ore-slam-non-peaked|aisle-slam] [mm|opt] <ncols> <nrows> <nparticles> <ntrials> <policy-parameter> <po>"
  exit 1
fi

slam_type=$1
shift 1
type=$1
nc=$2
nr=$3
np=$4
nt=$5
p=$6
po=$7

./slam --${slam_type} --seed $RANDOM -c $nc -r $nr -p 1 $p --pa .9 --po $po --kappa .1 -t $nt \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=iterated-ac3(level=0,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=iterated-ac3(level=1,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=iterated-ac3(level=2,inverse-check=true))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=hak(doubleloop=true,clusters=MIN,init=UNIFORM,tol=1e-3,maxiter=100,maxtime=1))" \

