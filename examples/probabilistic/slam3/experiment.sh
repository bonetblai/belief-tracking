#!/bin/bash

if [ $# -ne 7 ]; then
  echo "usage: experiment.sh [mm|opt] <ncols> <nrows> <nparticles> <ntrials> <policy-parameter> <po>"
  exit 1
fi

type=$1
nc=$2
nr=$3
np=$4
nt=$5
p=$6
po=$7

./slam --ore-slam --seed $RANDOM -c $nc -r $nr -p 1 $p --pa .9 --po $po --kappa .1 -t $nt \
    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=1))"
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=jt(updates=HUGIN))" \
#    --tracker="${type}-rbpf(nparticles=$np,force-resampling=true,sus=false,num-sampling-attempts=2,inference=hak(doubleloop=true,clusters=MIN,init=UNIFORM,tol=1e-3,maxiter=100,maxtime=1))"


