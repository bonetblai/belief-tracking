#!/bin/bash

if [ $# -ne 8 ]; then
  echo "usage: experiment.sh [ore-slam-peaked|ore-slam-non-peaked|aisle-slam] [mm|opt] <ncols> <nrows> <ntrials> <policy-parameter> <po> <kappa>"
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
kappa=$7

root=$HOME/software/github/belief-tracking/examples/probabilistic/slam3
$root/slam --${slam_type} --seed $RANDOM -c $nc -r $nr -p 1 $p --pa .9 --po $po --kappa $kappa -t $nt \
    --tracker="${type}-rbpf(nparticles=32,force-resampling=true,sus=false,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=64,force-resampling=true,sus=false,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=128,force-resampling=true,sus=false,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=256,force-resampling=true,sus=false,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=512,force-resampling=true,sus=false,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=1024,force-resampling=true,sus=false,inference=jt(updates=HUGIN))" \
    --tracker="${type}-rbpf(nparticles=32,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=64,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=128,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=256,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=512,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=1024,force-resampling=true,sus=false,inference=bp(updates=SEQRND,logdomain=false,tol=1e-5,maxtime=.5))" \
    --tracker="${type}-rbpf(nparticles=32,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=64,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=128,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=256,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=512,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))" \
    --tracker="${type}-rbpf(nparticles=1024,force-resampling=true,sus=false,inference=iterated-ac3(level=0,inverse-check=true))"

