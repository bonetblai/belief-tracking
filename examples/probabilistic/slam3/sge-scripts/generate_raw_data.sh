#!/bin/bash

parameters=$1
dir=$2

benchmark=tmp.benchmark
sort -u < $parameters > $benchmark
nb=`wc -l $benchmark | awk '{ print $1; }'`

root=$HOME/software/github/belief-tracking/examples/probabilistic/slam3

for ((nr=1;nr<=$nb;++nr)); do
  record=`awk "NR==$nr" $benchmark`
  slam_type=`echo $record | awk '{print $1;}'`
  sir_type=`echo $record | awk '{print $2;}'`
  dim=`echo $record | awk '{print $3;}'`
  np=`echo $record | awk '{print $4;}'`
  kappa=`echo $record | awk '{print $5;}'`
  grep -h final $root/$dir/output.type=${slam_type}.sir=${sir_type}.dim=${dim}x${dim}.np=${np}.kappa=${kappa}.id=*.txt > $root/$dir/raw_data.type=${slam_type}.sir=${sir_type}.dim=${dim}x${dim}.np=${np}.kappa=${kappa}.txt
done

rm $benchmark

