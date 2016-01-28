#!/bin/bash

echo "type,file,inference,sir,dim,np,steps,errors,unknowns,time"
for slam_type in ore-slam-peaked ore-slam-non-peaked; do
  for sir in opt mm; do
    for dim in 4x4 6x6 8x8 10x10 12x12 14x14; do
      for np in 32 64 128 256; do
        for dir in results/parameters-?n*.txt; do
          ./generate_stats_csv.sh HUGIN $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "jt,${sir},${dim},${np}"
          ./generate_stats_csv.sh "SEQRND,logdomain=false,tol=1e-5,maxtime=.1" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "bp(maxtime=.1),${sir},${dim},${np}"
          ./generate_stats_csv.sh "SEQRND,logdomain=false,tol=1e-5,maxtime=.5" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "bp(maxtime=.5),${sir},${dim},${np}"
          ./generate_stats_csv.sh "level=0" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "ac(level=0),${sir},${dim},${np}"
          ./generate_stats_csv.sh "level=1" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "ac(level=1),${sir},${dim},${np}"
          ./generate_stats_csv.sh "level=2" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "ac(level=2),${sir},${dim},${np}"
          ./generate_stats_csv.sh "level=3" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.kappa=.1.txt $slam_type "ac(level=3),${sir},${dim},${np}"
        done
      done
    done
  done
done

