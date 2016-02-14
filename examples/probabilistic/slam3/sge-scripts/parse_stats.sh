#!/bin/bash

echo "type,file,inference,sir,dim,np,steps,errors,unknowns,time"
for slam_type in mine-mapping-non-peaked; do
  for sir in opt mm; do
    for dim in 4x4 6x6 8x8 10x10 12x12 14x14; do
      for np in 32 64 128 256; do
        for dir in results.new/parameters-?n-paper.txt; do
          ./generate_stats_csv.sh HUGIN $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "jt,${sir},${dim},${np}"
          ./generate_stats_csv.sh "SEQRND,logdomain=false,tol=1e-5,maxtime=.1" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "bp(maxtime=.1),${sir},${dim},${np}"
          ./generate_stats_csv.sh "SEQRND,logdomain=false,tol=1e-5,maxtime=.5" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "bp(maxtime=.5),${sir},${dim},${np}"

          ./generate_stats_csv.sh "kappa=.1,level=0,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=0,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=0,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=0,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=0,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=0,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=1,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=1,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=1,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=1,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=1,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=1,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=2,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=2,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=2,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=2,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=2,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=2,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=3,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=3,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=3,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=3,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.1,level=3,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.1,level=3,lazy=false,simple=true)\",${sir},${dim},${np}"

          ./generate_stats_csv.sh "kappa=.3,level=0,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=0,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=0,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=0,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=0,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=0,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=1,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=1,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=1,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=1,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=1,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=1,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=2,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=2,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=2,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=2,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=2,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=2,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=3,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=3,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=3,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=3,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.3,level=3,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.3,level=3,lazy=false,simple=true)\",${sir},${dim},${np}"

          ./generate_stats_csv.sh "kappa=.5,level=0,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=0,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=0,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=0,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=0,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=0,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=1,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=1,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=1,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=1,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=1,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=1,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=2,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=2,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=2,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=2,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=2,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=2,lazy=false,simple=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=3,inverse-check=true,lazy=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=3,lazy=true)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=3,inverse-check=true,lazy=false,simple=false" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=3,lazy=false,simple=false)\",${sir},${dim},${np}"
          ./generate_stats_csv.sh "kappa=.5,level=3,inverse-check=true,lazy=false,simple=true" $dir/raw_data.type=${slam_type}.sir=${sir}.dim=${dim}.np=${np}.txt $slam_type "\"ac(kappa=.5,level=3,lazy=false,simple=true)\",${sir},${dim},${np}"
        done
      done
    done
  done
done

