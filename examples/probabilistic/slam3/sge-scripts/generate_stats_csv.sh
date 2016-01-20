#!/bin/bash

string=$1
raw_file=$2
stype=$3
extra=$4

extracted=tmp.txt
grep $string $raw_file | sed "s/=/ /g" | sed "s/,/ /g" > $extracted
nf=`head -1 $extracted | awk '{ print NF; }'`

#echo "inference,file,type,sir,dim,np,steps,unknowns,errors,time"
awk '{ for(i=1;i<=NF;++i) { if( $i == "#steps" ) ns=$(i+1); if( $i == "#unknowns" ) nu=$(i+1); if( $i == "#errors" ) ne=$(i+1); if( $i == "elapsed-time" ) et=$(i+1); } } printf "%s,\"%s\",%s,%s,%s,%s,%s\n", s1, s2, s3, ns, nu, ne, et; }' s1=$stype s2=$raw_file s3=$extra < $extracted

rm $extracted


