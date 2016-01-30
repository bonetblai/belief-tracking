#/bin/bash

parameters=$1
output_dir=$2
kappa=$3
nr=`wc -l $parameters | awk '{ print $1; }'`

mkdir -p output
sed "s/XXX/$nr/" < slam-template-ex-missing.sub > tmp.sub
qsub tmp.sub $parameters $output_dir $kappa
rm tmp.sub

