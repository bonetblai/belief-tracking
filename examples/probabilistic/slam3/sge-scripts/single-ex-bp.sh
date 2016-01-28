#/bin/bash

parameters=$1
output_dir=$2
maxtime=$3
nr=`wc -l $parameters | awk '{ print $1; }'`

mkdir -p output
sed "s/XXX/$nr/" < slam-template-ex-bp.sub > tmp.sub
qsub tmp.sub $parameters $output_dir $maxtime
rm tmp.sub

