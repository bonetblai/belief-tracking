#/bin/bash

parameters=$1
output_dir=$2
nr=`wc -l $parameters | awk '{ print $1; }'`

mkdir -p output
sed "s/XXX/$nr/" < slam-template-ex-ac2.sub > tmp.sub
qsub tmp.sub $parameters $output_dir
rm tmp.sub

