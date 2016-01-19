#/bin/bash

parameters=$1
output_dir=$2
nr=`wc -l $parameters | awk '{ print $1; }'`

sed "s/XXX/$nr/" < slam-template.sub > tmp.sub
qsub tmp.sub ${parameters}.txt sge-results/$output_dir
rm tmp.sub

