#!/bin/bash

string=$1
raw_file=$2
output=$3

extracted=tmp.$output.txt
grep $string $raw_file | sed "s/=/ /g" | sed "s/,/ /g" > $extracted
nf=`head -1 $extracted | awk '{ print NF; }'`

if [ "$nf" == "51" ]; then # this is AC3
  awk '{ printf "%s,%s,%s,%s,%s,%s\n", s1, s2, $21, $(NF-4), $(NF-2), $NF; }' s1=$output s2=$raw_file < $extracted > $output.csv
else
  awk '{ printf "%s,%s,%s,%s,%s,%s\n", s1, s2, $24, $(NF-4), $(NF-2), $NF; }' s1=$output s2=$raw_file < $extracted > $output.csv
fi
echo `wc -l $output.csv | awk '{ print $1; }'` "record(s) generated in '$output.csv'"

rm $extracted

