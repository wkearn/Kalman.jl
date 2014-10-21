#!/bin/bash

`julia -e "Pkg.update(); Pkg.clone("https://github.com/wkearn/Kalman.jl"); Pkg.checkout("Kalman","perf")`
PKG_DIR=`julia -e "print(Pkg.dir())"`
TEST_DIR="$PKG_DIR/Kalman/test"
n=$1
d=`date +%s`
jt=`julia $TEST_DIR/perf/perf.jl $n`
pt=`python $TEST_DIR/perf/perf.py $n`
ajt=`echo "$jt/$n" | bc -l`
apt=`echo "$pt/$n" | bc -l`
printf "%-10s%-15s%-6s%-15s%-15s\n" "julia" "$d" "$n" "$jt" "$ajt" >> $TEST_DIR/perf/benchmarks
printf "%-10s%-15s%-6s%-15s%-15s\n" "python" "$d" "$n" "$pt" "$apt"  >> $TEST_DIR/perf/benchmarks

