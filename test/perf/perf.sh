#!/bin/bash

PKG_DIR="/home/wkearn/.julia/v0.4"
TEST_DIR="$PKG_DIR/Kalman/test"
n=$1
d=`date +%s`
jt=`julia $TEST_DIR/perf/perf.jl $n`
pt=`python $TEST_DIR/perf/perf.py $n`
ajt=`echo "$jt/$n" | bc -l`
apt=`echo "$pt/$n" | bc -l`
printf "%-10s%-15s%-6s%-15s%-15s\n" "julia" "$d" "$n" "$jt" "$ajt" >> $TEST_DIR/perf/benchmarks
printf "%-10s%-15s%-6s%-15s%-15s\n" "python" "$d" "$n" "$pt" "$apt"  >> $TEST_DIR/perf/benchmarks

