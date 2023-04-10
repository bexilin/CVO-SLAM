#!/bin/bash

target_dir=$1

if [ -d "$target_dir" ]; then
    rm -r $target_dir
fi

mkdir $target_dir

cp -r ./* $target_dir
cd $target_dir
mkdir build
cd build
cmake .. -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc
make -j$(expr $(nproc) - 6)
# cp ../config/demo_superpoint.py .
# cp ../config/superpoint_v1.pth .


