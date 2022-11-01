#!/bin/bash
cd ~/HCR/PyPnC
if [ ! -d "build" ]; then
    mkdir build_towr
fi

cd build_towr
cmake ..
make -j