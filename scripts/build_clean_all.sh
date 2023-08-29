#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting Simulation Executables Generation...${NC}"


[ -d "build_simulation" ] && echo "Directory /build_simulation exists." && rm -R build_simulation

mkdir build_simulation && cd build_simulation/ && cmake  -DSimulation=ON ..  && make && cd ..

echo -e "${GREEN} Done Simulation Executables Generation${NC}"

[ -d "build_experiment" ] && echo "Directory /build_experiment exists." && rm -R build_experiment

mkdir build_experiment && cd build_experiment/ && cmake   -DSimulation=OFF ..  -DCMAKE_TOOLCHAIN_FILE=../cmake/pi.cmake  && make && cd ..
