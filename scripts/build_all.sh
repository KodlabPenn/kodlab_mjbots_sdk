#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting Simulation Executables Generation...${NC}"


[ ! -d "build_simulation" ] && echo "Make Directory /build_simulation." && mkdir build_simulation

cd build_simulation/ &&  make && cd ..

echo -e "${GREEN} Done Simulation Executables Generation${NC}"

[ -d "build_experiment" ] && echo "Directory /build_experiment exists." && mkdir build_experiment

cd build_experiment/ && make && cd ..
