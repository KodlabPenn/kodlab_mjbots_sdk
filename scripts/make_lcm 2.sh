#!/bin/bash


GREEN='\033[0;32m'
NC='\033[0m' # No Color

echo -e "${GREEN} Starting LCM type generation...${NC}"

cd "${0%/*}"
cd ../lcm_types
# Clean

echo "Deleting old files"
rm */*.hpp
rm */*.py
rm */*.pyc
rm *.hpp
rm *.py
rm *.pyc

# Make
lcm-gen -xp *.lcm


echo -e "${GREEN} Done with LCM type generation${NC}"
