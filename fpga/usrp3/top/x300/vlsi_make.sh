#!/bin/bash
#Build using vlsipool tools

module load python/3.8.5
. ./setupenv.sh --vivado-path /usr/caen/xilinx/vivado-2021.1/Vivado/
make X310_HG
