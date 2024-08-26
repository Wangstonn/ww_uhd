#!/bin/bash
# Load bitfile onto usrp

~/.local/lib/uhd-4.5/bin/uhd_image_loader --args "type=x300,addr=192.168.110.2" --fpga-path usrp_x310_fpga_HG.bit

~/.local/lib/uhd-4.5/bin/uhd_image_loader --args "type=x300,addr=192.168.10.2" --fpga-path usrp_x310_fpga_HG.bit
