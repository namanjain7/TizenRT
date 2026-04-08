#!/usr/bin/env python
###########################################################################
#
# Copyright 2024 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################

import os
import sys
import string
import subprocess
import config_util as util

os_folder = os.path.dirname(__file__) + '/..'
cfg_file = os_folder + '/.config'
tool_folder = os_folder + '/tools'
build_folder = os_folder + '/../build'
output_folder = build_folder + '/output/bin/'

# Create output directory if it doesnt exist
if not os.path.exists(output_folder):
    os.makedirs(output_folder)

path = "../../build/configs/" + board_name + "/scripts/xipelf/"
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), path)))

from get_binary_address import get_flash_address







get_flash_address(binary_type)  # Get flash address and size from board specific code

if binary_type == "common":
    print("RAM_ADD={}".format(common_ram_start))
    print("RAM_SIZE={}".format(hex(common_ram_size)))
elif binary_type == "app1":
    print("RAM_ADD={}".format(app1_ram_start))
    print("RAM_SIZE={}".format(app1_ram_size))
elif binary_type == "app2":
    print("RAM_ADD={}".format(app2_ram_start))
    print("RAM_SIZE={}".format(app2_ram_size))

class config_vals:
    flash_offset
    CONFIG_COMMON_BIN_STATIC_RAMSIZE
    CONFIG_APP1_BIN_DYN_RAMSIZE
    CONFIG_APP2_BIN_DYN_RAMSIZE
    board_name
    CONFIG_RAM_SIZE
    CONFIG_RAM_START

def get_config_vals():
    CONFIG_ARCH_BOARD = util.get_value_from_file(cfg_file, "CONFIG_ARCH_BOARD=").rstrip('\n')
    config_vals.board_name = CONFIG_ARCH_BOARD[1:-1]

    config_vals.CONFIG_RAM_SIZE = util.get_value_from_file(cfg_file, "CONFIG_RAM_SIZE=").rstrip('\n')
    config_vals.CONFIG_RAM_START = util.get_value_from_file(cfg_file, "CONFIG_RAM_START=").rstrip('\n')
    config_vals.CONFIG_APP1_BIN_DYN_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_APP1_BIN_DYN_RAMSIZE=").rstrip('\n')
    config_vals.CONFIG_COMMON_BIN_STATIC_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_COMMON_BIN_STATIC_RAMSIZE=").rstrip('\n')

    if util.check_config_existence(cfg_file, 'CONFIG_APP2_INFO') == True :
        CONFIG_APP2_BIN_DYN_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_APP2_BIN_DYN_RAMSIZE=").rstrip('\n')

def get_flash_bin_addr(binary_name, prev_flash_start_addr):


def get_ram_bin

def main(binary_name):
    get_config_vals()

    ram_end = int(config_vals.CONFIG_RAM_SIZE) + int(config_vals.CONFIG_RAM_START, 16)
    if ram_end % 4096 != 0 :
        print("RAM end should be 4KB aligned")
        sys.exit(1)

    ram_offset = ram_end
    ram_offset = ram_offset - int(config_vals.CONFIG_COMMON_BIN_STATIC_RAMSIZE)
    if ram_offset % 4096 != 0 :
        print("!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("CONFIG_COMMON_BIN_STATIC_RAMSIZE should be aligned to 4KB")
        sys.exit(1)

    common_ram_size = (int(config_vals.CONFIG_COMMON_BIN_STATIC_RAMSIZE) - 64 * 1024)
    common_ram_start = hex(ram_offset)

    ram_offset = ram_offset - int(config_vals.CONFIG_APP1_BIN_DYN_RAMSIZE)
    if ram_offset % 4096 != 0 :
        print("!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("CONFIG_APP1_BIN_DYN_RAMSIZE should be aligned to 4KB")
        sys.exit(1)

    app1_ram_start = hex(ram_offset)
    app1_ram_size = hex(int(config_vals.CONFIG_APP1_BIN_DYN_RAMSIZE))

    if util.check_config_existence(cfg_file, 'CONFIG_APP2_INFO') == True :
        CONFIG_APP2_BIN_DYN_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_APP2_BIN_DYN_RAMSIZE=").rstrip('\n')
        app2_ram_start = hex(ram_offset)
        app2_ram_size = hex(int(CONFIG_APP2_BIN_DYN_RAMSIZE))
        if ram_offset % 4096 != 0 :
            print("!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!")
            print("CONFIG_APP2_BIN_DYN_RAMSIZE should be aligned to 4KB")
            sys.exit(1)






    offset = get_flash_offset()
    if (binary_name == "common"):
        index = 0
        for name in NAME_LIST:
            if (name == "common"):
                part_size = int(SIZE_LIST[index]) * 1024
                break
            index += 1
        # For bk7239n, offset is already set to the common partition address by get_flash_vstart_loadable.py
        # No need to accumulate sizes - the offset is already at the correct location
        common_start = hex(offset + 0x10 + signing_offset)
        common_size = hex(part_size - 0x10 - signing_offset)
        print("FLASH_ADD={}".format(common_start))
        print("FLASH_SIZE={}".format(common_size))
    elif (binary_name == "app1" or binary_name == "app2"):
        index = 0
        current_offset = offset
        
        # Accumulate sizes starting from common partition to find app1/app2
        for name in NAME_LIST:
            part_size = int(SIZE_LIST[index]) * 1024
            
            if name == binary_name:
                app_start = hex(current_offset + 0x30 + signing_offset)
                app_size = hex(part_size - 0x30 - signing_offset)
                print("FLASH_ADD={}".format(app_start))
                print("FLASH_SIZE={}".format(app_size))
                return
            
            # Accumulate offset for all partitions starting from common
            if name in ('common', 'app1', 'app2'):
                current_offset = current_offset + part_size
            index += 1

if __name__ == "__main__":
    binary_type = sys.argv[1]
    main(binary_type)