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

CONFIG_ARCH_BOARD = util.get_value_from_file(cfg_file, "CONFIG_ARCH_BOARD=").rstrip('\n')
board_name = CONFIG_ARCH_BOARD[1:-1]

path = "../../build/configs/" + board_name + "/scripts/xipelf/"
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), path)))

from get_binary_address import get_flash_address

CONFIG_RAM_SIZE = util.get_value_from_file(cfg_file, "CONFIG_RAM_SIZE=").rstrip('\n')
CONFIG_RAM_START = util.get_value_from_file(cfg_file, "CONFIG_RAM_START=").rstrip('\n')

ram_end = int(CONFIG_RAM_SIZE) + int(CONFIG_RAM_START, 16)

if ram_end % 4096 != 0 :
    print("RAM end should be 4KB aligned")
    sys.exit(1)

ram_offset = ram_end
ram_size = 0

CONFIG_APP1_BIN_DYN_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_APP1_BIN_DYN_RAMSIZE=").rstrip('\n')
if util.check_config_existence(cfg_file, 'CONFIG_APP2_INFO') == True :
    CONFIG_APP2_BIN_DYN_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_APP2_BIN_DYN_RAMSIZE=").rstrip('\n')
CONFIG_COMMON_BIN_STATIC_RAMSIZE=util.get_value_from_file(cfg_file, "CONFIG_COMMON_BIN_STATIC_RAMSIZE=").rstrip('\n')

ram_offset = ram_offset - int(CONFIG_COMMON_BIN_STATIC_RAMSIZE)
if ram_offset % 4096 != 0 :
    print("!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("CONFIG_COMMON_BIN_STATIC_RAMSIZE should be aligned to 4KB")
    sys.exit(1)

common_ram_size = (int(CONFIG_COMMON_BIN_STATIC_RAMSIZE) - 64 * 1024)
common_ram_start = hex(ram_offset)

ram_offset = ram_offset - int(CONFIG_APP1_BIN_DYN_RAMSIZE)
if ram_offset % 4096 != 0 :
    print("!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!")
    print("CONFIG_APP1_BIN_DYN_RAMSIZE should be aligned to 4KB")
    sys.exit(1)

app1_ram_start = hex(ram_offset)
app1_ram_size = hex(int(CONFIG_APP1_BIN_DYN_RAMSIZE))

if util.check_config_existence(cfg_file, 'CONFIG_APP2_INFO') == True :
    app2_ram_start = hex(ram_offset)
    app2_ram_size = hex(int(CONFIG_APP2_BIN_DYN_RAMSIZE))
    if ram_offset % 4096 != 0 :
        print("!!!!!!!!!!!!!!!!!!!!!! ERROR !!!!!!!!!!!!!!!!!!!!!!!!!!")
        print("CONFIG_APP2_BIN_DYN_RAMSIZE should be aligned to 4KB")
        sys.exit(1)

binary_type = sys.argv[1]

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
