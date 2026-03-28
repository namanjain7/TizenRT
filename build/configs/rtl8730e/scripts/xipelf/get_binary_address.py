#!/usr/bin/env python
###########################################################################
#
# Copyright 2026 Samsung Electronics All Rights Reserved.
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

os_folder = os.path.dirname(__file__) + '/../../../../../os'
cfg_file = os_folder + '/.config'

# Get CONFIG_FLASH_VSTART_LOADABLE from config file
tools_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../os/tools'))
sys.path.append(tools_path)
import config_util as util

CONFIG_FLASH_VSTART_LOADABLE = util.get_value_from_file(cfg_file, "CONFIG_FLASH_VSTART_LOADABLE=").rstrip('\n')
CONFIG_USER_SIGN_PREPEND_SIZE = util.get_value_from_file(cfg_file, "CONFIG_USER_SIGN_PREPEND_SIZE=").rstrip('\n')

if CONFIG_USER_SIGN_PREPEND_SIZE == 'None' :
    signing_offset = 0
else :
    signing_offset = int(CONFIG_USER_SIGN_PREPEND_SIZE)

PARTITION_SIZE_LIST = util.get_value_from_file(cfg_file, "CONFIG_FLASH_PART_SIZE=").rstrip('\n').strip('"').rstrip(',')
PARTITION_NAME_LIST = util.get_value_from_file(cfg_file, "CONFIG_FLASH_PART_NAME=").rstrip('\n').strip('"').rstrip(',')

if PARTITION_SIZE_LIST == 'None' :
    sys.exit(0)

NAME_LIST = PARTITION_NAME_LIST.split(",")
SIZE_LIST = PARTITION_SIZE_LIST.split(",")

def get_flash_offset():
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../tools/amebasmart/gnu_utility')))
    from loadable_xip_elf import get_offset
    offset_shift = get_offset()
    offset = int(CONFIG_FLASH_VSTART_LOADABLE, 16) - int(offset_shift, 16)
    return offset

offset = get_flash_offset()

first_loadable_encountered = False


def get_flash_address(binary_name):
    global offset
    """
    Get flash address for specified binary (common/app1/app2) in dual OTA mode.
    For rtl8730e, offset starts from base address and accumulates through partitions.
    Returns FLASH_ADD and FLASH_SIZE for the specified binary.
    """
    index = 0
    ota_index = 1

    current_offset = offset
    
    for name in NAME_LIST:
        part_size = int(SIZE_LIST[index]) * 1024
        
        if name == "kernel":
            ota_index = (ota_index + 1) % 2
        elif name == binary_name:
            if binary_name == "common":
                binary_start = hex(current_offset + 0x10 + signing_offset)
                binary_size = hex(part_size - 0x10 - signing_offset)
            else:
                binary_start = hex(current_offset + 0x30 + signing_offset)
                binary_size = hex(part_size - 0x30 - signing_offset)
            
            print("FLASH_ADD={}".format(binary_start))
            print("FLASH_SIZE={}".format(binary_size))
            return

        current_offset = current_offset + part_size
        index += 1
