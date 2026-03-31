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
import subprocess

os_folder = os.path.dirname(__file__) + '/../../../../../os'
cfg_file = os_folder + '/.config'

CONFIG_FLASH_VSTART_LOADABLE = None
first_loadable_encountered = False

tools_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../os/tools'))
sys.path.append(tools_path)
import config_util as util

PARTITION_SIZE_LIST = util.get_value_from_file(cfg_file, "CONFIG_FLASH_PART_SIZE=").rstrip('\n').strip('"').rstrip(',')
PARTITION_NAME_LIST = util.get_value_from_file(cfg_file, "CONFIG_FLASH_PART_NAME=").rstrip('\n').strip('"').rstrip(',')

CONFIG_APP_BINARY_SEPARATION = util.get_value_from_file(cfg_file, "CONFIG_APP_BINARY_SEPARATION=").rstrip('\n')
CONFIG_SUPPORT_COMMON_BINARY = util.get_value_from_file(cfg_file, "CONFIG_SUPPORT_COMMON_BINARY=").rstrip('\n')
CONFIG_USE_BP = util.get_value_from_file(cfg_file, "CONFIG_USE_BP=").rstrip('\n')
CONFIG_NUM_APPS = util.get_value_from_file(cfg_file, "CONFIG_NUM_APPS=").rstrip('\n')

if PARTITION_SIZE_LIST == 'None' :
    sys.exit(0)

NAME_LIST = PARTITION_NAME_LIST.split(",")
SIZE_LIST = PARTITION_SIZE_LIST.split(",")

CONFIG_USER_SIGN_PREPEND_SIZE = util.get_value_from_file(cfg_file, "CONFIG_USER_SIGN_PREPEND_SIZE=").rstrip('\n')
if CONFIG_USER_SIGN_PREPEND_SIZE == 'None' :
    signing_offset = 0
else :
    signing_offset = int(CONFIG_USER_SIGN_PREPEND_SIZE)

def compute_flash_vstart_loadable():
    global CONFIG_FLASH_VSTART_LOADABLE
    _vstart_script = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../get_flash_vstart_loadable.py'))
    computed = subprocess.check_output([sys.executable, _vstart_script, os.path.abspath(cfg_file)]).decode('utf-8').strip()
    if computed:
        CONFIG_FLASH_VSTART_LOADABLE = computed
    else:
        CONFIG_FLASH_VSTART_LOADABLE = util.get_value_from_file(cfg_file, "CONFIG_FLASH_VSTART_LOADABLE=").rstrip('\n')

def get_flash_offset():
    global CONFIG_FLASH_VSTART_LOADABLE
    if CONFIG_FLASH_VSTART_LOADABLE is None:
        compute_flash_vstart_loadable()
    return int(CONFIG_FLASH_VSTART_LOADABLE, 16)

offset = get_flash_offset()

def get_flash_address(binary_name):
    global offset
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
