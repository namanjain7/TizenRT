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

CONFIG_FLASH_VSTART_LOADABLE = 0
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
    _vstart_script = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../get_flash_vstart_loadable.py'))
    computed = subprocess.check_output([sys.executable, _vstart_script, os.path.abspath(cfg_file)]).decode('utf-8').strip()
    if computed:
        CONFIG_FLASH_VSTART_LOADABLE = computed
    else:
        CONFIG_FLASH_VSTART_LOADABLE = util.get_value_from_file(cfg_file, "CONFIG_FLASH_VSTART_LOADABLE=").rstrip('\n')

def get_flash_offset():
    compute_flash_vstart_loadable()
    return 0

offset = get_flash_offset()

def get_flash_address(binary_name):
    global offset
    if (binary_name == "common"):
        index = 0
        for name in NAME_LIST:
            part_size = int(SIZE_LIST[index]) * 1024
            if (name == "common"):
                
                break
            else:
                index += 1
                offset += part_size
        common_start = hex(offset + 0x10 + signing_offset)
        common_size = hex(part_size - 0x10 - signing_offset)
        print("FLASH_ADD={}".format(common_start))
        print("FLASH_SIZE={}".format(common_size))
