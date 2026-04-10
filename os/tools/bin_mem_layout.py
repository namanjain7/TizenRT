#!/usr/bin/env python3
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
import json
import argparse

os_folder = os.path.dirname(os.path.abspath(__file__)) + '/..'
cfg_file = os.path.join(os_folder, '.config')
build_folder = os.path.join(os_folder, '..', 'build')
output_folder = os.path.join(build_folder, 'output', 'bin')
saved_config_file_path = os.path.join(output_folder, 'mem_layout_cache.json')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config_util as util

def ensure_output_dir():
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

def load_parameters():
    if os.path.exists(saved_config_file_path):
        try:
            with open(saved_config_file_path, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            pass
    return {"configs": {}, "memory_layout": {}}

def save_parameters(data):
    ensure_output_dir()
    with open(saved_config_file_path, 'w') as f:
        json.dump(data, f, indent=2)

def save_ram_end_json(ram_end):
    with open(saved_config_file_path, 'r') as f:
        data = json.load(f)
    data['configs']['ram_end'] = str(ram_end)
    with open(saved_config_file_path, 'w') as f:
        json.dump(data, f, indent=2)

def load_configs_from_file():
    configs = {}
    
    configs['CONFIG_ARCH_BOARD'] = util.get_value_from_file(cfg_file, "CONFIG_ARCH_BOARD=").rstrip('\n').strip('"')
    configs['CONFIG_TRPK_CONTAINS_MULTIPLE_BINARY'] = util.get_value_from_file(cfg_file, "CONFIG_TRPK_CONTAINS_MULTIPLE_BINARY=").rstrip('\n')
    configs['CONFIG_FLASH_VSTART_LOADABLE'] = util.get_value_from_file(cfg_file, "CONFIG_FLASH_VSTART_LOADABLE=").rstrip('\n')
    configs['CONFIG_RAM_START'] = util.get_value_from_file(cfg_file, "CONFIG_RAM_START=").rstrip('\n')
    configs['CONFIG_RAM_SIZE'] = util.get_value_from_file(cfg_file, "CONFIG_RAM_SIZE=").rstrip('\n')
    configs['CONFIG_COMMON_BIN_STATIC_RAMSIZE'] = util.get_value_from_file(cfg_file, "CONFIG_COMMON_BIN_STATIC_RAMSIZE=").rstrip('\n')
    configs['CONFIG_APP1_BIN_DYN_RAMSIZE'] = util.get_value_from_file(cfg_file, "CONFIG_APP1_BIN_DYN_RAMSIZE=").rstrip('\n')
    configs['CONFIG_APP2_BIN_DYN_RAMSIZE'] = util.get_value_from_file(cfg_file, "CONFIG_APP2_BIN_DYN_RAMSIZE=").rstrip('\n')
    configs['CONFIG_USER_SIGN_PREPEND_SIZE'] = util.get_value_from_file(cfg_file, "CONFIG_USER_SIGN_PREPEND_SIZE=").rstrip('\n')
    configs['CONFIG_FLASH_PART_SIZE'] = util.get_value_from_file(cfg_file, "CONFIG_FLASH_PART_SIZE=").rstrip('\n').strip('"').rstrip(',')
    configs['CONFIG_FLASH_PART_NAME'] = util.get_value_from_file(cfg_file, "CONFIG_FLASH_PART_NAME=").rstrip('\n').strip('"').rstrip(',')
    configs['CONFIG_APP_BINARY_SEPARATION'] = util.get_value_from_file(cfg_file, "CONFIG_APP_BINARY_SEPARATION=").rstrip('\n')
    configs['CONFIG_SUPPORT_COMMON_BINARY'] = util.get_value_from_file(cfg_file, "CONFIG_SUPPORT_COMMON_BINARY=").rstrip('\n')
    configs['CONFIG_NUM_APPS'] = util.get_value_from_file(cfg_file, "CONFIG_NUM_APPS=").rstrip('\n')
    
    return configs

def get_vendor_module():
    config_parameters = load_parameters()
    vendor_script_path = os.path.join(build_folder, 'configs', config_parameters['configs']['CONFIG_ARCH_BOARD'], 'scripts', 'xipelf', 'flash_offset.py')
    
    if vendor_script_path and os.path.exists(vendor_script_path):
        spec = importlib.util.spec_from_file_location("vendor_module", vendor_script_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    
    return None

def calculate_ram_start_address(configs, bin_name):
    if 'ram_end' not in configs:
        ram_start = int(configs['CONFIG_RAM_START'], 16)
        ram_size = int(configs['CONFIG_RAM_SIZE'])
        ram_end = ram_start + ram_size
    else:
        ram_end = configs['ram_end']

    ram_start = int(ram_end) - int(configs[bin_name])
    save_ram_end_json(ram_start)

    return ram_start

def get_memory_layout(binary_name, ota_index):
    config_parameters = load_parameters()

    if not config_parameters.get("configs"):
        config_parameters["configs"] = load_configs_from_file()
        save_parameters(config_parameters)

    ##### Remove this block of code when flash address logic is added ################
    if binary_name == "common":
        binary_flash_start = 0x1226d010
        binary_flash_size = 0x351ff0
    elif binary_name == "app1":
        binary_flash_start = 0x125bf030
        binary_flash_size = 0xfffd0
    ##################################################################################

    if (binary_name == 'common'):
        binary_ram_size = int(config_parameters["configs"]['CONFIG_COMMON_BIN_STATIC_RAMSIZE']) - 64 * 1024
        ram_start = calculate_ram_start_address(config_parameters["configs"], "CONFIG_COMMON_BIN_STATIC_RAMSIZE")
    elif (binary_name == 'app1'):
        binary_ram_size = int(config_parameters["configs"]['CONFIG_APP1_BIN_DYN_RAMSIZE'])
        ram_start = calculate_ram_start_address(config_parameters["configs"], "CONFIG_APP1_BIN_DYN_RAMSIZE")
    else:   # app2 case
        binary_ram_size = int(config_parameters["configs"]['CONFIG_APP2_BIN_DYN_RAMSIZE'])
        ram_start = calculate_ram_start_address(config_parameters["configs"], "CONFIG_APP2_BIN_DYN_RAMSIZE")
    

    print("FLASH_ADD=" + str(hex(binary_flash_start)))
    print("FLASH_SIZE=" + str(hex(binary_flash_size)))
    print("RAM_ADD=" + str(hex(ram_start)))
    print("RAM_SIZE=" + str(hex(binary_ram_size)))
    return
    
    


    if not config_parameters.get("memory_layout"):
        config_parameters["memory_layout"] = calculate_memory_layout(config_parameters["configs"])
        save_parameters(config_parameters)

    vendor_module = get_vendor_module()
    ota_index = vendor_module.get_ota_index()

    memory_layout = config_parameters["memory_layout"]
    
    if binary_name == "common":
        if not memory_layout.get("common"):
            memory_layout = calculate_memory_layout(config_parameters["configs"])
            config_parameters["memory_layout"] = memory_layout
            save_parameters(config_parameters)
        return memory_layout.get("common", {})
    else:
        if not memory_layout.get(binary_name, {}).get(str(ota_index)):
            memory_layout = calculate_memory_layout(config_parameters["configs"])
            config_parameters["memory_layout"] = memory_layout
            save_parameters(config_parameters)
        return memory_layout.get(binary_name, {}).get(str(ota_index), {})

def main():
    parser = argparse.ArgumentParser(description='Get memory layout for binaries')
    parser.add_argument('--binary-name', required=True, choices=['common', 'app1', 'app2'],
                        help='Binary name (common, app1, or app2)')
    parser.add_argument('--ota-index', type=int, default=0, choices=[0, 1],
                        help='OTA index (0 or 1, default: 0)')
    
    args = parser.parse_args()
    
    layout = get_memory_layout(args.binary_name, args.ota_index)
    
    return

if __name__ == "__main__":
    main()
