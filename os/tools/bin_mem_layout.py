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
import json
import config_util as util
import importlib.util
import argparse

os_folder = os.path.dirname(os.path.abspath(__file__)) + '/..'
cfg_file = os.path.join(os_folder, '.config')
build_folder = os.path.join(os_folder, '..', 'build')
output_folder = os.path.join(build_folder, 'output', 'bin')
cache_file = os.path.join(output_folder, 'mem_layout_cache.json')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import config_util as util

def ensure_output_dir():
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)

def load_cache():
    if os.path.exists(cache_file):
        try:
            with open(cache_file, 'r') as f:
                return json.load(f)
        except (json.JSONDecodeError, IOError):
            pass
    return {"configs": {}, "memory_layout": {}}

def save_cache(cache_data):
    ensure_output_dir()
    with open(cache_file, 'w') as f:
        json.dump(cache_data, f, indent=2)

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
    cache = load_cache()
    vendor_script_path = os.path.join(build_folder, 'configs', cache['configs']['CONFIG_ARCH_BOARD'], 'scripts', 'xipelf', 'flash_offset.py')
    
    if vendor_script_path and os.path.exists(vendor_script_path):
        spec = importlib.util.spec_from_file_location("vendor_module", vendor_script_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    
    return None

def calculate_memory_layout(configs):

    vendor_module = get_vendor_module()

    offset = vendor_module.get_flash_offset(configs)   # will return flash_offset which is differnt for each board TODO in board specific file

    ram_start = int(configs['CONFIG_RAM_START'], 16)
    ram_size = int(configs['CONFIG_RAM_SIZE'])
    ram_end = ram_start + ram_size

    if ram_end % 4096 != 0:
        print("RAM end should be 4KB aligned", file=sys.stderr)
        sys.exit(1)

    common_ram_size = int(configs['CONFIG_COMMON_BIN_STATIC_RAMSIZE']) - 64 * 1024
    app1_ram_size = int(configs['CONFIG_APP1_BIN_DYN_RAMSIZE'])
    app2_ram_size_val = configs.get('CONFIG_APP2_BIN_DYN_RAMSIZE')
    if app2_ram_size_val and app2_ram_size_val != 'None':
        app2_ram_size = int(app2_ram_size_val)
    else:
        app2_ram_size = app1_ram_size
    
    common_ram_start = ram_end - int(configs['CONFIG_COMMON_BIN_STATIC_RAMSIZE'])
    app1_ram_start = common_ram_start - app1_ram_size
    app2_ram_start = app1_ram_start - app2_ram_size

    name_list = configs['CONFIG_FLASH_PART_NAME'].split(",") if configs['CONFIG_FLASH_PART_NAME'] else []
    size_list = configs['CONFIG_FLASH_PART_SIZE'].split(",") if configs['CONFIG_FLASH_PART_SIZE'] else []
    
    signing_offset = 0
    if configs.get('CONFIG_USER_SIGN_PREPEND_SIZE') and configs['CONFIG_USER_SIGN_PREPEND_SIZE'] != 'None':
        signing_offset = int(configs['CONFIG_USER_SIGN_PREPEND_SIZE'])
    
    memory_layout = {
        "common": {},
        "app1": {"0": {}, "1": {}},
        "app2": {"0": {}, "1": {}}
    }

    ota_index = 0
    current_offset = offset
    change_ota_index = False

    for i, name in enumerate(name_list):
        name = name.strip()
        if i >= len(size_list):
            break
        part_size = int(size_list[i].strip()) * 1024
        print("part_name: " + str(name) + "  part size:  " + str(part_size) + "offset value: " + str(hex(current_offset)))

        if change_ota_index and name == "kernel":
            ota_index = (ota_index + 1) % 2

        if name == "kernel":
            change_ota_index = True
            continue

        elif name in ("common", "app1", "app2"):
            if name == "common":
                flash_start = current_offset + 0x10 + signing_offset
                flash_size = part_size - 0x10 - signing_offset
                ram_start_val = common_ram_start
                ram_size_val = common_ram_size
                print("flash_start in common: " + hex(flash_start))
            else:
                flash_start = current_offset + 0x30 + signing_offset
                flash_size = part_size - 0x30 - signing_offset
                print("flash_start in app1: " + hex(flash_start))
                if name == "app1":
                    ram_start_val = app1_ram_start
                    ram_size_val = app1_ram_size
                else:
                    ram_start_val = app2_ram_start
                    ram_size_val = app2_ram_size

            if current_offset % 4096 != 0:
                print(f"ERROR: flash start [{hex(current_offset)}] of {name} should be aligned to 4KB", file=sys.stderr)
                sys.exit(1)
            if part_size % 4096 != 0:
                print(f"ERROR: flash partition size [{part_size}] of {name} should be aligned to 4KB", file=sys.stderr)
                sys.exit(1)

            layout_entry = {
                "ram_start": hex(ram_start_val),
                "ram_size": hex(ram_size_val),
                "flash_start": hex(flash_start),
                "flash_size": hex(flash_size)
            }

            memory_layout[name][str(ota_index)] = layout_entry
        else:
            continue

        print("Update offset now" + str(hex(current_offset)) + " " + str(hex(part_size)))
        current_offset += part_size

    return memory_layout

def get_memory_layout(binary_name, ota_index):
    cache = load_cache()

    if not cache.get("configs"):
        cache["configs"] = load_configs_from_file()
        save_cache(cache)

    if not cache.get("memory_layout"):
        cache["memory_layout"] = calculate_memory_layout(cache["configs"])
        save_cache(cache)

    vendor_module = get_vendor_module()
    ota_index = vendor_module.get_ota_index()

    memory_layout = cache["memory_layout"]
    
    if binary_name == "common":
        if not memory_layout.get("common"):
            memory_layout = calculate_memory_layout(cache["configs"])
            cache["memory_layout"] = memory_layout
            save_cache(cache)
        return memory_layout.get("common", {})
    else:
        if not memory_layout.get(binary_name, {}).get(str(ota_index)):
            memory_layout = calculate_memory_layout(cache["configs"])
            cache["memory_layout"] = memory_layout
            save_cache(cache)
        return memory_layout.get(binary_name, {}).get(str(ota_index), {})

def main():
    parser = argparse.ArgumentParser(description='Get memory layout for binaries')
    parser.add_argument('--binary-name', required=True, choices=['common', 'app1', 'app2'],
                        help='Binary name (common, app1, or app2)')
    parser.add_argument('--ota-index', type=int, default=0, choices=[0, 1],
                        help='OTA index (0 or 1, default: 0)')
    
    args = parser.parse_args()
    
    layout = get_memory_layout(args.binary_name, args.ota_index)
    
    if layout:
        print(f"RAM_START={layout.get('ram_start', '0x0')}")
        print(f"RAM_SIZE={layout.get('ram_size', '0x0')}")
        print(f"FLASH_START={layout.get('flash_start', '0x0')}")
        print(f"FLASH_SIZE={layout.get('flash_size', '0x0')}")
    else:
        print("RAM_START=0x0", file=sys.stdout)
        print("RAM_SIZE=0x0", file=sys.stdout)
        print("FLASH_START=0x0", file=sys.stdout)
        print("FLASH_SIZE=0x0", file=sys.stdout)
        sys.exit(1)


if __name__ == "__main__":
    main()