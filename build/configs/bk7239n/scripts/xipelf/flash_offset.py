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

def get_flash_offset(configs):
    _vstart_script = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../build/configs/bk7239n/get_flash_vstart_loadable.py'))
    computed = subprocess.check_output([sys.executable, _vstart_script, os.path.abspath(cfg_file)]).decode('utf-8').strip()
    if computed:
        CONFIG_FLASH_VSTART_LOADABLE = computed
    else:
        CONFIG_FLASH_VSTART_LOADABLE = util.get_value_from_file(cfg_file, "CONFIG_FLASH_VSTART_LOADABLE=").rstrip('\n')

def get_ota_index():
    return 1
