/****************************************************************************
 *
 * Copyright 2025 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#ifndef __DRIVERS_LCD_COMMON_H
#define __DRIVERS_LCD_COMMON_H

#include <tinyara/lcd/lcd_dev.h>
#include <tinyara/lcd/mipi_lcd.h>

typedef enum lcd_mode_e {
        LCD_ON = 0,
        LCD_OFF = 1
} lcd_mode_t;

//typedef enum lcd_mode_e lcd_mode_t;

struct mipi_lcd_dev_s {
        /* Publicly visible device structure */

        struct lcd_dev_s dev;

        FAR struct mipi_dsi_device *dsi_dev;

        u8 *lcd_img_buffer[MAX_NO_PLANES];
        //u8 *BackupLcdImgBuffer;
        int fb_alloc_count;
        uint8_t power;
        lcd_mode_t lcdonoff;
        sem_t sem;
        struct mipi_lcd_config_s *config;
        bool backlight_power_inversion;
};

int send_init_cmd(struct mipi_lcd_dev_s *priv, lcm_setting_table_t *table);
int set_return_packet_len(struct mipi_lcd_dev_s *priv, u8 rx_len);
int read_response(struct mipi_lcd_dev_s *priv, lcm_setting_table_t command, u8 *rxbuf, u8 rx_len);

#endif

