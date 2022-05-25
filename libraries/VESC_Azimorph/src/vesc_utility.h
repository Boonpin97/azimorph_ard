/* 
  * This file is part of the arduino vesc library (https://github.com/arduino_vesc)
  * 
  * Copyright (c) 2019 Gianmarco Vitelli
  * 
  * This program is free software: you can redistribute it and/or modify  
  * it under the terms of the GNU General Public License as published by  
  * the Free Software Foundation, version 3.
  *
  * This program is distributed in the hope that it will be useful, but 
  * WITHOUT ANY WARRANTY; without even the implied warranty of 
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
  * General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License 
  * along with this program. If not, see <http://www.gnu.org/licenses/>.
  */
 #include <Arduino.h>
  
 class VescUtility {
     public:
     static void utility_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
     static void utility_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
     static void utility_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
     static void utility_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
     static void utility_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
     static void utility_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
     static void utility_append_float32_auto(uint8_t* buffer, float number, int32_t *index);
     static int16_t utility_get_int16(const uint8_t *buffer, int32_t *index);
     static uint16_t utility_get_uint16(const uint8_t *buffer, int32_t *index);
     static int32_t utility_get_int32(const uint8_t *buffer, int32_t *index);
     static uint32_t utility_get_uint32(const uint8_t *buffer, int32_t *index);
     static float utility_get_float16(const uint8_t *buffer, float scale, int32_t *index);
     static float utility_get_float32(const uint8_t *buffer, float scale, int32_t *index);
     static float utility_get_float32_auto(const uint8_t *buffer, int32_t *index);
 };