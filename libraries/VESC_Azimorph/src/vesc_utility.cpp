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
 #include <math.h>
 #include <stdbool.h>
 #include "vesc_utility.h"
  
 void VescUtility::utility_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
 }
  
 void VescUtility::utility_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) {
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
 }
  
 void VescUtility::utility_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
 }
  
 void VescUtility::utility_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) {
   buffer[(*index)++] = number >> 24;
   buffer[(*index)++] = number >> 16;
   buffer[(*index)++] = number >> 8;
   buffer[(*index)++] = number;
 }
  
 void VescUtility::utility_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
     VescUtility::utility_append_int16(buffer, (int16_t)(number * scale), index);
 }
  
 void VescUtility::utility_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
     VescUtility::utility_append_int32(buffer, (int32_t)(number * scale), index);
 }
  
 void VescUtility::utility_append_float32_auto(uint8_t* buffer, float number, int32_t *index) {
   int e = 0;
   float sig = frexpf(number, &e);
   float sig_abs = fabsf(sig);
   uint32_t sig_i = 0;
  
   if (sig_abs >= 0.5) {
     sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
     e += 126;
   }
  
   uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
   if (sig < 0) {
     res |= 1U << 31;
   }
  
   VescUtility::utility_append_uint32(buffer, res, index);
 }
  
 int16_t VescUtility::utility_get_int16(const uint8_t *buffer, int32_t *index) {
   int16_t res = ((uint16_t) buffer[*index]) << 8 |
           ((uint16_t) buffer[*index + 1]);
   *index += 2;
   return res;
 }
  
 uint16_t VescUtility::utility_get_uint16(const uint8_t *buffer, int32_t *index) {
   uint16_t res =  ((uint16_t) buffer[*index]) << 8 |
           ((uint16_t) buffer[*index + 1]);
   *index += 2;
   return res;
 }
  
 int32_t VescUtility::utility_get_int32(const uint8_t *buffer, int32_t *index) {
   int32_t res = ((uint32_t) buffer[*index]) << 24 |
           ((uint32_t) buffer[*index + 1]) << 16 |
           ((uint32_t) buffer[*index + 2]) << 8 |
           ((uint32_t) buffer[*index + 3]);
   *index += 4;
   return res;
 }
  
 uint32_t VescUtility::utility_get_uint32(const uint8_t *buffer, int32_t *index) {
   uint32_t res =  ((uint32_t) buffer[*index]) << 24 |
           ((uint32_t) buffer[*index + 1]) << 16 |
           ((uint32_t) buffer[*index + 2]) << 8 |
           ((uint32_t) buffer[*index + 3]);
   *index += 4;
   return res;
 }
  
 float VescUtility::utility_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
     return VescUtility::utility_get_int16(buffer, index) / scale;
 }
  
 float VescUtility::utility_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
     return VescUtility::utility_get_int32(buffer, index) / scale;
 }
  
 float VescUtility::utility_get_float32_auto(const uint8_t *buffer, int32_t *index) {
   uint32_t res = VescUtility::utility_get_uint32(buffer, index);
  
   int e = (res >> 23) & 0xFF;
   uint32_t sig_i = res & 0x7FFFFF;
   bool neg = res & (1U << 31);
  
   float sig = 0.0;
   if (e != 0 || sig_i != 0) {
     sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
     e -= 126;
   }
  
   if (neg) {
     sig = -sig;
   }
  
   return ldexpf(sig, e);
 }
