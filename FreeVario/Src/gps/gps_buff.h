/**	
 * \file            gps_buff.h
 * \brief           Buffer manager
 */
 
/*
 * Copyright (c) 2018 Tilen Majerle
 *  
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software, 
 * and to permit persons to whom the Software is furnished to do so, 
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of GPS NMEA parser.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 */
#ifndef GPS_HDR_BUFF_H
#define GPS_HDR_BUFF_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "stddef.h"
#include "stdint.h"
#include "string.h"

/**
 * \addtogroup      GPS_NMEA
 * \{
 */
 
/**
 * \defgroup        GPS_NMEA_BUFFER Ring buffer
 * \brief           Optional ring buffer for received GPS data
 * \{
 */

/**
 * \brief           GPS buffer structure
 */
typedef struct gps_buff {
    size_t size;                                /*!< Size of buffer in units of bytes */
    size_t in;                                  /*!< Input pointer to save next value */
    size_t out;                                 /*!< Output pointer to read next value */
    uint8_t* buff;                              /*!< Pointer to buffer data array */
    uint8_t flags;                              /*!< Flags for buffer */
} gps_buff_t;

uint8_t     gps_buff_init(gps_buff_t* buff, void* data, size_t len);
void        gps_buff_free(gps_buff_t* buff);
size_t      gps_buff_write(gps_buff_t* buff, const void* data, size_t count);
size_t      gps_buff_read(gps_buff_t* buff, void* data, size_t count);
size_t      gps_buff_get_free(gps_buff_t* buff);
size_t      gps_buff_get_full(gps_buff_t* buff);
void        gps_buff_reset(gps_buff_t* buff);
size_t      gps_buff_peek(gps_buff_t* buff, size_t skip_count, void* data, size_t count);
void *      gps_buff_get_linear_block_address(gps_buff_t* buff);
size_t      gps_buff_get_linear_block_length(gps_buff_t* buff);
size_t      gps_buff_skip(gps_buff_t* buff, size_t len);

/**
 * \}
 */

/**
 * \}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* GPS_HDR_BUFF_H */
