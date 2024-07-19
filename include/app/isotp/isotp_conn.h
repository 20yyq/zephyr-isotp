/*
 * @Author       : Eacher
 * @Date         : 2024-07-18 15:43:09
 * @LastEditTime : 2024-07-19 10:51:46
 * @LastEditors  : Eacher
 * --------------------------------------------------------------------------------<
 * @Description  : 
 * --------------------------------------------------------------------------------<
 * @FilePath     : /zephyrproject/veryark/application/include/app/isotp/isotp_conn.h
 */
#ifndef __VERYARK_ISOTP_CONN_H_
#define __VERYARK_ISOTP_CONN_H_

#include "isotp.h"

#define ISOTP_CAN_BITRATE        125000

int __init_can(void);
const struct device *get_can_dev(void);
int can_frame_send(uint8_t *data, uint8_t len);

#endif
