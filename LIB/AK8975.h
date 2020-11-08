#ifndef __AK8975_H
#define __AK8975_H

#include "iic.h"
#define AK8975_ADDR 0x0A

uint8_t ak8975_read_byte(uint8_t addr);

#endif

