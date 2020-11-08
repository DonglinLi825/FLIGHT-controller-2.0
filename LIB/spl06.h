#ifndef __SPL06_H
#define __SPL06_H
#include "delay.h"
#include "sys.h"
#include "SPI.h"

#define PRE_CS_LOW PBout(12)=0;
#define PRE_CS_HIGH PBout(12)=1;
#define PRESSURE_SENSOR     0
#define TEMPERATURE_SENSOR  1
/*SPL06校准数据*/
typedef struct 
{
	int16_t c0;		int16_t c1;
	int32_t c00;	int32_t c10;	int16_t c01;	int16_t c11;
	int16_t c20;	int16_t c21;	
	int16_t c30;
}__attribute__((__packed__))spl06_coefficients_t;
typedef spl06_coefficients_t* spl06_coefficients_p;
typedef struct
{
	uint32_t rawPressure:24;
	uint32_t rawTemperature:24;
	uint32_t pressure:24;
	uint32_t temperature:24;
}__attribute__((__packed__))spl06_data_t;
typedef spl06_data_t* spl06_data_p;
typedef struct 
{
	float pressure;
	float temperature;
	float high;
}__attribute__((__packed__))presData_t;


void spl06_init(void);
void spl06_write_byte(uint8_t reg,uint8_t val);
uint8_t spl06_read_byte(uint8_t addr);
void spl06_read(uint8_t addr,uint8_t* buffer,uint8_t size);
void spl06_dataRead(spl06_data_p data_p);
void spl06_coefficientsRead(spl06_coefficients_p coefficients);
void spl06_getPresData(presData_t* presData);
void spl0601_rateSet ( u8 sensor, u8 smpRate, u8 overSmpRate );

#endif


