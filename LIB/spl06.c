#include "spl06.h"

uint32_t K_PRESSURE =516096;
uint32_t K_TEMPERATURE=1572864;

spl06_coefficients_t spl06_coefficients;
void spl06_init()
{
	
	spl06_write_byte(0x0c,(1<7)|(0x09));				//软件复位
	delay_ms(100);
	spl0601_rateSet ( PRESSURE_SENSOR, 128, 16 );
	delay_ms(1);
    spl0601_rateSet ( TEMPERATURE_SENSOR, 8, 8 );
	delay_ms(1);
	spl06_write_byte( 0x08 , 0x07 );						//温度气压连续测量
	delay_us(10);	
	spl06_coefficientsRead(&spl06_coefficients);	
//	uint8_t temp=0;
//	spl06_write_byte(0x0c,(1<7)|(0x09));				//软件复位
//	delay_ms(40);
//	spl06_write_byte(0x06,0x65);						//pressure 64 samples per sec , 32 times over sampling		
//	delay_us(10);
//	spl06_write_byte(0x07,0x71);						//temperature 128 samples per sec , 2 times over sampling
//	delay_us(10);
//	spl06_write_byte(0x08,0x07);						//温度气压连续测量
//	delay_us(10);
//	temp=spl06_read_byte(0x09);							//读取配置寄存器
//	delay_us(10);
//	spl06_write_byte(0x09,temp|(1<<2));					//配置寄存器气压计P_SHIFT为1
//	delay_ms(10);
//	spl06_coefficientsRead(&spl06_coefficients);
	
}


/*				spl06向地址内写入一字节数据				*/
void spl06_write_byte(uint8_t reg,uint8_t val)
{	
//	reg&=0x7F;	
	PRE_CS_LOW;
	SPI2_ReadWriteOneByte(reg);
	SPI2_ReadWriteOneByte(val);
	PRE_CS_HIGH;
}
/*				spl06读取一个字节数据					*/
uint8_t spl06_read_byte(uint8_t addr)
{
	uint8_t result=0;
	PRE_CS_LOW;
	SPI2_ReadWriteOneByte(addr|0x80);
	result=SPI2_ReadWriteOneByte(0xff);
	PRE_CS_HIGH;
	return result;
}
/*				spl06读取连续地址数据					*/
void spl06_read(uint8_t addr,uint8_t* buffer,uint8_t size)
{
	addr|=0x80;	
	PRE_CS_LOW;
	SPI2_ReadWriteOneByte(addr);
	for(int i=0;i<size;i++)
		buffer[i]=SPI2_ReadWriteOneByte(0xff);
	PRE_CS_HIGH;
	
}
/*				读取SPL06传感器原始数据					*/
/*				因为数据是大端，需要小端转换			*/
void spl06_dataRead(spl06_data_p data_p)
{
	spl06_read(0x00,(uint8_t*)data_p ,sizeof(spl06_data_t)/2);
	
	data_p->pressure=__REV( (uint32_t) data_p->rawPressure)>>8;
	data_p->pressure=(data_p->pressure&0x800000 )?(0xFF000000|data_p->pressure):data_p->pressure;
	data_p->temperature=__REV( (uint32_t) data_p->rawTemperature)>>8;		
	data_p->temperature=(data_p->temperature&0x800000 )?(0xFF000000|data_p->temperature):data_p->temperature;
	
}
/*				读取SPL06传感器定标系数					*/
void spl06_coefficientsRead(spl06_coefficients_p coefficients)
{
	uint8_t rxBuf[18];
	spl06_read(0x10,rxBuf ,18);
	
	coefficients->c0 = ( rxBuf[0] << 4 ) | ( rxBuf[1] >> 4 );
	coefficients->c0 = ( coefficients->c0 & 0x0800 ) ? (0xF000|coefficients->c0) : coefficients->c0;
	coefficients->c1 = ( (rxBuf[1] & 0xf) << 8 ) | ( rxBuf[2] );
	coefficients->c1 = ( coefficients->c1 & 0x0800 ) ? (0xF000|coefficients->c1) : coefficients->c1;
	coefficients->c00 = ( rxBuf[3] << 12 ) | ( rxBuf[4] << 4 ) | ( rxBuf[5] >> 4 );
	coefficients->c00 = ( coefficients->c00 & 0x080000 ) ? (0xFFF00000|coefficients->c00) : coefficients->c00;
	coefficients->c10 = ( (rxBuf[5] & 0xf) << 16 ) | ( rxBuf[6] << 8 ) | ( rxBuf[7] >> 0 );
	coefficients->c10 = ( coefficients->c10 & 0x080000 ) ? (0xFFF00000|coefficients->c10) : coefficients->c10;
	coefficients->c01 = ( rxBuf[8] << 8 ) | ( rxBuf[9] << 0 );
	coefficients->c11 = ( rxBuf[9] << 8 ) | ( rxBuf[11] << 0 );
	coefficients->c20 = ( rxBuf[12] << 8 ) | ( rxBuf[13] << 0 );
	coefficients->c21 = ( rxBuf[14] << 8 ) | ( rxBuf[15] << 0 );
	coefficients->c30 = ( rxBuf[16] << 8 ) | ( rxBuf[17] << 0 );	

}
/*				利用定标系数和相关公式计算出实际气压值
				和温度以及高度信息等					*/
void spl06_getPresData(presData_t* presData)
{
	spl06_data_t rawData;
	spl06_dataRead(&rawData);

	float tSc=(float)(rawData.temperature) / (float) K_TEMPERATURE;
	float pSc=(float)(rawData.pressure) / (float) K_PRESSURE;
	float qua2=spl06_coefficients.c10+pSc*(spl06_coefficients.c20+pSc*spl06_coefficients.c30);
	float qua3=tSc*pSc*(spl06_coefficients.c11+pSc*spl06_coefficients.c21);

	presData->pressure=spl06_coefficients.c00+pSc*qua2+tSc*spl06_coefficients.c01+qua3;
	presData->temperature=spl06_coefficients.c0*0.5f+spl06_coefficients.c1*tSc;
	
	float alt3=( 101400 - presData->pressure ) / 1000.0f;
	presData->high=0.82f * alt3 * alt3 * alt3 + 0.09f * ( 101400 - presData->pressure ) * 100.0f ;
}

void spl0601_rateSet ( u8 sensor, u8 smpRate, u8 overSmpRate )
{
    uint8_t reg = 0;
    int32_t Pkt = 0;
    switch ( smpRate )
    {
    case 2:
        reg |= ( 1 << 4 );
        break;
    case 4:
        reg |= ( 2 << 4 );
        break;
    case 8:
        reg |= ( 3 << 4 );
        break;
    case 16:
        reg |= ( 4 << 4 );
        break;
    case 32:
        reg |= ( 5 << 4 );
        break;
    case 64:
        reg |= ( 6 << 4 );
        break;
    case 128:
        reg |= ( 7 << 4 );
        break;
    case 1:
    default:
        break;
    }
    switch ( overSmpRate )
    {
    case 2:
        reg |= 1;
        Pkt = 1572864;
        break;
    case 4:
        reg |= 2;
        Pkt = 3670016;
        break;
    case 8:
        reg |= 3;
        Pkt = 7864320;
        break;
    case 16:
        Pkt = 253952;
        reg |= 4;
        break;
    case 32:
        Pkt = 516096;
        reg |= 5;
        break;
    case 64:
        Pkt = 1040384;
        reg |= 6;
        break;
    case 128:
        Pkt = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        Pkt = 524288;
        break;
    }

    if ( sensor == 0 )
    {
        K_PRESSURE = Pkt;
        spl06_write_byte(0x06,reg);
        if ( overSmpRate > 8)
        {
            reg = spl06_read_byte( 0x09 );
            spl06_write_byte( 0x09 , reg | 0x04);
        }
    }
    if(sensor==1)
    {
        K_TEMPERATURE=Pkt;
        spl06_write_byte (0x07,reg|0x80); //Using mems temperature
        if ( overSmpRate > 8 )
        {
            reg = spl06_read_byte( 0x09 );
            spl06_write_byte ( 0x09, reg | 0x08 );
        }
    }

}




