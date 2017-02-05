/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/
#include "mma8451.h"
#include "fsl_i2c_hal.h"

/*==================[macros and definitions]=================================*/
#define MMA8451_I2C_ADDRESS     (0x1d)

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/
static uint8_t mma8451_read_reg(uint8_t addr)
{
    uint8_t ret;
    	
    I2C_HAL_MasterReceiveDataPolling(I2C0,
                    MMA8451_I2C_ADDRESS,
                    &addr,
                    1,
                    &ret,
                    1);
	
	return ret;
}

void mma8451_write_reg(uint8_t addr, uint8_t data)
{
	I2C_HAL_MasterSendDataPolling(I2C0,
	                MMA8451_I2C_ADDRESS,
	                &addr,
	                1,
	                &data,
	                1);
}

/* TODO: verificar esto */
// this delay is very important, it may cause w-r operation failure.
static void pause(void)
{
    int n,m = 0;
    for(n=0; n<40; n++)
    {
        m++;
    }
}

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
void mma8451_init(void)
{
	mma8451_write_reg(0x2a,0b00101101);
	
	/* TODO: verificar */
	pause();
}


int16_t iAcclReadX(void)
{
	int16_t readG = 0;

    if ((mma8451_read_reg(0x00) & 0x0f) != 0)
    {
    	readG   = mma8451_read_reg(0x01)<<8;
    	readG  |= mma8451_read_reg(0x02);

    	readG = readG >> 2;
    }
    return (int16_t)(((int32_t)readG * 100) / (int32_t)4096);
}

int16_t iAcclReadY(void)
{
	int16_t readG = 0;

    if ((mma8451_read_reg(0x00) & 0x0f) != 0)
    {
    	readG   = mma8451_read_reg(0x03)<<8;
    	readG  |= mma8451_read_reg(0x04);

    	readG = readG >> 2;
    }
    return (int16_t)(((int32_t)readG * 100) / (int32_t)4096);
}

int16_t iAcclReadZ(void) 
{
	int16_t readG = 0;
	
    if ((mma8451_read_reg(0x00) & 0x0f) != 0)
    {
    	readG   = mma8451_read_reg(0x05)<<8;
    	readG  |= mma8451_read_reg(0x06);
    	
    	readG = readG >> 2;
    }
    return (int16_t)(((int32_t)readG * 100) / (int32_t)4096);
}

/*==================[end of file]============================================*/
