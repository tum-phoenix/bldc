/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_i2c.h"
#include "mcpwm.h"
#include "hw.h"

#define HANDLE I2C2

// ENUM for registers
enum {
    IDENTIFICATION = 0x00, // 0x23
    FIRMWARE_MAJOR = 0x01,
    FIRMWARE_MINOR = 0x02,
    // 0x03 is reserved
    
    // Tachometer: 32bit signed integer (big endian) [ticks]
    TACHOMETER_1 = 0x04,
    TACHOMETER_2 = 0x05,
    TACHOMETER_3 = 0x06,
    TACHOMETER_4 = 0x07,
    
    // RPH: 32bit signed integer (big endian) [motor revolutions per hour]
    RPH_1 = 0x08,
    RPH_2 = 0x09,
    RPH_3 = 0x10,
    RPH_4 = 0x11,
    
    // TODO: motor current (directional/nondirectional?)/total current
    
    // Switching frequency: 16bit unsigned integer (big endian) [Hz]
    SWITCHING_FREQUENCY_HI = 0x12,
    SWITCHING_FREQUENCY_LO = 0x13,
    
    
} i2c_slave_registers;

typedef struct {
    size_t addr; //!< current memory address
} i2c_slave_state;

//! I2C slave state persistence
i2c_slave_state state;

void app_i2c_init(void) {
    // Init I2C
    I2C_InitTypeDef i2c;
    i2c.I2C_ClockSpeed = 400000;
    i2c.I2C_Mode = I2C_Mode_I2C;
    i2c.I2C_DutyCycle = I2C_DutyCycle_2;
    i2c.I2C_OwnAddress1 = (0x42);
    i2c.I2C_Ack = I2C_Ack_Enable;
    i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    
    // ACK enable
    I2C_AcknowledgeConfig(HANDLE, ENABLE);

    // I2C enable
    I2C_Cmd(HANDLE, ENABLE);
    
    // Init I2C
    I2C_Init(HANDLE, &i2c);
    
    // Setup NVIC
    // EV-Interrupt
    nvicEnableVector(I2C2_EV_IRQn, STM32_CAN_CAN1_IRQ_PRIORITY+1); //! Interrupt prio 1 lower than CAN

    // ER-Interrupt
    nvicEnableVector(I2C2_ER_IRQn, STM32_CAN_CAN1_IRQ_PRIORITY+1); //! Interrupt prio 1 lower than CAN
}

uint8_t get_big_endian_byte_32(int32_t value, uint8_t byte)
{
    uint8_t* ptr = (uint8_t*)&value;
    return ptr[sizeof(value) - (byte+1)];
}

uint8_t get_byte(uint8_t addr)
{
    // TODO: actual data handling
    return addr + 10;
    
    
    uint8_t value = 0x00;
    switch(addr)
    {
        case IDENTIFICATION:
            value = 0x23;
            break;
        case FIRMWARE_MAJOR:
            value = FW_VERSION_MAJOR;
            break;
        case FIRMWARE_MINOR:
            value = FW_VERSION_MINOR;
            break;
        case TACHOMETER_1:
        case TACHOMETER_2:
        case TACHOMETER_3:
        case TACHOMETER_4:
            value = get_big_endian_byte_32(mcpwm_get_tachometer_value(false), addr - TACHOMETER_1);
            break;
        case RPH_1:
        case RPH_2:
        case RPH_3:
        case RPH_4:
            value = get_big_endian_byte_32((int32_t)(60.0f * mcpwm_get_rpm()), addr - RPH_1);
            break;
        
        default:
            value = 0x0;
            break;
    }
    
    return value;
}


CH_IRQ_HANDLER(I2C2_EV_IRQHandler) {
    CH_IRQ_PROLOGUE();

    uint32_t event = I2C_GetLastEvent(HANDLE);
    switch(event)
    {
        case I2C_EVENT_SLAVE_RECEIVER_ADDRESS_MATCHED:
            // Our address has been adressed for writing -> start communication
            // Note: Writing is not supported, so we graceully ignore everything
            break;
        case I2C_EVENT_SLAVE_BYTE_RECEIVED:
            // Received byte from master
            // This is either address byte or data byte
            // but since we ignore any writes, we treat this as address
            state.addr = I2C_ReceiveData(HANDLE);
            break;
        case I2C_EVENT_SLAVE_TRANSMITTER_ADDRESS_MATCHED:
            // Adressed for reading, send current data
        case I2C_EVENT_SLAVE_BYTE_TRANSMITTED:
            // Master requests next byte
            
            // Both cases are effectively identical here
            
            I2C_SendData(HANDLE, get_byte(state.addr));
            // Increase address (for continuous reads)
            state.addr++;
            break;
        case I2C_EVENT_SLAVE_STOP_DETECTED:
            // End communcation
            
            // Clear addr flag
            while( I2C_SR1_ADDR == (HANDLE->SR1 & I2C_SR1_ADDR) )
            {
                HANDLE->SR1;
                HANDLE->SR2;
            }

            // Clear STOPF flag
            while( I2C_SR1_STOPF == (HANDLE->SR1 & I2C_SR1_STOPF) )
            {
                HANDLE->SR1;
                HANDLE->CR1 |= 0x1;
            }
            break;
    }
    CH_IRQ_EPILOGUE();
}

CH_IRQ_HANDLER(I2C2_ER_IRQHandler) {
    CH_IRQ_PROLOGUE();

    if(I2C_GetITStatus(HANDLE, I2C_IT_AF))
    {
        I2C_ClearITPendingBit(HANDLE, I2C_IT_AF);
    }

    CH_IRQ_EPILOGUE();
}
