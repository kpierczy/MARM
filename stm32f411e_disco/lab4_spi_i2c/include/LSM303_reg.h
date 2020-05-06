/*================================================================================
 *
 *    Filename : LSM303_reg.h
 *        Date : Wed May 06 2020
 *      Author : Krzysztof Pierczyk
 *     Version : 0.0.1
 *
 *    Platform : stm32f411e-DISCO
 *        Core : stm32f411vet
 *
 * Description : Macros for crucial registers of LSM303 (accelerometer & magneto-
 *               meter) as well as addresses of both devices on the I2C bus.
 *
 *===============================================================================*/


#ifndef LSM303_REG_H
#define LSM303_REG_H

#define LSM303_READ        0x01
#define LSM303_WRITE       0x00
#define LSM303_AUTOINC     0x80
#define LSM303_NO_AUTOINC  0x00

/*******************************************
 *        Accelerometer registers
 *******************************************/

// I2C Accellerometer address
#define LSM303_ADDR_A      0x32

// Configuration register
#define LSM303_CTRL_REG1_A 0x20
#define LSM303_CTRL_REG2_A 0x21
#define LSM303_CTRL_REG3_A 0x22
#define LSM303_CTRL_REG4_A 0x23
#define LSM303_CTRL_REG5_A 0x24
#define LSM303_CTRL_REG6_A 0x25

// Measurements register
#define LSM303_OUT_X_L_A   0x28
#define LSM303_OUT_X_H_A   0x29
#define LSM303_OUT_Y_L_A   0x2A
#define LSM303_OUT_Y_H_A   0x2B
#define LSM303_OUT_Z_L_A   0x2C
#define LSM303_OUT_Z_H_A   0x2D


/*******************************************
 *         Magnetometer registers
 *******************************************/

// I2C Magnetometer address
#define LSM303_ADDR_M      0x3C

// Configuration register
#define LSM303_CRA_REG_M   0x00
#define LSM303_CRB_REG_M   0x01
#define LSM303_MR_REG_M    0x02

// Measurements register
#define LSM303_OUT_X_H_M   0x03
#define LSM303_OUT_X_L_M   0x04
#define LSM303_OUT_Y_H_M   0x05
#define LSM303_OUT_Y_L_M   0x06
#define LSM303_OUT_Z_H_M   0x07
#define LSM303_OUT_Z_L_M   0x08

#endif