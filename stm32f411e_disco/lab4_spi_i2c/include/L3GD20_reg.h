#ifndef L3GD20_REG_H
#define L3GD20_REG_H

// Identification register
#define L3GD20_WHO_AM_I 0x0F

// Configuration registers
#define L3GD20_CTRL_REG1     0x20
#define L3GD20_CTRL_REG2     0x21
#define L3GD20_CTRL_REG3     0x22
#define L3GD20_CTRL_REG4     0x23
#define L3GD20_CTRL_REG5     0x24

// Measurements registers
#define L3GD20_OUT_X_L       0x28
#define L3GD20_OUT_X_H       0x29
#define L3GD20_OUT_Y_L       0x2A
#define L3GD20_OUT_Y_H       0x2B
#define L3GD20_OUT_Z_L       0x2C
#define L3GD20_OUT_Z_H       0x2D

// Transaction modifiers
#define L3GD20_READ          0x80
#define L3GD20_WRITE         0x00
#define L3GD20_AUTOINC       0x40
#define L3GD20_NO_AUTOINC    0x00

#endif