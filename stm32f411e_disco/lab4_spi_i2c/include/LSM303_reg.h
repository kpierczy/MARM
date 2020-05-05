/*******************************************
 *        Accelerometer registers
 *******************************************/

// Configuration register
#define LSM303_REGISTER_ACCEL_CTRL_REG1_A 0x20

// Measurements register
#define LSM303_REGISTER_ACCEL_OUT_X_L_A   0x28
#define LSM303_REGISTER_ACCEL_OUT_X_H_A   0x29
#define LSM303_REGISTER_ACCEL_OUT_Y_L_A   0x2A
#define LSM303_REGISTER_ACCEL_OUT_Y_H_A   0x2B
#define LSM303_REGISTER_ACCEL_OUT_Z_L_A   0x2C
#define LSM303_REGISTER_ACCEL_OUT_Z_H_A   0x2D


/*******************************************
 *         Magnetometer registers
 *******************************************/

// Configuration register
#define LSM303_REGISTER_MAG_MR_REG_M  0x02

// Measurements register
#define LSM303_REGISTER_MAG_OUT_X_H_M 0x03
#define LSM303_REGISTER_MAG_OUT_X_L_M 0x04
#define LSM303_REGISTER_MAG_OUT_Y_H_M 0x05
#define LSM303_REGISTER_MAG_OUT_Y_L_M 0x06
#define LSM303_REGISTER_MAG_OUT_Z_H_M 0x07
#define LSM303_REGISTER_MAG_OUT_Z_L_M 0x08