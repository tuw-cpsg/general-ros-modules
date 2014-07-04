/**
 * @file
 * @date 28.10.2013
 *
 * @brief Registers of IMU-3000.
 *
 * The registers are not complete! Only the used ones are defined.
 * For details see description of the datasheet.
 */

#ifndef __REG_IMU3000_H__
#define __REG_IMU3000_H__

#define WHO_AM_I	0x00

#define X_OFFS_USRH	0x0C
#define X_OFFS_USRL	0x0D
#define Y_OFFS_USRH	0x0E
#define Y_OFFS_USRL	0x0F
#define Z_OFFS_USRH	0x10
#define Z_OFFS_USRL	0x11

#define DLPF		0x16
#define FS1		4	// full scale range
#define FS0		3	// +/- 250/500/1000/2000 dps
#define FS		FS0
#define DLPF_CFG2	2
#define DLPF_CFG1	1
#define DLPF_CFG0	0
#define DLPF_CFG	DLPF_CFG0

#define TEMP_OUT_H	0x1B
#define TEMP_OUT_L	0x1C

#define GYRO_XOUT_H	0x1D
#define GYRO_XOUT_L	0x1E
#define GYRO_YOUT_H	0x1F
#define GYRO_YOUT_L	0x20
#define GYRO_ZOUT_H	0x21
#define GYRO_ZOUT_L	0x22

#define PWR_MGM		0x3E
#define H_RESET		7
#define SLEEP		6
#define STBY_XG		5
#define STBY_YG		4
#define STBY_ZG		3
#define CLK_SEL2	2
#define CLK_SEL1	1
#define CLK_SEL0	0
#define CLK_SEL		CLK_SEL0

#endif
