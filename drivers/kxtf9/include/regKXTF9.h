/**
 * @file
 * @date 30.10.2013
 *
 * @brief Registers of KXTF9.
 *
 * The registers are not complete! Only the used ones are defined.
 * For details see description of the datasheet.
 */

#ifndef __REG_KXTF9_H__
#define __REG_KXTF9_H__

#define XOUT_HPF_L	0x00
#define XOUT_HPF_H	0x01
#define YOUT_HPF_L	0x02
#define YOUT_HPF_H	0x03
#define ZOUT_HPF_L	0x04
#define ZOUT_HPF_H	0x05

#define XOUT_L		0x06
#define XOUT_H		0x07
#define YOUT_L		0x08
#define YOUT_H		0x09
#define ZOUT_L		0x0A
#define ZOUT_H		0x0B

#define WHO_AM_I	0x0F

#define CTRL_REG1	0x1B	// operating mode, resolution, etc.
#define GSEL0		3
#define GSEL1		4	// acceleration range +/- 2/4/8 g
#define RES		6	// 8bit (0) or 12bit (1)
#define PC1		7	// stand-by (0) or orperating (1)

#define DATA_CTRL_REG	0x21	// HP filter, output data rate, etc.
#define OSAC		0	// output data rate
#define OSAB		1
#define OSAA		2

// output data rate (Hz) settings (for DATA_CTRL_REG)
#define ODR_25		0x01
#define ODR_50		0x02	// default
#define ODR_100		0x03
#define ODR_200		0x04
#define ODR_400		0x05
#define ODR_800		0x06

#endif
