/* 
 * Copyright (c) 2014 Globalmems, Inc.  All rights reserved.
 *
 * This source is subject to the Globalmems Software License.
 * This software is protected by Copyright and the information and source code
 * contained herein is confidential. The software including the source code
 * may not be copied and the information contained herein may not be used or
 * disclosed except with the written permission of Globalmems Inc.
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
#ifndef GMA302_H
#define GMA302_H

/* Switch options */
#define GMA302	/* Enable flag use gma302 setup ,Disable use gma303 setup*/
#ifdef GMA302
#define SENSOR_NAME		"gma302"/* Device name for GMA302 misc. device */
#else
#define SENSOR_NAME		"gma303"/* Device name for GMA303 misc. device */
#endif

//#define GMA_DEBUG_DATA	/* Default Disable.	1:Enable Gsensor debug data. */
//#define EWMA_FILTER	/* Default:Disable EWMA */
//#define SMA_FILTER	/* Enable or Disable SMA. Default:Disable */
//#define AutoZeroX		/* Default Disable.	XYZ asix AutoZero (GRAVITY_ON_Z AUTO) */
//#define AutoZeroY		/* Default Disable.	XYZ asix AutoZero (GRAVITY_ON_Z AUTO) */
#define AutoZeroZ		/* Default Disable.	XYZ asix AutoZero (GRAVITY_ON_Z AUTO) */
/* Exponentially weighted moving average (EWMA) */
#ifdef EWMA_FILTER
#define EWMA_POSITIVE	10000	/* The values ​​are all positive */
#define EWMA_FACTOR		1024	/* Magnification 2^10=1024，All values ​​<< 10 */
#define EWMA_WEIGHT_X	2		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#define EWMA_WEIGHT_Y	2		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#define EWMA_WEIGHT_Z	4		/* 2(α=0.5)，4(α=0.25)，8(α=0.125)，16(α=0.0625) */
#endif  
/* for Simple Moving Average (SMA) */
#ifdef SMA_FILTER	/* Simple Moving Average */
#define SMA_AVG	4	/* AVG sensor data */
#endif

#define GSE_TAG	"(GMA 30x)"
#define GSE_LOG(fmt, args...)	printk(KERN_INFO GSE_TAG fmt, ##args)
#define ABS(a) ((a) < 0 ? -(a) : (a))

#define SENSOR_I2C_ADDR		0x18

/* Registers */
#define GMA1302_REG_PID 	0x00
#define GMA1302_REG_PD 		0x01
#define GMA1302_REG_ACTR 	0x02
#define GMA1302_REG_MTHR 	0x03
#define GMA1302_REG_STADR 	0x04
#define GMA1302_REG_STATUS 	0x05
#define GMA1302_REG_DX	 	0x06
#define GMA1302_REG_INTCR 	0x15
#define GMA1302_REG_CONTR1 	0x16
#define GMA1302_REG_CONTR2 	0x17
#define GMA1302_REG_CONTR3 	0x18
#define GMA1302_REG_OSM	 	0x38

#define GMA1302_MODE_RESET			0x02
#define GMA1302_MODE_POWERDOWN		0x05

#define GMA302_VAL_WMI					0x02
#define GMA303_VAL_WMI					0x03
#define GMA303_VAL_WMI_RD				0x33
#define GMA1302_VAL_OFFSET_TC_ON		0x40
#define GMA1302_VAL_DATA_READY_ON		0x2a
#define GMA1302_VAL_OFF					0x00
#define GMA1302_VAL_LPF_ON				0x09 /* low-pass filter on */
#define GMA1302_VAL_HPF_ON				0x1b /* high-pass filter on */
#define GMA1302_VAL_TRESHOLD_MAX		0x1F /* treshold set to max */
#define GMA1302_VAL_LOW_NOISE			0x5F /* Oversampling low noise */
#define GMA1302_VAL_ACTR_RESET			0x00 /* Reset DSP and AFE */
#define GMA1302_VAL_ACTR_STOP			0x01 /* Stop DSP*/
#define GMA1302_VAL_ACTR_CONTINUOUS		0x02 /* Enter continuous mode */
#define GMA1302_VAL_ACTR_NON_CONTINUOUS	0x04 /* Enter non-continuous mode */

#define SENSOR_DATA_SIZE 		3
#define AVG_NUM 				8	/* for calibration */
/* ABS axes parameter range [um/s^2] (for input event) */

#define Gravity0_5			LSG/2 	/* raw data 0.5g */
#define Gravity0_25			LSG/4 	/* raw data 0.25g */
#define Gravity0_1			LSG/10 	/* raw data 0.1g */
#define Gravity0_0625		LSG/16 	/* raw data 0.0625g */
#define Gravity0_0625		LSG/16 	/* raw data 0.0625g */
#define Gravity0_03125		LSG/32 	/* raw data 0.03125g */
#define Gravity0_01			LSG/64 	/* raw data 0.01g */
#define Gravity2_0			LSG*2 	/* raw data 2.0g */
#define Gravity4_0			LSG*4 	/* raw data 4.0g */


#endif               
