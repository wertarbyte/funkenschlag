/*** Funkenschlag configuration file ***/

// use TWI?
//#define ENABLE_TWI

/* ADC configuration */

// enable ADC4 and ADC5 - cannot be used in conjunction with TWI/I²c!
//#define USE_ADC4_ADC5

// enable PCF 8159 ADC via TWI
//#define USE_TWI_ADC

// enable I²C compass HMC5883
//#define USE_MAG

// change orientation of compass sensor
//#define MAG_ORIENTATION(X, Y, Z)  {mag_data[M_X]  = -(X); mag_data[M_Y]  = (Y); mag_data[M_Z]  = (Z);}

/* Datenschlag configuration */

#define DS_RETRANSMITS 0

// when defined, switch changes cancel all other transissions
//#define DS_BULLY_UPDATE

// transmit switches 1, 2 and 3 via data channel as AUX2-AUX4 (up to 8 inputs can be specified, 0 disables the AUX channel control)
//#define DS_SEND_AUX_SWITCHES {0, SRC_ID(SRC_SW, 1), SRC_ID(SRC_SW, 2), SRC_ID(SRC_SW, 3)}

// transmit gimbal data
//#define DS_SEND_GIMBAL_ANGLE

// read gimbal angle from input
//#define DS_GIMBAL_INPUT SRC_ID(SRC_TWI_ADC, 1)

// transmit reference heading
//#define DS_SEND_MAG_HEADING

//read headfree orientation from input
//#define DS_HEADING_INPUT SRC_ID(SRC_TWI_ADC, 0)
