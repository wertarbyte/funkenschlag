/*** Funkenschlag configuration file ***/


/* Datenschlag configuration */

#define DS_RETRANSMITS 0

// when defined, switch changes cancel all other transissions
//#define DS_BULLY_UPDATE

// transmit switches 1, 2 and 3 via data channel as AUX2-AUX4
#define DS_SEND_AUX_SWITCHES {-1, 1, 2, 3}

// transmit gimbal data
//#define DS_SEND_GIMBAL_ANGLE

// transmit reference heading
//#define DS_SEND_MAG_HEADING
