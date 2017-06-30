#ifndef __DUAGON_RPC_LOG_H
#define __DUAGON_RPC_LOG_H

enum {
	IONIA_LOG_CMD_INIT		= 0,	/* Reset modules? Self test ? tbd */
	IONIA_LOG_CMD_CHANNEL_ENABLE	= 1,	/* Enable 4 sensor input channels individually */
	IONIA_LOG_CMD_CHANNEL_MODE	= 2,	/* Select active/passive sensor tbd */
	IONIA_LOG_CMD_BUFFER_STATUS	= 3,	/* Fill state of the data buffer */
	IONIA_LOG_CMD_PATTERN_GEN	= 4,	/* Pattern generator enable */
	IONIA_LOG_CMD_SAMPLING_RATE	= 5,	/* Sampling rate tbd */
	IONIA_LOG_CMD_LED_CONFIG	= 6,	/* Change LED mode: user defined or input controlled tbd */
	IONIA_LOG_CMD_LED_SET		= 7,	/* Set status for 4 LEDs tbd */
	IONIA_LOG_CMD_BUFFER_CLEAR	= 8,	/* Clears the sampling data buffer */
	IONIA_LOG_CMD_TEMP_GET		= 11,	/* Return temperature */
	IONIA_LOG_CMD_SAMPLING_DATA_GET	= 12	/* Get Sensor Data Array */
};

#endif /* __DUAGON_RPC_LOG_H */
