/**	____________________________________________________________________
 *
 *	SBGC32 Serial API Library v2.2.1
 *
 *	@file		driverLinux.c
 *
 *	@brief		Linux OS driver source file
 *	____________________________________________________________________
 *
 *	@attention	<h3><center>
 *				Copyright © 2025 BaseCam Electronics™.<br>
 *				All rights reserved.
 *				</center></h3>
 *
 *				<center><a href="https://www.basecamelectronics.com">
 *				www.basecamelectronics.com</a></center>
 *
 *	Licensed under the Apache License, Version 2.0 (the "License");
 *	you may not use this file except in compliance with the License.
 *	You may obtain a copy of the License at
 *
 *	http://www.apache.org/licenses/LICENSE-2.0
 *
 *	Unless required by applicable law or agreed to in writing, software
 *	distributed under the License is distributed on an "AS IS" BASIS,
 *	WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or
 *	implied. See the License for the specific language governing
 *	permissions and limitations under the License.
 *	____________________________________________________________________
 */

#include "../../sbgc32.h"


#if (SBGC_USE_LINUX_DRIVER)

/* -----------------------------------------------------------------------
 *                        Receive Buffer for Hex Printing
 */
#define RX_PRINT_BUFF_SIZE 512

static ui8 rxPrintBuff[RX_PRINT_BUFF_SIZE];
static ui16 rxPrintIdx = 0;
static ui16 rxExpectedLen = 0;
static ui8 rxState = 0;  /* 0: wait start, 1: got cmd, 2: got len, 3: collecting */

static void PrintRecvBuffer(void)
{
	if (rxPrintIdx > 0) {
		printf("Recv: ");
		for (ui16 i = 0; i < rxPrintIdx; i++) {
			printf("%02X ", rxPrintBuff[i]);
		}
		printf("\n");
		rxPrintIdx = 0;
		rxState = 0;
		rxExpectedLen = 0;
	}
}

/* -----------------------------------------------------------------------
 *                              Executable Functions
 */
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-result"

/**	@addtogroup	Linux_Driver
 *	@{
 */
/**	@brief	Initializes the driver object of sbgcGeneral_t
 *
 *	@param	**driver - main hardware driver object
 *	@param	*dev - path to a connected SBGC32 device
 *	@param	serialSpeed - speed of the COM port
 */
void DriverSBGC32_Init (void **driver, const char *dev, unsigned long serialSpeed)
{
	*driver = sbgcMalloc(sizeof(sbgcDriver_t));

	sbgcDriver_t *drv = (sbgcDriver_t*)(*driver);

	ui8 devNameLength = strlen(dev) + 1;
	drv->device = (char*)sbgcMalloc(devNameLength);
	memcpy(drv->device, dev, devNameLength);

	drv->devFD = open(drv->device, O_RDWR | O_NOCTTY | O_NDELAY);

	if (drv->devFD == -1)
	{
		char errorStr [] = "Device not found!\n";
		DriverSBGC32_PrintDebugData(errorStr, strlen(errorStr));
		SerialAPI_FatalErrorHandler();
	}

	struct termios portConfigurations;

	tcgetattr(drv->devFD, &portConfigurations);

	cfsetispeed(&portConfigurations, serialSpeed);
	cfsetospeed(&portConfigurations, serialSpeed);

	portConfigurations.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
	portConfigurations.c_cflag |= CS8 | CREAD | CLOCAL;

	portConfigurations.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);

	portConfigurations.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	portConfigurations.c_oflag &= ~OPOST;

	tcsetattr(drv->devFD, TCSANOW, &portConfigurations);
}


/**	@brief	Deinitializes the periphery
 *
 *	@param	**driver - main hardware driver object
 */
void DriverSBGC32_Deinit (void **driver)
{
	unused_(driver);
}


/**	@brief	Gets current system time in milliseconds
 *
 *	@return	Current time
 */
sbgcTicks_t DriverSBGC32_GetTimeMs (void)
{
	struct timespec spec;

	clock_gettime(CLOCK_REALTIME, &spec);

	return ((spec.tv_sec & 0x000FFFFF) * 1000) + (sbgcTicks_t)((double)spec.tv_nsec / 1.0e6);
}


/**	@brief	Sends an amount of data to serial port
 *
 *	@param	*driver - main hardware driver object
 *	@param	*data - transferred data
 *	@param	size - size of transferred data
 *
 *	@return	Tx status
 */
ui8 DriverSBGC32_TransmitData (void *driver, ui8 *data, ui16 size)
{
	sbgcDriver_t *drv = (sbgcDriver_t*)driver;

	int bytes;

	/* Print transmitted data in hex format */
	printf("Send: ");
	for (ui16 i = 0; i < size; i++) {
		printf("%02X ", data[i]);
	}
	printf("\n");

	ioctl(drv->devFD, TIOCOUTQ, &bytes);

	if (((SBGC_DRV_TX_BUFF_TOTAL_SIZE - bytes) < size) || (bytes == -1))
		return SBGC_DRV_TX_BUFF_OVERFLOW_FLAG;

	write(drv->devFD, data, size);

	return SBGC_DRV_TX_OK_FLAG;
}


/**	@brief	Returns the number of available bytes
 *
 *	@param	*driver - main hardware driver object
 *
 *	@return	Number of available bytes
 */
ui16 DriverSBGC32_GetAvailableBytes (void *driver)
{
	sbgcDriver_t *drv = (sbgcDriver_t*)driver;

	int bytes;

	ioctl(drv->devFD, FIONREAD, &bytes);

	return bytes;
}


/**	@brief	Receives byte from serial port
 *
 *	@param	*driver - main hardware driver object
 *	@param	*data - data buffer
 *
 *	@return	Rx status
 */
ui8 DriverSBGC32_ReceiveByte (void *driver, ui8 *data)
{
	sbgcDriver_t *drv = (sbgcDriver_t*)driver;

	if (!DriverSBGC32_GetAvailableBytes(drv))
		return SBGC_DRV_RX_BUFF_EMPTY_FLAG;

	read(drv->devFD, data, 1);

	/* Accumulate received bytes and print complete message */
	if (rxPrintIdx < RX_PRINT_BUFF_SIZE) {
		rxPrintBuff[rxPrintIdx++] = *data;
	}

	/* State machine for SBGC protocol: 0x24 CMD LEN HCHK DATA[LEN] CRC16 */
	switch (rxState) {
		case 0:  /* Wait for start byte 0x24 */
			if (*data == 0x24) {
				rxState = 1;
			} else {
				rxPrintIdx = 0;  /* Reset if not start byte */
			}
			break;
		case 1:  /* Got start, next is CMD */
			rxState = 2;
			break;
		case 2:  /* Got CMD, next is data length */
			rxExpectedLen = *data + 4 + 2;  /* start + cmd + len + hchk + data[len] + crc16 */
			rxState = 3;
			break;
		case 3:  /* Collecting data */
			if (rxPrintIdx >= rxExpectedLen) {
				PrintRecvBuffer();
			}
			break;
	}

	return SBGC_DRV_RX_BUSY_FLAG;
}


/**	@brief	Prints debug data
 *
 *	@param	*data - debug data
 *	@param	length - size of debug data
 */
void DriverSBGC32_PrintDebugData (char *data, ui16 length)
{
	while (length)
	{
		printf("%c", *(data++));
		length--;
	}
}
/**	@}
 */

#pragma GCC diagnostic pop


#endif /* SBGC_USE_LINUX_DRIVER */

/* -------------------------------------------------------------------- */
/*                 https://www.basecamelectronics.com                   */
/* -------------------------------------------------------------------- */
