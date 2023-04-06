/**************************************
 * the pigpio application to communicat via UART
 * code protype comes from pigpio example
 *
 * The following functions are included
 *  - open a serial port with parameters: port_id, baud rate, and flag (always 0).
 *    return a handle >= 0.
 *  - close a serial port indicated with a parameter: handle.
 *    return 0 if Ok.
 *  - send data stored in a tx buffer
 *    return 0 if Ok.
 *  - read data into rx buffer
 *  - check if there is data already received in the serial port
 *
 **************************************/

#include <stdio.h>
// #include <wiringPi.h>
// #include <wiringSerial.h>
#include <pigpio.h>
#include <stdbool.h>
#include <Python.h>
#include "piSerial.h"

// bool GoGo = TRUE;
#define SERNAME "/dev/serial0"
#define SERBAUD       9600
#define SERBAUD115    115200
#define SLE_LDIS      0
#define SLE_TMPC      1
#define SER_SEL       1

/* Laser Commands */
/* global commands */
char read_laserParameter[4] = {0xFA, 0x06, 0x01, 0xFF};				// Read Parameter following 0xFF.
char read_laserNumber[4] = {0xFA, 0x06, 0x04, 0xFC};				// Read machine number following 0xFC.
char set_laserAddr[5] = {0xFA, 0x04, 0x01, 0x80, 0xFF};				// Set address following a address & CS. default address 0x80
char set_laserDistRevise[6] = {0xFA, 0x04, 0x06, 0x2B, 0x01, 0xFF}; // Revise distance, following symbol (0x2D or 0x2B), revised value, CS.
char set_laserInterval[5] = {0xFA, 0x04, 0x05, 0x01, 0xFF};			// Set datacontinuous interver, following intervaer value & CS.
char set_laserStart[5] = {0xFA, 0x04, 0x08, 0x00, 0xFF};			// Set distance starting and end point, following position value & CS
char set_laserRange[5] = {0xFA, 0x04, 0x09, 0x50, 0xFF};			// Set measuring range, following range value (0x05, 0x0A, 0x1E, 0x32, 0x50) & CS.
char set_laserFreq[5] = {0xFA, 0x04, 0x0A, 0x0A, 0xFF};				// Set frequency, following frequency value (0x05, 0x0A, 0x14)) & CS.
char set_laserResol[5] = {0xFA, 0x04, 0x0C, 0x01, 0xFF};			// Set resolution, following resolution value (1, 2) & CS.
char enable_laserPowerOn[5] = {0xFA, 0x04, 0x0D, 0x00, 0xFF};		// Set measurement starts when powered on (0, 1), following start flag & CS.
char measure_laserSingleB[4] = {0xFA, 0x06, 0x06, 0xFA};			// Single measurement broadcast following 0xFA.
/* module commands */
char read_laserCache[4] = {0x80, 0x06, 0x07, 0xFF};		  // Read cache following a CS. the first byte is laser's address. defaule address 0x80.
char measure_laserSingle[4] = {0x80, 0x06, 0x02, 0xFF};	  // Single measurement following a CS. the first byte is laser's address. defaule address 0x80.
char measure_laserContinu[4] = {0x80, 0x06, 0x03, 0xFF};  // Continuous measurement following a CS. the first byte is laser's address. defaule address 0x80.
char control_laserOff[5] = {0x80, 0x06, 0x05, 00, 0xFF};  // Control laser off llowing a CS. the first byte is laser's address. defaule address 0x80.
char control_laserOn[5] = {0x80, 0x06, 0x05, 01, 0xFF};	  // Control laser on llowing a CS. the first byte is laser's address. defaule address 0x80.
char control_laserShutdown[4] = {0x80, 0x04, 0x02, 0x7A}; // Shut down llowing a CS. the first byte is laser's address. defaule address 0x80.

SERCmdMsg LD_RDPARA = {
	1,
	read_laserParameter,
	4,
	1};
SERCmdMsg LD_MACHNUM = {
	2,
	read_laserNumber,
	4,
	1};
SERCmdMsg LD_SETADDR = {
	3,
	set_laserAddr,
	5,
	4};
SERCmdMsg LD_SETDISREV = {
	4,
	set_laserDistRevise,
	6,
	4};
SERCmdMsg LD_SETINTVL = {
	5,
	set_laserInterval,
	5,
	4};
SERCmdMsg LD_SETSTAEND = {
	6,
	set_laserStart,
	5,
	4};
SERCmdMsg LD_SETMEASRG = {
	7,
	set_laserRange,
	5,
	4};
SERCmdMsg LD_SETFREQ = {
	8,
	set_laserFreq,
	5,
	4};
SERCmdMsg LD_SETRESOL = {
	9,
	set_laserResol,
	5,
	4};
SERCmdMsg LD_ENPOWON = {
	10,
	enable_laserPowerOn,
	5,
	4};
SERCmdMsg LD_MEASSING_B = {
	11,
	measure_laserSingleB,
	4,
	0};
SERCmdMsg LD_READCACH = {
	12,
	read_laserCache,
	4,
	11};
SERCmdMsg LD_MEASSINGL = {
	13,
	measure_laserSingle,
	4,
	11};
SERCmdMsg LD_MEASCONTU = {
	14,
	measure_laserContinu,
	4,
	11};
SERCmdMsg LD_LASERON = {
	15,
	control_laserOn,
	5,
	5};
SERCmdMsg LD_LASEROFF = {
	16,
	control_laserOff,
	5,
	5};
SERCmdMsg LD_LASERSHTDWN = {
	17,
	control_laserShutdown,
	4,
	4};

SERCmdMsg *pLD_CMDMSG[20] =
	{
		&LD_RDPARA,
		&LD_MACHNUM,
		&LD_SETADDR,
		&LD_SETDISREV,
		&LD_SETINTVL,
		&LD_SETSTAEND,
		&LD_SETMEASRG,
		&LD_SETFREQ,
		&LD_SETRESOL,
		&LD_ENPOWON,
		&LD_MEASSING_B,
		&LD_READCACH,
		&LD_MEASSINGL,
		&LD_MEASCONTU,
		&LD_LASERON,
		&LD_LASEROFF,
		&LD_LASERSHTDWN};

/* serial port data */
// char txData[64] = {0};
// char rxData[64] = {0};

/* temperature controller commands */
const uint8_t MTD415_Cmds[32][20] =
{
	{"m?\n\0"},                   //reads the version of hardware and software. [MTD415T FW0.6.8]
	{"u?\n\0"},                   //read the UUID (Universal Unique Identifier). [xxx...]
	{"E?\n\0"},                   //read the error reagester
	{"c\n\0"},                    //reset the error register
	/* TEC Cmds */
	{"Lx\n\0"},                   //set the TEC current limit to x. x: 200 to 1500 [mA].
	{"L?\n\0"},                   //read the TEC current limit. [x<LF>] [mA].
	{"A?\n\0"},                   //read the actual TEC current. [x<LF>] x < 0 heating; x > 0 cooling.
	{"U?\n\0"},                   //read the actural TEC voltage. [x<LF>] [mV].
	/* Temperature Cmds */
	{"Tx\n\0"},                   //set the setting temperture to x. x: 5000 to 45000 [10^-3 C-degree].
	{"T?\n\0"},                   //read the set temperature. [x<LF>].
	{"Te?\n\0"},                  //read the actual temperature. [x<LF>].
	{"Wx\n\0"},                   //set the setting temperature window to x. x: 1 to 32000 [mK].
	{"W?\n\0"},                   //read the temperature window. [x<LF>] [mK].
	{"dx\n\0"},                   //set the delay time between reaching the temperature window and activating the State output pin to x. x: 1 to 32000 [sec]. 
	{"d?\n\0"},                   //read the delay time. [x<LF>] [sec].
	/* Control Loop Cmds */
	{"Gx\n\0"},                   //set the dritical gain to x. x: 10 to 100000 [mA/K].
	{"G?\n\0"},                   //read the critical gain. [x<LF>] [mA/K].
	{"Ox\n\0"},                   //set the critical period to x. x: 100 to 100000 [msec].
	{"O?\n\0"},                   //read the critical period. [x<LF>] [msec].
	{"Cx\n\0"},	                  //set the cycling time to x. x: 1 to 1000 [msec].
	{"C?\n\0"},                   //read the cycling time. [x<LF>] [msec].
	{"Px\n\0"},                   //set the P Share to x. x: 0 to 100000 [mA/K].
	{"P?\n\0"},                   //read the P Share. [x<LF>] [mA/K].
	{"Ix\n\0"},                   //set the I Share to x. x: 0 to 100000 [mA/(K*sec)].
	{"I?\n\0"},                   //read the I Share. [x<LF>] [mA\(K*sec)].
	{"Dx\n\0"},	                  //set the D Share to x. x: 0 to 100000 [mA*sec/K].
	{"D?\n\0"},                   //read the D Share. [x<LF>] [mA*sec/K].
	/* save settings */
	{"M\n\0"}                    //save the setup. The actual parameters that have been set using he commands T, W, L, d, G, O, P, I, D, C and S, are saved to the nonvolatile memeory.
};


char testSum(char *pTxbuf, int numbytes)
{
	char chkSum = 0;
	int i, numChk = numbytes;

	for (i = 0; i < numChk; i++)
	{
		chkSum += *(pTxbuf + i);
	}

	chkSum = ~chkSum + 1;

	return chkSum;
}

/* get sending Cmd/Msg Length */
int serSendCnt(char *string)
{
	int len=0;
	
	while(string[len]!=0)
		len++;
	return len;
}

int tspi_serial_start(int chan, UARTport *uart)
{
	int hd = -1;
	int vsel;
	/* set the serial mux output */
	gpioSetMode(SER_SEL, PI_OUTPUT);
	if (chan == SLE_TMPC)
		gpioWrite(SER_SEL, SLE_TMPC); // select temperature controler
	else
		gpioWrite(SER_SEL, SLE_LDIS); // select laser distance measuring (default)

	vsel = gpioRead(SER_SEL);

	/* open a serial port */
	if(chan == SLE_TMPC)
		hd = serOpen(SERNAME, SERBAUD115, 0); // open serial port for temp controller
	else
		hd = serOpen(SERNAME, SERBAUD, 0); // open serial port for laser distance measuring

	if (hd < 0)
		return hd; // open serial port failed.

	uart->serHandle = hd;
	uart->serID = chan;
	uart->Txbuf_lgth = 0;
	uart->Rxbuf_lgth = 0;

	/* return the hd */
	return hd;
}

int tspi_serial_close(int SERHandle, UARTport *uart)
{
	int hd, ok = -1;
	hd = SERHandle;

	/* close a serial port with SERHandle */
	ok = serClose(hd);

	/* clear uart */
	uart->serHandle = -1;
	uart->serID = -1;
	uart->pTxbuf = NULL;
	uart->pRxbuf = NULL;
	uart->Txbuf_lgth = 0;
	uart->Rxbuf_lgth = 0;

	/* return the close ok value */
	return ok;
}

int tspi_serial_write(UARTport *uart)
{
	int sendCnt = 0;
	int hd = uart->serHandle;

	char *lptxbuf = uart->pTxbuf;
	uint16_t lcnt = uart->Txbuf_lgth;

	/* send data via UART */
	sendCnt = serWrite(hd, lptxbuf, lcnt);

	/* return number of byte sended */
	return sendCnt;
}

int tspi_serial_read(UARTport *uart, int byteCnt)
{
	int hd, receiveCnt;
	int numbyte;

	char *lprxbuf = uart->pRxbuf;

	hd = uart->serHandle;
	// numbyte = byteCnt;

	/* check the data ready to read */
	numbyte = tspi_serial_dataRdy(hd); // for debug. then rm into calling routine.

	if (numbyte > byteCnt)
	{
		printf("get more data: %d %d\n", numbyte, byteCnt);
	}
	else
		numbyte = byteCnt;

	// if(numbyte >= lreqnm)
	{
		/* read data */
		receiveCnt = serRead(hd, uart->pRxbuf, numbyte);
	}
	// else{
	//	receiveCnt = 0;
	// }

	/* return number of byte received */
	return receiveCnt;
}

int tspi_serial_dataRdy(int SERHandle)
{
	int numbyte = 0;

	/* check available data on uart */
	numbyte = serDataAvailable(SERHandle);

	/* return number of available data */
	return numbyte;
}

void tspi_serPreExch(UARTport *uart, SERCmdMsg *cmg)
{
	unsigned char sumChk;
	char *lpTxbuf = uart->pTxbuf;
	uint16_t lnumbytes = cmg->txLgth - 1;
	int i;

	/* prepare uart */
	printf("tx length %d \n", lnumbytes + 1);

	for (i = 0; i < lnumbytes; i++)
	{
		*(lpTxbuf + i) = *(cmg->pcmd + i);
		//printf("0x%x - 0x%x; ", *(lpTxbuf + i), *((cmg->pcmd) + i));
	}

	/* calculate checksum */
	if(uart->serID == SLE_LDIS)
	{
		sumChk = testSum(lpTxbuf, lnumbytes);
		*(lpTxbuf + lnumbytes) = sumChk;
	}else// if(uart->serID == SLE_LDIS)
	{
		*(lpTxbuf + lnumbytes) = *(cmg->pcmd + lnumbytes);
	}
	//sumChk = testSum(lpTxbuf, lnumbytes);
	//*(lpTxbuf + lnumbytes) = sumChk;

	uart->Txbuf_lgth = cmg->txLgth;
	uart->Rxbuf_lgth = cmg->rxLgth;

	printf("pre data: sum - 0x%x txlg - %d rxlg - %d. \n", sumChk, uart->Txbuf_lgth, uart->Rxbuf_lgth);
	return;
}

int tspi_serPosExch(UARTport *uart, SERCmdMsg *cmg)
{
	bool okFlag = true;
	float dist = 0;
	char result = 0;
	int cmdId = cmg->cmdID;

	switch (cmdId)
	{
	case LASER_REDPAR_CMD:
		uart->pRxbuf[3]; // address.
		uart->pRxbuf[4]; // light
		uart->pRxbuf[5]; //???
		uart->pRxbuf[6]; // temperature
		break;
	case LASER_REDNUM_CMD:
		// cmdResult->laserAddr = RxBuffer[3];
		// cmdResult->light = RxBuffer[4];
		// cmdResult->returned = RxBuffer[5];
		// cmdResult->temperature = RxBuffer[6];
		break;
	case LASER_SETADD_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETDSR_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETINL_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETSTA_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETRNG_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETFRQ_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_SETRES_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_ENAPOW_CMD:
		result = uart->pRxbuf[1];
		if (result == 0x04)
			okFlag = false;
		break;
	case LASER_MESIGB_CMD:
		break;
	case LASER_REDCAH_CMD:
	case LASER_MEASIG_CMD:
	case LASER_MEACON_CMD:
		result = uart->pRxbuf[3];
		if (result == 0x45)
		{
			result = uart->pRxbuf[4];
			if (result == 0x52)
				okFlag = false;
			dist = 0;
		}
		else
		{
			dist = (uart->pRxbuf[3] - 0x30) * 100 + (uart->pRxbuf[4] - 0x30) * 10 +
				   (uart->pRxbuf[5] - 0x30) * 1 + (uart->pRxbuf[7] - 0x30) * 0.1 +
				   (uart->pRxbuf[8] - 0x30) * 0.01 + (uart->pRxbuf[9] - 0x30) * 0.001;
		}
		// cmdResult->distance = dist;
		break;
	case LASER_CTLOFF_CMD:
	case LASER_CTRLON_CMD:
		result = uart->pRxbuf[3];
		if (result == 0x0)
			okFlag = false;
		break;
	case LASER_CTLSHT_CMD:
		break;
	}
	return okFlag;
}

int tspi_serExchData(UARTport *uart, SERCmdMsg *pcdmg)
{
	const int maxTry = 300;
	int OK = false;
	int hd, counter = 0, numBytes = 0, rxNum = 0, n;
	char dumBuf[32] = {0};
	char *lprxbuf;
	uint16_t ltxlth, lrxlth;

	hd = uart->serHandle;
	ltxlth = uart->Txbuf_lgth;
	lrxlth = uart->Rxbuf_lgth;

	/* Debug code */
	//printf("TX Data0: \n");
	//for (n = 0; n < pcdmg->txLgth; n++)
	//{
	//	printf("0x%x - 0x%x ", *(uart->pTxbuf + n), *(pcdmg->pcmd + n));
	//}

	//printf(" No.Rx %d pcdmg 0x%x.\n\n", lrxlth, pcdmg->pcmd);

	/* prepare send data */
	//tspi_serPreExch(uart, pcdmg);

	/* Debug code */
	printf("\nTX Data: ");
	for (n = 0; n < ltxlth; n++)
	{
		printf("0x%x ", *(uart->pTxbuf + n));
	}

	printf(".\n\n");

	//return n;
	/* end of Debug */

	/* flush the receiver buffer */
	numBytes = tspi_serial_dataRdy(hd);
	while (1)
	{
		lprxbuf = uart->pRxbuf;
		// lrxlth = uart->Rxbuf_lgth;
		uart->pRxbuf = dumBuf;
		uart->Rxbuf_lgth = numBytes;
		tspi_serial_read(uart, numBytes);
		
		/* Debug code */
		printf("dump Data: ");
		for (n = 0; n < numBytes; n++)
		{
			printf("0x%x ", *(uart->pRxbuf + n));
		}
		
		printf(".\n\n");
		/* end of Debug code */

		if(numBytes = tspi_serial_dataRdy(hd) == 0)
		{
			uart->pRxbuf = lprxbuf;
			break;
		}
		// uart->Rxbuf_lgth = lrxlth;
	}

	/* send data */
	tspi_serial_write(uart);
	gpioDelay(2000);

	numBytes = tspi_serial_dataRdy(hd);
	do
	{
		if (numBytes >= lrxlth)
		{
			rxNum += tspi_serial_read(uart, numBytes);
			
			printf("rx-%d\n", rxNum);
			gpioDelay(10000);
			numBytes = tspi_serial_dataRdy(hd);
			if(numBytes > 0)
			{
				continue;
			}

			break;
		}
		else
		{
			gpioDelay(10000);
			counter++;
			numBytes = tspi_serial_dataRdy(hd);
			//printf("count-%d, No.-%d, %d\n", counter, numBytes, lrxlth);
		}
	} while (/*(numBytes <= lrxlth) &&*/ (counter < maxTry));

		uart->Rxbuf_lgth = rxNum;
		/* Debug code */
		printf("Received Data %d ,%d: ", counter, numBytes);
		for (n = 0; n < rxNum; n++)
		{
			printf("0x%x, 0x%x; ", *(uart->pRxbuf + n), *(uart->pTxbuf + n));
		}
		
		printf(".\n\n");

		/* end of Debug code */
		
		/* process received data */
	//tspi_serPosExch(uart, pcdmg);

	return rxNum;
}

int tspi_serCdmg(UARTport *uart, SERCmdMsg *pcdmg, int CmdID)
{
	int numofbytes = 0, sendNo;
	//SERCmdMsg *lpcdmg = pcdmg;

	if(uart->serID == SLE_TMPC)/* if LDIS */
	{
		pcdmg->cmdID = CmdID;
		pcdmg->pcmd = &MTD415_Cmds[CmdID - 1][0];
		pcdmg->txLgth = serSendCnt(pcdmg->pcmd);
		pcdmg->rxLgth = 2;
		sendNo = pcdmg->txLgth;
	}
	else{
		pcdmg = pLD_CMDMSG[CmdID - 1];
		int sendNo = pcdmg->txLgth;
	}

	// check lpcdmg contents
	//printf("CmdID: %d - %d; Count %d; SendLgt %d.\n", CmdID, pcdmg->cmdID, pcdmg->rxLgth, sendNo);

	for (int i = 0; i < sendNo; i++)
	{
		printf("Cmd: 0x%x ", *((pcdmg->pcmd) + i));
	}
	//printf("0x%x\n", pcdmg->pcmd);

	/* prepare the data for sending */
	tspi_serPreExch(uart, pcdmg);

	numofbytes = tspi_serExchData(uart, pcdmg);

	return numofbytes;
}

void UART_distMain(int isConti)
{
	int hd, reps, addr;
	int value;
	int numbytes;
	char command[16];
	UARTport muart;
	SERCmdMsg cmdmsg;

	char txbuf[32] = {0}, rxbuf[32] = {0};

	float distance = 0;

	/* start UART */
	hd = tspi_serial_start(SLE_LDIS, &muart);
	if (hd < 0)
	{
		printf("start uart failed.\n");
		return hd;
	}

	/* set tx/rx buffers */
	muart.pTxbuf = &txbuf[0];
	muart.pRxbuf = &rxbuf[0];

	/* Select Dis_Laser operations */

	printf("UART started.\n\n    Input LD Command: ");
	scanf("%s", command);

	while (1) // input uart app commands.
	{
		// printf("Dis Command %s - %d.\n", command, addr);

		if (!strcmp("LDparm", command)) // check LDIS parameters
		{
			// printf("Set UART ID.\n    Input Command: "); // selection: 0-LaserDis; 1-TempCtrl.
			// scanf("%d", command);

			// execute code;
			printf("Execute %s, No. bytes %d.\n", command, sizeof(read_laserParameter));
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_REDPAR_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDNo", command)) // check LDIS serial number
		{
			// execute code;
			printf("Execute %s, No. bytes %d.\n", command, sizeof(read_laserNumber));
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_REDNUM_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetAddr", command)) // set LDIS address. default 0x80.
		{
			printf("    Input Address: ");
			scanf("%x", &addr);

			// execute code;
			printf("Execute %s, No. bytes %d, %x.\n", command, sizeof(set_laserAddr), addr);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETADD_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetDistRev", command)) // set LDIS Distance Revise. default.
		{
			printf("    Input Revice: ");
			scanf("%x", &value);

			// execute code;
			printf("Execute %s, No. bytes %d.\n", command, sizeof(set_laserDistRevise));
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETDSR_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetInt", command)) // set LDIS Continue Measuring Internal. default.
		{
			printf("    Input Interval: ");
			scanf("%x", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETINL_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetStaEnd", command)) // set LDIS Start/End Point. default.
		{
			printf("    Input Start/End: ");
			scanf("%x", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETSTA_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetRange", command)) // set LDIS Measuring Range. default.
		{
			printf("    Input Range: ");
			scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETRNG_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetFreq", command)) // set LDIS Measuring Frequence. default.
		{
			printf("    Input Frequence: ");
			scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETFRQ_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDSetRes", command)) // set LDIS Measuring Resolution. default.
		{
			printf("    Input Resolution: ");
			scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_SETRES_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDatPowerOn", command)) // set LDIS start at the Power On. default.
		{
			printf("    Input Flag of Power On: ");
			scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_ENAPOW_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDMeasSigB", command)) // set LDIS Measuring Single Broad. default.
		{
			// printf("    Input Frequence: ");
			// scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_MESIGB_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDReadCache", command)) // read LDIS cache if in measuring single Broad. default.
		{
			// printf("    Input Resolution: ");
			// scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_REDCAH_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDMeasSingle", command)) // set LDIS Measuring Single. default.
		{
			// printf("    Input Flag of Power On: ");
			// scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_MEASIG_CMD);

			float distance = ((rxbuf[3] - 0x30) * 100 
			+ (rxbuf[4] - 0x30) * 10 + (rxbuf[5] - 0x30) * 1
			+ (rxbuf[7] - 0x30) * 0.1 + (rxbuf[8] - 0x30) * 0.01 
			+ (rxbuf[9] - 0x30) * 0.001);
			printf("distanc: %4.2fm \n", distance);


			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDMeasContinue", command)) // set LDIS Measuring Continue. default.
		{
			printf("    Input Number of Measurement: ");
			scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_MEACON_CMD);
			// stop continue measurement
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_MEASIG_CMD);
			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDLaserOff", command)) // set Laser Off. default.
		{
			// printf("    Input Resolution: ");
			// scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_CTLOFF_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDLaserOn", command)) // set Laser On. default.
		{
			// printf("    Input Flag of Power On: ");
			// scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_CTRLON_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("LDLaserShutDown", command)) // set Laser Shutdown. default.
		{
			// printf("    Input Frequence: ");
			// scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n", command);
			reps = tspi_serCdmg(&muart, &cmdmsg, LASER_CTLSHT_CMD);

			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("CloseLDist", command)) // close current UART. default.
		{
			// printf("    Input UART ID: ");
			// scanf("%d", &value);

			break;
		}
		else if (!strcmp("SerDbgData", command)) // sent or received data ...
		{
			/* input concern data ID */
			//printf("    Input Cmd ID: ");
			//scanf("%d", &value);

			/* display data */
			printf("LD Cmd ID %d: ", muart.serID);
			for (int n = 0; n < muart.Rxbuf_lgth; n++)
				printf("0x%x ", *((muart.pRxbuf) + n));

			printf("\n\n");

			// printf("The Cmd Paras: 0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txbuf[0], txbuf[1], txbuf[2], txbuf[3], txbuf[4], txbuf[5], txbuf[6], txbuf[7], txbuf[8]);
			// printf("Tx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", txbuf[9], txbuf[10], txbuf[11], txbuf[12], txbuf[13], txbuf[14], txbuf[15], txbuf[16], txbuf[17]);

			// printf("Rx0:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxbuf[0], rxbuf[1], rxbuf[2], rxbuf[3], rxbuf[4], rxbuf[5], rxbuf[6], rxbuf[7], rxbuf[8]);
			// printf("Rx1:  0x%x, 0x%x, 0x%x, 0x%x 0x%x, 0x%x, 0x%x, 0x%x 0x%x \n", rxbuf[9], rxbuf[10], rxbuf[11], rxbuf[12], rxbuf[13], rxbuf[14], rxbuf[15], rxbuf[16], rxbuf[17]);

			// printf("Data: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", adcData.response, adcData.channel0, adcData.channel1, adcData.channel2, adcData.channel3);

			// double step = 1200000.0 / 8388607.0;
			// double v1, v2, v3, v4;

			// v1 = (double)(~(0xe00000));

			// printf("v1 %f.\n", v1);

			// v1 = 0.0 - v1;

			// printf("0-v1 %f.\n", v1);
			/*
			if(adcData.channel0 > 0x7fffff)
			{
				v1 = (double)(~(adcData.channel0 | 0xff000000)+1);
				v1 = -v1;
			}
			else
			{
				v1 = (double)adcData.channel0;
			}

			if(adcData.channel1 > 0x7fffff)
			{
				v2 = (double)(~(adcData.channel1 | 0xff000000)+1);
				v2 = -v2;
			}
			else
			{
				v2 = (double)adcData.channel1;
			}

			if(adcData.channel2 > 0x7fffff)
			{
				v3 = (double)(~(adcData.channel2 | 0xff000000)+1);
				v3 = -v3;
			}
			else
			{
				v3 = (double)adcData.channel2;
			}

			if(adcData.channel3 > 0x7fffff)
			{
				v4 = (double)(~(adcData.channel3 | 0xff000000)+1);
				v4 = -v4;
			}
			else
			{
				v4 = (double)adcData.channel3;
			}

			v1 *= step;
			v2 *= step;
			v3 *= step;
			v4 *= step;

			printf("Data: 0x%x, %f, %f, %f, %f\n", adcData.response, v1, v2, v3, v4);
			*/
			printf("    Input LD Command: ");
			scanf("%s", command);
		}
		else
		{
			printf("\nIncorrect LD command!\n\n");
			printf("The commands should be:\n");
			printf("    \"LDparm\" to get MTD version of Hardware and Software.\n");
			printf("    \"LDMeasSingle\" to get MTD UUID.\n");
			printf("    \"LDMeasContinue\" to get MTD Error information;\n");
			printf("    \"LDLaserOff\" to get MTD Error information;\n");
			printf("    \"LDLaserOn\" TEC commands.\n");
			printf("    \"LDLaserShutDown\" Temperature commands.\n\n");
			//printf("    \"MTDLpCtrlSet\" Loop Controller commands.\n\n");
			//printf("    \"MTDSave\" Save parameters into non-volatile memory.\n\n");
			
			printf("    \"CloseLDist\" to stop the MTD Settings.\n\n");

			printf("    Input Command: ");
			scanf("%s", command);
		}
	}

#if 0  // remove old code
	if(!isCouti)
	{
		//distance = measurDig(hd);
		muart->pTxbuf = &LDsinglemeas; // set continuse measurement
		muart->Txbuf_lgth = sizeof(LDsinglemeas);
		
		/* flush the receiver buffer */
		//read all received data
		
		/* send command */
		tspi_serial_write(hd,muart);
		delay(500);
		
		/* receive data to receive buffer */
		numbytes = tspi_serial_dataRdy(hd);
		do{
			if(numbytes > muart->Rxbuf_lgth)
				tspi_serial_read(muart, numbytes);
			else{
				delay(200);
				numbytes = tspi_serial_dataRdy(hd);
			}
		}while();
		
		/* process of the command response */
		
		/* set end of routine. */
		
	}
	
	/* set LDIS mode */
	muart->pTxbuf = &LDcontimeas; // set continuse measurement
	muart->Txbuf_lgth = sizeof(LDcontimeas);
    tspi_serial_write(hd,muart);
    
	/* read the distance value via uart */
    int counter = 0, counterErr = 0; GoGo = TRUE;
    while(GoGo)
    {  
        
        if (counter > 2 || counterErr > 50) GoGo = FALSE;
        if (serialDataAvail(fd) > 0)
        {
            delay(50);
            counter++;
            for(int i=0;i<11;i++)
            {
                data[i]=serialGetchar(fd);
                //printf("%x ",data[i]);
            }
            //printf("\n");
            unsigned char Check=0;
            for(int i=0;i<10;i++)
            {
                Check=Check+data[i];
            }
            Check=~Check+1;
            //printf("%x \n" ,Check);
            if(data[10]==Check)
            {
                if(data[3]=='E'&&data[4]=='R'&&data[5]=='R')
                {
                    printf("Out of range");
                }
                else
                {
                distance=0;
                distance=(data[3]-0x30)*100+(data[4]-0x30)*10+(data[5]-0x30)*1+(data[7]-0x30)*0.1+(data[8]-0x30)*0.01+(data[9]-0x30)*0.001;
                printf("Distance = ");
                printf("%5.1f",distance);
                printf(" m\n");
                }
            }
            else
            {
                printf("Invalid Data!\n");
            }
        }else{
            counterErr++;
        }
        delay(20);
    }
    //serialPrintf(fd,laseroff);
    //delay(500);
    //serialPrintf(fd,shutdown);     
    //delay(500);
#endif // end of the old code

	tspi_serial_close(hd, &muart); // serialClose(hd);
	printf("LDistance Sensor UART Closed. \n");
	return;
}

//mtdCmdInf comdInf;

void preSerCmd(mtdCmdInf* comdInf)
{
	int oper, value;
	
	printf("    Input Operation: ");  // 0 - set; 1 - get
	scanf("%d", &oper);
		
	printf("    Input set value: ");  // 0 - set; 1 - get
	scanf("%d", &value);
	
	comdInf->oper = oper;
	comdInf->value = value;
	comdInf->cmdParFlay = true;
	
	return;
}
#if 0 // replace by tempCtrl_py()
void UART_tempCMain(int isConti)
{
    int hd, reps, addr;
    int oper, value;
    int numbytes;
	char command[16];
	UARTport muart;
	SERCmdMsg cmdmsg;
	
	char txbuf[32], rxbuf[32];
    
	float distance=0;

    /* start UART */
	hd = tspi_serial_start(SLE_TMPC, &muart);
    if(hd < 0) 
	{
		printf("start uart failed.\n");
		return hd;
	}

	/* set tx/rx buffers */
	muart.pTxbuf = &txbuf[0];
	muart.pRxbuf = &rxbuf[0];
	
	/* Select Dis_Laser operations */
	
	printf("UART started.\n\n    Input MTD Command: ");
	scanf("%s", command);

	while (1) // input uart app commands.
	{
		// printf("Dis Command %s - %d.\n", command, addr);
		
		if (!strcmp("MTDversion", command)) // check MTD415 version
		{
			
			cmdmsg.comdInf.cmdParFlay = false;

			// execute code;
			printf("Execute %s, No. bytes %d.\n", command, sizeof(read_laserParameter));
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_VER_CMD);
						
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDuuid", command)) // check MTD415 uuid
		{
			cmdmsg.comdInf.cmdParFlay = false;
		
			// execute code;
			printf("Execute %s, No. bytes %d.\n",command, sizeof(read_laserNumber));
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_UUID_CMD);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDErr", command)) // get MTD415 error data.
		{
			cmdmsg.comdInf.cmdParFlay = false;
		
			// execute code;
			printf("Execute %s, No. bytes %d, %x.\n", command, sizeof(set_laserAddr), addr);
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_GERR_CMD);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDClrErr", command)) // clean MTD415 error data.
		{
			cmdmsg.comdInf.cmdParFlay = false;
		
			// execute code;
			printf("Execute %s, No. bytes %d.\n",command, sizeof(set_laserDistRevise));
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_CERR_CMD);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDCurntSet", command)) // set/get MTD415 TEC current/voltage.
		{
			/* set commad parameters */			
			preSerCmd(&cmdmsg.comdInf);
			printf("MTDCur op-%d, val-%d\n", cmdmsg.comdInf.oper, cmdmsg.comdInf.value);

			oper = cmdmsg.comdInf.oper; //0-set limit; 1-get limit; 2-get current; 3-get voltage.
			if(oper < 0 || oper > 3)
			{
				oper = 1; //if value out of the range, get current limit.
				cmdmsg.comdInf.oper = oper;
			}
						
			value = cmdmsg.comdInf.value;
			switch(oper)
			{
				case 0:
				if(value < 200 || value > 1500)
					value = 1000; //if value out of the range, set to default 1000mA.
				break;
				case 1:
				case 2:
				case 3:
				cmdmsg.comdInf.cmdParFlay = false;
				break;
			}
			cmdmsg.comdInf.value = value;
									
			// execute code;
			printf("Execute %s.\n",command);
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_TECUS_CMD + oper);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}	
		else if (!strcmp("MTDTempSet", command)) // set/get MTD415 tempture setting.
		{
			/* set commad parameters */			
			preSerCmd(&cmdmsg.comdInf);

			oper = cmdmsg.comdInf.oper; //0-set temp point; 1-get temp point; 2-get temp; 3-set temp window; 4-get temp window; 5-set delay; 6-get dalay.
			if(oper < 0 || oper > 6)
			{
				oper = 1; //if value out of the range, get temp point.
				cmdmsg.comdInf.oper = oper;
			}
						
			value = cmdmsg.comdInf.value;
			switch(oper)
			{
				case 0:
				if(value < 5000 || value > 45000) //5 ~ 45 c-degree.
					value = 25000; //if value out of the range, set to default 25 c-degree.
				break;
				case 3:
				if(value < 1 || value > 32000) //1 ~ 32000 mK-degree.
					value = 1000; //if value out of the range, set to default 1 degree window.
				break;
				case 5:
				if(value < 1 || value > 32000) //1 ~ 32000 sec.
					value = 6; //if value out of the range, set to default 6 second delay.
				case 6:
				case 1:
				case 2:
				case 4:
				cmdmsg.comdInf.cmdParFlay = false;
				break;
			}
			cmdmsg.comdInf.value = value;			

			// execute code;
			printf("Execute %s.\n",command);
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_TEMPS_CMD + oper);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDLpCtrlSet", command)) // set/get MTD415 control loop setting..
		{
			/* set commad parameters */			
			preSerCmd(&cmdmsg.comdInf);

			oper = cmdmsg.comdInf.oper; 
			/********************************
			 * control loop set commands:
			 *  0-set gain; 1-get gain;
			 *  2-set period; 3-get period;
			 *  4-set cycling time; 5-get cycling time;
			 *  6-set P share; 7-get P share;
			 *  8-set I share; 9-get I share;
			 *  10-set D share; 11-get D share.
			 * 
			 * *****************************/
			if(oper < 0 || oper > 11)
			{
				oper = 1; //if value out of the range, get gain.
				cmdmsg.comdInf.oper = oper;
			}
			
			value = cmdmsg.comdInf.value;
			switch(oper)
			{
				case 0:
				if(value < 10 || value > 100000) //critical loop gain 10 ~ 100000 mA/K.
					value = 2000; //if value out of the range, set to default 2000 mA/K.
				break;
				case 2:
				if(value < 100 || value > 100000) //critical period 100 ~ 100000 msec.
					value = 2000; //if value out of the range, set to default 2000 msec.
				break;
				case 4:
				if(value < 1 || value > 1000) //cycling time 1 ~ 1000 msec.
					value = 50; //if value out of the range, set to default 50 msec.
				break;
				case 6:
				if(value < 0 || value > 100000) //P share 0 ~ 100000 mA/K.
					value = 1000; //if value out of the range, set to default 1000 mA/K.
				break;
				case 8:
				if(value < 0 || value > 100000) //I share 0 ~ 100000 mA/(K*sec).
					value = 200; //if value out of the range, set to default 200 mA/(K*sec).
				break;
				case 10:
				if(value < 0 || value > 100000) //D share 0 ~ 100000 (mA*sec)/K.
					value = 100; //if value out of the range, set to default 100 (mA*sec)/K.
				break;
				case 1:
				case 3:
				case 5:
				case 7:
				case 9:
				case 11:
				cmdmsg.comdInf.cmdParFlay = false;
				break;
			}
			cmdmsg.comdInf.value = value;

			// execute code;
			printf("Execute %s.\n",command);
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_CLPGS_CMD + oper);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDSave", command)) // set Laser Off. default.
		{
			cmdmsg.comdInf.cmdParFlay = false;

			// execute code;
			printf("Execute %s.\n",command);
			reps = tspi_serCdmg(&muart, &cmdmsg, MTD_PMSAV_CMD);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDDbgData", command)) // set Laser On. default.
		{
			//printf("    Input Flag of Power On: ");
			//scanf("%d", &value);

			// execute code;
			printf("Execute %s.\n",command);
			//reps = tspi_serCdmg(&muart, &cmdmsg, LASER_CTRLON_CMD);
			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("MTDHelp", command)) // set Laser Shutdown. default.
		{
			/* no use now */			
			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else if (!strcmp("CloseMTD", command)) // close current UART. default.
		{
			//printf("    Input UART ID: ");
			//scanf("%d", &value);

			break;
		}
		else if (!strcmp("SerDbgData", command)) // sent or received data ...
		{
			/* input concern data ID */
			printf("    Input Cmd ID: ");
			scanf("%d", &value);
			
			/* display data */
			printf("MTD Cmd ID %d", value);
			for(int n = 0; n < (pLD_CMDMSG[value])->txLgth; n++)
				printf("0x%x ", (*(pLD_CMDMSG[value]->pcmd)+n));
			
			printf("\n");

			printf("    Input MTD Command: ");
			scanf("%s", command);
		}
		else
		{
			printf("\nIncorrect MTD command!\n\n");
			printf("The commands should be:\n");
			printf("    \"MTDversion\" to get MTD version of Hardware and Software.\n");
			printf("    \"MTDuuid\" to get MTD UUID.\n");
			printf("    \"MTDErr\" to get MTD Error information;\n");
			printf("    \"MTDClrErr\" to get MTD Error information;\n");
			printf("    \"MTDCurntSet\" TEC commands.\n");
			printf("    \"MTDTempSet\" Temperature commands.\n\n");
			printf("    \"MTDLpCtrlSet\" Loop Controller commands.\n\n");
			printf("    \"MTDSave\" Save parameters into non-volatile memory.\n\n");
			
			printf("    \"CloseMTD\" to stop the MTD Settings.\n\n");

			printf("    Input Command: ");
			scanf("%s", command);
      		
			//printf("Unvalid MTD command!\n");
			//break;
		}
	}
	
    tspi_serial_close(hd, &muart);//serialClose(hd);
    printf("MTD UART Closed. \n"); 
    return;
}
#endif

/*******************************************************
 * It calls a python rouitne to set the temperature controller.
 * - Parameters:
 *   argc: number of parameters
 *   argv1: module name
 *   argv2: function name
 *   argv3: parameter1
 *   argv4: parameter2
 *   ......
 * 
 * *****************************************************/
// cresult = call_Python_Stitch(6, "Image_Stitching", "main", "Images", "output.jpeg","--images","--output");
				
void tempCtrll_py(int argc, char *argv1, char *argv2, char *argv3)
{
	PyObject *pName, *pModule, *pDict, *pFunc, *pValue, *pmyresult, *args, *kwargs;
	int i;

    //gpioSetMode(SER_SEL, PI_OUTPUT);
	gpioWrite(SER_SEL, SLE_TMPC); // select temperature controler
    
	// Set PYTHONPATH TO working directory used for GPS parser
	//setenv("PYTHONPATH", "/home/pi/gpsPy:/home/pi/nmea_parser-master:/home/pi/nmea_parser-master/nmea:/home/pi/nmea_parser-master/nmea/core", 1);
	setenv("PYTHONPATH", "/home/pi/mtd415py:/home/pi/mtd415lib/thorlabs-mtd415t:/home/pi/mtd415lib/thorlabs-mtd415t/thorlabs_mtd415t", 1);
	//printf("PATH: %s\n", getenv("PATH"));
	//printf("PYTHONPATH: %s\n", getenv("PYTHONPATH"));
	printf("in the ctrl_py(%d):\n   %s\n   %s\n   %s\n", argc, argv1, argv2, argv3);
	//return;

	wchar_t *program = Py_DecodeLocale(argv1, NULL);
	if (program == NULL)
	{
		fprintf(stderr, "Fatal error: cannot decode argv[0]\n");
		exit(1);
	}
	Py_SetProgramName(program); /* optional but recommended */
	// Initialize the Python Interpreter
	Py_Initialize();
	//PySys_SetPath("/home/pi/nmea_parser-master");
	//printf("PATH: %s\n", getenv("PATH"));
	//printf("PYTHONPATH: %s\n", getenv("PYTHONPATH"));
	
	// Build the name object
	pName = PyUnicode_DecodeFSDefault(argv1);
	//pName = PyUnicode_FromString("nmeaParser");

	// Load the module object
	pModule = PyImport_Import(pName);
	if(pModule == NULL)
	{
		fprintf(stderr, "Fatal error: cannot load the module\n");
		exit(1);
	}

	// pDict is a borrowed reference
	pDict = PyModule_GetDict(pModule);
	if(pDict == NULL)
	{
		fprintf(stderr, "Fatal error: cannot get a Dict\n");
		exit(1);
	}
	// pFunc is also a borrowed reference
	pFunc = PyDict_GetItemString(pDict, argv2);
	if(pFunc == NULL)
	{
		fprintf(stderr, "Fatal error: cannot get a function\n");
		exit(1);
	}
	args = PyTuple_Pack(1,PyUnicode_DecodeFSDefault(argv3));
	// kwargs = PyTuple_Pack(2,PyUnicode_DecodeFSDefault(argv5), PyUnicode_DecodeFSDefault(argv6));
	// args = Py_BuildValue("ssss", argv5, argv3, argv6, argv4);
	// kwargs = Py_BuildValue("ss", argv5, argv6);

	if (PyCallable_Check(pFunc))
	{
		//pmyresult = PyObject_CallObject(pFunc, args/*NULL*/);
		pmyresult = PyObject_CallObject(pFunc, NULL);
		i = 0;
	}
	else
	{
		PyErr_Print();
		i = 1;
		return;

	}
	PyUnicode_CheckExact(pmyresult);
	//printf("in the ctrl_py\n");

	if (PyUnicode_Check(pmyresult))
	{
		//clrscr();
		PyObject *temp_bytes = PyUnicode_AsEncodedString(pmyresult, "UTF-8", "strict"); // Owned reference
		if (temp_bytes != NULL)
		{
			char *resultStr = PyBytes_AS_STRING(temp_bytes); // Borrowed pointer
			resultStr = strdup(resultStr);
			Py_DECREF(temp_bytes);
			printf(resultStr);
			/* split string resultStr by "," */
			//char *p = strtok(resultStr, ", ");
    		//while(p)
    		//{
        	//	printf("%s \n", p); //print newline
        	//	p = strtok(NULL, ", ");
    		//}
		}
		else
		{
			printf("in the ctrl_py\n");
			return;
			// TODO: Handle encoding error.
		}
	}
	else{
		printf("no string return\n");
	}

	// Clean up
	Py_DECREF(pModule);
	Py_DECREF(pName);

	// Finish the Python Interpreter
	Py_Finalize();

	gpioWrite(SER_SEL, SLE_LDIS); // set low (borrow SLE_LDIS)

	return;
}