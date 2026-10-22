
#include <stdio.h>
#include <memory.h>

#include <unistd.h>
#include "RFT_UART_SAMPLE.h"

//////////////////////////////////////////////////////////////////////////////
// Global Functions
CRT_RFT_UART::CRT_RFT_UART( void )
{
	initialize_variables();		
}

void CRT_RFT_UART::initialize_variables(void)
{
	m_bConnected = FALSE;		        // Open/Closed indicating flag

	m_nCurrMode = CMD_NONE;
	m_bIsRcvd_Response_Pkt = false;

	m_nRcvdBufferIdx = 0;
	
	memset(m_rcvdBuff, 0, sizeof(unsigned char)*RFT_UART_RX_BUFF_SIZE);
	memset(m_txBuff, 0, sizeof(unsigned char)*RFT_UART_TX_BUFF_SIZE);
}


CRT_RFT_UART::~CRT_RFT_UART( void )
{
	closePort( );
}

/*
bool CRT_RFT_UART::openPort( char *devName, BYTE bPort, DWORD rate, DWORD byteSize )
{
	if( m_bConnected )
		return true;
	if( !createComPort( devName, bPort ) )
	{
		printf("com port creation error\n");
		return false;
	}

	m_bConnected = setupConnection( rate, byteSize );
	if (!m_bConnected )
	{
		return false;
	}

	int result = pthread_create( &m_readThread, NULL, readThread, (void *)this );
	if( result != 0 ) 
	{
		printf("\nCan't create read thread");
		closePort();
	}
	else
	{ // Thread Creation Succeeded
		; // Thread Priority Setting.
	}

	return true;
}
*/
bool CRT_RFT_UART::openPort(char *devName, BYTE /*bPort*/, DWORD rate, DWORD byteSize)
{
    if (m_bConnected)
        return true;

  
    if (!createComPort(devName,0)) 
    {
        printf("com port creation error\n");
        return false;
    }

   
    m_bConnected = setupConnection(rate, byteSize);
    if (!m_bConnected)
    {
        printf("setup connection failed\n");
        return false;
    }

    
    return true;
}
/*
bool CRT_RFT_UART::createComPort( char *devName, BYTE bPort )
{
	char szPort[20];

	//sprintf( szPort, "/dev/ttyS%d", bPort);
	sprintf( szPort, "%s%d", devName, bPort);

	printf( "%s\n", szPort ); // for test...

	// NON-Blocking Mode File Open..
	fd = open( szPort, O_RDWR | O_NOCTTY | O_NDELAY);

	if( fd == -1 )
		return false;
	else
		fcntl( fd, F_SETFL, O_NDELAY );

	return true;
}
*/
bool CRT_RFT_UART::createComPort(char *devName, BYTE bPort)
{
    (void)bPort;
    char szPort[64];

 
    snprintf(szPort, sizeof(szPort), "%s", devName);

    //printf("%s\n", szPort); 

    
    fd = open(szPort, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("open"); 
        return false;
    }
    else
    {
        fcntl(fd, F_SETFL, O_NDELAY);
    }

    return true;
}

bool CRT_RFT_UART::setupConnection(DWORD dwBaudrate, DWORD bByteSize )
{
	struct termios oldtio, newtio;

	tcgetattr(fd,&oldtio); /* save current serial port settings */
	bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

	newtio.c_cflag = dwBaudrate | bByteSize | CLOCAL | CREAD;
	tcsetattr(fd,TCSANOW,&newtio);

	return true;
}

void CRT_RFT_UART::discardInput( void )
{
	tcflush(fd, TCIFLUSH);
	m_nRcvdBufferIdx = 0;
}

bool CRT_RFT_UART::closePort( void )
{
  if( !m_bConnected )
		return false;

	if( m_bConnected )
	{
		// set connected flag to FALSE
		m_bConnected = FALSE;
		// disable event notification and wait for thread to halt

		close( fd );
	}
	
	return true;
}

bool CRT_RFT_UART::isConnected( void )
{
	return (m_bConnected);
}

bool CRT_RFT_UART::readWorker( void )        // read worker.
{

	#define BUFF_SIZE 1024

	int      nReceived;
	char 	   buff[BUFF_SIZE];
	
	if( m_bConnected )
	{
		// printf("RFT UART IF READ THREAD START...\n");
		nReceived = 0;
		nReceived = read( fd, buff, BUFF_SIZE );

		if( nReceived <= 0 )
		{
			return false;
		}
		
		// data copy
		for( int i = 0; i < nReceived; i++)
		{
			m_rcvdBuff[m_nRcvdBufferIdx + i] = buff[i];
		}
		m_nRcvdBufferIdx += nReceived;

		if (m_nCurrMode == CMD_NONE)
		{
			m_nRcvdBufferIdx = 0;
			// printf("CMD NONE: Serial data received\n");
			return false;
		}

		// wait packet.... 
		if (m_nRcvdBufferIdx < UART_RESPONSE_PACKET_SIZE)
		{
			// printf("RECEIVED: %d\n", m_nRcvdBufferIdx);
			return false;
		}

		// find SOP
		int found_idx = -1;
		for (int i = 0; i < m_nRcvdBufferIdx; i++)
		{
			if (m_rcvdBuff[i] == SOP)
			{
				found_idx = i;
				break;
			}
		}

		if (found_idx == -1)
		{
			m_nRcvdBufferIdx = 0;
			// printf("SOP NOT FOUNDED\n");
			return false;
		}

		// if the index of SOP is not first(0), shift received data....
		if (found_idx != 0) 
		{
			for (int i = 0; i < (m_nRcvdBufferIdx - found_idx); i++)
			{
				m_rcvdBuff[i] = m_rcvdBuff[i + found_idx];
			}
			// printf("SHIFT\n");
			m_nRcvdBufferIdx = m_nRcvdBufferIdx - found_idx;
		}

		// packet handling....
		if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE )
		{
			RFT_Data_Handler();
			// printf("RFT UART IF READ THREAD FINISHED...\n");
			return true;
		}
		else
		{
			// printf("RFT UART IF READ THREAD FAILED...\n");
		}		
	}
	
	return false;

}

void CRT_RFT_UART::RFT_Data_Handler( void )
{
	switch (m_nCurrMode)
	{
	case CMD_GET_PRODUCT_NAME:
		rcvd_ProductName();
		
		printf("%s\n", m_RFT_IF_PACKET.m_rcvd_product_name );
		
		break;

	case CMD_GET_SERIAL_NUMBER:
		rcvd_SerialNumber();

		printf("%s\n", m_RFT_IF_PACKET.m_rcvd_serial_number );

		break;

	case CMD_GET_FIRMWARE_VER:
		rcvd_Firmwareversion();

		printf("%s\n", m_RFT_IF_PACKET.m_rcvd_firmware_version );

		break;

	case CMD_SET_COMM_BAUDRATE:
		rcvd_Response_Set_Comm_Speed();
		break;

	case CMD_GET_COMM_BAUDRATE:
		rcvd_CommSpeed();
		break;

	case CMD_SET_FT_FILTER:
		rcvd_Response_Set_FT_Filter_Type();
		break;

	case CMD_GET_FT_FILTER:
		rcvd_FT_Filter_Type();
		break;

	case CMD_FT_ONCE:
		rcvd_FT();

		//printf("%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f\r",
			//m_RFT_IF_PACKET.m_rcvdForce[0], m_RFT_IF_PACKET.m_rcvdForce[1], m_RFT_IF_PACKET.m_rcvdForce[2],
			//m_RFT_IF_PACKET.m_rcvdForce[3], m_RFT_IF_PACKET.m_rcvdForce[4], m_RFT_IF_PACKET.m_rcvdForce[5] );
		break;

	case CMD_FT_CONT:
		rcvd_FT();
		
		//printf("%0.3f, %.03f, %.03f, %0.3f, %.03f, %.03f\r",
			//m_RFT_IF_PACKET.m_rcvdForce[0], m_RFT_IF_PACKET.m_rcvdForce[1], m_RFT_IF_PACKET.m_rcvdForce[2],
			//m_RFT_IF_PACKET.m_rcvdForce[3], m_RFT_IF_PACKET.m_rcvdForce[4], m_RFT_IF_PACKET.m_rcvdForce[5] );
		
		break;

	case CMD_FT_CONT_STOP:
		break;

	case CMD_SET_CONT_OUT_FRQ:
		rcvd_Response_Set_FT_Cont_Interval();
   
		break;

	case CMD_GET_CONT_OUT_FRQ:
		rcvd_FT_Cont_Interval();
		break;

	case CMD_GET_OVERLOAD_COUNT:
		rcvd_FT_Overload_Count();
		break;

	default: 
		break;
	}
}

bool CRT_RFT_UART::writeBuffer( BYTE* lpBuffer, DWORD dwBytesToWrite, LPDWORD lpdwBytesWritten )
{
	DWORD     dwBytesWritten = 0;
	DWORD     dwTotalWritten = 0;
	dwTotalWritten = 0;

	// WRITE
	while( (dwBytesToWrite - dwTotalWritten) > 0 )
	{
		dwBytesWritten = write( fd, (lpBuffer + dwTotalWritten), (dwBytesToWrite - dwTotalWritten) );
		if( dwBytesWritten )
		{
			dwTotalWritten += dwBytesWritten;
		}
		else
		{
			usleep( 100 );
		}
	}
	*lpdwBytesWritten = dwTotalWritten;

	return true;
}


bool CRT_RFT_UART::rqst_ProductName( void )
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_product_name(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_SerialNumber(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_serial_name(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::rqst_Firmwareverion(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_firmware_version(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_CommSpeed(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_comm_baudrate(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];
	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_filter_type(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_force_once(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Continuous(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_force(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Stop(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_stop_force_out(m_txBuff);

	m_bIsRcvd_Response_Pkt = true; // FT output stop command don't have response packet.
	m_nCurrMode = CMD_NONE;

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_output_frq(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::rqst_FT_OverloadCnt(void)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_read_overload_count(m_txBuff);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_Comm_Speed(int comm_speed)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_comm_baudrate(m_txBuff, comm_speed);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_FT_Filter_Type(int filter_type, int sub_type)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_filter_type(m_txBuff, filter_type, sub_type);


	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}


bool CRT_RFT_UART::set_FT_Cont_Interval(int interval)
{
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0;
	}

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_output_frq(m_txBuff, interval);

	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);
	//printf("written: %d\n", written);
	return result;
}

bool CRT_RFT_UART::set_FT_Bias(int is_on)
{
	/* Bias setting command don't have response packet
	if (m_nRcvdBufferIdx)
	{
		printf("BUFF IDX CLEAR - NEW CMD SENDING\n");
		m_nRcvdBufferIdx = 0; 
	}
	*/

	int txPktSize = UART_COMMAND_PACKET_SIZE;

	m_RFT_IF_PACKET.UPG_set_bias(m_txBuff, is_on);

	/* Bias setting command don't have response packet
	m_bIsRcvd_Response_Pkt = false;
	m_nCurrMode = m_txBuff[1];
	*/

	DWORD written;
	bool result = (bool)writeBuffer(m_txBuff, txPktSize, &written);

	//printf("written: %d\n", written);
	return result;
}


void CRT_RFT_UART::rcvd_ProductName(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_PRODUCT_NAME))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_SerialNumber(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_SERIAL_NUMBER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Firmwareversion(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_FIRMWARE_VER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_CommSpeed(void)
{

	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_COMM_BAUDRATE))
		{
			m_bIsRcvd_Response_Pkt = true;

			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_FT_FILTER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

struct timeval start_point, end_point;
unsigned int rcvdCnt = 0;
bool isFirst = true;

void CRT_RFT_UART::rcvd_FT(void)
{
	int pkt_size = UART_RESPONSE_PACKET_SIZE;
	int num_of_rcvd_pkt = m_nRcvdBufferIdx / pkt_size;
	int num_of_next_pkt_data = m_nRcvdBufferIdx % pkt_size;
	int transfer_data_size;
	int source_start_idx;

	// if the number of received packet is more than 1, 
	// move the last packet data to first location of buffer to interprete the last received data.
	if (num_of_rcvd_pkt > 1) 
	{
		transfer_data_size = pkt_size + num_of_next_pkt_data;
		source_start_idx = (num_of_rcvd_pkt - 1) * pkt_size;

		for (int i = 0; i < transfer_data_size; i++)
			m_rcvdBuff[i] = m_rcvdBuff[source_start_idx + i];

		//printf("DATA TRANSFER - BEFORE\n");
	}

	if (m_nRcvdBufferIdx >= pkt_size)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, m_nCurrMode))
		{
			m_bIsRcvd_Response_Pkt = true;

			// if received data size bigger than packet size,
			// shift that to first location of buffer.
			if (num_of_next_pkt_data)
			{
				transfer_data_size = num_of_next_pkt_data;
				source_start_idx = pkt_size;
				for (int i = 0; i < transfer_data_size; i++)
					m_rcvdBuff[i] = m_rcvdBuff[source_start_idx + i];

				//printf("DATA TRANSFER - AFTER\n");

				m_nRcvdBufferIdx = num_of_next_pkt_data;
			}
			else
			{
				m_nRcvdBufferIdx = 0; 
			}
			
			/*
			// for receive counting test...
			//////////////////////////////////////////
			if (isFirst)
			{
				gettimeofday(&start_point, NULL); // get start time.
				rcvdCnt = num_of_rcvd_pkt;
				isFirst = false;
			}
			else
			{
				rcvdCnt += num_of_rcvd_pkt;
				gettimeofday(&end_point, NULL); // get end time
				
				double passedTime = (double)(end_point.tv_sec)+(double)(end_point.tv_usec)/1000000.0 
				                  - (double)(start_point.tv_sec)-(double)(start_point.tv_usec)/1000000.0;

				if (passedTime > 10.0)
				{
					printf("RDV TEST: [%d][%f]\n", rcvdCnt, passedTime);
					isFirst = true;
				}
			}
			////////////////////////////////////////////
			*/
			
		}
	}
}



void CRT_RFT_UART::rcvd_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_CONT_OUT_FRQ))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Response_Set_Comm_Speed(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_COMM_BAUDRATE))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Response_Set_FT_Filter_Type(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_FT_FILTER))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_FT_Overload_Count(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_GET_OVERLOAD_COUNT))
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}

void CRT_RFT_UART::rcvd_Response_Set_FT_Cont_Interval(void)
{
	if (m_nRcvdBufferIdx >= UART_RESPONSE_PACKET_SIZE)
	{
		if (m_RFT_IF_PACKET.UART_packet_processing(m_rcvdBuff, CMD_SET_CONT_OUT_FRQ)) // 
		{
			m_bIsRcvd_Response_Pkt = true;
			m_nRcvdBufferIdx = 0;
		}
	}
}


//////////////////////////////////////////////////////////////////////////////
// END OF FILE
