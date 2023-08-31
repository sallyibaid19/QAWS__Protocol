#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/*
typedef unsigned char   UINT8_T;
typedef char            TEXT_T;
typedef signed char     INT8_T;
typedef unsigned int    UINT16_T;
typedef int             INT16_T;
typedef unsigned long   UINT32_T;
typedef long            INT32_T;
typedef void            VOID_T;
typedef unsigned char		BOOL_T;

*/

#define USART_IDLE 0       // receiver is waiting for a new message to start
#define USART_RECEIVING 1  // receiving data
#define USART_NEWMESSAGE 2 // a new message has been received and reciever is now disabled
#define USART_BADMESSAGE 3 // a bad message has been received and reciever is now disabled
#define USART2_MAX_RXMESSAGE 16
#define USART2_MAX_TXDATA 15
#define ECMD_QUERY_CHECKSUM 7
#define ECMD_QUERY_CHECKSUM_LENGTH 0
#define EREPLY_QUERY_CHECKSUM 7
#define EREPLY_QUERY_CHECKSUM_LENGTH 2
// command 1 ask engine for data
#define ECMD_QUERY 1
#define ECMD_QUERY_LENGTH 0
#define EREPLY_QUERY 1
#define EREPLY_QUERY_LENGTH 8
unsigned char ballsLeft;
unsigned int position, travel, slippage;
unsigned char motorSpeed, motorLimit, direction;
unsigned char engineBuff[10];
unsigned char *p;
unsigned char usart2_txbuffer[USART2_MAX_TXDATA + 1];
unsigned char usart2_state = USART_IDLE;
unsigned char usart2_rxbuffer[USART2_MAX_RXMESSAGE];

//=======================================================================
unsigned int txreg2;
#define usart2_txready 1

// convert hex byte i to to ASC string in p (0xA5 -> "A5")
void hex2asc(unsigned char i, unsigned char *p)
{
    unsigned char c;

    c = i >> 4; // high nibble

    if (c >= 10)
    {
        *p++ = 'A' + (c - 10);
    }
    else
    {
        *p++ = '0' + c;
    }

    c = i & 0x0f; // low nibble

    if (c >= 10)
    {
        *p = 'A' + (c - 10);
    }
    else
    {
        *p = '0' + c;
    }
}
void usart2_send(unsigned char *data, unsigned char len)
{
    unsigned char checksum = 0;
    unsigned char asc[2];
    //	usart2_dir = 1;					// transmitt
    while (usart2_txready == 0)
        ; // if it does not clear down within watchdog timeout let it reset
    txreg2 = '>';

    while (len)
    {
        hex2asc(*data, asc);
        checksum -= *data;
        printf("checksum=  %#x \n", checksum);

        while (usart2_txready == 0)
            ; // if it does not clear down within watchdog timeout let it reset
        txreg2 = asc[0];
        printf("txreg2=  %#x \n", txreg2);
        while (usart2_txready == 0)
            ; // if it does not clear down within watchdog timeout let it reset
        txreg2 = asc[1];

        data++;
        len--;
    }

    hex2asc(checksum, asc);
    checksum -= *data;

    while (usart2_txready == 0)
        ; // if it does not clear down within watchdog timeout let it reset
    txreg2 = asc[0];

    while (usart2_txready == 0)
        ; // if it does not clear down within watchdog timeout let it reset
    txreg2 = asc[1];

    while (usart2_txready == 0)
        ; // if it does not clear down within watchdog timeout let it reset
    txreg2 = 0x0A;

    while (usart2_txready == 0)
        ; // if it does not clear down within watchdog timeout let it reset
    //	MillisecDelay(1);
    // usart2_dir = 0;					// back to receive
}
//======================================================================
// Send an USART message to the engine.

unsigned char WriteEngine(unsigned char command, unsigned char *command_data, unsigned char command_data_length)
{
    unsigned char i = 1;

    if (command_data_length > USART2_MAX_TXDATA)
        return 1; // fail
    usart2_txbuffer[0] = command;
    while (i <= command_data_length)
    {
        usart2_txbuffer[i] = *command_data;
        i++;
        command_data++;
    }

    usart2_send(usart2_txbuffer, i);

    return 0; // OK
}

unsigned char QueryEngine(unsigned char command, unsigned char *command_data, unsigned char command_data_length)
{
    //	usart2_reset_receiver(); // reset the receiver
    if (WriteEngine(command, command_data, command_data_length))
        return -1;

    // usart_timer_set(250);	// 250mS timeout handled in usart interrupt

    //	while(usart_timer_value())
    //	{
    if (usart2_state == USART_NEWMESSAGE)
    {
        printf("usart2_rxbuffer %#x ", usart2_rxbuffer[0]);
        return (unsigned char)usart2_rxbuffer[0];
    }
    //	}
    return -1; // failed
}
unsigned char decode_int8(unsigned char *s)
{
    unsigned char i = s[0];
    return i;
}
unsigned int decode_int16(unsigned char *s) // was unsigned
{
    unsigned int i = s[1]; // was unsigned
    i = i << 8;
    i += s[0];

    return i;
}

unsigned char QueryEngineChecksum(unsigned int *p)
{
    unsigned char i;
    i = QueryEngine(ECMD_QUERY_CHECKSUM, NULL, ECMD_QUERY_CHECKSUM_LENGTH);
    printf("i = QueryEngine =  %#x", i);
    if (i < 0)
        return -1; // query failed
    if (i == EREPLY_QUERY_CHECKSUM)
    {
        *p = decode_int16(&usart2_rxbuffer[1]);
        return 0; // OK
    }
    return -1; // error
}
unsigned char QueryEngineData(void)
{
    unsigned char rc = -1;
    unsigned char c;

    // usart1_proccmd();
    // usart1_procmsg();
    rc = QueryEngine(ECMD_QUERY, NULL, ECMD_QUERY_LENGTH);
    if (rc >= 0)
    {
        //	MillisecDelay(3);       // to allow engine to get back to receive

        if (rc == EREPLY_QUERY)
        {
            //	sysData.optoState = decode_int8(&usart2_rxbuffer[1]); // the status byte

            position = decode_int16(&usart2_rxbuffer[2]);
            //	sysData.mmTeePosition = StepsToMillimetres(position);

            slippage = decode_int16(&usart2_rxbuffer[4]);

            motorSpeed = decode_int8(&usart2_rxbuffer[6]);
            motorLimit = decode_int8(&usart2_rxbuffer[7]);
            direction = decode_int8(&usart2_rxbuffer[8]);
            rc = 0; // all OK
        }
        else
        {
            rc = -1; // bad responce
        }
    }
    return rc;
}
int main()
{
    printf("start.....");
    usart2_txbuffer[0] = ECMD_QUERY;
    unsigned char *data;
    unsigned char checksum = 0;
    unsigned char asc[2];
     unsigned char i;
   unsigned int j = 0xffff;
    usart2_state = USART_NEWMESSAGE;

    hex2asc(usart2_txbuffer[0], asc);
    // hex2asc(*usart2_txbuffer, asc);
    checksum = usart2_txbuffer[0];
    printf("checksum=  %#x \n", checksum);
     printf("Hexadecimal value: %02X\n", usart2_txbuffer[0]);
    printf("ASCII representation: %c%c\n", asc[0], asc[1]);

    i = WriteEngine(ECMD_QUERY, NULL, ECMD_QUERY_LENGTH);
    printf("i = QueryEngine =  %#x \n", i);

    
#if(0)
    unsigned int j = 0xffff;
    usart2_state = USART_NEWMESSAGE;
    unsigned char rc = -1;
    unsigned char i;
    //*****************
    

    unsigned char c;
    rc = QueryEngine(ECMD_QUERY, NULL, ECMD_QUERY_LENGTH);
    // rc = QueryEngine(1, NULL , 0);
    printf("rc = QueryEngine=  %c \n", rc);
    i = WriteEngine(ECMD_QUERY, NULL, ECMD_QUERY_LENGTH);
    printf("i = QueryEngine =  %c \n", i);
    QueryEngineChecksum(&j);
    printf("QueryEngineChecksum =  %s", QueryEngineChecksum(&j));
    //****************************
 usart2_txbuffer[0] = ECMD_QUERY;
    unsigned char *data;
    unsigned char checksum = 0;
    unsigned char asc[2];
    
   
    hex2asc(usart2_txbuffer[0], asc);
    // hex2asc(*usart2_txbuffer, asc);
    checksum = usart2_txbuffer[0];
    printf("checksum=  %c \n", checksum);
     printf("Hexadecimal value: %02X\n", usart2_txbuffer[0]);
    printf("ASCII representation: %c%c\n", asc[0], asc[1]);
#endif
    return 0;
}