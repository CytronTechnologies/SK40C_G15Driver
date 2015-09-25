/* 
 * File:   ARDUINOG15.h
 * Author: yh
 *
 * Created on September 5, 2012, 1:57 PM
 */
#include <htc.h>
typedef unsigned char byte;
typedef unsigned short word;

//definition:
#warning " Pls define Correct G15 Driver Control Pin here!"
#define CTRL_G15 RC4
#define _XTAL_FREQ 20000000

#define ConvertAngle2Pos(Angle) (word) ((word)(Angle)*1088UL/360UL)
#define ConvertPos2Angle(Pos) (float)(Pos)*360.0/1088.0
#define ConvertTime(Time) (word)(Time*10UL)

#define ON 1
#define OFF 0
#define CW 1
#define CCW 0

//Alarm Mask	1111 1111
#define ALARM_INST              0x40
#define ALARM_OVERLOAD		0x20
#define ALARM_CHECKSUM		0x10
#define ALARM_RANGE             0x08
#define ALARM_OVERHEAT		0x04
#define ALARM_ANGLELIMIT 	0x02
#define ALARM_VOLTAGE		0x01


//******************************************************************
//*	INSTRUCTIONS
//******************************************************************
#define iPING 		0x01 //obtain a status packet
#define iREAD_DATA	0x02 //read Control Table values
#define iWRITE_DATA	0x03 //write Control Table values
#define iREG_WRITE 	0x04 //write and wait for ACTION instruction
#define iACTION 	0x05 //triggers REG_WRITE instruction
#define iRESET 		0x06 //set factory defaults
#define iSYNC_WRITE     0x83 //simultaneously control multiple actuators

#define setTX()  CTRL_G15=1
#define setRX()  CTRL_G15=0

word send_packet(byte ID, char inst, byte* data, byte param_len);
//*=========Wheel Mode=====================================================================================
//360 degree continous rotation. change CW and CCW Angle Limits to same value
word SetWheelMode(byte ServoID); 	//10 bits speed (0-1024)
word ExitWheelMode(byte ServoID);
word SetWheelSpeed(byte ServoID, word Speed, byte CW_CCW); 
//*=========Normal Positioning Mode========================================================================
//(Rotation limited by Angle Limit and Direction of Rotation determined by operation section of Angle Limit)
word SetPos(byte ServoID, word Position, byte Write_Reg);
//*========Direction Positioning Mode======================================================================
//(Rotation direction and angle is NOT limited by Angle Limit Control Register value)
word RotateCW (byte ServoID, word Position, byte Write_Reg);
word RotateCCW (byte ServoID, word Position, byte Write_Reg);
//*=======Torque Enable and Speed Control==================================================================
word SetTorqueOnOff(byte ServoID, byte on_off, byte Write_Reg);
word SetSpeed(byte ServoID, word Speed, byte Write_Reg);
word SetTimetoGoal(byte ServoID, word Time,byte Write_Reg);

//*=======Set Maximum Limits===============================================================================
word SetAngleLimit(byte ServoID, word CW_angle, word CCW_angle);
word SetTorqueLimit(byte ServoID, word TorqueLimit); //in RAM area
word SetTemperatureLimit(byte ServoID, byte Temperature);
word SetVoltageLimit(byte ServoID, byte VoltageLow, byte VoltageHigh);

word SetID(byte ServoID, byte NewID);

word SetLED(byte ServoID, byte on_off, byte Write_Reg);
word SetAlarmLED(byte ServoID, byte AlarmLED);
word SetAlarmShutDown(byte ServoID, byte Alarm);

//*========Servo Positioning Control Parameters============================================================
word SetMarginSlopePunch(byte ServoID, byte CWMargin, byte CCWMargin, byte CWSlope, byte CCWSlope, word Punch);

word SetBaudRate(byte ServoID, long bps);

word FactoryReset(byte ServoID);

word Ping(byte ServoID, byte* data);

word GetPos(byte ServoID, byte* data);
word GetSpeed(byte ServoID, byte* data);
word GetLoad(byte ServoID, byte* data);
word GetVoltage(byte ServoID, byte* data);
word GetTemperature(byte ServoID, byte* data);
word GetTorqueOnOff(byte ServoID, byte* data);
word IsMoving(byte ServoID, byte* data);

void SetAction(void);


extern void UART_SEND(unsigned char data);
extern unsigned char UART_REC(void);
extern unsigned char ERR_FLAG;


enum{
      MODEL_NUMBER_L,                   // 0x00
      MODEL_NUMBER_H,                   // 0x01
      VERSION,                          // 0x02
      ID, 				// 0x03
      BAUD_RATE, 			// 0x04
      RETURN_DELAY_TIME, 		// 0x05
      CW_ANGLE_LIMIT_L, 		// 0x06
      CW_ANGLE_LIMIT_H, 		// 0x07
      CCW_ANGLE_LIMIT_L, 		// 0x08
      CCW_ANGLE_LIMIT_H, 		// 0x09
      RESERVED1, 			// 0x0A
      LIMIT_TEMPERATURE, 		// 0x0B
      DOWN_LIMIT_VOLTAGE, 		// 0x0C
      UP_LIMIT_VOLTAGE, 		// 0x0D
      MAX_TORQUE_L,                     // 0x0E
      MAX_TORQUE_H,                     // 0x0F
      STATUS_RETURN_LEVEL, 		// 0x10
      ALARM_LED, 			// 0x11
      ALARM_SHUTDOWN,                   // 0x12
      RESERVED2, 			// 0x13
      DOWN_CALIBRATION_L, 		// 0x14
      DOWN_CALIBRATION_H, 		// 0x15
      UP_CALIBRATION_L, 		// 0x16
      UP_CALIBRATION_H, 		// 0x17
      TORQUE_ENABLE,                    // 0x18
      LED, 				// 0x19
      CW_COMPLIANCE_MARGIN,             // 0x1A
      CCW_COMPLIANCE_MARGIN,            // 0x1B
      CW_COMPLIANCE_SLOPE, 		// 0x1C
      CCW_COMPLIANCE_SLOPE,             // 0x1D
      GOAL_POSITION_L,                  // 0x1E
      GOAL_POSITION_H,                  // 0x1F
      MOVING_SPEED_L,                   // 0x20
      MOVING_SPEED_H,                   // 0x21
      TORQUE_LIMIT_L,                   // 0x22
      TORQUE_LIMIT_H,                   // 0x23
      PRESENT_POSITION_L, 		// 0x24
      PRESENT_POSITION_H, 		// 0x25
      PRESENT_SPEED_L,                  // 0x26
      PRESENT_SPEED_H,                  // 0x27
      PRESENT_LOAD_L,                   // 0x28
      PRESENT_LOAD_H,                   // 0x29
      PRESENT_VOLTAGE,                  // 0x2A
      PRESENT_TEMPERATURE, 		// 0x2B
      REGISTERED_INSTRUCTION,           // 0x2C
      RESERVE3, 			// 0x2D
      MOVING,                           // 0x2E
      LOCK,                             // 0x2F
      PUNCH_L,                          // 0x30
      PUNCH_H                           // 0x31
};


word send_packet(byte ID, char inst, byte* data, byte param_len)
{
    int i;
    byte packet_len = 0;
    byte TxBuff[16];
    byte Status[16];
    byte checksum; 		//Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
    word error=0;
    
    setTX(); 					//set for transmit mode

    checksum=0;                         //clear Checksum val
    TxBuff[0] = 0xFF;			//0xFF not included in checksum
    TxBuff[1] = 0xFF;
    TxBuff[2] = ID; 		checksum += TxBuff[2];	//0-254, 0xFE = broadcast id
    TxBuff[3] = param_len + 2;	checksum += TxBuff[3];	//INSTRUCTION + PARAMETERS( START ADDR + VALUES ) + CHECKSUM                                                                                                          //0xFF and ID not included
    TxBuff[4] = inst;		checksum += TxBuff[4];	//

    for(i = 0; i < param_len; i++)
    {
        TxBuff[i+5] = data[i];
        checksum += TxBuff[i+5];
    }
    TxBuff[i+5] = ~checksum; 		//Checksum with Bit Inversion

    packet_len = TxBuff[3] + 4;		//# of bytes for the whole packet

    for(i=0; i<packet_len;i++)          //sending out the packet
    {
        UART_SEND(TxBuff[i]);           //uart function must wait for complete transmission
    }

      
    // we'll only get a reply if it was not broadcast
    if((ID != 0xFE) || (inst == iPING))
    {
        if(inst == iREAD_DATA)			//if a read instruction
        {
            param_len = data[1];
            packet_len = data[1] + 6;           // data[1] = length of the data to be read
        }
        else
        {
            packet_len = 6;                     //return status packet always 6 bytes
        }

        setRX();                            //set to receive mode
        error=0;                            //clear error

        for (i = 0; i < packet_len ; i++)   //start receive packet
    	{
            Status[i] = UART_REC();
            if(ERR_FLAG)        //check for time out
            {
                error|=0x0100;  //time out error or incomplete packet
               break;           //break from receive
            }
        }
        if ((Status[0] !=0xFF) || (Status[1] != 0xFF))
        {
            error|=0x0200;
            //return (error);			//1000 00001	//wrong header
        }
        if (Status[2] != ID)
        {
            error|=0x0400;
            //return (error);			//ID mismatch
        }
        if(Status[4] != 0)
        {
            error|=(word)Status[4];
            //return(error);
        }

        //calculate checksum
        checksum=0;
        for(i = 2; i < packet_len; i++)	//whole package including checksum but excluding header
        {
            checksum += Status[i];		//correct end result must be 0xFF
        }
        if(checksum != 0xFF)
        {
            error |= 0x0800;       		//return packet checksum mismatch error
            //return (error);
        }
        if(Status[4]==0x00 && (error&0x0100)==0x00)	//copy data only if there is no packet error
        {
            if(inst == iPING)
            {
                // ID is passed to the data[0]
                data[0] = Status[2];
            }
            else if(inst == iREAD_DATA)
            {
                for(i = 0; i < param_len; i++)  //Requested Parameters
                    data[i] = (byte) Status[i+5];
            }
        }
    }

    return(error); // return error code
}

word SetWheelMode(byte ServoID)	//10 bits speed (0-1024)
{
    word Error=0;

    Error=SetAngleLimit(ServoID, 0,0);	//enable wheel mode
    if(Error!=0) return (Error);

    Error = SetTorqueOnOff(ServoID, 1, iWRITE_DATA);	//enable torque

    return(Error);
}

word ExitWheelMode(byte ServoID)
{
	return(SetAngleLimit(ServoID, 0,1087));  //reset to default angle limit
}

word SetWheelSpeed(byte ServoID, word Speed, byte CW_CCW)
{
	Speed=Speed&0x03FF; 	//0000 0011 1111 1111 eliminate bits which are non speed
	if(CW_CCW){		//if CW
		Speed=Speed|0x0400;
	}
	return(SetSpeed(ServoID, Speed,iWRITE_DATA));
}

word SetPos(byte ServoID, word Position, byte Write_Reg)
{
    byte TxBuff[3];

    TxBuff[0] = GOAL_POSITION_L;	//Control Starting Address
    TxBuff[1] = (byte) (Position&0x00FF);  			//goal pos bottom 8 bits
    TxBuff[2] = (byte)(Position>>8); 			//goal pos top 8 bits

    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 3));
}
word RotateCW (byte ServoID, word Position, byte Write_Reg)
{

    Position = Position|0xC000;  //directional positioning mode CW

    return(SetPos(ServoID, Position, Write_Reg));

}
word RotateCCW (byte ServoID, word Position, byte Write_Reg)
{
    Position = Position|0x8000;  //directional positioning mode
    Position = Position&0xBFFF;  //CCW	1011 1111 1111 1111

    return(SetPos(ServoID, Position, Write_Reg));

}
word SetTorqueOnOff(byte ServoID, byte on_off, byte Write_Reg)
{
    byte TxBuff[2];

    TxBuff[0] = TORQUE_ENABLE;		//Control Starting Address
    TxBuff[1] = on_off; 			//ON = 1, OFF = 0

    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 2));
}

word SetSpeed(byte ServoID, word Speed, byte Write_Reg)
{
    byte TxBuff[3];

    TxBuff[0] = MOVING_SPEED_L;		//Control Starting Address
    TxBuff[1] = (byte)(Speed&0x00FF);			//speed bottom 8 bits
    TxBuff[2] = (byte)(Speed>>8); 			//speed top 8 bits

    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 3));
}

word SetTimetoGoal(byte ServoID, word Time, byte Write_Reg)
{
	Time = Time&0x0FFF; 	//			0000 1111 1111 1111
	Time = Time|0x8000; 	//bit 15 represents the time to goal pos mode

	return(SetSpeed(ServoID, Time, Write_Reg));
}

word SetAngleLimit(byte ServoID, word CW_angle, word CCW_angle)
{
    byte TxBuff[5];
    word error;

    TxBuff[0] = CW_ANGLE_LIMIT_L;			//Control Starting Address
    TxBuff[1] = (byte)(CW_angle&0x00FF);  	//CW limit bottom 8 bits
    TxBuff[2] = (byte)(CW_angle>>8); 			//CW limit top 8 bits
    TxBuff[3] = (byte)(CCW_angle&0x00FF); 	//CCW limit bottom 8 bits
    TxBuff[4] = (byte)(CCW_angle>>8); 		//CCW limit top 8 bits

    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 5);
    __delay_ms(10);

    // write the packet, return the error code
    return(error);
}

word SetTorqueLimit(byte ServoID, word TorqueLimit)
{
    byte TxBuff[3];

    TxBuff[0] = TORQUE_LIMIT_L;				//Starting Address
    TxBuff[1] = (byte)(TorqueLimit&0x00FF);  	//Torque limit bottom 8 bits
    TxBuff[2] = (byte)(TorqueLimit>>8); 		//Torque limit top 8 bits

    // write the packet, return the error code
    return(send_packet(ServoID, iWRITE_DATA, TxBuff, 3));
}

word SetTemperatureLimit(byte ServoID, byte Temperature)
{
    byte TxBuff[2];
    word error;

    TxBuff[0] = LIMIT_TEMPERATURE;			//Starting Address
    TxBuff[1] = Temperature;  				//temperature

    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
    __delay_ms(10);
    // write the packet, return the error code
    return(error);
}

word SetVoltageLimit(byte ServoID, byte VoltageLow, byte VoltageHigh)
{
    byte TxBuff[3];
    word error;

    TxBuff[0] = DOWN_LIMIT_VOLTAGE;				//Starting Address
    TxBuff[1] = VoltageLow;  	//lower voltage limit
    TxBuff[2] = VoltageHigh; 		//Higher voltage limit

    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 3);
    __delay_ms(10);
    // write the packet, return the error code
    return(error);
}

word SetID(byte ServoID, byte NewID)
{
    byte TxBuff[2];
    word error;
    TxBuff[0] = ID;			//Control Starting Address
    TxBuff[1] = NewID;

    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
    __delay_ms(10);
    return(error);
}

word SetLED(byte ServoID, byte on_off, byte Write_Reg)
{
    byte TxBuff[2];

    TxBuff[0] = LED;				//Control Starting Address
    TxBuff[1] = on_off; 			//ON = 1, OFF = 0

    // write the packet, return the error code
    return(send_packet(ServoID, Write_Reg, TxBuff, 2));
}

word SetAlarmLED(byte ServoID, byte AlarmLED)
{
    byte alarmval=0x00;
    byte TxBuff[2];
    word error;

    alarmval=alarmval|AlarmLED;

    TxBuff[0] = ALARM_LED;		//Control Starting Address
    TxBuff[1] = alarmval;			//alarm val

    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
    __delay_ms(10);
    
    // write the packet, return the error code
    return(error);
}

word SetAlarmShutDown(byte ServoID, byte Alarm)
{
    byte alarmval=0x00;
    byte TxBuff[2];
    word error;

    alarmval=alarmval|Alarm;

    TxBuff[0] = ALARM_SHUTDOWN;		//Control Starting Address
    TxBuff[1] = alarmval;			//alarm

    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
    __delay_ms(10);
    // write the packet, return the error code
    return(error);
}


word SetMarginSlopePunch(byte ServoID, byte CWMargin, byte CCWMargin, byte CWSlope, byte CCWSlope, word Punch)
{
    byte TxBuff[5];
    word error=0;

    TxBuff[0] = CW_COMPLIANCE_MARGIN;		//Control Starting Address
    TxBuff[1] = CWMargin;
    TxBuff[2] = CCWMargin;
    TxBuff[3] = CWSlope;
    TxBuff[4] = CCWSlope;

    // write the packet, return the error code
    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 5);

    if(error!=0)
            return (error);

    TxBuff[0] = PUNCH_L;				//Control Starting Address
    TxBuff[1] = (byte)(Punch&0x00FF);		//punch Lower 8 bits
    TxBuff[2] = (byte)(Punch>>8); 		//punch Higher 8 bits

    // write the packet, return the error code
    error=send_packet(ServoID, iWRITE_DATA, TxBuff, 3);

    return(error);
}

word SetBaudRate(byte ServoID, long bps)
{
    byte TxBuff[2];
    word error;

    TxBuff[0] = BAUD_RATE;		//Control Starting Address
    TxBuff[1] = (2000000/bps)-1;	//Calculate baudrate
					//Speed (BPS) = 32M / (16*(n + 1))=2000000/(n+1)

    error = send_packet(ServoID, iWRITE_DATA, TxBuff, 2);
    __delay_ms(10);
    // write the packet, return the error code
    return(error);
}

//******************************************************************
//*	RESET TO FACTORY SETTINGS
//* 	eg:	FactoryReset(1,1);// Reset servo 1
//******************************************************************
word FactoryReset(byte ServoID)
{
    byte TxBuff[1];	//dummy byte
    word error;

    error=send_packet(ServoID, iRESET, TxBuff, 0);
    __delay_ms(100);

    // write the packet, return the error code
    return(error);
}

word Ping(byte ServoID, byte* data)
{
	// write the packet, return the error code
    return(send_packet(ServoID, iPING, data, 0));
}

word GetPos(byte ServoID, byte* data)
{
    data[0] = PRESENT_POSITION_L;	// Starting Addr where data to be read
    data[1] = 0x02;			// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word GetSpeed(byte ServoID, byte* data)
{
    data[0] = PRESENT_SPEED_L;	// Starting Addr where data to be read
    data[1] = 0x02;			// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word GetLoad(byte ServoID, byte* data)
{
    data[0] = PRESENT_LOAD_L;	// Starting Addr where data to be read
    data[1] = 0x02;			// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word GetVoltage(byte ServoID, byte* data)
{
    data[0] = PRESENT_VOLTAGE;	// Starting Addr where data to be read
    data[1] = 0x01;			// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word GetTemperature(byte ServoID, byte* data)
{
    data[0] = PRESENT_TEMPERATURE;	// Starting Addr where data to be read
    data[1] = 0x01;			// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word GetTorqueOnOff(byte ServoID, byte* data)
{
    data[0] = TORQUE_ENABLE;		// Starting Addr where data to be read
    data[1] = 0x01;					// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

word IsMoving(byte ServoID, byte* data)
{
    data[0] = MOVING;				// Starting Addr where data to be read
    data[1] = 0x01;					// # of bytes to be read

    return (send_packet(ServoID, iREAD_DATA, data, 2));
}

void SetAction(void)
{
    byte TxBuff[1];	//dummy byte

    send_packet(0xFE, iACTION, TxBuff, 0);
}
