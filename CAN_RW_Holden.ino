/*

   )     )   (    (              )                         (                  *         )       (     (    (     
( /(  ( /(   )\ ) )\ )        ( /(                         )\ )   (  (      (  `     ( /(    (  )\ )  )\ ) )\ )  
)\()) )\()) (()/((()/(   (    )\())  (   (   (     (   (  (()/(   )\))(   ' )\))(    )\()) ( )\(()/( (()/((()/(  
((_)\ ((_)\   /(_))/(_))  )\  ((_)\   )\  )\  )\    )\  )\  /(_)) ((_)()\ ) ((_)()\  ((_)\  )((_)/(_)) /(_))/(_)) 
_((_)  ((_) (_)) (_))_  ((_)  _((_) ((_)((_)((_)  ((_)((_)(_))_| _(())\_)()(_()((_)   ((_)((_)_(_))_ (_)) (_))   
| || | / _ \ | |   |   \ | __|| \| | \ \ / / | __| \ \ / / | |_   \ \((_)/ /|  \/  |  / _ \ | _ )|   \|_ _||_ _|  
| __ || (_) || |__ | |) || _| | .` |  \ V /  | _|   \ V /  | __|   \ \/\/ / | |\/| | | (_) || _ \| |) || |  | |   
|_||_| \___/ |____||___/ |___||_|\_|   \_/   |___|   \_/   |_|      \_/\_/  |_|  |_|  \___/ |___/|___/|___||___|
-----------------------------------------------------------------------------------------------------------------
-----------------------------------------------------------------------------------------------------------------

----------------
Hardware needed:
----------------
1 x MCP2515 CAN Bus Module TJA1050 Receiver
1 x OBDII Connector
1 x Arduino Nano, Uno, or Mega (Uno is best bang for buck)


----------------
Arduino Libraries needed:
----------------
MCP_CAN https://www.arduino.cc/reference/en/libraries/mcp_can/

----------------
Connections:
----------------
Pin 02 - Int (Interrupt, "I Have Data" pin)
Pin 10 - CS (SS) (Chip Select, if using multiple SPI devices choose different CS for each one i.e. for LCD screen)
Pin 11 - MOSI (SO on MCP2515, SI on Ardunio, Use same pins for all SPI devices)
Pin 12 - MISO (SI on MCP2515, SO on Arduino, Use same pins for all SPI devices)
Pin 13 - SCK (Clock, Use same pins for all SPI devices)
VCC    - 5V (Must be 5V, DO NOT use a 3.3V Arduino, you'll blow up the pins)
GND    - GND
CAN H goes to OBDII Pin 6
CAN L goes to OBDII Pin 14
*/

#include <mcp_can.h> //Make sure mcp_can library is extracted to your Arduino library folder in My Documents
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char ext = 0;
unsigned char rxBuf[8];
char msgString[128];                        // Array to store serial string
long messages=0;
long timePrev=millis();

#define CAN0_INT 2                           // Set INT to pin 2 on Nano and Uno
MCP_CAN CAN0(10);                            // Set CS to 10 on Nano and Uno

int upto=0;
int uptoMess=0;
struct can_frame {
    unsigned long    can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    unsigned char    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    unsigned char    data[4] __attribute__((aligned(8)));
    unsigned long    response;
    String           ColloquialName;
    unsigned char    Num;
    float            Offset;
    float            Scale;
    bool             TwoByte;
    String           Units;
};
#define msgSize 12
can_frame canMsg[msgSize];
float responseData[msgSize];
void SetupMessages();
byte SendCanBus(can_frame &canMsg1);

void setup()
{
  SetupMessages();

  Serial.begin(115200);
  
  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if(CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");
  
  CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.enOneShotTX();

  pinMode(CAN0_INT, INPUT);                            // Configuring pin for /INT input
  
  Serial.println("MCP2515 Library Receive Example Modded for Holden...");
  Serial.println("------- CAN Read FILTERED!!! ----------");
}

void(* resetFunc) (void) = 0; //declare reset function @ address 0

void loop()
{
  if(!digitalRead(CAN0_INT))                         // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &ext, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)

    if ((len==8)&&(
      ((rxId & 0xFF0)==0x7E0) ||
       ((rxId & 0xFF0)==0x5E0) //(Don't care aobut 5Ex, not using multiple PIDs, can remove)
       ))
    {
      if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      } else {
        bool found=false;
        for (int j=0;j<msgSize;j++){
          //Check correct PID
          if ((rxBuf[2]==canMsg[j].data[0]) && (rxBuf[3]==canMsg[j].data[1])) {
            found=true;
            //Check correct module responded
            if (rxId==canMsg[j].response) {
              //Check if response is valid
              if ((rxBuf[1]==0x62)) {
                //Two or single byte response
                if ((canMsg[j].TwoByte)&&(rxBuf[0]==0x5)) {
                  responseData[j]=((256.0f*(float)rxBuf[4]+(float)rxBuf[5])+canMsg[j].Offset)*canMsg[j].Scale;
                }
                else if ((!canMsg[j].TwoByte)&&(rxBuf[0]==0x4)) {
                  responseData[j]=(((float)rxBuf[4])+canMsg[j].Offset)*canMsg[j].Scale;
                }
              }
              else {
                Serial.print(canMsg[j].data[0],HEX);
                Serial.print(",");
                Serial.print(canMsg[j].data[1],HEX);
                Serial.println(" Bad Response, message not OK");
              }
            }
          }
        }

        if (!found) {
          //Unknown response, spill data
          Serial.print(rxId,HEX);
          Serial.print(" ");

          for(byte i = 0; i<len; i++){
            if (rxBuf[i]<0x10) {
              Serial.print("0");
            }
            Serial.print(rxBuf[i],HEX);
          }
          Serial.println("");
        }
        else {
          //Only update serial if first PID is received (prevent spamming unnecessarily)
          if ((rxBuf[2]==canMsg[0].data[0]) && (rxBuf[3]==canMsg[0].data[1])) {
            //Print Values
            for (int j=0;j<msgSize;j++){
              Serial.print(canMsg[j].ColloquialName); Serial.print(" ");
              Serial.print(responseData[j],2); Serial.print(" ");
              Serial.print(canMsg[j].Units); Serial.print(" ");
            }
            Serial.println("");
          }
        }
      }
    }

    //Reset if locked up
    messages++;
    if ((messages>20000) || (len>32)){
      Serial.print("Reset ");
      Serial.println(millis()-timePrev);
      resetFunc();
    }
  }
  else {
    //Send Stuff Here (cycle through all codes)
    upto++;
    if (upto==100) {
      SendCanBus(canMsg[uptoMess]);
      uptoMess++;
      if (uptoMess>=msgSize) uptoMess=0;
      upto=0;
    }
  }
  
  //Reset if locked up
  if (millis()-timePrev>6000){
    timePrev=millis();
    messages=0;
  }
}

byte SendCanBus(can_frame &canMsg1) {
  unsigned char    bdata[8] __attribute__((aligned(8)));
  bdata[0]=0x03;
  bdata[1]=0x22;
  bdata[2]=canMsg1.data[0];
  bdata[3]=canMsg1.data[1];
  bdata[4]=0x00;
  bdata[5]=0x00;
  bdata[6]=0x00;
  bdata[7]=0x00;

  byte sndStat = CAN0.sendMsgBuf(canMsg1.can_id, 0, canMsg1.can_dlc, bdata);
  if(sndStat == CAN_OK){
    //Serial.println("Success");
  } else {
    Serial.println("Error Sending Message...");
  }
  return sndStat;
}


void SetupMessages() {
  //-1F8 response
  int i=0;

  //Engine Oil Pressure
  //7E8 0462147031AAAAAA Response
  canMsg[i].Num = 0;
  canMsg[i].ColloquialName = "Eng Oil P";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x14; //PID Byte #1
  canMsg[i].data[1] = 0x70; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 4.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "kPa";

  //Transmission Temp
  //7EA 0462194075AAAAAA Response
  i++;
  canMsg[i].Num = 1;
  canMsg[i].ColloquialName = "Trans T";
  canMsg[i].can_id  = 0x7E2;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x19; //PID Byte #1
  canMsg[i].data[1] = 0x40; //PID Byte #2
  canMsg[i].response = 0x7EA;
  canMsg[i].Offset = -40.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "C";

  //Engine Coolant Temp
  i++;
  canMsg[i].Num = 2;
  canMsg[i].ColloquialName = "Eng Cool T";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x00; //PID Byte #1
  canMsg[i].data[1] = 0x05; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = -40.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "C";

  //Engine Oil Temp
  i++;
  canMsg[i].Num = 3;
  canMsg[i].ColloquialName = "Eng Oil T";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x11; //PID Byte #1
  canMsg[i].data[1] = 0x54; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = -40.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "C";

  //Misfire
  i++;
  canMsg[i].Num = 4;
  canMsg[i].ColloquialName = "C1.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x12; //PID Byte #1
  canMsg[i].data[1] = 0x05; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 5;
  canMsg[i].ColloquialName = "C2.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x12; //PID Byte #1
  canMsg[i].data[1] = 0x06; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 6;
  canMsg[i].ColloquialName = "C3.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x12; //PID Byte #1
  canMsg[i].data[1] = 0x07; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 7;
  canMsg[i].ColloquialName = "C4.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x12; //PID Byte #1
  canMsg[i].data[1] = 0x08; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 8;
  canMsg[i].ColloquialName = "C5.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x11; //PID Byte #1
  canMsg[i].data[1] = 0xEA; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 9;
  canMsg[i].ColloquialName = "C6.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x11; //PID Byte #1
  canMsg[i].data[1] = 0xEB; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 10;
  canMsg[i].ColloquialName = "C7.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x11; //PID Byte #1
  canMsg[i].data[1] = 0xEC; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

  //Misfire
  i++;
  canMsg[i].Num = 11;
  canMsg[i].ColloquialName = "C8.MF";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x11; //PID Byte #1
  canMsg[i].data[1] = 0xED; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "";

/*
-----
NOTE:
-----
If you delete or add messages above, make sure you change "msgSize"
*/

/*
  //Fuel Level
  i++;
  canMsg[i].Num = 4;
  canMsg[i].ColloquialName = "Fuel L";
  canMsg[i].can_id  = 0x7E0;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x00; //PID Byte #1
  canMsg[i].data[1] = 0x2F; //PID Byte #2
  canMsg[i].response = 0x7E8;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 100.0f/255.0f;
  canMsg[i].TwoByte = false;
  canMsg[i].Units = "%";
*/

/*
  //Transmission Pressure
  i++;
  canMsg[i].Num = 5;
  canMsg[i].ColloquialName = "Trans Oil Pres";
  canMsg[i].can_id  = 0x7E1;
  canMsg[i].can_dlc = 8;
  canMsg[i].data[0] = 0x28; //PID Byte #1
  canMsg[i].data[1] = 0x62; //PID Byte #2
  canMsg[i].response = 0x7E9;
  canMsg[i].Offset = 0.0f;
  canMsg[i].Scale = 1.0f;
  canMsg[i].TwoByte = true;
  canMsg[i].Units = "kPa";
*/

  //---------------------------
  //0x7e0 Engine Control Module
  //---------------------------
  //220005 Engine Coolant Temp (A-40)
  //221154 Engine Oil temp (A-40)
  //22002F Fuel Level (A*100/255)
  //221470 Engine Oil Pressure (A*4 kPA)
  //---------------------------

  //---------------------------
  //0x7e2 Transm Control Module
  //---------------------------
  //222862 Transmission Oil Pressure (S_A*256+B) 7E1
  //221940 Transmission Oil Temp (A-40)
  //---------------------------

  for (i=0;i<msgSize;i++){
    responseData[i]=0;
  }
}


/*
Extended PIDs
-------------
Send:
7DF 8 0322-PID-0000000000
	    | |
	    |	Service Code
	    |
	    Number of additional Bytes not including this byte (i.e. length of Service Code + PID in bytes)
7E0,7E1,7E2,7E3,7E4 also all valid modules to send to

Transmission Temperature,TFT,221940,A*9/5-40,-30,300,F,6C18F1,,,,,
Oil Pressure,OIL,221470,A*29/50,0,150,PSI,6C10F1,,,,,
Injector Pulse Width 1,IPW1,221193,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 2,IPW2,221194,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 3,IPW3,221195,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 4,IPW4,221196,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 5,IPW5,221197,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 6,IPW6,221198,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 7,IPW7,221199,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Injector Pulse Width 8,IPW8,22119A,((A*256+B)*200/131)/100,0,1000,ms,6C10F1,,,,,
Balance Rate 1,BR1,22162F,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 2,BR2,221630,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 3,BR3,221631,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 4,BR4,221632,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 5,BR5,221633,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 6,BR6,221634,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 7,BR7,221635,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Balance Rate 8,BR8,221636,((A*256+B)*5/32-5120)/10,0,512,mm3/str,6C10F1,,,,,
Name,ShortName,ModeAndPID,Equation,Min Value,Max Value,Units,OBD Header
Control Module Voltage,Cont. Mod. Volt,220042,(A*256+B)/1000,0,15,V,7E0
Engine Coolant Temp,Coolant Temp,220005,(A-40),-40,200,C,7E0
Engine Oil Temp,Oil Temp,221154,(A-40),-40,200,C,7E0
Fuel Level,Fuel Level,22002F,A*100/255,0,100,%,7E0
Fuel Remaining,Fuel Remaining,22002F,(A/255)*9.3122,0,9.3,Gallons,7E0
Speed kmh,Speed,22000D,A,0,200,kmh,7E0
*M Commanded Throttle Pos,Commanded Throttle Pos,22004C,A*100/255,0,100,%,7E0
*M Engine Torque,Engine Torque,22203F,((256*A)+B)/4,0,200,Nm,7E0
*M Fuel System Status,M11 31 B1,221131,A,0,255,B1,7E0
*M Intake Air Temp IAT,IAT,22000F,(A-40)/2,-20,100,C,7E0
*M Spark Advance,Spark Advance,22000E,A/2-64,-64,64,Deg,7E0
PRNDL,PRNDL,222889,A,0,8,,7E1
Transmission Pressure,Trans Pressure,222862,(S_A*256+B),0,20000,,7E1
BRAKE,H1 24 34,222434,(A*256+B)/64,0,420,V,7E1
Tran Temp,Tran Temp,221940,(A-40),-40,215,C,7E2
Transmission ISS  19 41,Tran ISS,221941,((S_A*256+B))/4,-32000,32000,RPM,7E2
Outside Temp Filtered,OAT Raw,22801E,((A-40)/8),-20,50,C,7E4
Barometric Pressure,BARO,2102,H/255*119,0,119,kPa,,
Coolant Temperature,ЕСТ,2102,A/255*192-49,-50,140,C,,
Desired Idle,NeedIdleRPM,210d,M*256+N,500,1500,rpm,,
Desired Ignition timing,DesIT,2103,(G-64)/2.677,-30,72,deg,,
FL Wheel Speed,FLSpeed,210b,AA,0,255,km/h,,
Fuel level,FuelAvg,210b,X/4,0,60,L,,
Ignition timing Cyl1,IT1,2103,(192-A)/2.677,-30,72,deg,,
Ignition timing Cyl2,IT2,2103,(192-B)/2.677,-30,72,deg,,
Ignition timing Cyl3,IT3,2103,(192-C)/2.677,-30,72,deg,,
Ignition timing Cyl4,IT4,2103,(192-D)/2.677,-30,72,deg,,
Injection duration current,InjDur,2102,(P*256+Q)/250,0,0,ms,,
Injection duration on start,InjDurSt,2102,(N*256+O)/62.5,0,1050,ms,,
Instant Fuel Level,FuelCur,2102,Z/255*45,0,45,l,,
Instant Fuel Percent,Fuel%,2102,Z/255*100,0,100,%,,
Intake Air Temperature,IAT,2102,B/255*192-49,-50,140,C,,
Manifold Absolute Pressure,МАР,2102,J/255*119,0,119,kPa,,
Mass Airflow,MAF,2102,(L*256+M)/47,0,1400,mg/t,,
Real Throttle Position,RealTPS,2102,AC,0,255,steps,,
Signal Throtle Position,SignalTPS,2106,C/255*100,0,100,%,,
Throttle Position,TPS,2102,C/255*100,0,100,%,,
Time from the start engine,EngRunTime,2117,(I*256+J)/10,0,6553.5,sec,,
Voltage O2 Sensor2,O2S2,2104,(G*256+F)*4.88,0,319810,mV,,
Voltage,Volt,2102,D/8.88,0,16,V,,
Knock Retard,KR,2211A6,A*22.5/256,0,50,Deg,,0,
Knock Retard 2,KR,22125D,A*22.5/256,0,50,Deg,,0,
Knock Active Count,KC,22125E,A,0,10000,Count,,2,1000,
Cylinder 1 Injector Pluse Width,Cly1.IPW,221193,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 2 Injector Pluse Width,Cly2.IPW,221194,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 3 Injector Pluse Width,Cly3.IPW,221195,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 4 Injector Pluse Width,Cly4.IPW,221196,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 5 Injector Pluse Width,Cly5.IPW,221197,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 6 Injector Pluse Width,Cly6.IPW,221198,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 7 Injector Pluse Width,Cly7.IPW,221199,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 8 Injector Pluse Width,Cly8.IPW,22119A,(A*256+B)/65.5,0,100,ms,,0,
Cylinder 1 Balance Rate,Cly1.BR,22162F,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 2 Balance Rate,Cly2.BR,221630,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 3 Balance Rate,Cly3.BR,221631,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 4 Balance Rate,Cly4.BR,221632,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 5 Balance Rate,Cly5.BR,221633,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 6 Balance Rate,Cly6.BR,221634,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 7 Balance Rate,Cly7.BR,221635,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Cylinder 8 Balance Rate,Cly8.BR,221636,(((A*256+B)-32768)*0.15625)/10,-100,100,,,0,
Ignition 1 Voltage,Ign.Volt,221141,A/10,0,20,V,,1,
H2OS Sensor,H2OS,221145,A*4.34,0,1000,mV,,0,100,
Commanded Idle Speed,Idle RPM,221192,A*12.5,0,10000,RPM,,3,1000,
Cylinder 1 Misfire History,C1.MF.Hist,221201,A*256+B,0,20000,Count,,3,1000,
Cylinder 2 Misfire History,C2.MF.Hist,221202,A*256+B,0,20000,Count,,3,1000,
Cylinder 3 Misfire History,C3.MF.Hist,221203,A*256+B,0,20000,Count,,3,1000,
Cylinder 4 Misfire History,C4.MF.Hist,221204,A*256+B,0,20000,Count,,3,1000,
Cylinder 5 Misfire History,C5.MF.Hist,2211F8,A*256+B,0,20000,Count,,3,1000,
Cylinder 6 Misfire History,C6.MF.Hist,2211F9,A*256+B,0,20000,Count,,3,1000,
Cylinder 7 Misfire History,C7.MF.Hist,2211FA,A*256+B,0,20000,Count,,3,1000,
Cylinder 8 Misfire History,C8.MF.Hist,2211FB,A*256+B,0,20000,Count,,3,1000,
Cylinder 1 Misfire Current,Cly1.MF,221205,A,0,255,Count,,1,
Cylinder 2 Misfire Current,Cly2.MF,221206,A,0,255,Count,,1,
Cylinder 3 Misfire Current,Cly3.MF,221207,A,0,255,Count,,1,
Cylinder 4 Misfire Current,Cly4.MF,221208,A,0,255,Count,,1,
Cylinder 5 Misfire Current,Cly5.MF,2211EA,A,0,255,Count,,1,
Cylinder 6 Misfire Current,Cly6.MF,2211EB,A,0,255,Count,,1,
Cylinder 7 Misfire Current,Cly7.MF,2211EC,A,0,255,Count,,1,
Cylinder 8 Misfire Current,Cly8.MF,2211ED,A,0,255,Count,,1,
Transmission Fluid Temperature 1,TFT,221940,A-40,0,200,C,,3,
Transmission Fluid Temperature 2,TFT,221940,A-40,0,200,C,7E1,3,
Transmission Fluid Temperature 3,TFT,221940,A-40,0,200,C,7E2,3,
Last Shift Time,Lst.Shft,221992,A/40,0,70,sec,,1,
1-2 Shift Time,1-2.Shft,221993,A/40,0,70,sec,,1,
2-3 Shift Time,2-3.Shft,221994,A/40,0,70,sec,,1,
3-4 Shift Time,3-4.Shft,221995,A/40,0,70,sec,,1,
1-2 Shift Error,Shft.Er12,221997,A/40,0,70,sec,,1,
2-3 Shift Error,Shft.Er23,221998,A/40,0,70,sec,,1,
3-4 Shift Error,Shft.Er34,221999,A/40,0,70,sec,,1,
Current Gear,Gear,22199A,A,0,10,,,1,
PC Solenoid Actual Current,PC.Sol,22199E,A*0.0195,0,50,Amp,,0,
PC Solenoid Reference Current,PC.Sol.Ref,22199F,A*0.0195,0,50,Amp,,0,
IAC Position,IAC.Pos,221172,A,0,255,,,1,
Commanded Air To Fuel Ratio,AFR(COM),22119E,A/10,0,30,,,2,
Torque Converter Clutch Slip,TCC.Slip,221191,(S_A*256+B)/8,0,10000,RPM,,1,1000,
Aircon High side pressure,AC.Pres,221564,A*2.02,0,100,Psi,,1,
ABS Front Left Wheel Speed,ABS.FLW,224051,A,0,255,KPH,760,1,
ABS Front Right Wheel Speed,ABS.FRW,224052,A,0,255,KPH,760,1,
ABS Rear Left Wheel Speed,ABS.RLW,224053,A,0,255,KPH,760,1,
ABS Rear Right Wheel Speed,ABS.RRW,224054,A,0,255,KPH,760,1,
Engine Oil Pressure,Oil.Pres,221470,A*0.578,0,100,Psi,,0,
Engine Oil Temperature,Oil.Temp,221154,A-40,0,200,C,,3,
Fuel Tank Pressure,FTPres,22F432,(A*256+B)*0.00003,0,100,Psi,,1,
Intak air temperature 2,IAT2,221538,A-40,0,00,C,,3,
Outside air temperature,OutTemp,221161,A-40,0,200,C,,3,
"Barometric Pressure -F","Baro Pres","221251","A",0,37,"kPa",
Allison Trans Fluid Temp (5-Speed),TFT,221940,A*9/5-40,-30,300,F,7E2,,,,,
Engine Torque,TRQ,2219DE,((A*256+B)*10/2)/10,0,1000,Ft-Lbs,7E0,,,,,
Trans Fluid Temperature,TFT,221940,A*9/5-40,-30,300,F,7E0,,,,,
Remaining Oil Life,OilLife,22119F,(A*200/51)/10,0,100,%,7E0,,,,,
Air to Fuel Ratio,AFR,22119E,(A)/10,0,25,,7E0,,,,,
Barometer V8,BARO,22119D,(A*3)/10,0,100,inHg,7E0,,,,,
Barometer V6,BARO,221251,(A*3)/10,0,100,inHg,7E0,,,,,
Knock Retard,KR,2211A6,(A*45/50)/10,0,25,Deg,7E0,,,,,
Elapsed Time Since Engine Start,ET,2211A1,(A*256+B),0,65535,Sec,7E0,,1000,,,
Transmission Fluid Temperature,TFT,221940,(A-40)*9/5+32,0,250,F,7E2,2,
Transmission Fluid Temperature,TFT,221940,A-40,0,150,C,7E2,2,
*/

/*********************************************************************************************************
  END FILE
*********************************************************************************************************/
