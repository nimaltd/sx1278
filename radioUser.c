
#include "radio.h"
#include "gpio.h"
#include <string.h>

#include "cmsis_os.h"
#include "Functions.h"
#include "sx1278-LoRaMisc.h"
#include "sx1278-LoRa.h"
char  txPacket[16];
char  SerialTxPacket[16];

//Radio_TxPacket(data,DataLength);
//#########################################################################################
void  Radio_UserInit(void)
{
  SX1278LoRaSetRFFrequency(440000000);
  SX1278LoRaSetRFPower(17);
  //SX1278LoRaSetRxPacketTimeout(50);
  //SX1278LoRaSetTxPacketTimeout(50);
  //SX1278LoRaSetSignalBandwidth(2);
  SX1278LoRaSetPreambleLength(128);
  SX1278LoRaSetLowDatarateOptimize(true);  
  
}
//#########################################################################################
void  Radio_RxDone(uint8_t  *Packet,uint16_t  PacketLength)
{
  char *str;
  uint8_t   BajehIndex;
  uint16_t  Number;
  static    uint16_t  LastNumber=0;
  printf("%s\r\n",(char*)Packet);
  printf("RSSI:%5.2f   SNR:%d\r\n",SX1278LoRaGetPacketRssi(),SX1278LoRaGetPacketSnr());
  str= strstr((char*)Packet,"AT+SEND");
  if(sscanf(str,"AT+SEND=%hhu,%hu\r\n",&BajehIndex,&Number)==2)
  {
    if(BajehIndex == StaitionIndex)
    {      
      NobatMe = Number;
      sprintf(SerialTxPacket,"AT+SET=%d\r\n",Number);
      Serial_Puts(SerialTxPacket);
      memset(txPacket,0,sizeof(txPacket));
      if(LastNumber == Number)
        sprintf(txPacket,"RF CURRENT OK:%u\r\n",BajehIndex);
      else
        sprintf(txPacket,"RF NEXT OK:%u\r\n",BajehIndex);
      Radio_TxPacket((uint8_t*)txPacket,strlen(txPacket)); 
      LastNumber = Number;
    }
    else
    {
      NobatLast = Number;
    }
  }
  
}
//#########################################################################################
void  Radio_RxTimeout(void)
{
  
}
//#########################################################################################
void  Radio_TxDone(void)
{
  
}
//#########################################################################################
void  Radio_Misc_Per100ms(void)
{
  if(KeyCurrentPressed==1)
  {
    memset(txPacket,0,sizeof(txPacket));
    sprintf(txPacket,"RF CURRENT:%u\r\n",StaitionIndex);
    Radio_TxPacket((uint8_t*)txPacket,strlen(txPacket)); 
    
    KeyCurrentPressed=0;
  }
  if(KeyNextPressed==1)
  {
    memset(txPacket,0,sizeof(txPacket));
    sprintf(txPacket,"RF NEXT:%u\r\n",StaitionIndex);
    Radio_TxPacket((uint8_t*)txPacket,strlen(txPacket)); 
    KeyNextPressed=0;
  }

}
//#########################################################################################
