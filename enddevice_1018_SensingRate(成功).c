/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
//#include "SampleAppHw.h"
//#include "temperature.c"
//#include "temperture.c"
//#include "temperatureNEW.c"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
//#include "acc.c"
#include "light.c"
#include "hal.h"

#if defined( HAL_UART )
  #include "uart.c"
#endif

#if defined( TEMP )
  #include "temperatureNEW.c"
#endif

// This list should be filled with Application specific Cluster IDs.  //這些常數都定義在SampleApp.h裡面
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint8 init = 1;
int SensingRate = 10000;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );  //afIncomingMSGPacket_t 定義在AF.h
void SampleApp_SendPeriodicMessage( void );

void SampleApp_SendFlashMessage_Endevice_Df( uint16 flashTime );
void SampleApp_SendFlashMessage_Endevice_Ob( uint16 flashTime );
void SampleApp_SendFlashMessage_Endevice_SensingrateACK( uint16 flashTime );
void SampleApp_SendFlashMessage_Endevice_Temp( uint16 flashTime );

void SampleApp_SendFlashMessage3( uint16 flashTime );//only return light value

void SampleApp_SendFlashMessage_Coordinator_Ob( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_Df( uint16 flashTime );

char * strcat(char *dest, const char *src);
int itoa(int n, char* out);
void reverse(char* str, int length);

char * strcat(char *dest, const char *src)
{
    uint8 i,j;
    for (i = 0; dest[i] != '\0'; i++)
        ;
    for (j = 0; src[j] != '\0'; j++)
        dest[i+j] = src[j];
    dest[i+j] = '\0';
    return dest;
}
int itoa(int n, char* out)
{
    // if negative, need 1 char for the sign
    int sign = n < 0? 1: 0;
    int i = 0;
    if (n == 0) {
        out[i++] = '0';
    } else if (n < 0) {
        out[i++] = '-';
        n = -n;
    }
    while (n > 0) {
        out[i++] = '0' + n % 10;
        n /= 10;
    }
    out[i] = '\0';
    reverse(out + sign, i - sign);
    return 0;
}
void reverse(char* str, int length){
    int i = 0, j = length-1;
    char tmp;
    while (i < j) {
        tmp = str[i];
        str[i] = str[j];
        str[j] = tmp;
        i++; j--;
    }
}



/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  open(SampleApp_TaskID);
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().
  
 #if defined ( SOFT_START )
  // The "Demo" target is setup to have SOFT_START and HOLD_AUTO_START
  // SOFT_START is a compile option that allows the device to start
  //  as a coordinator if one isn't found.
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered 
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // SOFT_START
  
#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to 
  //  start the device.
  ZDOInitDevice(0);
#endif
  
  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = 0x0000;  //送給coor
  //end device
  /*
  coordinator to end device must 0xffff
  end device to coordinator must 0x0000
  */
  
  //SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  //coordinator
  //SampleApp_Flash_DstAddr.addr.shortAddr = 0xFFFF;
   
  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1  
  SampleApp_Group.ID = SAMPLEAPP_FLASH_GROUP;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
  
#if defined( HAL_UART )
  open(SampleApp_TaskID);
#endif
#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */

uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:  //目前用到此case
          //HalUARTWrite(SERIAL_APP_PORT, power, 5);
          //HalLedBlink( HAL_LED_2, 4, 50, 500 );
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( SampleApp_NwkState == DEV_ZB_COORD )
          {
            //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 5000  );
          }
          else if (SampleApp_NwkState == DEV_ROUTER) {
          }
          else if (SampleApp_NwkState == DEV_END_DEVICE) {
            osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 10000  );
            //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
            //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 10  );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );  //釋放MSGpkt該指標的記憶體

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
	if(init==1){
		SampleApp_SendFlashMessage_Endevice_Df( SAMPLEAPP_FLASH_DURATION );
		init++;
	}
	
    SampleApp_SendFlashMessage_Endevice_Ob( SAMPLEAPP_FLASH_DURATION );
    //HalLedBlink(HAL_LED_4,2,50,200); 

    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SensingRate  ); //送出封包的週期時間(毫秒)
    //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT  );
    
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
  return 0;
}

void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_SW_6 ) //燒錄板上面的s2
  {
    //HalLedBlink( HAL_LED_4, 4, 50, 500 );
    //SampleApp_SendFlashMessage_Endevice_Df( SAMPLEAPP_FLASH_DURATION ); //送註冊訊息給dongle
    SampleApp_SendFlashMessage_Endevice_Ob( SAMPLEAPP_FLASH_DURATION ); //送OB訊息給dongle
    //SampleApp_SendFlashMessage_Coordinator_Df( SAMPLEAPP_FLASH_DURATION );
    //SampleApp_SendFlashMessage3( SAMPLEAPP_FLASH_DURATION );
  }
  else if ( keys & HAL_KEY_SW_5 ) //燒錄板上面的s3 //reset
  {
    HalLedBlink( HAL_LED_4, 4, 50, 500 ); //黃登亮四下  
    //SampleApp_SendFlashMessage_Endevice_Ob( SAMPLEAPP_FLASH_DURATION ); //送OB訊息給dongle,但不知為何寫在這顆按鈕會錯(會狂傳ob回來)
  }
}


void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt ) //這邊是處理由dongle送給enddevice的封包
{
  //uint16 shortAddr;
  //uint8 data[4] = {'F','U','C','K'} ;
 
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
 
      break;
      
    case SAMPLEAPP_FLASH_CLUSTERID:
     
      //shortAddr = pkt->srcAddr.addr.shortAddr;
      //data[0] = LO_UINT16(shortAddr);
      //data[1] = HI_UINT16(shortAddr);
      //osal_memcpy( data, pkt->cmd.Data, pkt->cmd.DataLength);
        
        
 //if coordinator  
      //HalUARTWrite(SERIAL_APP_PORT, pkt->cmd.Data, pkt->cmd.DataLength);
 //if end device    
      if (pkt->cmd.Data[0] == 'D' && pkt->cmd.Data[1] == 'F') {
         HalLedBlink( HAL_LED_2, 4, 50, 500 ); 
         SampleApp_SendFlashMessage_Endevice_Df( SAMPLEAPP_FLASH_DURATION );
          //SampleApp_SendFlashMessage_Endevice_Ob( SAMPLEAPP_FLASH_DURATION );
           /*
         //IO_FUNC_PORT_PIN(1, 2, IO_FUNC_GIO);                   // 設置P1.0為GPIO
         IO_DIR_PORT_PIN(1, 2, IO_OUT);                         // 設置P1.0方向為輸出
         if (P1_2 == 1)
           P1_2 = 0;
         else
           P1_2 = 1; 
         //IO_FUNC_PORT_PIN(0, 6, IO_FUNC_GIO);                   // 設置P1.0為GPIO
         IO_DIR_PORT_PIN(0, 6, IO_OUT);                         // 設置P1.0方向為輸出
         P0_6 = 0; 
          */
      } else if (pkt->cmd.Data[0] == 'O' && pkt->cmd.Data[1] == 'B') {  //收到OB指令-->亮藍燈 
          HalLedBlink( HAL_LED_2, 4, 50, 500 ); 
          SampleApp_SendFlashMessage_Endevice_Ob( SAMPLEAPP_FLASH_DURATION );
      } else if (pkt->cmd.Data[0] == 'L' && pkt->cmd.Data[1] == 'F') {  //收到關的指令-->關黃燈 
          HalLedSet( HAL_LED_4, HAL_LED_MODE_OFF);
      } else if (pkt->cmd.Data[0] == 1 && pkt->cmd.Data[1] == 1) {  //單獨command的指令
          HalLedSet( HAL_LED_4, HAL_LED_MODE_ON);
      } else if (pkt->cmd.Data[0] == 1 && pkt->cmd.Data[1] == 0) {  
          HalLedSet( HAL_LED_4, HAL_LED_MODE_OFF);
      }	else if (pkt->cmd.Data[0] == 'L' && pkt->cmd.Data[1] == 'N') {  //收到開的指令-->開黃燈
		  HalLedSet( HAL_LED_4, HAL_LED_MODE_ON);	  
      } else if (pkt->cmd.Data[0] == 'S' && pkt->cmd.Data[1] == 'R') {
                                HalLedBlink( HAL_LED_4, 4, 50, 500 ); 
			if (pkt->cmd.Data[2] == '1'){
                                HalLedBlink( HAL_LED_2, 4, 50, 500 ); 
				SensingRate = 1000;
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '2'){
                                HalLedBlink( HAL_LED_2, 4, 50, 500 ); 
				SensingRate = 2000;
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '3'){
				SensingRate = 3000;	
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '4'){
				SensingRate = 4000;
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '5'){
				SensingRate = 5000;
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '6'){
				SensingRate = 6000;	
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '7'){
				SensingRate = 7000;
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '8'){
				SensingRate = 8000;	
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}else if (pkt->cmd.Data[2] == '9'){
				SensingRate = 9000;
				SampleApp_SendFlashMessage_Endevice_SensingrateACK( SAMPLEAPP_FLASH_DURATION );
			}
	  }
      break;  
  }
}

void SampleApp_SendPeriodicMessage( void )
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
}


void SampleApp_SendFlashMessage_Endevice_Df( uint16 flashTime )  //傳送DF資料的function
{

  //UINT16  i;
  //UINT16 buflen;
  //uint8 *ieeeAddr;
  //UINT16 shortAddr;
 
  uint8 buffer[100];
  //uint8 shorta[2];
  uint8 packet_size;
 
  //uint8 coordAddr[2];
  //buflen = 0;

  osal_memset(buffer,0,sizeof(buffer));
  
  
  //get ieee address
  packet_size = sizeof("Register OctopusX 001 Light ")-1;
  osal_memcpy(buffer,"Register OctopusX 001 Light ",packet_size);
  
  //for (i=0; i<100; i++) {
  //  if (buffer[i]=='\0') {
  //     buflen = i;
  //     break;
  //  }
  //}
  //ieeeAddr = NLME_GetExtAddr(); //抓硬體位置
  //shortAddr = NLME_GetShortAddr(); //抓IP addr

  
  //osal_cpyExtAddr (buffer+buflen, ieeeAddr);
  //buffer[buflen+8] = ' '; //在IEEE ADDR後面加一個空格
  /*


   */
  //buffer[buflen+buflen] = (uint8)((shortAddr&0xff00)>>8);
  //buffer[buflen+buflen+1] = (uint8)shortAddr;
  

  //shorta[0] = HI_UINT16(shortAddr);
  //shorta[1] = LO_UINT16(shortAddr);
  //osal_memcpy(buffer+buflen+9,shorta,2);

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       packet_size,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    //HalLedBlink( HAL_LED_2, 4, 50, 500 );  //如果有送DF資料就亮燈; led2=藍燈
  }
  else
  {
  }
}


/*void SampleApp_SendFlashMessage_Endevice_Temp( uint16 flashTime )
{
  uint8 thValue[4]; //溫濕度的value，需要各兩個bytes
  thValue[0] = (uint8)HI_UINT16(readTemperature());
  thValue[1] = (uint8)LO_UINT16(readTemperature());
  thValue[2] = (uint8)HI_UINT16(readHumidity());
  thValue[3] = (uint8)LO_UINT16(readHumidity());
  
  UINT16  i;
 
  uint8 buffer[10];
  uint8 packet_size;

  osal_memset(buffer,0,sizeof(buffer));
  
  packet_size = sizeof(thValue);
  
  for (i=0; i<3; i++)
     buffer[i] = thValue[i];
  
  AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       packet_size,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS );
}*/


void SampleApp_SendFlashMessage_Endevice_Ob( uint16 flashTime ) //傳送OB資料的function
{

#if defined( ACC_READ )    //條件式編譯
  uint8 pValue[9];
  readAcc( pValue ); //讀取三軸加速度計的值
  
  uint16 i;
  uint16 buflen;
  uint8 *ieeeAddr; //指向uint8變數型態的指標
  uint16 shortAddr;
 
  uint8 buffer[100];
  uint8 packet_size;
  uint8 ieeeAddr_size;
 
  
  buflen = 0;
  osal_memset(buffer,0,sizeof(buffer));   //一開始先把全部初始化為0
  //get ieee address
  packet_size = sizeof("OB http://koichidino.twbbs.org/USP/RegisterSensor.xml ")-1;  //減掉字串結數字元的1個size "\0"
  osal_memcpy(buffer,"OB http://koichidino.twbbs.org/USP/RegisterSensor.xml ",packet_size);
  for (i=0; i<100; i++) {
    if (buffer[i]=='\0') {
       buflen = i;
       break;
    }
  }
  ieeeAddr = NLME_GetExtAddr();  //傳回位址給該指標,指標接收位址
  osal_cpyExtAddr (buffer+buflen, ieeeAddr); //將此指標指到的位址的值copy到buffer裡面(從buffer+buflen開始複製)
  ieeeAddr_size = sizeof(*ieeeAddr);
  buffer[buflen + ieeeAddr_size] = ' '; //加空格以便切割字串
  packet_size = packet_size + ieeeAddr_size + 1;  //sizeof(*ieeeAddr),該指標指向的值的大小,*取值運算子, +1為空格
  for (i=0; i<9; i++) //ACC接受的值有9個
     buffer[buflen + ieeeAddr_size + 1 + i] = pValue[i];  

  packet_size = packet_size + sizeof(pValue); 
  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       packet_size,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    //HalLedBlink( HAL_LED_2, 4, 50, 500 );  //如果有送OB(ACC)資料就亮燈; led2=藍燈
  }
  else
  {
  }
#else
  uint8 pValue[3]; //光的value,需要三個bytes
  readLight(pValue); //讀取光的value
  
 
#if defined (TEMP)
  uint8 thValue[4]; //溫濕度的value，需要各兩個bytes
  thValue[0] = (uint8)HI_UINT16(readTemperature());
  thValue[1] = (uint8)LO_UINT16(readTemperature());
  thValue[2] = (uint8)HI_UINT16(readHumidity());
  thValue[3] = (uint8)LO_UINT16(readHumidity());
#endif  
  
  uint16 i;
  uint16 buflen;
  //uint8 *ieeeAddr;
  uint8 buffer[100];
  uint8 packet_size;
 
  
  buflen = 0;
  osal_memset(buffer,0,sizeof(buffer));     //800
  packet_size = sizeof("Data 001 ")-1;
  osal_memcpy(buffer,"Data 001 ",packet_size);
  for (i=0; i<100; i++) {
    if (buffer[i]=='\0') {
       buflen = i;
       break;
    }
  }
  //ieeeAddr = NLME_GetExtAddr();
 // osal_cpyExtAddr (buffer+buflen, ieeeAddr);
  //buffer[buflen+8] = ' ';
  //add tag "light"
  //osal_memcpy(buffer+buflen+9,"Light ",sizeof("Light "));
  //use ACC instead real value 3-bytes
  //osal_memcpy(pValue,"ACC",3);
  for (i=0; i<3; i++)
     buffer[buflen+i] = pValue[i]; 
  for (i=0; i<4; i++)
     buffer[buflen+3+i] = thValue[i];
  packet_size = packet_size + sizeof(pValue)+4;
  //packet_size = packet_size + sizeof(pValue);
  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       packet_size,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  /*------------
  //get ieee address
  osal_memcpy(buffer,"OB 001 ",sizeof("OB 001 ")); //7+1
  for (i=0; i<100; i++) {
    if (buffer[i]=='\0') {
       buflen = i; //i=54
       break;
    }
  }
  ieeeAddr = NLME_GetExtAddr();
  osal_cpyExtAddr (buffer+buflen, ieeeAddr);
  buffer[buflen+8] = ' ';
  //add tag "light"
  osal_memcpy(buffer+buflen+9,"Light ",sizeof("Light ")); //sizeof("Light ")=6+1
  //use ACC instead real value 3-bytes
  //osal_memcpy(pValue,"ACC",3);*/
  /*for (i=0; i<3; i++)
     buffer[7+i] = pValue[i]; //buffer[buflen+8+1+i+6] = pValue[i];  

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       10,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )  
    ---------- */
  {
    //HalLedBlink( HAL_LED_2, 4, 50, 500 );  //如果有送OB(light)資料就亮燈; led2=藍燈
  }
  else
  {
  }
#endif  
}




void SampleApp_SendFlashMessage3( uint16 flashTime )
{
//only return light value
  //only return light value
  //only return light value
  //only return light value
#if defined( ACC_READ )  
  uint8 pValue[9];
  readAcc( pValue );


  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       9,
                       pValue,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
#else
  uint8 pValue[3];
  readLight(pValue);
  

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       pValue,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
#endif  
}

void SampleApp_SendFlashMessage_Endevice_SensingrateACK( uint16 flashTime ) 
{ 
  uint8 buffer[30];
  uint8 packet_size = sizeof("Sensing rate set to: 1000")-1;
  
  char buffer1[30] = "Sensing rate set to: ";
  char buffer2[4];
  itoa(SensingRate,buffer2);
  osal_memset(buffer,0,sizeof(buffer));
  strcat(buffer1, buffer2);
 
  osal_memcpy(buffer, buffer1, packet_size);
  
  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       packet_size,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
}

void SampleApp_SendFlashMessage_Coordinator_Ob( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'O';
  cmd[1] = 'B';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       2,
                       cmd,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
}

void SampleApp_SendFlashMessage_Coordinator_Df( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'D';
  cmd[1] = 'F';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       2,
                       cmd,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
  }
}
