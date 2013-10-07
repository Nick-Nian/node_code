/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"
//#include "tamperature.c"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "acc.c"
#include "light.c"
#if defined( HAL_UART )
  #include "uart.c"
#endif
#define SAMPLEAPP_UART_RECV_EVT               0x0002
// This list should be filled with Application specific Cluster IDs.
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

/* Here I define my own System _Event */

/*------------------------------------*/
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
//uint8 click = 0;
char id_string[3];
uint8 button=2;
char *id_ptr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );

void SampleApp_SendFlashMessage_Endevice_Df( uint16 flashTime );
void SampleApp_SendFlashMessage_Endevice_Ob( uint16 flashTime );

void SampleApp_SendFlashMessage3( uint16 flashTime );//only return light value

void SampleApp_SendFlashMessage_Coordinator_Ob( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_Df( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_Lf( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_Ln( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_C( uint16 flashTime, char *id, uint8 button );

void SampleApp_SendFlashMessage_Coordinator_SR1( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR2( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR3( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR4( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR5( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR6( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR7( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR8( uint16 flashTime );
void SampleApp_SendFlashMessage_Coordinator_SR9( uint16 flashTime );

int atoi(char *p);
int atoi(char *p) {
 int k = 0;
 while (*p) {
 k = (k<<3)+(k<<1)+(*p)-'0';
 p++;
 }
 return k;
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
  //uart
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
  SampleApp_Flash_DstAddr.addr.shortAddr = 0xFFFF;  //廣播
  //end device
  /*
  coordinator to end device must 0xffff
  end device to coordinator must 0x0000
  */
  //mpleApp_Flash_DstAddr.addr.shortAddr = 0x0000;
  //SampleApp_Flash_DstAddr.addr.shortAddr =SAMPLEAPP_FLASH_GROUP;
  //coordinator
   
   
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

  
  if ((events & SEND_UARTMSG_EVT) && click) {
    //HalUARTWrite(SERIAL_APP_PORT, otaBuf, 2);
    if (otaBuf[0] =='O'){//4F
	    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
            SampleApp_SendFlashMessage_Coordinator_Ob( SAMPLEAPP_FLASH_DURATION );}
    else if (otaBuf[0] =='R'){//44 
        HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);	
        SampleApp_SendFlashMessage_Coordinator_Df( SAMPLEAPP_FLASH_DURATION );}
    else if (otaBuf[0] =='L' && otaBuf[1] =='F'){//46
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);	
                SampleApp_SendFlashMessage_Coordinator_Lf( SAMPLEAPP_FLASH_DURATION );}
    else if (otaBuf[0] =='L' && otaBuf[1] =='N'){//53 41
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_Ln( SAMPLEAPP_FLASH_DURATION );}
	else if (otaBuf[0] =='C'){
	   for(int i=1; i < 4; i++)  
		id_string[i-1] = otaBuf[i];
	   //id_ptr = id_string;
           //if(id_string[0] == '0' && id_string[1] == '0' && id_string[2] == '1')
             //HalLedBlink( HAL_LED_4, 4, 50, 500 );
	   if(otaBuf[4] == 'N'){ 
           HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
	       button = 1;
	   }else if(otaBuf[4] == 'F'){
           HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
	       button = 0;
           }
	    SampleApp_SendFlashMessage_Coordinator_C( SAMPLEAPP_FLASH_DURATION, id_string, button );
        }
	else if (otaBuf[0] =='S' && otaBuf[1] =='R'){
	   if (otaBuf[2] =='1'){
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR1( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='2'){
	    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR2( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='3'){
	    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR3( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='4'){
	    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR4( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='5'){
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR5( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='6'){
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR6( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='7'){
	    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR7( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='8'){
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR8( SAMPLEAPP_FLASH_DURATION );}
	   else if (otaBuf[2] =='9'){
		HalLedSet( HAL_LED_ALL, HAL_LED_MODE_TOGGLE);
		SampleApp_SendFlashMessage_Coordinator_SR9( SAMPLEAPP_FLASH_DURATION );}
	}
    //osal_memset(otaBuf,0,sizeof(otaBuf));
    otaBuf = NULL;
    click = 0;
    return (events ^ SYS_EVENT_MSG);
  }
 
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
        case AF_INCOMING_MSG_CMD:
          //HalUARTWrite(SERIAL_APP_PORT, power, 5);
          HalLedBlink( HAL_LED_2, 4, 50, 500 );
          SampleApp_MessageMSGCB( MSGpkt );
          break;
          
    
        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          //SampleApp_SendFlashMessage_Coordinator_Df( SAMPLEAPP_FLASH_DURATION );
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( SampleApp_NwkState == DEV_ZB_COORD )
          {
            //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 5000  );
          }
          else if (SampleApp_NwkState == DEV_ROUTER) {
          }
          else if (SampleApp_NwkState == DEV_END_DEVICE) {
            //al_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 5000  );
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
      osal_msg_deallocate( (uint8 *)MSGpkt );
     // osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_UART_RECV_EVT,0);
      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
  
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
  
   // SampleApp_SendFlashMessage_Endevice_Df( SAMPLEAPP_FLASH_DURATION );
    //HalLedBlink(HAL_LED_4,2,50,200); 

    //osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT, 5000  );
    
    
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }/*
    if (eventS & ZB_ENTRY_EVENT)
  {
    HalUARTWrite(SERIAL_APP_PORT, "Command sent", 12);
    HalUARTRead(SERIAL_APP_PORT, commandBuf, 2);
    HalLedBlink( HAL_LED_2, 4, 50, 500 );
    //HalUARTWrite(SERIAL_APP_PORT, "Command sent", 12);
    if (commandBuf[0] =='O' && commandBuf[1]=='B')
       SampleApp_SendFlashMessage_Coordinator_Ob( SAMPLEAPP_FLASH_DURATION );
    else if (commandBuf[0] =='D' && commandBuf[1]=='F')  
       SampleApp_SendFlashMessage_Coordinator_Df( SAMPLEAPP_FLASH_DURATION );
    osal_memset(commandBuf,0,sizeof(commandBuf));
    //return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }*/
  
  return 0;
}

void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_SW_6 )
  {
    
    //mpleApp_SendFlashMessage_Endevice_Df( SAMPLEAPP_FLASH_DURATION );
    SampleApp_SendFlashMessage_Coordinator_Ob( SAMPLEAPP_FLASH_DURATION );
    //SampleApp_SendFlashMessage3( SAMPLEAPP_FLASH_DURATION );
  }
  else if ( keys & HAL_KEY_SW_5 )
  {
    HalLedBlink( HAL_LED_4, 4, 50, 500 );
    //SampleApp_SendFlashMessage_Endevice_Ob( SAMPLEAPP_FLASH_DURATION );
  }
}


void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  //uint16 shortAddr;
  //uint8 data[4] = {'F','U','C','K'} ;
 
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
 
      break;
    case SAMPLEAPP_FLASH_CLUSTERID:
 //if coordinator  
      HalUARTWrite(SERIAL_APP_PORT, pkt->cmd.Data, pkt->cmd.DataLength); //如果收到封包，送什麼東西給console去顯示
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
void SampleApp_SendFlashMessage_Coordinator_Lf( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'L';
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
void SampleApp_SendFlashMessage_Coordinator_Ln( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'L';
  cmd[1] = 'N';

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
void SampleApp_SendFlashMessage_Coordinator_SR1( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '1';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR2( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '2';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR3( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '3';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR4( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '4';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR5( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '5';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR6( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '6';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR7( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '7';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR8( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '8';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_SR9( uint16 flashTime )
{
  uint8 cmd[9];
  cmd[0] = 'S';
  cmd[1] = 'R';
  cmd[2] = '9';

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
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
void SampleApp_SendFlashMessage_Coordinator_C( uint16 flashTime, char *id, uint8 button )
{
  uint8 id_int = atoi(id);
  if(id_int == 1)
    HalLedBlink( HAL_LED_4, 4, 50, 500 );
  
  uint8 cmd[9];
  cmd[0] = id_int;
  cmd[1] = button;
 
  
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