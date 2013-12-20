/**************************************************************************************************
Filename:       SampleApp.c
Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
Revision:       $Revision: 19453 $

Description:    Sample Application (no Profile).


Copyright 2007 Texas Instruments Incorporated. All rights reserved.

IMPORTANT: Your use of this Software is limited to those specific rights
granted under the terms of a software license agreement between the user
who downloaded the software, his/her employer (which must be your employer)
and Texas Instruments Incorporated (the "License").  You may not use this
Software unless you agree to abide by the terms of the License. The License
limits your use, and you acknowledge, that the Software may not be modified,
copied or distributed unless embedded on a Texas Instruments microcontroller
or used solely and exclusively in conjunction with a Texas Instruments radio
frequency transceiver, which is integrated into your product.  Other than for
the foregoing purpose, you may not use, reproduce, copy, prepare derivative
works of, modify, distribute, perform, display or sell this Software and/or
its documentation for any purpose.

YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

Should you have any questions regarding your right to use this Software,
contact Texas Instruments Incorporated at www.TI.com.
 **************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
SW1:  Sends a flash command to all devices in Group 1.
SW2:  Adds/Removes (toggles) this device in and out
of Group 1.  This will enable and disable the
reception of the flash command.
 *********************************************************************/

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

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "IC.H"
#include "stdlib.h"
#include "string.h"
#include "ZComDef.h"
#include "OSAL_Nv.h"
#include "OnBoard.h"

/*********************************************************************
 * MACROS
 */
#define HandleFlashStatus(status) \
	do { \
		switch(status) { \
			case ZSUCCESS: \
				       break; \
			case NV_ITEM_UNINIT: \
					     HalUARTWrite(0,"NV_ITEM_UNINIT",14); break; \
			case NV_OPER_FAILED: \
					     HalUARTWrite(0,"NV_OPER_FAILED",14); break; \
			default: break;\
		} \
	}while(0)

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

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

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
uint8 SampleApp_SendPeriodicMessage( void );
/*void SampleApp_SendPeriodic5SecMessage( void );*/
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
void MilliWait(uint16 time);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
	SampleApp_TaskID = task_id;
	SampleApp_NwkState = DEV_INIT;
	SampleApp_TransID = 0;

	/***********串口初始化************/
	MT_UartInit();//初始化
	MT_UartRegisterTaskID(task_id);//登记任务号
	HalUARTWrite(0,"任务启动...\n",12);



	// Device hardware initialization can be added here or in main() (Zmain.c).
	// If the hardware is application specific - add it here.
	// If the hardware is other parts of the device add it in main().
	IC_Init(); //IC 初始化

#if defined ( BUILD_ALL_DEVICES )
	// The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
	// We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
	// together - if they are - we will start up a coordinator. Otherwise,
	// the device will start as a router.
	if ( readCoordinatorJumper() )
		zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
	else
		zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
	// HOLD_AUTO_START is a compile option that will surpress ZDApp
	//  from starting the device and wait for the application to
	//  start the device.
	ZDOInitDevice(0);
#endif

	// Setup for the periodic message's destination address
	// Broadcast to everyone
	//SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
	SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
	SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
	//SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
	SampleApp_Periodic_DstAddr.addr.shortAddr = 0x0000;

	// Setup for the flash command's destination address - Group 1
	SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
	SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
	SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

	// Fill out the endpoint description.
	SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
	SampleApp_epDesc.task_id = &SampleApp_TaskID;
	SampleApp_epDesc.simpleDesc
		= (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
	SampleApp_epDesc.latencyReq = noLatencyReqs;

	// Register the endpoint description with the AF
	afRegister( &SampleApp_epDesc );

	// Register for all key events - This app will handle all key events
	RegisterForKeys( SampleApp_TaskID );

	// By default, all devices start out in Group 1
	SampleApp_Group.ID = 0x0001;
	osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
	aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

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
	(void)task_id;  // Intentionally unreferenced parameter

	if ( events & SYS_EVENT_MSG )
	{
		MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
		while ( MSGpkt )
		{
			switch ( MSGpkt->hdr.event )
			{

				case CMD_SERIAL_MSG:  //串口收到数据后由MT_UART层传递过来的数据，用网蜂方法接收，编译时不定义MT相关内容，
					SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
					break;

					// Received when a key is pressed
				case KEY_CHANGE:
					SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
					break;

					// Received when a messages is received (OTA) for this endpoint
				case AF_INCOMING_MSG_CMD:
					SampleApp_MessageMSGCB( MSGpkt );
					break;

					// Received whenever the device changes state in the network
				case ZDO_STATE_CHANGE:
					SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
					if ( (SampleApp_NwkState == DEV_ZB_COORD)
							|| (SampleApp_NwkState == DEV_ROUTER)
							|| (SampleApp_NwkState == DEV_END_DEVICE) )
					{
						// Start sending the periodic message in a regular interval.
						osal_start_timerEx( SampleApp_TaskID,
								SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
								SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
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

			// Next - if one is available
			MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
		}

		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}

	// Send a message out - This event is generated by a timer
	//  (setup in SampleApp_Init()).
	if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
	{
		// Send the periodic message
		SampleApp_SendPeriodicMessage();

		// Setup to send message again in normal period (+ a little jitter)
		osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
				(/*SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT*/ 400 + (osal_rand() & 0x00FF)) );

		// return unprocessed events
		return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
	}

	// Discard unknown events
	return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
	(void)shift;  // Intentionally unreferenced parameter

	if ( keys & HAL_KEY_SW_1 )
	{
		/* This key sends the Flash Command is sent to Group 1.
		 * This device will not receive the Flash Command from this
		 * device (even if it belongs to group 1).
		 */
		SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
	}

	if ( keys & HAL_KEY_SW_2 )
	{
		/* The Flashr Command is sent to Group 1.
		 * This key toggles this device in and out of group 1.
		 * If this device doesn't belong to group 1, this application
		 * will not receive the Flash command sent to group 1.
		 */
		aps_Group_t *grp;
		grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
		if ( grp )
		{
			// Remove from the group
			aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
		}
		else
		{
			// Add to the flash group
			aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
		}
	}
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
	//uint8 asc_16[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	uint16 flashTime;

	switch ( pkt->clusterId )
	{
		case SAMPLEAPP_PERIODIC_CLUSTERID:
			if ( SampleApp_NwkState == DEV_END_DEVICE )
			{
				//HalLcdWriteString("Recive!",3);
				if(*(&pkt->cmd.Data[0])==1){
					HalLcdWriteString("LED IS OPENED.",3);
					HalLedBlink( HAL_LED_4, 4, 50, 300 );                  
				}
			}
			break;

		case SAMPLEAPP_FLASH_CLUSTERID:
			flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
			HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
			break;
	}
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
/*             +-------------------+
	       |启动等待串口写门号 |
	       |写之后发送协调器   |
	       +-------------------+
	       |先设置管理卡       |
	       |后可添加普通卡     |
	       +-------------------+
	       |刷卡开门，亮灯显示 |
	       |记录开门记录       |
	       +-------------------+
	       |无任务休眠，每5秒  |
	       |唤醒与协调器联系   |
	       +-------------------+
	       */
uint8 SampleApp_SendPeriodicMessage( void )
{
	uint8 asc_16[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	char Card_Id[8];  
	/*char *Card_Id_str = NULL;*/
	/*char *nv_intend = "abcdefgh"; //测试flash write*/
	char *nv_room_data,*nv_valid_card,*nv_valid_mgcard;
	/*uint8 flash_status = 0;*/
	/*如果ZCD_NV_ROOM_ID为空(0xff); (指示) goto failed. (等待串口写入)*/
	/*如果ZCD_NV_USER_VALID_CARD合法 ->开门并指示 否则 失败并指示*/
	/*如果ZCD_NV_USER_MG_CARD 为管理卡 -> 添加合法卡并指示*/
	if(IC_Test()==1) {
		for(int i=0;i<4;i++)
		{
			Card_Id[i*2]=asc_16[qq[i]/16];
			Card_Id[i*2+1]=asc_16[qq[i]%16];        
		}
		/*HalUARTWrite(0,Card_Id,8);*/
		//Handleflashstatus 是一个宏定义 用于调试flash读和写状态
		HandleFlashStatus(osal_nv_item_init(ZCD_NV_ROOM_ID,6,NULL));
		HandleFlashStatus(osal_nv_read(ZCD_NV_ROOM_ID,0,6,nv_room_data));
		if(*nv_room_data == 0xFF){ //楼号房间号为无效
			HalUARTWrite(0,"楼号房间号未写入,使用串口进行设置",34);
			goto nv_fail_invalid;
		}
		/*HalUARTWrite(0,nv_data,6);*/
		HandleFlashStatus(osal_nv_item_init(ZCD_NV_USER_VALID_CARD,8,NULL));
		HandleFlashStatus(osal_nv_read(ZCD_NV_USER_VALID_CARD,0,8,nv_valid_card));
		if(*nv_valid_card == 0xFF) {
			HalUARTWrite(0,"无效卡号，需要刷管理卡再添加",28);
			//goto nv_fail_invalid;
		}
		HandleFlashStatus(osal_nv_item_init(ZCD_NV_USER_MG_CARD,8,NULL));
		HandleFlashStatus(osal_nv_read(ZCD_NV_USER_MG_CARD,0,8,nv_valid_mgcard));
		if(*nv_valid_mgcard == 0xFF) {
			HalUARTWrite(0,"无效管理卡，需要使用串品进行设置",32);
		}                
		/*HalUARTWrite(0,nv_valid_mgcard,8);*/
		/*memcpy(Card_Id_str,nv_valid_mgcard,8);*/
		if (memcmp(nv_valid_mgcard,Card_Id,8) == 0) { //判断是否为管理卡,之后可以加入开卡,写入合法rfid卡id到flash
			HalUARTWrite(0,"检测到管理卡,刷另一张卡可以添加为开门卡",40);
                        //HalLedBlink( HAL_LED_4, 4, 50, 600 );
			HalUARTWrite(0,"延时完成",8);
			if(IC_Test()==1) {
				HalUARTWrite(0,"检测到添加的卡",14);
				for(int i=0;i<4;i++)
				{
					Card_Id[i*2]=asc_16[qq[i]/16];
					Card_Id[i*2+1]=asc_16[qq[i]%16];        
				}
				/*HalUARTWrite(0,"比较是否还是管理卡",18);*/
				//不是管理卡，且和原来的卡不一样 TODO
				/*if(memcmp(Card_Id_str,Card_Id,8) != 0) { //再一次检测卡，添加开门卡不能为管理卡*/
				HandleFlashStatus(osal_nv_item_init(ZCD_NV_USER_VALID_CARD,8,NULL));
				HandleFlashStatus(osal_nv_write(ZCD_NV_USER_VALID_CARD,0,8,Card_Id));
				HalUARTWrite(0,"添加成功->",10);
				HalUARTWrite(0,Card_Id,8);
				/*}*/
			}
		}
		//如果上面判断不是管理卡就进行这里的合法卡判断
//			//      //uint8 osal_nv_write( uint16 id, uint16 ndx, uint16 len, void *buf )
//			//	    /*flash_status = osal_nv_item_init(ZCD_NV_USER_VALID_CARD,8,NULL);*/
//			//	    osal_nv_item_init(ZCD_NV_USER_MG_CARD,8,NULL);
//			//	    /*flash_status = osal_nv_write(ZCD_NV_USER_VALID_CARD,0,8,nv_intend);*/
//			//	    osal_nv_read(ZCD_NV_USER_MG_CARD,0,8,nv_data);
//			//	    /*HalLcdWriteString("A managment card",2);*/
//			//	    HalLcdWriteString(nv_data,2);
//			//            
//			if(IC_Test()==1) {
//				for(int i=0;i<4;i++)
//				{
//					Card_Id[i*2]=asc_16[qq[i]/16];
//					Card_Id[i*2+1]=asc_16[qq[i]%16];        
//				}
//				//不是管理卡，且和原来的卡不一样
//				memcpy(Card_Id_str,Card_Id,8)
//					if(osal_nv_write(ZCD_NV_USER_VALID_CARD,0,8,Card_Id) == 0)
//					{
//						osal_nv_read(ZCD_NV_USER_VALID_CARD,0,8,Card_Id);
//						HalLcdWriteString(Card_Id,4);
//					}
//			}
			//    } else 
			//    {
			//      //加入认证比较是否为合法的rfid卡来开门 
			//    }
			//    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
			//                         SAMPLEAPP_PERIODIC_CLUSTERID,
			//                         8,
			//                         /*(uint8*)&SampleAppPeriodicCounter,*/
			//                         Card_Id,
			//                         &SampleApp_TransID,
			//                         AF_DISCV_ROUTE,
			//                         AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
			//    {
			//      /*HalLcdWriteString("The Card ID:",3);*/
			//      HalLcdWriteString(Card_Id,4);
			//    }
			//    else
			//    {
			//      // Error occurred in request to send.
			//    }
	}
	return 0;
nv_fail_invalid:
	//HalUARTWrite(0,"flash invalid",13);
	return 1;
}

/*void SampleApp_SendPeriodic5SecMessage(void)*/
/*{*/
/*;*/
/*}*/
/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
	uint8 buffer[3];
	buffer[0] = (uint8)(SampleAppFlashCounter++);
	buffer[1] = LO_UINT16( flashTime );
	buffer[2] = HI_UINT16( flashTime );

	if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
				SAMPLEAPP_FLASH_CLUSTERID,
				3,
				buffer,
				&SampleApp_TransID,
				AF_DISCV_ROUTE,
				AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	{
	}
	else
	{
		// Error occurred in request to send.
	}
}


void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)
{
	uint8 len,*str=NULL;  //len有用数据长度
	str=cmdMsg->msg;        //指向数据开头
	len=*str;               //msg里的第1个字节代表后面的数据长度

	if(*(str+sizeof(uint8)) == 'R') {
		HandleFlashStatus(osal_nv_item_init(ZCD_NV_ROOM_ID,6,NULL));
		HandleFlashStatus(osal_nv_write(ZCD_NV_ROOM_ID,0,6,str+3*sizeof(uint8)));
	}
	if(*(str+sizeof(uint8)) == 'M') {
		HandleFlashStatus(osal_nv_item_init(ZCD_NV_USER_MG_CARD,8,NULL));
		//(len) 4D 0A 35 43 43 46 42 35 32 37 0A
		HandleFlashStatus(osal_nv_write(ZCD_NV_USER_MG_CARD,0,8,str+3*sizeof(uint8))); 
	}
	/********打印出串口接收到的数据，用于提示*********/
	for(int i=1;i<=len;i++)
		HalUARTWrite(0,str+i,1 ); 
	HalUARTWrite(0,"\n",1 );//换行  

	/*****************发送出去***参考网蜂 1小时无线数据传输教程***********************/
	/*if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,*/
	/*[> SAMPLEAPP_COM_CLUSTERID,//自己定义一个 <]*/
	/*SAMPLEAPP_SERIAL_CLUSTERID,*/
	/*len+1,                  // 数据长度         */
	/*str,                    //数据内容*/
	/*&SampleApp_TransID, */
	/*AF_DISCV_ROUTE,*/
	/*AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )*/
	/*{*/
	/*}*/
	/*else*/
	/*{*/
	/*// Error occurred in request to send.*/
	/*}*/
	/*MicroWait(65535);*/
	/*SystemReset();*/
}
/*********************************************************************
 *********************************************************************/
//延时函数  ms
void MilliWait(uint16 time)
{
  MicroWait(1000);    
}
