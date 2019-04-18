/*
/ _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
\____ \| ___ |    (_   _) ___ |/ ___)  _ \
_____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
(C)2013 Semtech

Description: LoRaMac classA device implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

/*! \file classA/SensorNode/main.c */

#include "Platform.h"
#include <string.h>
#include <math.h>
#include "board.h"
#include "utils.h"
#include "gps.h"
#include "LoRaMac.h"
#include "Region.h"
#include "Commissioning.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "custom_board.h"
#include "lis3dh_driver.h"
#include "rak_i2c_sht31.h"
/*!
* Defines the application data transmission duty cycle. value in [ms].
*/
#define APP_TX_DUTYCYCLE                            (30*1000)

/*!
* Defines a random delay for application data transmission duty cycle. 1s,
* value in [ms].
*/
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
* Default datarate
*/
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
* LoRaWAN confirmed messages
*/
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
* LoRaWAN Adaptive Data Rate
*
* \remark Please note that when ADR is enabled the end-device should be static
*/
#define LORAWAN_ADR_ON                              1

#if defined( REGION_EU868 )

#include "LoRaMacTest.h"

/*!
* LoRaWAN ETSI duty cycle control enable/disable
*
* \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
*/
#define LORAWAN_DUTYCYCLE_ON                        true

//#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, 0, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, 0, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, 0, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

/*!
* LoRaWAN application port
*/
#define LORAWAN_APP_PORT                            2

/*!
* User application data buffer size
*/
#if defined( REGION_CN470 ) || defined( REGION_CN779 ) || defined( REGION_EU433 ) || defined( REGION_EU868 ) || defined( REGION_IN865 ) || defined( REGION_KR920 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( REGION_AS923 ) || defined( REGION_AU915 ) || defined( REGION_US915 ) || defined( REGION_US915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#else

#error "Please define a region in the compiler options."

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
* Device address
*/
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
* Application port
*/
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
* User application data size
*/
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
* User application data buffer size
*/
#define LORAWAN_APP_DATA_MAX_SIZE                           242

/*!
* User application data
*/
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
* Indicates if the node is sending confirmed or unconfirmed messages
*/
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
* Defines the application data transmission duty cycle
*/
static uint32_t TxDutyCycleTime;

/*!
* Timer to handle the application data transmission duty cycle
*/
static TimerEvent_t TxNextPacketTimer;

/*!
* Specifies the state of the application LED
*/
static bool AppLedStateOn = false;

/*!
* Timer to handle the state of LED1
*/
//static TimerEvent_t Led1Timer;

/*!
* Timer to handle the state of LED2
*/
//static TimerEvent_t Led2Timer;

/*!
* Indicates if a new packet can be sent
*/
static bool NextTx = true;

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

//extern void cli_init(void);
//extern void cli_start(void);
//extern void cli_process(void);

static const lora_cfg_t g_def_cfg =
{
    .sof     = 0x00,
    .dev_eui = LORAWAN_DEVICE_EUI,
    .app_eui = LORAWAN_APPLICATION_EUI,
    .app_key = LORAWAN_APPLICATION_KEY,
    .dev_addr = LORAWAN_DEVICE_ADDRESS,
    .nwkskey = LORAWAN_NWKSKEY,
    .appskey = LORAWAN_APPSKEY,
};

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,
    
    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
    * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
    * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
    * last page of flash available to write data. */
    .start_addr = 0x0007F000,
    .end_addr   = 0x00080000,
};

/*!
* Device states
*/
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
* LoRaWAN compliance tests support data
*/
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }
    
    switch (p_evt->id)
    {
      case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
        
      case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
        
      default:
        break;
    }
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}

void u_fs_init(void)
{
    ret_code_t rc;
    //NRF_LOG_INFO(LOG_INFO, "fstorage example started!");
    
    nrf_fstorage_api_t * p_fs_api;
    
    //NRF_LOG_INFO(LOG_INFO, "SoftDevice is present.");
    //NRF_LOG_INFO(LOG_INFO, "Initializing nrf_fstorage_sd implementation...");
    /* Initialize an fstorage instance using the nrf_fstorage_sd backend.
    * nrf_fstorage_sd uses the SoftDevice to write to flash. This implementation can safely be
    * used whenever there is a SoftDevice, regardless of its status (enabled/disabled). */
    p_fs_api = &nrf_fstorage_sd;
    
    rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
    
}

void u_fs_check_lora_cfg(lora_cfg_t *cfg)
{
    printf("sof: %x\r\n",cfg->sof);
    if(cfg->sof != 0x55)
    {
        memcpy((uint8_t*)cfg,&g_def_cfg,sizeof(g_def_cfg));
    }
}

void u_fs_read_lora_cfg(lora_cfg_t *cfg)
{
    ret_code_t rc;
    rc = nrf_fstorage_read(&fstorage, fstorage.start_addr, cfg, sizeof(lora_cfg_t));
    APP_ERROR_CHECK(rc);
}

void u_fs_write_lora_cfg(lora_cfg_t *cfg)
{
    ret_code_t rc;
    
    rc = nrf_fstorage_erase(&fstorage, fstorage.start_addr, 1, NULL);
    NRF_LOG_INFO("erase %d", rc);
    APP_ERROR_CHECK(rc);
    //wait_for_flash_ready(&fstorage);

    cfg->sof = 0x55;
    rc = nrf_fstorage_write(&fstorage, fstorage.start_addr, cfg, sizeof(lora_cfg_t), NULL);
    NRF_LOG_INFO("write %d", rc);
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
    printf("LoRaWAN parameters configured successfully\r\n");
}



void dump_hex2str(uint8_t *buf , uint8_t len)
{
    for(uint8_t i=0; i<len; i++) {
        printf("%02X ", buf[i]);
    }
    printf("\r\n");
}

static void PrintTxFrame(void) {
  int i;

  printf("TX Frame: ");
  for(i=0; i<AppDataSize; i++) {
    printf("%02X ", AppData[i]);
  }
  printf("\r\n");
}

extern peripherals_data per_data;

static void PrepareTxFrame(uint8_t port)
{
    switch( port )
    {
      case 2:
#if !PL_USE_GPS
        { /* dummy data */
            AppData[0] = 0x01;
            AppData[1] = 0x02;
            AppData[2] = 0x03;
            AppData[3] = 0x04;
            AppData[4] = 0x05;
            AppDataSize = 5;
        }
#else
        {
          /*lat=47.054777 lon=8.585165 alt=659*/
          /* https://www.thethingsnetwork.org/docs/applications/ttnmapper/ */
          /* https://github.com/AmedeeBulle/ttn-mapper/blob/master/ttn-mapper-gps/ttn_mapper.cpp#L116 */
          float lat = 47.054777;
          float lon = 8.585165;
          uint16_t alt = 659;
          float hdop = 1.0; /* Horizontal Dilution of Precision, lower is better */ /* value of >0x30xx is considered as a fix */
          uint32_t val;

          if (per_data.gps_quality>=0x3000) {
            hdop = (per_data.gps_quality-0x3000);
            hdop /= 100.0f;
            hdop = 20.0f-hdop;
            hdop -= 15; /* artificial adjustment */
            if (hdop<1.0f) {
              hdop = 1.0f;
            } else if (hdop>20.0f) {
              hdop = 20.0f;
            }
          } else {
            //hdop = 20.0; /* considering as bad */
            hdop = 1.0;
          }
          lat = per_data.gps_latitude;//47.054777;
          lon = per_data.gps_longitude;//8.585165;
          alt = per_data.gps_altitude; //659;

          val = ((lat*(lat>=0?1:-1)+90)/180)*16777215;
          AppData[0]  = val>>16; /* lat */
          AppData[1]  = val>>8;
          AppData[2]  = val;
          val = ((lon*(lon>=0?1:-1)+180)/360)*16777215;
          AppData[3]  = val>>16; /* lon */
          AppData[4]  = val>>8;
          AppData[5]  = val;
          AppData[6]  = alt>>8; /* alt */
          AppData[7]  = alt;
          AppData[8]  = (uint8_t)(hdop*10.0); /* 1.0: best, 20.0: very poor */
          AppDataSize = 9;
        }
#if 0 /* tnn decoder function */
        /* https://www.thethingsnetwork.org/forum/t/ttn-mapper-and-no-data-appearing-for-ttgo-t-beam/15474/5 */
function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;

    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign) {
      decoded.altitude = 0xFFFF0000 | altValue;
    } else {
      decoded.altitude = altValue;
    }
    decoded.hdop = bytes[8] / 10.0;
    return decoded;
}
#endif
#endif
        break;
      case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
              case 4:
                ComplianceTest.State = 1;
                break;
              case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
      default:
        break;
    }
}

/*!
* \brief   Prepares the payload of the frame
*
* \retval  [0: frame could be send, 1: error]
*/
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;
    
    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }
    
    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
* \brief Function executed on TxNextPacket Timeout event
*/
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;
    
    TimerStop( &TxNextPacketTimer );
    
    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );
    
    printf("LoRa OnTxNextPacketTimerEvent\r\n");
    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
* \brief Function executed on Led 1 Timeout event
*/
//static void OnLed1TimerEvent( void )
//{
//    TimerStop( &Led1Timer );
//    // Toggle LED 1 
//    GpioToggle( &Led1 );
//    TimerStart( &Led1Timer );
//}


/*!
* \brief   MCPS-Confirm event function
*
* \param   [IN] mcpsConfirm - Pointer to the confirm structure,
*               containing confirm attributes.
*/
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
          case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                break;
            }
          case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                break;
            }
          case MCPS_PROPRIETARY:
            {
                break;
            }
          default:
            break;
        }
        
    }
    NextTx = true;
}

/*!
* \brief   MCPS-Indication event function
*
* \param   [IN] mcpsIndication - Pointer to the indication structure,
*               containing indication attributes.
*/
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    printf("LoRa McpsIndication\r\n");
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }
    
    switch( mcpsIndication->McpsIndication )
    {
      case MCPS_UNCONFIRMED:
        {
            break;
        }
      case MCPS_CONFIRMED:
        {
            break;
        }
      case MCPS_PROPRIETARY:
        {
            break;
        }
      case MCPS_MULTICAST:
        {
            break;
        }
      default:
        break;
    }
    
    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot
    
    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }
    
    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
          case 1: // The application LED can be controlled on port 1 or 2
          case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
            }
            break;
          case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                   ( mcpsIndication->Buffer[0] == 0x01 ) &&
                       ( mcpsIndication->Buffer[1] == 0x01 ) &&
                           ( mcpsIndication->Buffer[2] == 0x01 ) &&
                               ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                    //GpsStop( );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                  case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;
                    
                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    //GpsStart( );
                    break;
                  case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                  case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                  case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                  case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;
                    
                    AppData[0] = 4;
                    for( uint8_t i = 1; i < MIN( AppDataSize, LORAWAN_APP_DATA_MAX_SIZE ); i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                  case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                  case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;
                        
                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;
                        
                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( REGION_EU868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                        //GpsStart( );
                        
                        mlmeReq.Type = MLME_JOIN;
                        
                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;
                        
                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                  case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                  default:
                    break;
                }
            }
            break;
          default:
            break;
        }
    }
    
    // Switch LED 1 ON for each received downlink
    //GpioWrite( &Led1, 0 );
    //TimerStart( &Led1Timer );
}

/*!
* \brief   MLME-Confirm event function
*
* \param   [IN] mlmeConfirm - Pointer to the confirm structure,
*               containing confirm attributes.
*/
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    printf("LoRa MlmeConfirm\r\n");
    switch( mlmeConfirm->MlmeRequest )
    {
      case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
                printf("OTAA Join Success \r\n");
			#if PL_USE_OLED
			  Write_OLED_string("Join success!");
			#endif
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
      case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
      default:
        break;
    }
    NextTx = true;
}


LoRaMacPrimitives_t LoRaMacPrimitives;
LoRaMacCallback_t LoRaMacCallbacks;
MibRequestConfirm_t mibReq;

void lora_init(void)
{
    BoardInitMcu();
    DeviceState = DEVICE_STATE_INIT;
}


lora_cfg_t *lora_cfg=&g_lora_cfg;

void lora_process(void)
{
    switch(DeviceState)
    {
      case DEVICE_STATE_INIT:
        {
          printf("LoRa DEVICE_STATE_INIT\r\n");
            LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
            LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
            LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
            LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
#if defined( REGION_AS923 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AS923 );
#elif defined( REGION_AU915 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_AU915 );
#elif defined( REGION_CN470 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN470 );
#elif defined( REGION_CN779 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_CN779 );
#elif defined( REGION_EU433 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU433 );
#elif defined( REGION_EU868 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_EU868 );
#elif defined( REGION_IN865 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_IN865 );
#elif defined( REGION_KR920 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_KR920 );
#elif defined( REGION_US915 )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915 );
#elif defined( REGION_US915_HYBRID )
            LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks, LORAMAC_REGION_US915_HYBRID );
#else
#error "Please define a region in the compiler options."
#endif
            TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
            
            //TimerInit( &Led1Timer, OnLed1TimerEvent );
            //TimerSetValue( &Led1Timer, 2000 );   
            
            mibReq.Type = MIB_ADR;
            mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            mibReq.Type = MIB_PUBLIC_NETWORK;
            mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            DeviceState = DEVICE_STATE_JOIN;
            NRF_LOG_INFO("goto to join");
            break;
        }
      case DEVICE_STATE_JOIN:
        {
          printf("LoRa DEVICE_STATE_JOIN\r\n");
#if( OVER_THE_AIR_ACTIVATION != 0 )
            MlmeReq_t mlmeReq;
            
            // Initialize LoRaMac device unique ID
            //BoardGetUniqueId( DevEui );
            
            printf("OTAA:\r\n");
            printf("Dev_EUI: ");
            dump_hex2str(lora_cfg->dev_eui , 8);
            printf("AppEui: ");
            dump_hex2str(lora_cfg->app_eui , 8);
            printf("AppKey: ");
            dump_hex2str(lora_cfg->app_key , 16);
            
            mlmeReq.Type = MLME_JOIN;
            
            mlmeReq.Req.Join.DevEui = lora_cfg->dev_eui;//DevEui;
            mlmeReq.Req.Join.AppEui = lora_cfg->app_eui;//AppEui;
            mlmeReq.Req.Join.AppKey = lora_cfg->app_key;//AppKey;
            mlmeReq.Req.Join.NbTrials = 3;
            
#if defined ( REGION_US915 )  
            
            uint16_t ch_mask[5];
            ch_mask[0] =0xff00;
            ch_mask[1] =0x0000;
            ch_mask[2] =0x0000;
            ch_mask[3] =0x0000;
            ch_mask[4] =0x0000;
            
            mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;  
            mibReq.Param.ChannelsDefaultMask = ch_mask;
            LoRaMacMibSetRequestConfirm( &mibReq ); 
            
            mibReq.Type = MIB_CHANNELS_MASK;  
            mibReq.Param.ChannelsDefaultMask = ch_mask;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
#endif            
            if( NextTx == true )
            {
                LoRaMacStatus_t status;
                status = LoRaMacMlmeRequest( &mlmeReq );
                NRF_LOG_INFO("OTAA Join Start...%d \r\n", status);
                printf("OTAA Join Start...\r\n");
			#if PL_USE_OLED
			  Write_OLED_string("OTAA join...");
			#endif
           }
            DeviceState = DEVICE_STATE_SLEEP;
            NRF_LOG_INFO("goto to sleep");
            printf("goto to sleep\r\n");
#else
            // Choose a random device address if not already defined in Commissioning.h
            if( DevAddr == 0 )
            {
                // Random seed initialization
                srand1( BoardGetRandomSeed( ) );
                
                // Choose a random device address
                DevAddr = randr( 0, 0x01FFFFFF );
            }
            
            printf("ABP: \r\n");
            printf("Dev_EUI: ");
            dump_hex2str(lora_cfg->dev_eui , 8);
            printf("DevAddr: %08X\r\n", lora_cfg->dev_addr);
            printf("NwkSKey: ");
            dump_hex2str(lora_cfg->nwkskey , 16);
            printf("AppSKey: ");
            dump_hex2str(lora_cfg->appskey , 16);
            
            mibReq.Type = MIB_NET_ID;
            mibReq.Param.NetID = LORAWAN_NETWORK_ID;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            mibReq.Type = MIB_DEV_ADDR;
            mibReq.Param.DevAddr = lora_cfg->dev_addr;//DevAddr;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            mibReq.Type = MIB_NWK_SKEY;
            mibReq.Param.NwkSKey = lora_cfg->nwkskey;//NwkSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            mibReq.Type = MIB_APP_SKEY;
            mibReq.Param.AppSKey = lora_cfg->appskey;//AppSKey;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            mibReq.Type = MIB_NETWORK_JOINED;
            mibReq.Param.IsNetworkJoined = true;
            LoRaMacMibSetRequestConfirm( &mibReq );
            
            DeviceState = DEVICE_STATE_SEND;
#endif
            break;
        }
      case DEVICE_STATE_SEND:
        {
            printf("LoRa DEVICE_STATE_SEND\r\n");
            if( NextTx == true )
            {
             // if (GpsHasFix()) { /*! \todo */
                PrepareTxFrame(2);
                PrintTxFrame();
                NextTx = SendFrame();
             // } else {
             //   NRF_LOG_INFO("No GPS fix.")
             //   printf("no GPS fix.\r\n");
             // }
            }
            if( ComplianceTest.Running == true )
            {
                // Schedule next packet transmission
                TxDutyCycleTime = 5000; // 5000 ms
            }
            else
            {
                // Schedule next packet transmission
                TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
            }
            DeviceState = DEVICE_STATE_CYCLE;
            break;
        }
      case DEVICE_STATE_CYCLE:
        {
            printf("LoRa DEVICE_STATE_CYCLE\r\n");
            DeviceState = DEVICE_STATE_SLEEP;
            
            // Schedule next packet transmission
            TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
            TimerStart( &TxNextPacketTimer );
            break;
        }
      case DEVICE_STATE_SLEEP:
        {
            // Wake up through events
            TimerLowPowerHandler();
            break;
        }
      default:
        {
            DeviceState = DEVICE_STATE_INIT;
            break;
        }
    }
}
