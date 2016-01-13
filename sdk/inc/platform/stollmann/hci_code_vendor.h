
#ifndef __HCI_CODE_VENDOR_H
#define __HCI_CODE_VENDOR_H



/** CSR-Vendor specific cmds and events: defines and constans  */
#define HCI_CSR_BCCMDVARID_SPECIAL_START    0x1000

#define HCI_CSR_BCCMDVARID_NOACCESS         0
#define HCI_CSR_BCCMDVARID_READONLY         0x2000
#define HCI_CSR_BCCMDVARID_WRITEONLY        0x4000
#define HCI_CSR_BCCMDVARID_READWRITE        0x6000

#define HCI_CSR_EXTN_PACKET                 0xFC00
#define HCI_CSR_EXTN_PACKET_MAX_SIZE        258
#define HCI_CSR_VDBG_HEADER_SIZE            5

/** Vendor specific cmds and events: status code  */
#define HCI_CSR_VDBG_STATUS_OK                  0x0000
#define HCI_CSR_VDBG_STATUS_NO_SUCH_VARID       0x0001
#define HCI_CSR_VDBG_STATUS_TOO_BIG             0x0002
#define HCI_CSR_VDBG_STATUS_NO_VALUE            0x0003
#define HCI_CSR_VDBG_STATUS_BAD_REQ             0x0004
#define HCI_CSR_VDBG_STATUS_NO_ACCESS           0x0005
#define HCI_CSR_VDBG_STATUS_READ_ONLY           0x0006
#define HCI_CSR_VDBG_STATUS_WRITE_ONLY          0x0007
#define HCI_CSR_VDBG_STATUS_ERROR               0x0008
#define HCI_CSR_VDBG_STATUS_PERMISSION_DENIED   0x0009

/** Vendor specific cmds and events: channel ID in payload description     */
#define HCI_CSR_VDBG_CHNL_ID_BCCMD 0x02 /* used for HCI tunneling */
#define HCI_CSR_VDBG_CHNL_ID_HQ 0x03    /* used for HQ protocol */

/** CSR-Vendor specific cmds and events: fragmentation in payload description */
#define HCI_CSR_FIRST_FRAGMENT              0x40
#define HCI_CSR_LAST_FRAGMENT               0x80

/** CSR-Vendor specific cmds and events: messages types                    */
#define HCI_CSR_GETREQ                      0x0000
#define HCI_CSR_GETRESP                     0x0001
#define HCI_CSR_SETREQ                      0x0002

/** CSR-Vendor specific cmds and events: system control commands           */
#define HCI_CSR_BCCMDVARID_WARM_RESET       (HCI_CSR_BCCMDVARID_WRITEONLY +0x0002)
#define HCI_CSR_BCCMDVARID_CONFIG_UART      (HCI_CSR_BCCMDVARID_READWRITE +0x0802)
#define HCI_CSR_BCCMDVARID_MAX_TX_PWR       (HCI_CSR_BCCMDVARID_READWRITE +0x0827)
#define HCI_CSR_BCCMDVARID_DEFAULT_TX_PWR   (HCI_CSR_BCCMDVARID_READWRITE +0x082B)

#if (F_BT_LOW_ENERGY)
#define HCI_CSR_BCCMDVARID_BLE_DEFAULT_TX_POWER (HCI_CSR_BCCMDVARID_READWRITE +0x087A)
#endif

/** CSR-Vendor specific cmds and events: connection status commands        */
#define HCI_CSR_BCCMDVARID_RSSI_ACL         (HCI_CSR_BCCMDVARID_SPECIAL_START \
                                              + HCI_CSR_BCCMDVARID_READONLY +0x001D)

/**CSR-Vendor specific cmds and events: ADC commands                      */
#define HCI_CSR_BCCMDVARID_ADC              (HCI_CSR_BCCMDVARID_WRITEONLY +0x0829)
#define HCI_CSR_BCCMDVARID_ADC_RESULT       (HCI_CSR_BCCMDVARID_SPECIAL_START \
                                              + HCI_CSR_BCCMDVARID_READONLY  +0x0007)

/**CSR-Vendor specific cmds and events: connection control commands       */
#define HCI_CSR_BCCMDVARID_MC_MAP_SCO_PCM   (HCI_CSR_BCCMDVARID_WRITEONLY +0x081C)

/** CSR-Vendor specific cmds and events: PIO commands                      */
#define HCI_CSR_BCCMDVARID_PIO_DIRECTION_MASK (HCI_CSR_BCCMDVARID_READWRITE +0x081E)
#define HCI_CSR_BCCMDVARID_PIO                (HCI_CSR_BCCMDVARID_READWRITE +0x081F)

/**  CSR-Vendor specific cmds and events: test commands                     */
#define HCI_CSR_BCCMDVARID_PROVOKE_FAULT       (HCI_CSR_BCCMDVARID_WRITEONLY +0x0822)

/**CSR-Vendor specific cmds and events: Persistent store commands         */
#define HCI_CSR_BCCMDVARID_PSKEY_READWRITE (HCI_CSR_BCCMDVARID_SPECIAL_START \
                                             + HCI_CSR_BCCMDVARID_READWRITE + 3)


/**  CSR-Vendor specific cmds and events: DSP Manager  */
#define HCI_CSR_VARID_DSPMANAGER_CREATE_OPERATOR_C                        0x5075
#define HCI_CSR_VARID_DSPMANAGER_CONFIG_REQUEST                           0x101C


/** S keys - destination storage *** */
#define HCI_CSR_PSKEY_STORAGE_DEFAULT             0x0000 /**< psram, psi. psf and then psrom */
#define HCI_CSR_PSKEY_STORAGE_PSI                 0x0001
#define HCI_CSR_PSKEY_STORAGE_PSF                 0x0002
#define HCI_CSR_PSKEY_STORAGE_PSI_PSF             0x0003 /**< psi then psf */
#define HCI_CSR_PSKEY_STORAGE_PSROM               0x0004
#define HCI_CSR_PSKEY_STORAGE_PSI_PSF_PSROM       0x0007
#define HCI_CSR_PSKEY_STORAGE_PSRAM               0x0008
#define HCI_CSR_PSKEY_STORAGE_PSRAM_PSI           0x0009
#define HCI_CSR_PSKEY_STORAGE_PSRAM_PSI_PSF       0x000B
#define HCI_CSR_PSKEY_STORAGE_PSRAM_PSI_PSF_PSROM 0x000F

/** PS keys *** */
#define HCI_CSR_PSKEY_NULL                  0x0000 /**< not valid PSKEY - used as marker at end of pskey table  */
#define HCI_CSR_PSKEY_BDADDR                0x0001
#define HCI_CSR_PSKEY_MAX_SCOS              0x000E
#define HCI_CSR_PSKEY_H_HC_FC_MAX_ACL_PKT_LEN 0x0011
#define HCI_CSR_PSKEY_H_HC_FC_MAX_ACL_PKTS  0x0013
#define HCI_CSR_PSKEY_LC_MAX_TX_POWER       0x0017
#define HCI_CSR_PSKEY_LC_DEFAULT_TX_POWER         0x0021
#define HCI_CSR_PSKEY_LC_MAX_TX_POWER_NO_RSSI     0x002D
#define HCI_CSR_PSKEY_LC_ENHANCED_POWER_TABLE 0x0031
#define HCI_CSR_PSKEY_PATCH0                0x0096 /**< first PSKEY patch                                       */
#define HCI_CSR_PSKEY_PATCH1                0x0097
#define HCI_CSR_PSKEY_PATCH17               0x00A7
#define HCI_CSR_PSKEY_PATCH25               0x00AF
#define HCI_CSR_PSKEY_PATCH30               0x00B4
#define HCI_CSR_PSKEY_PATCH31               0x00B5
#define HCI_CSR_PSKEY_PATCH32               0x00B6
#define HCI_CSR_PSKEY_PATCH33               0x00B7
#define HCI_CSR_PSKEY_PATCH34               0x00B8
#define HCI_CSR_PSKEY_PATCH35               0x00B9
#define HCI_CSR_PSKEY_PATCH49               0x00C7 /**< last PSKEY patch                                        */
#define HCI_CSR_PSKEY_LOCAL_NAME0           0x00DC
#define HCI_CSR_PSKEY_LOCAL_NAME1           (HCI_CSR_PSKEY_LOCAL_NAME0 + 1)
#define HCI_CSR_PSKEY_LOCAL_NAME2           (HCI_CSR_PSKEY_LOCAL_NAME0 + 2)
#define HCI_CSR_PSKEY_LOCAL_NAME3           (HCI_CSR_PSKEY_LOCAL_NAME0 + 3)
#define HCI_CSR_PSKEY_LOCAL_NAME4           (HCI_CSR_PSKEY_LOCAL_NAME0 + 4)
#define HCI_CSR_PSKEY_LOCAL_NAME5           (HCI_CSR_PSKEY_LOCAL_NAME0 + 5)
#define HCI_CSR_PSKEY_LOCAL_NAME6           (HCI_CSR_PSKEY_LOCAL_NAME0 + 6)
#define HCI_CSR_PSKEY_LOCAL_NAME7           (HCI_CSR_PSKEY_LOCAL_NAME0 + 7)
#define HCI_CSR_PSKEY_LOCAL_NAME8           (HCI_CSR_PSKEY_LOCAL_NAME0 + 8)
#define HCI_CSR_PSKEY_LOCAL_NAME9           (HCI_CSR_PSKEY_LOCAL_NAME0 + 9)
#define HCI_CSR_PSKEY_LOCAL_NAME10          (HCI_CSR_PSKEY_LOCAL_NAME0 + 10)
#define HCI_CSR_PSKEY_LOCAL_NAME11          (HCI_CSR_PSKEY_LOCAL_NAME0 + 11)
#define HCI_CSR_PSKEY_LOCAL_NAME12          (HCI_CSR_PSKEY_LOCAL_NAME0 + 12)
#define HCI_CSR_PSKEY_LOCAL_NAME13          (HCI_CSR_PSKEY_LOCAL_NAME0 + 13)
#define HCI_CSR_PSKEY_LOCAL_NAME14          (HCI_CSR_PSKEY_LOCAL_NAME0 + 14)
#define HCI_CSR_PSKEY_LOCAL_NAME15          (HCI_CSR_PSKEY_LOCAL_NAME0 + 15)
#define HCI_CSR_PSKEY_LOCAL_NAME16          (HCI_CSR_PSKEY_LOCAL_NAME0 + 16)
#define HCI_CSR_PSKEY_LOCAL_NAME17          (HCI_CSR_PSKEY_LOCAL_NAME0 + 17)
#define HCI_CSR_PSKEY_HOSTIO_USE_HCI_EXTN_CCFC  0x01A6
#define HCI_CSR_PSKEY_HOSTIO_MAP_SCO_PCM    0x01AB
#define HCI_CSR_PSKEY_LOCAL_LENGTH          0x00EE
#define HCI_CSR_PSKEY_LOCAL_SUPPORTED_FEATURES              0x00EF
#define HCI_CSR_PSKEY_PCM_CONFIG32          0x01B3
#define HCI_CSR_PSKEY_UART_BAUDRATE         0x01BE
#define HCI_CSR_PSKEY_UART_BITRATE          0x01EA  /**< This key replaces PSKEY_UART_BAUDRATE for BC7+ chips */

#define HCI_CSR_PSKEY_UART_CONFIG_H4DS            0X01CB
#define HCI_CSR_PSKEY_H4DS_WAKE_DURATION          0X01CC
#define HCI_CSR_PSKEY_H4DS_MAXWU                  0X01CD
#define HCI_CSR_PSKEY_H4DS_LE_TIMER_PERIOD        0X01CF
#define HCI_CSR_PSKEY_H4DS_TWU_TIMER_PERIOD       0X01D0
#define HCI_CSR_PSKEY_H4DS_UART_IDLE_TIMER_PERIOD 0X01D1

#define HCI_CSR_PSKEY_ANA_FTRIM             0x01F6
#define HCI_CSR_PSKEY_HOST_INTERFACE        0x01F9
#define HCI_CSR_PSKEY_ANA_FREQ              0x01FE
#define HCI_CSR_PSKEY_TXRX_PIO_CONTROL      0x0209
#define HCI_CSR_PSKEY_TX_OFFSET_HALF_MHZ    0x0217
#define HCI_CSR_PSKEY_DEEP_SLEEP_WAKE_CTS   0x023C
#define HCI_CSR_PSKEY_TX_PRE_LVL            0x0240
#define HCI_CSR_PSKEY_RX_SINGLE_ENDED       0x0242
#define HCI_CSR_PSKEY_CLOCK_REQUEST_ENABLE  0x0246
#define HCI_CSR_PSKEY_TEMPERATURE_VS_DELTA_TX_BB_MR_PAYLOAD 0x03AA
#define HCI_CSR_PSKEY_TEMPERATURE_VS_DELTA_TX_BB_MR_HEADER  0x03AB
#define HCI_CSR_PSKEY_MR_ANA_RX_FTRIM                       0x03BF
#define HCI_CSR_PSKEY_TEMPERATURE_VS_DELTA_ANA_FTRIM        0x03D7
#define HCI_CSR_PSKEY_TEMPERATURE_VS_DELTA_TX_PRE_LVL       0x03D9
#define HCI_CSR_PSKEY_TEMPERATURE_VS_DELTA_INTERNAL_PA      0x03DA
#define HCI_CSR_PSKEY_TEMPERATURE_CALIBRATION               0x03DB
#define HCI_CSR_PSKEY_DEEP_SLEEP_USE_EXTERNAL_CLOCK         0x03C3


#endif /**< __HCI_CODE_H */
