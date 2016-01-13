#ifndef _TEST_UPPER_STACK_API_H_
#define _TEST_UPPER_STACK_API_H_

typedef enum _TTestResult TTestResult;

TTestResult test_upperstack_CmdRemoteBdSet( PGATTTest pGattTest,
        TTestParseResult *pParseResult );


TTestResult test_upperstack_CmdRemoteBdList( PGATTTest pGattTest,
        TTestParseResult *pParseResult );
TTestResult test_upperstack_CmdRemoteBdRemove( PGATTTest pGattTest,
        TTestParseResult *pParseResult );

/*----------------------------------------------------------------------------
 * command lshndl
 * --------------------------------------------------------------------------*/

TTestResult test_upperstack_CmdHandleUUIDDisplay( PGATTTest pGattTest,
        TTestParseResult *pParseResult );
TTestResult test_upperstack_CmdShortOutput(PGATTTest pGattTest,
        TTestParseResult *pParseResult );
TTestResult test_upperstack_Buftest(PGATTTest pGattTest,
                            TTestParseResult *pParseResult);

TTestResult test_upperstack_SetDataLength(PGATTTest pGattTest,
                            TTestParseResult *pParseResult);

TTestResult test_upperstack_Reset(PGATTTest PGATTTest,
                                           TTestParseResult *pParseResult);

  /*-- gatdbapi.c: BlueAPI interface routines --*/
TTestResult test_upperstack_Register(PGATTTest PGATTTest,
                                           TTestParseResult *pParseResult);

TTestResult test_upperstack_Release(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_Inquiry(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_NameDiscovery(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_GATTSecurityReq(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_ServiceRegister(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_ServiceRelease(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_AttribUpdate(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
int             test_upperstack_AttribUpdateContinue( PGATTTest PGATTTest );



TTestResult test_upperstack_CmdSetAdvertisingEnable(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdSetAdvertisingParameters(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdSetDirectedAdvertising(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_SetAdvertisingData(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);



TTestResult test_upperstack_ConUpdateReq(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_ConUpdateResp(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdAddToWhitelist(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdRemoveFromWhitelist(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdClearWhitelist(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);

TTestResult test_upperstack_CmdVendorSetVoicePara(PGATTTest PGATTTest,
                                  TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdVendorSetBleTxPower(PGATTTest PGATTTest,
                                  TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdSetRandomAddress(PGATTTest PGATTTest,
                                  TTestParseResult *pParseResult);

TTestResult test_upperstack_CmdConLEChan(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdDiscLEChan(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdCredit(PGATTTest PGATTTest,
										  TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdLEData(PGATTTest PGATTTest,
										  TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdLESec(PGATTTest PGATTTest,
										  TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdScanEnable(PGATTTest PGATTTest, 
                                                                        TTestParseResult *pParseResult);


TTestResult test_upperstack_dlps(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult);

TTestResult test_upperstack_ota(PGATTTest pGattTest,
                                  TTestParseResult *pParseResult);

TTestResult test_upperstack_ConnectReq(PGATTTest PGATTTest,
                                            TTestParseResult *pParseResult);
TTestResult test_upperstack_DisconnectReq(PGATTTest PGATTTest,
                                            TTestParseResult *pParseResult);

TTestResult test_upperstack_dump(PGATTTest PGATTTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_ServiceDiscovery(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_RelationshipDiscovery(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CharacteristicDiscovery(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_CharacDescriptorDiscovery(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_ListUUIDs(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_AttribRead(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_AttribReadUUID(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_AttribReadMulti(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
TTestResult test_upperstack_AttribWrite(PGATTTest PGATTTest,
                                          TTestParseResult *pParseResult);
#if (GATTDEMO_PREPARE_WRITE_SUPPPRT)
TTestResult test_upperstack_AttribPrepareWrite(PGATTTest pGattTest,
                                          TTestParseResult *pParseResult);
#endif

TTestResult test_upperstack_CmdTraceLevel(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_CmdSetReadDataLength(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_report_tx(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_report_tx_clear(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_report_rx(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_report_rx_set_pkt_len(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_report_rx_clear(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

TTestResult test_upperstack_report_rx_display(PGATTTest pGattTest,
        TTestParseResult *pParseResult);

/* security specific stuff: gatdbapisec.c */
TTestResult test_upperstack_CmdDevCfg            (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdDevCfgGet            (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdBndList(PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdBndDel(PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdNvClear           (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdNvShow            (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdPrivacyMode       (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdBrMode            (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdBrAuth            (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdPairableMode      (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdAuthConfirm       (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdAuthKeypress      (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdAuthKeyboard      (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdAuthSetOOB        (PGATTTest PGATTTest, TTestParseResult *pParseResult);
TTestResult test_upperstack_CmdAuthPin           (PGATTTest PGATTTest, TTestParseResult *pParseResult);














#endif
