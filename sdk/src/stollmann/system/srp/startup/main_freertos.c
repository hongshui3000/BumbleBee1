/**********************************************************************!MA*
 *
 * $Header: /var/lib/cvs/sw/src/system/srp/startup/main_freertos.c,v 1.5 2013/10/17 09:33:08 mn Exp $
 *
 * File:        $RCSfile: main_freertos.c,v $
 * Version:     $Name:
 *
 * Archive:     $Source: /var/lib/cvs/sw/src/system/srp/startup/main_freertos.c,v $
 * Revision:    $Revision: 1.5 $
 * Date:        $Date: 2013/10/17 09:33:08 $
 * Author:      $Author: mn $
 *
 * ------------------------------------------------------------------------
 * !MODULE      [  ]
 * ------------------------------------------------------------------------
 * !FILE        [  ]
 * !PROGRAM     [  ]
 * !VERSION     [$Name:]
 * !GROUP       [  ]
 * !AUTHOR      [$Author: mn $]
 * ------------------------------------------------------------------------
 *
 *          Copyright (c)           2013 Stollmann E+V GmbH
 *                                  Mendelssohnstr. 15D
 *                                  22761 Hamburg
 *                                  Phone: 040/89088-0
 *          All Rights Reserved
 *
 * ------------------------------------------------------------------------
 * !DESCRIPTION
 *
 *          BlueMod+SR Board Package specific definitions
 *
 * ------------------------------------------------------------------------
 * !INDEX
 *  ...
 * ------------------------------------------------------------------------
 * !CONTENTS
 * ------------------------------------------------------------------------
 * !INCLUDE_REFERENCES
 * ------------------------------------------------------------------------
 * !HISTORY
 *  Date      Author          Comment
 *  tt.mm.jj                  Initial revision
 *  tt.mm.jj
 * ------------------------------------------------------------------------
 *
 * $Log: main_freertos.c,v $
 * Revision 1.5  2013/10/17 09:33:08  mn
 * issue 0011138
 * HCI API (Low Layer API).
 *
 * Revision 1.4  2013/09/03 07:40:49  mn
 * issue 0010764
 * Added LTP: BtStack library test.
 *
 * Revision 1.3  2013/08/16 06:49:36  mn
 * issue 0010764
 * Added hardware detection.
 *
 * Revision 1.2  2013/08/15 10:44:37  mn
 * issue 0010764
 * Added SPPDEMO.
 *
 * Revision 1.1  2013/05/27 10:20:52  mn
 * issue 0008559
 * First commit.
 *
 **********************************************************************!HE*/

#include <stdio.h>

/****************************************************************************/
/* FreeRTOS                                                                 */
/****************************************************************************/

#include <FreeRTOS.h>
#include <task.h>

#include <hw_srp.h>

#include <misc.h>

/****************************************************************************/
/* Prototypes                                                               */
/****************************************************************************/

void hciApiInit(void);          /* HCIAPI */
#if defined(SRP0007)
void ltp_main(void);
#endif
#if defined(SPPDEMO)
void sppdInit(void);         /* sppdemo_freertos.c */
#elif defined(HIDDEMO)
void hiddInit(void);        /* hiddemo_freertos.c */
#elif defined(HDPDEMO)
void hdpdInit(void);        /* hiddemo_freertos.c */
#elif defined(TESTPERIPHERAL)
int peripheral_main(void);      /* simpleBLEPeripheralApp */
#elif defined(TESTCENTRAL)
int central_main(void);         /* simpleBLECentralApp */
#elif defined(TESTBROADCASTER)
int broadcaster_main(void);     /* simpleBLEBroadcasterApp */
#elif defined(TESTOBSERVER)
int observer_main(void);        /* simpleBLEObserverApp */
#elif defined(TESTPERIPHDEMO)
int test_peripheral_main(void);     /* test_peripheral_application */
#elif defined(TESTCENTRALDEMO)
int test_central_main(void);        /* test_central_application */
#elif defined(TESTLEGACYDEMO)
int Testlegacy_main(void);
#elif defined(TESTAVRCPDEMO)
int TestAvrcp_main(void);
#elif defined(TESTRFCOMMDEMO)
int TestRfcomm_main(void);
#elif defined(TESTDUALDEMO)
int Testdual_main(void);
#elif defined(TESTBQB)
int bqb_main(void);
#else
void gattdInit(void);        /* gattdemo_freertos.c */
#endif
void stInitFreeRTOS(void);   /* stfreertos.c */


/****************************************************************************/
/* main                                                                     */
/****************************************************************************/

int main( void )
{
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  hwDetectHwVersion();  /* BlueMOD+SR */
  hciApiInit();
  stInitFreeRTOS();
#if defined(SRP0007)
  ltp_main();
#else
#if defined(SPPDEMO)
  sppdInit();
#elif defined(HIDDEMO)
  hiddInit();        /* hiddemo_freertos.c */
#elif defined(HDPDEMO)
  hdpdInit();        /* hdpdemo_freertos.c */
#elif defined(TESTCENTRALDEMO)
  test_central_main();          /* test_central_application */
#elif defined(TESTPERIPHDEMO)
  test_peripheral_main();       /* test_peripheral_application */
#elif defined(TESTCENTRAL)
  central_main();               /* simpleBLECentralApp */
#elif defined(TESTBROADCASTER)
  broadcaster_main();           /* simpleBLEBroadcasterApp */
#elif defined(TESTOBSERVER)
  observer_main();              /* simpleBLEObserverApp */
#elif defined(TESTPERIPHERAL)
  peripheral_main();            /* simpleBLEPeripheralApp */
#elif defined(TESTLEGACYDEMO)
  Testlegacy_main();
#elif defined(TESTAVRCPDEMO)
  TestAvrcp_main();
#elif defined(TESTA2DPAVRCPDEMO)
  extern int TestA2dpAvrcp_main(void);
  TestA2dpAvrcp_main();
#elif defined(TESTRFCOMMDEMO)
  TestRfcomm_main();
#elif defined(TESTDUALDEMO)
  Testdual_main();
#elif defined(TESTBQB)
  bqb_main();
#else
  gattdInit();
#endif
#endif

    vTaskStartScheduler();    /* Start the scheduler. */

    /* no return */
    return 0;
}
