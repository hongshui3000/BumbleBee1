/***************************************************************************
 Copyright (C) Mindtree Ltd.
 This module is a confidential and proprietary property of MindTree and
 a possession or use of this module requires written permission of MindTree.
 ***************************************************************************/

/** 
 * \file bz_config.h
 *  BlueWiz Configuration File. This file contains all the macros that control
 *  the components to be compiled. It offers more fine grained control over
 *  the size of the firmware. Make the value any macro to 0, if you don't want
 *  that component to be part of compilation. Otherwise, make it as 1.
 * 
 * \author Santhosh kumar M
 * \date 2007-02-01
 */

#ifndef _BZ_CONFIG_H_
#define _BZ_CONFIG_H_


/* ========================= HCI related Macros ========================= */
#ifdef COMPILE_FIVE_SLOT_PACKET
#define HCI_ACL_DATA_REPORT_SIZE      1021
#elif COMPILE_THREE_SLOT_PACKET
#define HCI_ACL_DATA_REPORT_SIZE      552
#else
#define HCI_ACL_DATA_REPORT_SIZE      81
#endif

/* ========================= LMP related Macros ========================= */

/**
 * Checks the parameters received from the remote device against the spec. It
 * will enable checking of the parameters for its boundary conditions and
 * whether they are valid or not. It doesn't check for applicability of these
 * parameters at the current instant.
 *
 * Default: 0
 */
#define LMP_PARAMS_CHECK                    0

/**
 * Checks the Transaction ID received from the remote device.
 *
 * Default: 0
 */
#define LMP_TID_CHECK                       0

/* ========================= LC related Macros ========================== */



/* ====================== Feature Specific Macros ======================= */

/* ========================= SCO related Macros ========================= */

/* ======================== ESCO related Macros ========================= */

/* ===================== Authentication related Macros ================== */


#endif /* _BZ_CONFIG_H_ */
