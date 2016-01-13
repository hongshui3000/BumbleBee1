/**
********************************************************************************************************
Copyright (c) 2015, Realtek Semiconductor Corporation. All rights reserved.
*********************************************************************************************************
* @file        att_code.h
* @brief      ATT Protocol Layer : Public Codes
* @details   
*
* @author   	gordon
* @date      	2015-07-10
* @version	v0.1
*/

#ifndef __ATT_CODE_H
#define __ATT_CODE_H

#define ATT_MTU_SIZE                     23  /**< Minimum ATT MTU size */
#define ATT_MAX_MTU_SIZE                 517 /**< Maximum ATT MTU size */

/**
* Attribute Opcode                                                         
*/
#define ATT_ERROR_RESPONSE               0x01 /**< ATT Error Response */
#define ATT_EXCHANGE_MTU_REQUEST         0x02 /**< ATT Exchange MTU Request */
#define ATT_EXCHANGE_MTU_RESPONSE        0x03 /**< ATT Exchange MTU Response */
#define ATT_FIND_INFO_REQUEST            0x04 /**< ATT Find Information Request */
#define ATT_FIND_INFO_RESPONSE           0x05 /**< ATT Find Information Response */
#define ATT_FIND_BY_TYPE_VALUE_REQUEST   0x06 /**< ATT Find By Type Value Request */
#define ATT_FIND_BY_TYPE_VALUE_RESPONSE  0x07 /**< ATT Find By Type Vaule Response */
#define ATT_READ_BY_TYPE_REQUEST         0x08 /**< ATT Read By Type Request */
#define ATT_READ_BY_TYPE_RESPONSE        0x09 /**< ATT Read By Type Response */
#define ATT_READ_REQUEST                 0x0A /**< ATT Read Request */
#define ATT_READ_RESPONSE                0x0B /**< ATT Read Response */
#define ATT_READ_BLOB_REQUEST            0x0C /**< ATT Read Blob Request */
#define ATT_READ_BLOB_RESPONSE           0x0D /**< ATT Read Blob Response */
#define ATT_READ_MULTI_REQUEST           0x0E /**< ATT Read Multiple Request */
#define ATT_READ_MULTI_RESPONSE          0x0F /**< ATT Read Multiple Response */
#define ATT_READ_BY_GROUP_TYPE_REQUEST   0x10 /**< ATT Read By Group Type Request */
#define ATT_READ_BY_GROUP_TYPE_RESPONSE  0x11 /**< ATT Read By Group Type Response */
#define ATT_WRITE_REQUEST                0x12 /**< ATT Write Request */
#define ATT_WRITE_RESPONSE               0x13 /**< ATT Write Response */
#define ATT_PREPARE_WRITE_REQUEST        0x16 /**< ATT Prepare Write Request */
#define ATT_PREPARE_WRITE_RESPONSE       0x17 /**< ATT Prepare Write Response */
#define ATT_EXECUTE_WRITE_REQUEST        0x18 /**< ATT Execute Write Request */
#define ATT_EXECUTE_WRITE_RESPONSE       0x19 /**< ATT Execute Write Response */
#define ATT_HANDLE_VALUE_NOTIFICATION    0x1B /**< ATT Handle Value Notification */
#define ATT_HANDLE_VALUE_INDICATION      0x1D /**< ATT Handle Value Indication */
#define ATT_HANDLE_VALUE_CONFIRMATION    0x1E /**< ATT Handle Value Confirmation */

#define ATT_WRITE_COMMAND                0x52 /**< ATT Write Command */
#define ATT_SIGNED_WRITE_COMMAND         0xD2 /**< ATT Signed Write Command */

#define ATT_OPCODE_MASK                  0x3f

#define ATT_COMMAND_FLAG_BIT             0x40 /**< Command Flag (bit 6) */
#define ATT_AUTHEN_SIG_FLAG_BIT          0x80 /**< Authentication Signature Flag (bit 7) */

#endif /**< __ATT_CODE_H */

