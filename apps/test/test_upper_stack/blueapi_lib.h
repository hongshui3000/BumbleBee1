
#ifndef __BLUEAPI_LIB_H
#define __BLUEAPI_LIB_H

#ifndef __BLUEAPI_TYPES_H
#include <blueapi_types.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* various enum => string functions for HDP Stack/Apps */

const char * blueAPI_StringizeBd(uint8_t * bd);

const char * blueAPI_CauseString(TBlueAPI_Cause cause);

const char * blueAPI_CommandString(TBlueAPI_Command Command);


const char * blueAPI_MCLStatusString(TBlueAPI_MCLStatus status);

const char * blueAPI_ACLStatusString(TBlueAPI_ACLStatus status);

const char * blueAPI_InternalEventTypeString(TBlueAPI_InternalEventType eventType);

const char * blueAPI_AuthRequirementsString(TBlueAPI_AuthRequirements authReq);

const char * blueAPI_IOCapabilitiesString(TBlueAPI_IOCapabilities ioCaps);

#ifdef __cplusplus
}
#endif

#endif  /* __BLUEAPI_LIB_H */
