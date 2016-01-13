enum { __FILE_NUM__ = 0 };


#include "test_mesh_cmd.h"
#include <blueapi.h>
#include "trace.h"
#include <string.h>

#include "mesh_common.h"
#include "mesh_iop.h"

/**
    4	Smart Mesh Core Specification IOP Test Cases
*/

/**
4.1	IOP-01 Generate IVzero from a NetworkKey
Take a Network key and a Network key diversifier and generate the corresponding IVzero.
"	Precondition:
A device has a Network key and a Network key diversifier.
"	Test Procedure:
The test partner generates IVzero from this Network key.
"	Expected Outcome:
The IVzero is the same as the test partner.

IVzero = AES-CMAC (AES-CMAC (NetKey, "smit"), "smiz")
*/
void mesh_core_spec_iop_01(uint8_t* NetKey, uint8_t len)
{
   MeshNetKeySet(NetKey, len);
}

/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
NetworkIV = AES-CMAC (AES-CMAC (NetKey, IVzero || IVindex), "smiv")
*/
void mesh_core_spec_iop_02()
{
    
}

/**
4.3	IOP-03 Transmit Mesh Packets with Max Payload
Transmit a mesh packet in a non-connectable advertising packet (AD_Mesh).
Note: This is the opposite test to Test 4.4.
"	Precondition:
The IUT is a broadcaster and Tester is an observer.
"	Test Procedure:
The IUT transmits an advertising packet that contains a "Mesh Packet" - using the Advertising Tag "AD_Mesh". (See section 0)
The Tester observes advertising packets - optionally filtered on the AdvAddr of the IUT.
"	Expected Outcome:
The Tester receives one or more advertising packets from the IUT with the AD_Mesh data type.
*/

void mesh_core_spec_iop_03()
{
    
}


/**
4.4	IOP-04 Receive Mesh Packets with Max Payload
Receive a mesh packet in a non-connectable advertising packet.
Note: This is the opposite test to Test 4.3.
"	Precondition:
The IUT is an observer and Tester is a broadcaster.
"	Test Procedure:
The Tester transmits an advertising packet that contains a "Mesh Packet" - using the Advertising Tag "AD_Mesh". (See section 0)
The IUT observes advertising packets from the Tester - optionally filtered on the Testers AdvAddr.
"	Expected Outcome:
The IUT receives one or more advertising packets from the Tester with the AD_Mesh data type.

*/

void mesh_core_spec_iop_04()
{
    
}

/**
4.5	IOP-05 Transmit Network Authenticated Packets
Transmit a mesh packet authenticated with a known NetworkKey.
Note: This is the opposite test to Test 4.6.
"	Precondition:
The IUT is a broadcaster and Tester is an observer.
The IUT and Tester both know the same NetworkKey. 
"	Test Procedure:
The IUT transmits an advertising packet that contains a "Mesh Packet" that is authenticated with the NetworkKey.
The Tester observes advertising packets - optionally filtered on the AdvAddr of the IUT.
"	Expected Outcome:
The Tester receives the advertising packet that is correctly authenticated with the shared NetworkKey and has the correct NetworkID for that NetworkKey.

*/

void mesh_core_spec_iop_05()
{
    
}

/**
4.6	IOP-06 Receive Network Authenticated Packets
Receive a mesh packet authenticated with a known NetworkKey
Note: This is the opposite test to Test 4.5.
"	Precondition:
The Tester is a broadcaster and IUT is an observer.
The IUT and Tester both know the same NetworkKey.
"	Test Procedure:
The Tester transmits an advertising packet that contains a "Mesh Packet" that is authenticated with the NetworkKey.
The IUT observes advertising packets - optionally filtered on the AdvAddr of the Tester.
"	Expected Outcome:
The IUT receives the advertising packet that is correctly authenticated with the shared NetworkKey and has the correct NetworkID for that NetworkKey.

*/

void mesh_core_spec_iop_06()
{
    
}


/**
4.7	IOP-07 Transmit Application Authenticated Packets
Transmit a mesh packet authenticated with a known ApplicationKey and NetworkKey.
Note: This is the opposite test to Test 4.8.
"	Precondition:
The IUT is a broadcaster and Tester is an observer.
The IUT and Tester both know the same ApplicationKey and NetworkKey. 
"	Test Procedure:
The IUT transmits an advertising packet that contains a "Mesh Packet" that is authenticated with the ApplicationKey and NetworkKey.
The Tester observes advertising packets - optionally filtered on the AdvAddr of the IUT.
"	Expected Outcome:
The Tester receives the advertising packet that is correctly authenticated with the shared ApplicationKey and has the correct NetworkID for that 
ApplicationKey.

*/

void mesh_core_spec_iop_07()
{
    
}


/**
4.8	IOP-08 Receive Application Authenticated Packets
Receive a mesh packet authenticated with a known ApplicationKey and NetworkKey.
Note: This is the opposite test to Test 4.7
"	Precondition:
The Tester is a broadcaster and IUT is an observer.
The IUT and Tester both know the same ApplicationKey and NetworkKey.
"	Test Procedure:
The Tester transmits an advertising packet that contains a "Mesh Packet" that is authenticated with the ApplicationKey.
The IUT observes advertising packets - optionally filtered on the AdvAddr of the Tester.
"	Expected Outcome:
The IUT receives the advertising packet that is correctly authenticated with the shared ApplicationKey and has the correct NetworkID for that ApplicationKey.

*/

void mesh_core_spec_iop_08()
{
    
}


/**
4.9	IOP-09 Relay Mesh Packets received with TTL>1
Receive a mesh packet and relay that packet if appropriate, confirm that packet has been relayed.
"	Precondition:
The Tester is a broadcaster and an observer.
The IUT is a broadcaster and an observer.
"	Test Procedure:
The Tester transmits a mesh packet with a TTL of 3 and DST not matching the Device Address of the IUT.
"	Expected Outcome:
The Tester receives a relayed copy of the mesh packet with a TTL of 2 from the IUT.

*/

void mesh_core_spec_iop_09()
{
    
}


/**
4.10	IOP-10 Not Relaying TTL=1 Mesh Packets
Receive a mesh packet and relay that packet if appropriate, confirm that packet has been relayed.
Note: This test is similar to Test 4.9 except that the TTL has a different value and the message should not be relayed.
"	Precondition:
The Tester is a broadcaster and an observer.
The IUT is a broadcaster and an observer.
"	Test Procedure:
The Tester transmits a mesh packet with a TTL of 1 and DST not matching the Device Address of the IUT.
"	Expected Outcome:
The Tester does not receive a relayed copy of the mesh packet from the IUT.

*/

void mesh_core_spec_iop_10()
{
    
}


/**
4.11	IOP-11 Not Relaying TTL=0 Mesh Packets
Receive a mesh packet and relay that packet if appropriate, confirm that packet has been relayed.
Note: This test is similar to Test 4.9 except that the TTL has a different value and the message should not be relayed.
"	Precondition:
The Tester is a broadcaster and an observer.
The IUT is a broadcaster and an observer.
"	Test Procedure:
The Tester transmits a mesh packet with a TTL of 0 and DST not matching the Device Address of the IUT.
"	Expected Outcome:
The Tester does not receive a relayed copy of the mesh packet from the IUT.

*/

void mesh_core_spec_iop_11()
{
    
}


/**
4.12	IOP-12 Not Relaying Packets with Target Device Address
Receive a mesh packet addressed to the local application and confirm that the packet has not been relayed
"	Precondition:
The Tester is a broadcaster and an observer.
The IUT is a broadcaster and an observer.
"	Test Procedure:
The Tester transmits a mesh packet with a TTL of 2 and DST matching the Device Address of the IUT.
"	Expected Outcome:
The Tester does not receive a relayed copy of the mesh packet from the IUT.

*/

void mesh_core_spec_iop_12()
{
    
}


/**
4.13	IOP-13 Confirm Mesh Message Cache Functionality
Receive a mesh packet and relay that packet if appropriate and confirm that packet has been relayed.
"	Precondition:
The Tester is a broadcaster and an observer.
The IUT is a broadcaster and an observer.
"	Test Procedure:
The Tester transmits a mesh packet with a TTL of 3 and DST not matching Device Address of the IUT.
At a reasonable time later (suggestion 3 seconds), the Tester transmits the same mesh packet with a TTL of 2.
"	Expected Outcome:
The Tester receives a relayed copy of the mesh packet with a TTL of 2 from the IUT and the Tester does not receive a relayed copy of the mesh packet with a 
TTL of 1.

*/

void mesh_core_spec_iop_13()
{
    
}


/**
4.14	IOP-14 Check Invalid ApplicationKey

"	Precondition:
The Tester is a Broadcaster. The IUT is an Observer.
"	Test Procedure:
The Tester sends a valid packet to the IUT with a packet authenticated with a valid ApplicationKey and with a valid NetworkKey.
The Tester then sends a valid packet to the IUT with the packet authenticated with an invalid ApplicationKey but with a valid NetworkKey.
"	Expected Outcome:
The IUT receives correctly the valid packet, but does not receive the invalid packet and 
*/

void mesh_core_spec_iop_14()
{
    
}


/**
4.15	IOP-15 Check Invalid NetworkKey

"	Precondition:
The Tester is a Broadcaster. The IUT is an Observer.
"	Test Procedure:
The Tester sends a valid packet to the IUT with a packet authenticated with a valid ApplicationKey and with a valid NetworkKey.
The Tester then sends a valid packet to the IUT with the packet authenticated with an invalid NetworkKey.
"	Expected Outcome:
The IUT receives correctly the valid packet, but does not receive the invalid packet.

*/

void mesh_core_spec_iop_15()
{
    
}


/**
4.16	IOP-16 Check Relay without knowledge of Application Key (good packet)
Receive a mesh packet authenticated with an unknown ApplicationKey and known NetworkKey, relay that packet, and confirm that packet has been relayed.
"	Precondition:
The Tester is a Broadcaster. The IUT is an Observer.
"	Test Procedure:
The Tester sends a mesh packet with a TTL of 3 and DST not matching Device Address of IUT. Packet is authenticated with a valid ApplicationKey and with a 
valid NetworkKey.
"	Expected Outcome:
The Tester receives a relayed copy of the mesh packet with a TTL of 2 from the IUT and confirms that packet is authenticated with a valid ApplicationKey.

*/

void mesh_core_spec_iop_16()
{
    
}


/**
4.17	IOP-17 Check Relay without knowledge of Application Key (bad packet)
Receive a mesh packet authenticated with an unknown ApplicationKey and known NetworkKey, relay that packet, and confirm that packet has been relayed.
"	Precondition:
The Tester is a Broadcaster. The IUT is an Observer.
"	Test Procedure:
The Tester sends a mesh packet with a TTL of 3 and DST not matching Device Address of IUT. Packet is authenticated with an invalid ApplicationKey and with a 
valid NetworkKey.
"	Expected Outcome:
The Tester receives a relayed copy of the mesh packet with a TTL of 2 from the IUT and confirms that packet is authenticated with an invalid ApplicationKey.

*/

void mesh_core_spec_iop_17()
{
    
}


/**
4.18	IOP-18 Check Relay with knowledge of Application Key (good packet)
Receive a mesh packet authenticated with known ApplicationKey and known NetworkKey, relay that packet, and confirm that packet has been relayed.
"	Precondition:
The Tester is a Broadcaster. The IUT is an Observer.
"	Test Procedure:
The Tester sends a mesh packet with a TTL of 3 and DST not matching Device Address of IUT. Packet is authenticated with a valid ApplicationKey and with a 
valid NetworkKey.
"	Expected Outcome:
The Tester receives a relayed copy of the mesh packet with a TTL of 2 from the IUT and confirms that packet is authenticated with a valid ApplicationKey.

*/

void mesh_core_spec_iop_18()
{
    
}


/**
4.19	IOP-19 Check Relay with knowledge of Application Key (bad packet)
Receive a mesh packet authenticated with known ApplicationKey and known NetworkKey, relay that packet, and confirm that packet has been relayed.
"	Precondition:
The Tester is a Broadcaster. The IUT is an Observer.
"	Test Procedure:
The Tester sends a mesh packet with a TTL of 3 and DST not matching Device Address of IUT. Packet is authenticated with a valid ApplicationKey and with a 
valid NetworkKey.
"	Expected Outcome:
The Tester receives a relayed copy of the mesh packet with a TTL of 2 from the IUT and confirms that packet is authenticated with an invalid ApplicationKey.

*/

void mesh_core_spec_iop_19()
{
    
}


/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_20()
{
    
}


/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_21()
{
    
}


/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_22()
{
    
}


/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_23()
{
    
}


/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_24()
{
    
}


/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_25()
{
    
}

/**
4.2	IOP-02 Generate NetworkIV
Take a NetworkKey, IVzero and IVindex and generate the corresponding NetworkIV.
"	Precondition:
A device has a NetworkKey, an IVzero (e.g. after Test 4.1) and no IVindex.
"	Test Procedure:
The IVindex is given to the test partner, and the test partner generates NetworkIV from the NetworkKey, IVzero and IVindex.
"	Expected Outcome:
The NetworkIV is the same as the test partner.
*/

void mesh_core_spec_iop_26()
{
    
}


/**
5	Smart Mesh Supported Profile IOP Test Cases
*/

/**
5.1	IOP-Supported01 Discovering Mesh Profiles for Supported Server
Discover the set of mesh profiles that a device supports
"	Precondition:
IUT is a Smart Mesh Server. The Tester is a Smart Mesh Client.
"	Test Procedure:
The Tester sends a Supported Get Profiles message with the Page set to 15. The Tester will expect to receive the Supported Profiles message. The Tester will 
store the Profiles field and if the Page is greater than 0 will send another Supported Get Profiles message with the Page set to one less than the received 
Page number and await another reply.
"	Expected Outcome:
The Tester collects all the Profiles and it is the same value that is declared by the IUT.
*/
void mesh_iop_supported01()
{
    
}

/**
5.2	IOP-Supported02 Discovering Mesh Profiles for Supported Client

"	Precondition:
IUT is a Smart Mesh Client. The Tester is a Smart Mesh Server.
"	Test Procedure:
The IUT shall send a Supported Get Profiles message with the Page set to 15 to the Tester. The Tester will respond with the Supported Profiles message. The 
IUT will store the Profiles field and if the Page is greater than 0 will send another Supported Get Profiles message with the Page set to one less than the 
received Page number and await another reply.
"	Expected Outcome:
The IUT collects all the Profiles and it is the same value that is expected by the Tester.

*/
void mesh_iop_supported02()
{
    
}


/**
6	Smart Mesh Power Profile IOP Test Cases
*/
/**
6.1	IOP-Power01 Process "Power Set State without Ack" for Power Server

"	Precondition:
The IUT supports the Power Server, and the Tester supports the Power Client.
"	Test Procedure:
The Tester sends a Power Set State without Ack to the IUT with the State field set to either On or Off.
"	Expected Outcome:
The IUT reports that the power has changed to a new state that is the same as the value of the state field sent by the Tester.
*/
void mesh_iop_power01()
{
    
}

/**
6.2	IOP-Power02 Send "Power Set State without Ack" for Power Client

"	Precondition:
The IUT supports the Power Client, and the Tester supports the Power State.
"	Test Procedure:
The IUT sends a Power Set State without Ack message to the Tester with the State field set to either On or Off as determined by the Upper Tester.
"	Expected Outcome:
The Tester receives a Power Set State without Ack message with the State field set to the value requested by the Upper Tester.
*/
void mesh_iop_power02()
{
    
}


/**
6.3	IOP-Power03 Send a "Power Set State with Ack" message for Power Server

"	Precondition:
The IUT supports the Power Server, and the Tester supports the Power Client.
"	Test Procedure:
The Tester sends a Power Set State with Ack to the IUT with the State field set to either On or Off.
"	Expected Outcome:
The IUT reports that the power has changed to a new state and the Tester receives a Power State message with the same TID as the Power Set State with Ack and 
the same State value of the state field sent by the Tester.
*/
void mesh_iop_power03()
{
    
}

/**
6.4	IOP-Power04 Send a "Power Set State with Ack" message for Power Client

"	Precondition:
The IUT supports the Power Client, and the Tester supports the Power State.
"	Test Procedure:
The IUT sends a Power Set State with Ack message to the Tester with the State field set to either On or Off as determined by the Upper Tester.
The Tester can ignore one or more Power Set State with Ack messages to test reliable message delivery
"	Expected Outcome:
The Tester receives a Power Set State with Ack message with the State field set to the value requested by the Upper Tester and the IUT reports receiving the 
Power State message.
*/

void mesh_iop_power04()
{
    
}

/**
6.5	IOP-Power05 Send "Power Get State" message for Power Server

"	Precondition:
The IUT supports the Power Server, and the Tester supports the Power Client.
"	Test Procedure:
The Tester sends a Power Get State to the IUT. The Tester can ignore one or more Power State messages sent by the IUT to trigger "reliable message delivery".
"	Expected Outcome:
The Tester receives a Power State message with the same TID as that used in the Power Get State message.
*/

void mesh_iop_power05()
{
    
}

/**
6.6	IOP-Power06 Send a "Power Get State" message for Power Client

"	Precondition:
The IUT supports the Power Client, and the Tester supports the Power State.
"	Test Procedure:
The IUT sends a Power Get State to the Tester. The Tester replies with a Power State message. The Tester can ignore one or more Power Get State messages sent 
by the IUT to test reliable message delivery.
"	Expected Outcome:
The IUT reports receiving a Power State message with the State field set to the value sent by the Tester in the Power State message.
*/

void mesh_iop_power06()
{
    
}

/**
6.7	IOP-Power07 Confirm Replay Attack Prevention (Unreliable) for Power Server

"	Precondition:
IUT is a Power Server. Tester is a Power Client.
"	Test Procedure:
The Tester sends a "Power Set State (On) without Ack" message to the IUT. The IUT should "turn on".
The Tester sends a "Power Set State (Off) without Ack" message to the IUT. The IUT should "turn off".
The Tester sends the first "Power Set State (On) without Ack" message to a device, using the same SEQ (sequence number) that was used in the first message. 
The IUT should ignore the message.
"	Expected Outcome:
The IUT shall end in the "off" state.
*/

void mesh_iop_power07()
{
    
}

/**
6.8	IOP-Power08 Confirm Replay Attack Prevention (Unreliable) for Power Client

"	Precondition:
IUT is a Power Client. Tester is a Power Server.
"	Test Procedure:
The Tester sends a "Power Set State (On) without Ack" message to the IUT. The IUT should "turn on".
The Tester sends a "Power Set State (Off) without Ack" message to the IUT. The IUT should "turn off".
The Tester sends the first "Power Set State (On) without Ack" message to a device, using the same SEQ (sequence number) that was used in the first message. 
The IUT should ignore the message.
"	Expected Outcome:
The IUT shall end in the "off" state.
*/

void mesh_iop_power08()
{
    
}

/**
6.9	IOP-Power09 Confirm Replay Attack prevention (Reliable) for Power Server

"	Precondition:
IUT is a Power Server. Tester is a Power Client.
"	Test Procedure:
The Tester sends a "Power Set State (On) with Ack" message to the IUT. The IUT should "turn on" and reply with a Power State message.
The Tester sends a "Power Set State (Off) with Ack" message to the IUT. The IUT should "turn off" and reply with a Power State message.
The Tester sends the first "Power Set State (On) with Ack" message to a device, using the same SEQ (sequence number) that was used in the first message. 
"	Expected Outcome:
The IUT shall reply with two Power State messages and shall end in the "off" state.
*/

void mesh_iop_power09()
{
    
}

/**
6.10	IOP-Power10 Confirm Replay Attack prevention (Reliable) for Power Client

"	Precondition:
IUT is a Power Client. Tester is a Power Server.
"	Test Procedure:
The Tester sends a "Power Set State (On) with Ack" message to the IUT. The IUT should "turn on" and reply with a Power State message.
The Tester sends a "Power Set State (Off) with Ack" message to the IUT. The IUT should "turn off" and reply with a Power State message.
The Tester sends the first "Power Set State (On) with Ack" message to a device, using the same SEQ (sequence number) that was used in the first message. 
"	Expected Outcome:
The IUT shall reply with two Power State messages and shall end in the "off" state.
*/

void mesh_iop_power10()
{
    
}




