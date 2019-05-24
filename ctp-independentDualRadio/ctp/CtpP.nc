/*
 * Copyright (c) 2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the University of California nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 * Copyright (c) 2006 Stanford University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the Stanford University nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL STANFORD
 * UNIVERSITY OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "Ctp.h"

/**
 * A data collection service that uses a tree routing protocol
 * to deliver data to collection roots, following TEP 119.
 *
 * @author Rodrigo Fonseca
 * @author Omprakash Gnawali
 * @author Kyle Jamieson
 * @author Philip Levis
 */


configuration CtpP {
  provides {
    interface StdControl;
    interface Send[uint8_t client];
    interface Receive[collection_id_t id];
    interface Receive as Snoop[collection_id_t];
    interface Intercept[collection_id_t id];

    interface Packet;
    interface CollectionPacket;
    interface CtpPacket;

    interface CtpInfo1;
    interface CtpInfo2;
    interface CtpInfoForward;
    //interface LinkEstimator;
    interface CtpCongestion;
    interface RootControl1;
    interface RootControl2; 
    interface UnicastNameFreeRouting1;
    interface UnicastNameFreeRouting2;
  }

  uses {
    interface CollectionId[uint8_t client];
    interface CollectionDebug;
  }
}

implementation {
  enum {
    CLIENT_COUNT = uniqueCount(UQ_CTP_CLIENT),
    FORWARD_COUNT = 12,
    TREE_ROUTING_TABLE_SIZE = 10,
    QUEUE_SIZE = CLIENT_COUNT + FORWARD_COUNT,
    CACHE_SIZE = 10,
  };

  //components ActiveMessageC;
  components new CtpForwardingEngineP() as Forwarder;
  components MainC, LedsC;
  
  Send = Forwarder;
  StdControl = Forwarder;
  Receive = Forwarder.Receive;
  Snoop = Forwarder.Snoop;
  Intercept = Forwarder;
  Packet = Forwarder;
  CollectionId = Forwarder;
  CollectionPacket = Forwarder;
  CtpPacket = Forwarder;
  CtpCongestion = Forwarder;
  
  components new PoolC(message_t, FORWARD_COUNT) as MessagePoolP;
  components new PoolC(fe_queue_entry_t, FORWARD_COUNT) as QEntryPoolP;
  Forwarder.QEntryPool -> QEntryPoolP;
  Forwarder.MessagePool -> MessagePoolP;

  components new QueueC(fe_queue_entry_t*, QUEUE_SIZE) as SendQueueP;
  Forwarder.SendQueue -> SendQueueP;

  components new LruCtpMsgCacheC(CACHE_SIZE) as SentCacheP;
  Forwarder.SentCache -> SentCacheP;

  components new TimerMilliC() as RoutingBeaconTimer1;
  components new TimerMilliC() as RouteUpdateTimer1;
  components new TimerMilliC() as RoutingBeaconTimer2;
  components new TimerMilliC() as RouteUpdateTimer2;

  components new LinkEstimatorP() as Estimator1;
  components new LinkEstimatorP() as Estimator2;

  Forwarder.LinkEstimator1 -> Estimator1;
  Forwarder.LinkEstimator2 -> Estimator2;
/*
  components new AMSenderC(AM_CTP_DATA);
  components new AMReceiverC(AM_CTP_DATA);
  components new AMSnooperC(AM_CTP_DATA);
*/
  components RF231ActiveMessageC;
  components RF212ActiveMessageC;

  components new CtpRoutingEngineP1(TREE_ROUTING_TABLE_SIZE, 128, 512000) as Router1;
  components new CtpRoutingEngineP2(TREE_ROUTING_TABLE_SIZE, 128, 512000) as Router2;

  StdControl = Router1;
  StdControl = Estimator1;
  StdControl = Router2;
  StdControl = Estimator2;
  RootControl1 = Router1;
  RootControl2 = Router2;
  UnicastNameFreeRouting1 = Router1;
  UnicastNameFreeRouting2 = Router2;
  
  MainC.SoftwareInit -> Router1;
  MainC.SoftwareInit -> Router2;
  Router1.BeaconSend -> Estimator1.Send;
  Router2.BeaconSend -> Estimator2.Send;
  Router1.BeaconReceive -> Estimator1.Receive;
  Router2.BeaconReceive -> Estimator2.Receive;
  Router1.LinkEstimator -> Estimator1.LinkEstimator;
  Router2.LinkEstimator -> Estimator2.LinkEstimator;

  Router1.CompareBit -> Estimator1.CompareBit;
  Router2.CompareBit -> Estimator2.CompareBit;

  Router1.AMPacket -> RF231ActiveMessageC;
  Router2.AMPacket -> RF212ActiveMessageC;
  Router1.RadioControl -> RF231ActiveMessageC;
  Router2.RadioControl -> RF212ActiveMessageC; 
  Router1.BeaconTimer -> RoutingBeaconTimer1;
  Router2.BeaconTimer -> RoutingBeaconTimer2;
  Router1.RouteTimer -> RouteUpdateTimer1;
  Router2.RouteTimer -> RouteUpdateTimer2;
  Router1.CollectionDebug = CollectionDebug;
  Router2.CollectionDebug = CollectionDebug;
  Forwarder.CollectionDebug = CollectionDebug;
  Forwarder.CtpInfo1 -> Router1;
  Forwarder.CtpInfo2 -> Router2;
  Router1.CtpCongestion -> Forwarder;
  Router2.CtpCongestion -> Forwarder;
  CtpInfo1 = Router1;
  CtpInfo2 = Router2;
  CtpInfoForward = Forwarder;
  
  components SerialLoggerC;
  Router1.SerialLogger -> SerialLoggerC;
  Router2.SerialLogger -> SerialLoggerC;
  Forwarder.SerialLogger -> SerialLoggerC;

  
  components new TimerMilliC() as RetxmitTimer1;
  components new TimerMilliC() as RetxmitTimer2;
  Forwarder.RetxmitTimer1 -> RetxmitTimer1;
  Forwarder.RetxmitTimer2 -> RetxmitTimer2;

  components RandomC;
  Router1.Random -> RandomC;
  Router2.Random -> RandomC;
  Forwarder.Random -> RandomC;

 MainC.SoftwareInit -> Forwarder;
  Forwarder.SubSend1 -> RF231ActiveMessageC.AMSend[AM_CTP_DATA];
  Forwarder.SubSend2 -> RF212ActiveMessageC.AMSend[AM_CTP_DATA];
  Forwarder.SubReceive1 -> RF231ActiveMessageC.Receive[AM_CTP_DATA];
  Forwarder.SubReceive2 -> RF212ActiveMessageC.Receive[AM_CTP_DATA];
  Forwarder.SubSnoop1 -> RF231ActiveMessageC.Snoop[AM_CTP_DATA];
  Forwarder.SubSnoop2 -> RF212ActiveMessageC.Snoop[AM_CTP_DATA];
  Forwarder.SubPacket1 -> RF231ActiveMessageC;
  Forwarder.SubPacket2 -> RF212ActiveMessageC;
  Forwarder.RootControl1 -> Router1;
  Forwarder.RootControl2 -> Router2;
  Forwarder.UnicastNameFreeRouting1 -> Router1.Routing;
  Forwarder.UnicastNameFreeRouting2 -> Router2.Routing;

  components DualRadioControlC;
  DualRadioControlC.Radio1Control -> RF231ActiveMessageC;
  DualRadioControlC.Radio2Control -> RF212ActiveMessageC;
  Forwarder.RadiosControl -> DualRadioControlC;
  
  Forwarder.Radio1Ack -> RF231ActiveMessageC;
  Forwarder.Radio2Ack -> RF212ActiveMessageC;
  Forwarder.AMPacket1 -> RF231ActiveMessageC;
  Forwarder.AMPacket2 -> RF212ActiveMessageC;
  Forwarder.Leds -> LedsC;
  
  //components new AMSenderC(AM_CTP_ROUTING) as SendControl;
  //components new AMReceiverC(AM_CTP_ROUTING) as ReceiveControl;

  //LinkEstimator = Estimator1;
  
  Estimator1.Random -> RandomC;
  Estimator2.Random -> RandomC;

  Estimator1.AMSend -> RF231ActiveMessageC.AMSend[AM_CTP_ROUTING];
  Estimator2.AMSend -> RF212ActiveMessageC.AMSend[AM_CTP_ROUTING];
  Estimator1.SubReceive -> RF231ActiveMessageC.Receive[AM_CTP_ROUTING];
  Estimator2.SubReceive -> RF212ActiveMessageC.Receive[AM_CTP_ROUTING];
  Estimator1.SubPacket -> RF231ActiveMessageC;
  Estimator2.SubPacket -> RF212ActiveMessageC;
  Estimator1.SubAMPacket -> RF231ActiveMessageC;
  Estimator2.SubAMPacket -> RF212ActiveMessageC;

#if defined(CC2420X)
  components CC2420XActiveMessageC as PlatformActiveMessageC;
#elif defined(PLATFORM_TELOSB) || defined(PLATFORM_MICAZ)
#ifndef TOSSIM
  components CC2420ActiveMessageC as PlatformActiveMessageC;
#else
  components DummyActiveMessageP as PlatformActiveMessageC;
#endif
#elif defined (PLATFORM_MICA2) || defined (PLATFORM_MICA2DOT)
  components CC1000ActiveMessageC as PlatformActiveMessageC;
#elif defined(PLATFORM_EYESIFXV1) || defined(PLATFORM_EYESIFXV2)
  components WhiteBitAccessorC as PlatformActiveMessageC;
#elif defined(PLATFORM_IRIS) || defined(PLATFORM_MESHBEAN)
  components RF230ActiveMessageC as PlatformActiveMessageC;
#elif defined(PLATFORM_MESHBEAN900)
  components RF212ActiveMessageC as PlatformActiveMessageC;
#elif defined(PLATFORM_UCMINI)
  components RFA1ActiveMessageC as PlatformActiveMessageC;
#else
  components DummyActiveMessageP as PlatformActiveMessageC;
#endif

  Estimator1.LinkPacketMetadata -> RF231ActiveMessageC;
  Estimator2.LinkPacketMetadata -> RF212ActiveMessageC;

  // eventually
  //  Estimator.LinkPacketMetadata -> ActiveMessageC;

  MainC.SoftwareInit -> Estimator1;
  MainC.SoftwareInit -> Estimator2;
}
