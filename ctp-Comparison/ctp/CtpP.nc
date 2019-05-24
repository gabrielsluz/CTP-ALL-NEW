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

    interface CtpInfo;
    interface CtpInfoForward;
    //interface LinkEstimator;
    interface CtpCongestion;
    interface RootControl; 
    interface UnicastNameFreeRouting;
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

  //Serial Logger
  components SerialLoggerC;
  Forwarder.SerialLogger -> SerialLoggerC;

  components new TimerMilliC() as RoutingBeaconTimer;
  components new TimerMilliC() as RouteUpdateTimer;
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

  components new CtpRoutingEngineP(TREE_ROUTING_TABLE_SIZE, 128, 512000) as Router;

  StdControl = Router;
  StdControl = Estimator1;
  StdControl = Estimator2;
  RootControl = Router;
  UnicastNameFreeRouting = Router;
  
  MainC.SoftwareInit -> Router;
  Router.BeaconSend1 -> Estimator1.Send;
  Router.BeaconSend2 -> Estimator2.Send;
  Router.BeaconReceive1 -> Estimator1.Receive;
  Router.BeaconReceive2 -> Estimator2.Receive;
  Router.LinkEstimator1 -> Estimator1.LinkEstimator;
  Router.LinkEstimator2 -> Estimator2.LinkEstimator;

  Router.CompareBit1 -> Estimator1.CompareBit;
  Router.CompareBit2 -> Estimator2.CompareBit;

  //Router.AMPacket -> ActiveMessageC;
  //Router.RadioControl -> ActiveMessageC;
  Router.AMPacket1 -> RF231ActiveMessageC;
  Router.AMPacket2 -> RF212ActiveMessageC;
  //Router.RadioControl -> RF231ActiveMessageC;
  //Router.RadioControl2 -> RF212ActiveMessageC;
  Router.BeaconTimer -> RoutingBeaconTimer;
  Router.RouteTimer -> RouteUpdateTimer;
  Router.CollectionDebug = CollectionDebug;
  Forwarder.CollectionDebug = CollectionDebug;
  Forwarder.CtpInfo -> Router;
  Router.CtpCongestion -> Forwarder;
  CtpInfo = Router;
  CtpInfoForward = Forwarder;

  Router.SerialLogger -> SerialLoggerC;
  
  components new TimerMilliC() as RetxmitTimer1;
  components new TimerMilliC() as RetxmitTimer2;
  Forwarder.RetxmitTimer1 -> RetxmitTimer1;
  Forwarder.RetxmitTimer2 -> RetxmitTimer2;

  components RandomC;
  Router.Random -> RandomC;
  Forwarder.Random -> RandomC;

  MainC.SoftwareInit -> Forwarder;
  //Forwarder.SubSend1 -> AMSenderC;
  Forwarder.SubSend1 -> RF231ActiveMessageC.AMSend[AM_CTP_DATA];
  Forwarder.SubSend2 -> RF212ActiveMessageC.AMSend[AM_CTP_DATA];
  //Forwarder.SubReceive -> AMReceiverC;
  Forwarder.SubReceive1 -> RF231ActiveMessageC.Receive[AM_CTP_DATA];
  Forwarder.SubReceive2 -> RF212ActiveMessageC.Receive[AM_CTP_DATA];
  //Forwarder.SubSnoop -> AMSnooperC;
  Forwarder.SubSnoop1 -> RF231ActiveMessageC.Snoop[AM_CTP_DATA];
  Forwarder.SubSnoop2 -> RF212ActiveMessageC.Snoop[AM_CTP_DATA];
  Forwarder.SubPacket1 -> RF231ActiveMessageC;
  Forwarder.SubPacket2 -> RF212ActiveMessageC;
  Forwarder.RootControl -> Router;
  Forwarder.UnicastNameFreeRouting -> Router.Routing;

  components DualRadioControlC;
  DualRadioControlC.Radio1Control -> RF231ActiveMessageC;
  DualRadioControlC.Radio2Control -> RF212ActiveMessageC;
  Forwarder.RadiosControl -> DualRadioControlC;
  Router.RadiosControl -> DualRadioControlC;
  
  Forwarder.Radio1Ack -> RF231ActiveMessageC;
  Forwarder.Radio2Ack -> RF212ActiveMessageC;
  Forwarder.AMPacket1 -> RF231ActiveMessageC;
  Forwarder.AMPacket2 -> RF212ActiveMessageC;
  Forwarder.Leds -> LedsC;
  
  //components new AMSenderC(AM_CTP_ROUTING) as SendControl;
  //components new AMReceiverC(AM_CTP_ROUTING) as ReceiveControl;

 // LinkEstimator = Estimator1;
  
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



  Estimator1.LinkPacketMetadata -> RF231ActiveMessageC;
  Estimator2.LinkPacketMetadata -> RF212ActiveMessageC;

  // eventually
  //  Estimator1.LinkPacketMetadata -> ActiveMessageC;

  MainC.SoftwareInit -> Estimator2;
  MainC.SoftwareInit -> Estimator1;
}
