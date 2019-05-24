
configuration MpdrP {
  provides {
    interface StdControl;
    interface MpdrRouting;
    interface AMSend;
    interface Receive;
    interface Packet;
    interface MpdrStats;
  }
  uses {
    interface Pool<message_t>;
    interface Queue<message_t*>;
  }
}

implementation {
  components new MpdrForwardingEngineP() as Forwarder;
  StdControl = Forwarder;
  AMSend = Forwarder;
  Receive = Forwarder;
  Packet = Forwarder;
  MpdrStats = Forwarder;

  components new MpdrRoutingEngineP() as Router;
  StdControl = Router;
  MpdrRouting = Router;

  components RF231ActiveMessageC;
  components RF212ActiveMessageC;

  Forwarder.Routing -> Router;
  Forwarder.Radio1Send -> RF231ActiveMessageC.AMSend[22];
  Forwarder.Radio2Send -> RF212ActiveMessageC.AMSend[22];
  Forwarder.Radio1Receive -> RF231ActiveMessageC.Receive[22];
  Forwarder.Radio2Receive -> RF212ActiveMessageC.Receive[22];
  Forwarder.Radio1Packet -> RF231ActiveMessageC;
  Forwarder.Radio2Packet -> RF212ActiveMessageC;
  Forwarder.Radio1Ack -> RF231ActiveMessageC;
  Forwarder.Radio2Ack -> RF212ActiveMessageC;

  Forwarder.Pool = Pool;
  Forwarder.Queue = Queue;

  components LocalTimeMilliC;
  components new TimerMilliC() as RetryTimerC;

  Forwarder.LocalTime -> LocalTimeMilliC;
  Forwarder.RetryTimer -> RetryTimerC;

  Router.RoutingSend1 -> RF231ActiveMessageC.AMSend[23];
  Router.RoutingSend2 -> RF212ActiveMessageC.AMSend[23];
  Router.RoutingAck -> RF231ActiveMessageC;
  Router.RoutingReceive1 -> RF231ActiveMessageC.Receive[23];
  Router.RoutingReceive2 -> RF212ActiveMessageC.Receive[23];
  Router.RadioChannel1 -> RF231ActiveMessageC;
  Router.RadioChannel2 -> RF212ActiveMessageC;

  components SerialLoggerC;
  Router.SerialLogger -> SerialLoggerC;
  Forwarder.SerialLogger-> SerialLoggerC;
}
