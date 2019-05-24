#include "SerialLogger.h"

configuration SerialLoggerC {
  provides {
    interface SplitControl;
    interface SerialLogger;
  }
}

implementation {
  components SerialLoggerP;
  components SerialActiveMessageC;
  components new SerialAMSenderC(0xFF) as SerialSenderC;
  components new PoolC(message_t, 100) as SerialPoolC;
  components new QueueC(message_t*, 100) as SerialQueueC;

  SerialLoggerP.SerialControl -> SerialActiveMessageC;
  SerialLoggerP.SerialSend -> SerialSenderC;
  SerialLoggerP.Pool -> SerialPoolC;
  SerialLoggerP.Queue -> SerialQueueC;

  SplitControl = SerialLoggerP;
  SerialLogger = SerialLoggerP;
}
