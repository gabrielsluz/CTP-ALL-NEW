#include "Mpdr.h"

configuration MpdrC {
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
  components MpdrP;

  MpdrP.Pool = Pool;
  MpdrP.Queue = Queue;
  
  StdControl = MpdrP;
  MpdrRouting = MpdrP;
  AMSend = MpdrP;
  Receive = MpdrP;
  Packet = MpdrP;
  MpdrStats = MpdrP;
}
