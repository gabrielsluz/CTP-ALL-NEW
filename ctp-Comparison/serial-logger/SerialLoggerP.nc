#include "SerialLogger.h"
#include "Timer.h"

module SerialLoggerP {
  provides {
    interface SplitControl;
    interface SerialLogger;
  }
  uses {
    interface SplitControl as SerialControl;
    interface AMSend as SerialSend;
    interface Pool<message_t>;
    interface Queue<message_t*>;
  }
}

implementation {

  bool serialBusy = FALSE;

  task void sendTask() {
    if (serialBusy) {
      return;
    } else if (call Queue.empty()) {
      return;
    } else {
      message_t* smsg = call Queue.dequeue();
      error_t result = call SerialSend.send(AM_BROADCAST_ADDR, smsg,
                                            sizeof(serial_log_message_t));
      if (result == SUCCESS) {
        serialBusy = TRUE;
      } else {
        call Pool.put(smsg);
        if (!call Queue.empty()) {
          post sendTask();
        }
      }
    }
  }

  command error_t SplitControl.start() {
    return call SerialControl.start();
  }

  event void SerialControl.startDone(error_t error) {
    signal SplitControl.startDone(error);
  }

  command error_t SplitControl.stop() {
    return call SerialControl.stop();
  }

  event void SerialControl.stopDone(error_t error) {
    signal SplitControl.stopDone(error);
  }

  command void SerialLogger.log(uint16_t evt, uint16_t data) {
    if (call Pool.empty()) {
      return;
    } else {
      message_t* msg = call Pool.get();
      serial_log_message_t* smsg = (serial_log_message_t*)
                  call SerialSend.getPayload(msg, sizeof(serial_log_message_t));
      if (smsg == NULL) {
        return;
      }
      smsg->evt = evt;
      smsg->data = data;
      if (call Queue.enqueue(msg) == SUCCESS) {
        post sendTask();
      } else {
        call Pool.put(msg);
      }
    }

  }

  event void SerialSend.sendDone(message_t * msg, error_t error) {
    call Pool.put(msg);
    serialBusy = FALSE;
    if (!call Queue.empty()) {
      post sendTask();
    }
  }

}
