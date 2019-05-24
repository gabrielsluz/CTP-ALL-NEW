#define INIT_TIME 500
#define FINISH_TIME 2500000

#define NUM_MSGS 2000
#define SEND_PERIOD 2000
#define SEND_DELAY 5000

module TestCtpC {
  uses{
    interface Boot;

    interface Timer<TMilli> as InitTimer;
    interface Timer<TMilli> as FinishTimer;
    interface Timer<TMilli> as SendTimer;

    interface SplitControl as SerialControl;
    interface SplitControl as RadiosControl;
    interface StdControl as CtpRouteControl;

		interface SerialLogger;


    interface RootControl;
    interface CtpInfo;
    interface CollectionPacket;
    interface CtpInfoForward;
    interface Receive;
    interface Intercept;
    interface Send;

    interface Cache<message_t*> as ReceivedCache;
  }

}


implementation {

	bool is_root;
  bool transmitting = FALSE;
  uint16_t sendCount = 0;
  uint32_t receivedCount = 0;
  message_t msgBuffer;
  uint32_t startTime = 0;
  uint32_t endTime = 0;
  uint16_t duplicate = 0;
  uint16_t cycles = 0;


  void initializeNode() {

    if (TOS_NODE_ID == 40 )  {
      call RootControl.setRoot();
      call SerialLogger.log(LOG_ROOT, TOS_NODE_ID);
      call FinishTimer.startOneShot(FINISH_TIME);

    } else{
      transmitting = TRUE;
      call SerialLogger.log(LOG_INITIALIZED, TOS_NODE_ID);
      call SendTimer.startPeriodicAt(SEND_DELAY, SEND_PERIOD);
      call FinishTimer.startOneShot(FINISH_TIME);
    }
  }


  void SendMessage(){
    message_t * msg;
    DataMsg * payload;
    error_t result;
    uint8_t i;
     if (NUM_MSGS > 0 && sendCount >= NUM_MSGS) {
      return;
    }
    msg = &msgBuffer;
    payload = (DataMsg*) call Send.getPayload(msg, sizeof(DataMsg));
    for (i = 0; i < MSG_SIZE; i++) {
      payload->data[i] = i;
    }
    result = call Send.send(msg, sizeof(DataMsg));
    if (result == SUCCESS) {
      sendCount++;
      //call SerialLogger.log(LOG_SENDING,sendCount);
    }
    else{
      //call SerialLogger.log(LOG_SEND_FAILED,result);
    }
  }


  event void Boot.booted(){
    call SerialControl.start();
  }

  event void SerialControl.startDone(error_t err) {
    if (err != SUCCESS) {
      call SerialControl.start();
    } else {
      call RadiosControl.start();
    }
  }

  event void SerialControl.stopDone(error_t err) {}

  event void RadiosControl.startDone(error_t error) {
    if (error != SUCCESS) {
      call RadiosControl.start();
    } else {
      call CtpRouteControl.start();
      call InitTimer.startOneShot(INIT_TIME);
    }
  }

	event void RadiosControl.stopDone(error_t err) {}



  event void InitTimer.fired() {
    initializeNode();
  }

    event void SendTimer.fired() {
    if(TOS_NODE_ID % 2 == 1){
      if (transmitting) {
       // call SerialLogger.log(LOG_SEND_TIMER,1);
        SendMessage();
        if (sendCount >= NUM_MSGS) {
          transmitting = FALSE;
        }
      } else {
        call SendTimer.stop();
      }
    }
    else {
        call SendTimer.stop();
    }
  }

	event bool Intercept.forward(message_t *msg, void *payload, uint8_t len) {}

	event void Send.sendDone(message_t *msg, error_t error) {}

	event message_t * Receive.receive(message_t *msg, void *payload, uint8_t len) {

    if (startTime == 0) {
      startTime = call FinishTimer.getNow();
    }
    endTime = call FinishTimer.getNow();
    

    if(call ReceivedCache.lookup(msg)){
      call SerialLogger.log(LOG_DUPLICATE_AT_ROOT,1);
      duplicate ++;
    }
    else{
      call ReceivedCache.insert(msg);
      receivedCount++;
      call SerialLogger.log(LOG_RECEIVED_PACKET,endTime);
      cycles = endTime>>16;
      call SerialLogger.log(LOG_OVERFLOW_COUNTER,cycles);
      
      call SerialLogger.log(LOG_RECEIVED_COUNT,receivedCount);
      call SerialLogger.log(LOG_OVERFLOW_COUNTER,receivedCount>>16);
    }
    return msg;
  }
	

  event void FinishTimer.fired(){
    call SendTimer.stop();
		call SerialLogger.log(LOG_FINISH,TOS_NODE_ID);
    if(call RootControl.isRoot()){
      call SerialLogger.log(LOG_ROOT,TOS_NODE_ID);
      call SerialLogger.log(LOG_RECEIVED_COUNT,receivedCount);
      call SerialLogger.log(LOG_OVERFLOW_COUNTER,receivedCount>>16);
      call SerialLogger.log(LOG_THROUGHPUT_TIME,endTime - startTime);
      cycles = (endTime - startTime)>>16;
      call SerialLogger.log(LOG_OVERFLOW_COUNTER,cycles);
    }
    else{
      call SerialLogger.log(LOG_NOT_ROOT,TOS_NODE_ID);
      call SerialLogger.log(LOG_SENT_COUNT,sendCount);
    }
    call SerialLogger.log(LOG_TOTAL_BEACONS, call CtpInfo.totalBeacons());
    call SerialLogger.log(LOG_OVERFLOW_COUNTER,(call CtpInfo.totalBeacons())>>16);
    call SerialLogger.log(LOG_DUPLICATES,call CtpInfoForward.totalDuplicates() + duplicate);
    call SerialLogger.log(LOG_AVERAGE_THL,call CtpInfoForward.averageTHL());
    call SerialLogger.log(LOG_MAX_THL,call CtpInfoForward.maxTHL());
    call SerialLogger.log(LOG_TOTAL_MESSAGES, call CtpInfoForward.totalMsgs());
    cycles = (call CtpInfoForward.totalMsgs())>>16;
    call SerialLogger.log(LOG_OVERFLOW_COUNTER,cycles);
  }

}

