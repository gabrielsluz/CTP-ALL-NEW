#include "Mpdr.h"

#define RETRY_TIME 2

generic module MpdrForwardingEngineP() {
  provides {
    interface StdControl;
    interface AMSend;
    interface Receive;
    interface Packet;
    interface MpdrStats;
  }
  uses {
    interface AMSend as Radio1Send;
    interface AMSend as Radio2Send;
    interface Receive as Radio1Receive;
    interface Receive as Radio2Receive;
    interface Packet as Radio1Packet;
    interface Packet as Radio2Packet;
    interface PacketAcknowledgements as Radio1Ack;
    interface PacketAcknowledgements as Radio2Ack;

    interface MpdrRouting as Routing;
    interface Pool<message_t>;
    interface Queue<message_t*>;
    interface SerialLogger;
    interface LocalTime<TMilli>;
    interface Timer<TMilli> as RetryTimer;
  }
}

implementation {

  uint8_t radio = 1;
  uint8_t prefRadio = 0;
  bool requireAck = TRUE;
  bool sourceSending = FALSE;
  uint8_t maxRetransmissions = 10;
  uint16_t nextDsn = 1;

  message_t* msg1;
  message_t* msg2;
  bool radio1Busy = FALSE;
  bool radio2Busy = FALSE;
  bool retransmitting1 = FALSE;
  bool retransmitting2 = FALSE;
  uint8_t numRetransmissions1 = 0;
  uint8_t numRetransmissions2 = 0;
  uint16_t lastDsn1 = 0;
  uint16_t lastDsn2 = 0;

  // Stats
  uint16_t statSentRadio1 = 0;
  uint16_t statSentRadio2 = 0;
  uint16_t statReceivedRadio1 = 0;
  uint16_t statReceivedRadio2 = 0;
  uint16_t statDropped1 = 0;
  uint16_t statDropped2 = 0;
  uint32_t statStartTime1 = 0;
  uint32_t statStartTime2 = 0;
  uint32_t statEndTime1 = 0;
  uint32_t statEndTime2 = 0;
  uint16_t statRetransmissions1 = 0;
  uint16_t statRetransmissions2 = 0;
  uint16_t statMaxQueueSize = 0;
  uint16_t statDuplicated1 = 0;
  uint16_t statDuplicated2 = 0;
  uint16_t statPoolErrors = 0;
  uint16_t statQueueErrors = 0;
  uint16_t statEbusyRadio1 = 0;
  uint16_t statEbusyRadio2 = 0;

  void sendRadio1() {
    mpdr_msg_hdr_t* msg_hdr;
    uint8_t len;
    error_t result;
    uint8_t queue_size;
    /*uint16_t* seqno;*/
    if (radio1Busy) {
      return;
    }
    if (call Queue.empty()) {
      return;
    }
    queue_size = call Queue.size();
    if (queue_size > statMaxQueueSize) {
      statMaxQueueSize = queue_size;
    }
    if (!retransmitting1) {
      msg1 = call Queue.dequeue();
    }
    msg_hdr = call Radio1Send.getPayload(msg1, sizeof(mpdr_msg_hdr_t));
    len = call Radio1Packet.payloadLength(msg1);
    if (requireAck) {
      result = call Radio1Ack.requestAck(msg1);
    } else {
      result = call Radio1Ack.noAck(msg1);
    }
    /*seqno = (uint16_t*) (((void*)msg_hdr) + sizeof(mpdr_msg_hdr_t) + 1);
    if (!retransmitting1) {
      call SerialLogger.log(LOG_SENDING_SEQNO_1, *seqno);
    } else {
      call SerialLogger.log(LOG_RETRANSMITTING_SEQNO_1, *seqno);
    }*/
    result = call Radio1Send.send(msg_hdr->next_hop, msg1, len);
    if (result == SUCCESS) {
      radio1Busy = TRUE;
    } else {
      // Drop the packet for now
      statDropped1++;
      call Pool.put(msg1);
      call SerialLogger.log(LOG_RADIO_1_SEND_RESULT, result);
    }
    if (statStartTime1 == 0) {
      statStartTime1 = call LocalTime.get();
    }
    statEndTime1 = call LocalTime.get();
  }

  void sendRadio2() {
    mpdr_msg_hdr_t* msg_hdr;
    uint8_t len;
    error_t result;
    uint8_t queue_size;
    /*uint16_t* seqno;*/
    if (radio2Busy) {
      return;
    }
    if (call Queue.empty()) {
      return;
    }
    queue_size = call Queue.size();
    if (queue_size > statMaxQueueSize) {
      statMaxQueueSize = queue_size;
    }
    if (!retransmitting2) {
      msg2 = call Queue.dequeue();
    }
    msg_hdr = call Radio2Send.getPayload(msg2, sizeof(mpdr_msg_hdr_t));
    len = call Radio2Packet.payloadLength(msg2);
    if (requireAck) {
      result = call Radio2Ack.requestAck(msg2);
    } else {
      result = call Radio2Ack.noAck(msg2);
    }
    /*seqno = (uint16_t*) (((void*)msg_hdr) + sizeof(mpdr_msg_hdr_t) + 1);
    if (!retransmitting2) {
      call SerialLogger.log(LOG_SENDING_SEQNO_2, *seqno);
    } else {
      call SerialLogger.log(LOG_RETRANSMITTING_SEQNO_2, *seqno);
    }*/
    result = call Radio2Send.send(msg_hdr->next_hop, msg2, len);
    if (result == SUCCESS) {
      radio2Busy = TRUE;
    } else {
      // Drop the packet for now
      statDropped2++;
      call Pool.put(msg2);
      call SerialLogger.log(LOG_RADIO_2_SEND_RESULT, result);
    }
    if (statStartTime2 == 0) {
      statStartTime2 = call LocalTime.get();
    }
    statEndTime2 = call LocalTime.get();
  }

  void sendMessage() {
    message_t* msg;
    mpdr_msg_hdr_t* msg_hdr;
    am_addr_t addr, next1 = 0, next2 = 0;
    error_t result;
    if (prefRadio == 1) {
      if (radio1Busy) {
        return;
      }
      radio = 1;
    } else if (prefRadio == 2) {
      if (radio2Busy) {
        return;
      }
      radio = 2;
    } else {
      if (radio1Busy && radio2Busy) {
        return;
      } else if (!radio1Busy && radio2Busy) {
        radio = 1;
      } else if (radio1Busy && !radio2Busy) {
        radio = 2;
      }
    }
    msg = call Queue.head();
    if (radio == 1) {
      msg_hdr = call Radio1Send.getPayload(msg, sizeof(mpdr_msg_hdr_t));
    } else {
      msg_hdr = call Radio2Send.getPayload(msg, sizeof(mpdr_msg_hdr_t));
    }
    addr = msg_hdr->destination;
    if (prefRadio == 0 || sourceSending) {
      result = call Routing.getSendAddresses(addr, &next1, &next2);
    } else {
      next1 = call Routing.getNextHop(msg_hdr->destination);
      next2 = next1;
      result = SUCCESS;
    }
    if (result == FAIL) {
      call SerialLogger.log(LOG_ADDR_ERROR, result);
      return;
    }
    if (radio == 1) {
      msg_hdr->next_hop = next1;
      sendRadio1();
    } else {
      msg_hdr->next_hop = next2;
      sendRadio2();
    }
    if (radio == 1) {
      radio = 2;
    } else {
      radio = 1;
    }
  }

  command error_t StdControl.start() {
    dbg("StdControl", "MpdrForwardingEngineP started\n");
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    dbg("StdControl", "MpdrForwardingEngineP stopped\n");
    return SUCCESS;
  }

  command error_t AMSend.send(am_addr_t addr, message_t* msg, uint8_t len) {
    message_t* send_msg;
    mpdr_msg_hdr_t* msg_hdr;
    error_t result;
    send_msg = call Pool.get();
    if (send_msg == NULL) {
      statPoolErrors++;
      return FAIL;
    }
    memcpy(send_msg, msg, sizeof(message_t));
    msg_hdr = call Radio1Send.getPayload(send_msg, sizeof(mpdr_msg_hdr_t));
    msg_hdr->source = TOS_NODE_ID;
    msg_hdr->destination = addr;
    msg_hdr->dsn = nextDsn;
    nextDsn++;
    call Packet.setPayloadLength(send_msg, len);
    result = call Queue.enqueue(send_msg);
    if (result != SUCCESS) {
      statQueueErrors++;
      call Pool.put(send_msg);
      return FAIL;
    }
    if (call Routing.getNumPaths() == 1) {
      sourceSending = TRUE;
      prefRadio = 1;
    }
    sendMessage();
    return SUCCESS;
  }

  command error_t AMSend.cancel(message_t* msg) {
    // TODO: implement the cancel
    return FAIL;
  }

  command uint8_t AMSend.maxPayloadLength() {
    return call Packet.maxPayloadLength();
  }

  command void* AMSend.getPayload(message_t* msg, uint8_t len) {
    return call Packet.getPayload(msg, len);
  }


  event message_t* Radio1Receive.receive(message_t* msg, void* payload,
                                         uint8_t len) {
    mpdr_msg_hdr_t* msg_hdr = (mpdr_msg_hdr_t*) payload;
    message_t* send_msg;
    /*mpdr_msg_hdr_t* send_msg_hdr;*/
    /*am_addr_t next_hop;*/
    error_t result;

    /*uint16_t* seqno = (uint16_t*) (payload + sizeof(mpdr_msg_hdr_t) + 1);
    call SerialLogger.log(LOG_RECEIVED_RADIO_1_SEQNO, *seqno);*/

    if (msg_hdr->dsn == lastDsn1) {
      statDuplicated1++;
      return msg;
    }

    lastDsn1 = msg_hdr->dsn;
    statReceivedRadio1++;

    if (statStartTime1 == 0) {
      statStartTime1 = call LocalTime.get();
    }
    statEndTime1 = call LocalTime.get();

    if (msg_hdr->destination != TOS_NODE_ID) {
      /*
        Forward message
      */
      send_msg = call Pool.get();
      if (send_msg == NULL) {
        statPoolErrors++;
        return msg;
      }
      memcpy(send_msg, msg, sizeof(message_t));
      call Packet.setPayloadLength(send_msg, len - sizeof(mpdr_msg_hdr_t));
      result = call Queue.enqueue(send_msg);
      if (result == SUCCESS) {
        prefRadio = 2;
        sendMessage();
      } else {
        statQueueErrors++;
      }
    } else {
      /*
        Receive message
      */
      payload += sizeof(mpdr_msg_hdr_t);
      len -= sizeof(mpdr_msg_hdr_t);
      signal Receive.receive(msg, payload, len);
    }
    return msg;
  }

  event message_t* Radio2Receive.receive(message_t* msg, void* payload,
                                         uint8_t len) {
    mpdr_msg_hdr_t* msg_hdr = (mpdr_msg_hdr_t*) payload;
    message_t* send_msg;
    /*mpdr_msg_hdr_t* send_msg_hdr;*/
    /*am_addr_t next_hop;*/
    error_t result;

    /*uint16_t* seqno = (uint16_t*) (payload + sizeof(mpdr_msg_hdr_t) + 1);
    call SerialLogger.log(LOG_RECEIVED_RADIO_2_SEQNO, *seqno);*/

    if (msg_hdr->dsn == lastDsn2) {
      statDuplicated2++;
      return msg;
    }

    lastDsn2 = msg_hdr->dsn;
    statReceivedRadio2++;

    if (statStartTime2 == 0) {
      statStartTime2 = call LocalTime.get();
    }
    statEndTime2 = call LocalTime.get();

    if (msg_hdr->destination != TOS_NODE_ID) {
      /*
        Forward message
      */
      send_msg = call Pool.get();
      if (send_msg == NULL) {
        statPoolErrors++;
        return msg;
      }
      memcpy(send_msg, msg, sizeof(message_t));
      call Packet.setPayloadLength(send_msg, len - sizeof(mpdr_msg_hdr_t));
      result = call Queue.enqueue(send_msg);
      if (result == SUCCESS) {
        prefRadio = 1;
        sendMessage();
      } else {
        statQueueErrors++;
      }
    } else {
      /*
        Receive message
      */
      payload += sizeof(mpdr_msg_hdr_t);
      len -= sizeof(mpdr_msg_hdr_t);
      signal Receive.receive(msg, payload, len);
    }
    return msg;
  }

  event void RetryTimer.fired() {
    if (retransmitting1) {
      sendRadio1();
    }
    if (retransmitting2) {
      sendRadio2();
    }
  }

  event void Radio1Send.sendDone(message_t* msg, error_t error) {
    bool dropped = FALSE;
    radio1Busy = FALSE;
    if (msg1 != msg) {
      call SerialLogger.log(LOG_QUEUE_HEAD_ERROR_1, 0);
      return;
    }
    if (error == EBUSY) {
      statEbusyRadio1++;
      retransmitting1 = TRUE;
      call RetryTimer.startOneShot(RETRY_TIME);
      return;
    }
    if (requireAck) {
      if (!call Radio1Ack.wasAcked(msg)) {
        if (!retransmitting1) {
          retransmitting1 = TRUE;
          statRetransmissions1++;
          numRetransmissions1++;
          call RetryTimer.startOneShot(RETRY_TIME);
          return;
        } else {
          if (numRetransmissions1 < maxRetransmissions) {
            statRetransmissions1++;
            numRetransmissions1++;
            call RetryTimer.startOneShot(RETRY_TIME);
            return;
          } else {
            dropped = TRUE;
          }
        }
      }
    }
    numRetransmissions1 = 0;
    retransmitting1 = FALSE;
    if (dropped) {
      statDropped1++;
    } else {
      statSentRadio1++;
    }
    call Pool.put(msg);
    msg1 = NULL;
    /*if (error != SUCCESS) {
      call SerialLogger.log(LOG_SEND_DONE_1_ERROR, error);
    }*/
    if (!call Queue.empty()) {
      sendMessage();
    }
  }

  event void Radio2Send.sendDone(message_t* msg, error_t error) {
    bool dropped = FALSE;
    radio2Busy = FALSE;
    if (msg2 != msg) {
      call SerialLogger.log(LOG_QUEUE_HEAD_ERROR_2, 0);
      return;
    }
    if (error == EBUSY) {
      statEbusyRadio2++;
      retransmitting2 = TRUE;
      call RetryTimer.startOneShot(RETRY_TIME);
      return;
    }
    if (requireAck) {
      if (!call Radio2Ack.wasAcked(msg)) {
        if (!retransmitting2) {
          retransmitting2 = TRUE;
          statRetransmissions2++;
          numRetransmissions2++;
          call RetryTimer.startOneShot(RETRY_TIME);
          return;
        } else {
          if (numRetransmissions2 < maxRetransmissions) {
            statRetransmissions2++;
            numRetransmissions2++;
            call RetryTimer.startOneShot(RETRY_TIME);
            return;
          } else {
            dropped = TRUE;
          }
        }
      }
    }
    numRetransmissions2 = 0;
    retransmitting2 = FALSE;
    if (dropped) {
      statDropped2++;
    } else {
      statSentRadio2++;
    }
    call Pool.put(msg);
    msg2 = NULL;
    /*if (error != SUCCESS) {
      call SerialLogger.log(LOG_SEND_DONE_2_ERROR, error);
    }*/
    if (!call Queue.empty()) {
      sendMessage();
    }
  }

  event void Routing.pathsReady(am_addr_t destination) {
  }

  /*
    Packet commands
  */
  command void Packet.clear(message_t* msg) {
    call Radio1Packet.clear(msg);
    call Radio2Packet.clear(msg);
  }

  command uint8_t Packet.payloadLength(message_t* msg) {
    uint8_t len1 = call Radio1Packet.payloadLength(msg);
    uint8_t len2 = call Radio2Packet.payloadLength(msg);
    if (len1 == len2) {
      return len1 - sizeof(mpdr_msg_hdr_t);
    } else {
      if (len1 < len2) {
        return len1 - sizeof(mpdr_msg_hdr_t);
      } else {
        return len2 - sizeof(mpdr_msg_hdr_t);
      }
    }
  }

  command void Packet.setPayloadLength(message_t* msg, uint8_t len) {
    call Radio1Packet.setPayloadLength(msg, len + sizeof(mpdr_msg_hdr_t));
    call Radio2Packet.setPayloadLength(msg, len + sizeof(mpdr_msg_hdr_t));
  }

  command uint8_t Packet.maxPayloadLength() {
    uint8_t max1 = call Radio1Packet.maxPayloadLength();
    uint8_t max2 = call Radio2Packet.maxPayloadLength();
    if (max1 == max2) {
      return max1 - sizeof(mpdr_msg_hdr_t);
    }
    if (max1 < max2) {
      return max1 - sizeof(mpdr_msg_hdr_t);
    }
    return max2 - sizeof(mpdr_msg_hdr_t);
  }

  command void* Packet.getPayload(message_t* msg, uint8_t len) {
    void* payload1 = call Radio1Packet.getPayload(msg, len);
    void* payload2 = call Radio2Packet.getPayload(msg, len);
    if (payload1 == payload2) {
      return payload1 + sizeof(mpdr_msg_hdr_t);
    }
    if (payload1 > payload2) {
      return payload1 + sizeof(mpdr_msg_hdr_t);
    }
    return payload2 + sizeof(mpdr_msg_hdr_t);
  }

  /*
    Stats Commands
  */

  command uint16_t MpdrStats.getSentRadio1() {
    return statSentRadio1;
  }

  command uint16_t MpdrStats.getSentRadio2() {
    return statSentRadio2;
  }

  command uint16_t MpdrStats.getReceivedRadio1() {
    return statReceivedRadio1;
  }

  command uint16_t MpdrStats.getReceivedRadio2() {
    return statReceivedRadio2;
  }

  command uint16_t MpdrStats.getDropped1() {
    return statDropped1;
  }

  command uint16_t MpdrStats.getDropped2() {
    return statDropped2;
  }

  command uint32_t MpdrStats.getTimeRadio1() {
    return statEndTime1 - statStartTime1;
  }

  command uint32_t MpdrStats.getTimeRadio2() {
    return statEndTime2 - statStartTime2;
  }

  command uint16_t MpdrStats.getRetransmissions1() {
    return statRetransmissions1;
  }

  command uint16_t MpdrStats.getRetransmissions2() {
    return statRetransmissions2;
  }

  command uint16_t MpdrStats.getMaxQueueSize() {
    return statMaxQueueSize;
  }

  command uint16_t MpdrStats.getDuplicated1() {
    return statDuplicated1;
  }

  command uint16_t MpdrStats.getDuplicated2() {
    return statDuplicated2;
  }

  command uint16_t MpdrStats.getPoolErrors() {
    return statPoolErrors;
  }

  command uint16_t MpdrStats.getQueueErrors() {
    return statQueueErrors;
  }

  command uint16_t MpdrStats.getEbusyRadio1() {
    return statEbusyRadio1;
  }

  command uint16_t MpdrStats.getEbusyRadio2() {
    return statEbusyRadio2;
  }

  command void MpdrStats.clear() {
    statSentRadio1 = 0;
    statSentRadio2 = 0;
    statReceivedRadio1 = 0;
    statReceivedRadio2 = 0;
    statDropped1 = 0;
    statDropped2 = 0;
    statStartTime1 = 0;
    statStartTime2 = 0;
    statRetransmissions1 = 0;
    statRetransmissions2 = 0;
    statMaxQueueSize = 0;
    statDuplicated1 = 0;
    statDuplicated2 = 0;
    statPoolErrors = 0;
    statQueueErrors = 0;
    statEbusyRadio1 = 0;
    statEbusyRadio2 = 0;
  }

}
