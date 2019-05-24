#include "DualLinkEstimator.h"

generic module DualLinkEstimatorP() {
  provides {
    interface StdControl;
    interface DualLinkEstimator;
  }
  uses {
    interface AMSend as Radio1Send;
    interface AMSend as Radio2Send;
    interface Receive as Radio1Receive;
    interface Receive as Radio2Receive;
    interface Timer<TMilli> as Radio1Timer;
    interface Timer<TMilli> as Radio2Timer;
    interface Random;
  }
}

implementation {

  uint16_t seqno1 = 0;
  uint16_t seqno2 = 0;

  message_t radio1Buffer;
  message_t radio2Buffer;

  bool sending1 = FALSE;
  bool sending2 = FALSE;

  neighbor_table_t table1;
  neighbor_table_t table2;

  void sendMessage1() {
    message_t* msg;
    estimator_msg_t* payload;
    if (NUM_MSGS > 0 && seqno1 >= NUM_MSGS)  {
      sending1 = FALSE;
    }
    if (!sending1) {
      return;
    }
    msg = &radio1Buffer;
    payload = call Radio1Send.getPayload(msg, sizeof(estimator_msg_t));
    payload->source = TOS_NODE_ID;
    payload->seqno = seqno1;
    seqno1++;
    call Radio1Send.send(AM_BROADCAST_ADDR, msg, sizeof(estimator_msg_t));
  }

  void sendMessage2() {
    message_t* msg;
    estimator_msg_t* payload;
    if (NUM_MSGS > 0 && seqno2 >= NUM_MSGS) {
      sending2 = FALSE;
    }
    if (!sending2) {
      return;
    }
    msg = &radio2Buffer;
    payload = call Radio2Send.getPayload(msg, sizeof(estimator_msg_t));
    payload->source = TOS_NODE_ID;
    payload->seqno = seqno2;
    seqno2++;
    call Radio2Send.send(AM_BROADCAST_ADDR, msg, sizeof(estimator_msg_t));
  }

  uint16_t getNewInterval() {
    return call Random.rand16() % 100 + 400;
  }

  command error_t StdControl.start() {
    uint16_t newInterval;
    sending1 = TRUE;
    sending2 = TRUE;
    newInterval = getNewInterval();
    call Radio1Timer.startOneShot(newInterval);
    newInterval = getNewInterval();
    call Radio2Timer.startOneShot(newInterval);
    return SUCCESS;
  }

  command error_t StdControl.stop() {
    sending1 = FALSE;
    sending2 = FALSE;
    return SUCCESS;
  }

  event void Radio1Timer.fired() {
    uint16_t newInterval;
    sendMessage1();
    if (sending1) {
      newInterval = getNewInterval();
      call Radio1Timer.startOneShot(newInterval);
    }
  }

  event void Radio2Timer.fired() {
    uint16_t newInterval;
    sendMessage2();
    if (sending2) {
      newInterval = getNewInterval();
      call Radio2Timer.startOneShot(newInterval);
    }
  }

  event void Radio1Send.sendDone(message_t* msg, error_t err) {}

  event void Radio2Send.sendDone(message_t* msg, error_t err) {}

  bool updateNeighbor(am_addr_t addr, uint16_t rseqno, neighbor_table_t* table) {
    uint8_t i;
    for (i = 0; i < table->size; i++) {
      if (table->neighbors[i].address == addr) {
        table->neighbors[i].received++;
        table->neighbors[i].sent = rseqno;
        return TRUE;
      }
    }
    return FALSE;
  }

  bool addNeighbor(am_addr_t addr, uint16_t rseqno, neighbor_table_t* table) {
    if (table->size < MAX_NEIGHBOR_TABLE_SIZE) {
      table->neighbors[table->size].address = addr;
      table->neighbors[table->size].received = 1;
      table->neighbors[table->size].sent = rseqno;
      table->size++;
      return TRUE;
    } else {
      return FALSE;
    }
  }

  event message_t* Radio1Receive.receive(message_t* msg, void* payload, uint8_t len) {
    estimator_msg_t* rcvd = (estimator_msg_t*) payload;
    if (!updateNeighbor(rcvd->source, rcvd->seqno, &table1)) {
      addNeighbor(rcvd->source, rcvd->seqno, &table1);
    }
    return msg;
  }

  event message_t* Radio2Receive.receive(message_t* msg, void* payload, uint8_t len) {
    estimator_msg_t* rcvd = (estimator_msg_t*) payload;
    if (!updateNeighbor(rcvd->source, rcvd->seqno, &table2)) {
      addNeighbor(rcvd->source, rcvd->seqno, &table2);
    }
    return msg;
  }

  command uint16_t DualLinkEstimator.getLinkQuality(am_addr_t neighbor, uint8_t radio) {
    uint8_t i;
    uint16_t received, sent;
    neighbor_table_t* table;
    if (radio == 1) {
      table = &table1;
    } else {
      table = &table2;
    }
    for (i = 0; i < table->size; i++) {
      if (table->neighbors[i].address == neighbor) {
        received = table->neighbors[i].received;
        sent = table->neighbors[i].sent;
        if (NUM_MSGS == 0) {
          return 100 * received / sent;
        } else {
          return 100 * received / NUM_MSGS;
        }
      }
    }
    return 0xFFFF;
  }

  command am_addr_t DualLinkEstimator.getNeighbor(uint8_t i, uint8_t radio) {
    neighbor_table_t* table;
    if (radio == 1) {
      table = &table1;
    } else {
      table = &table2;
    }
    return table->neighbors[i].address;
  }

  command uint8_t DualLinkEstimator.getNumOfNeighbors(uint8_t radio) {
    neighbor_table_t* table;
    if (radio == 1) {
      table = &table1;
    } else {
      table = &table2;
    }
    return table->size;
  }

}
