#ifndef TESTCTP_H
#define TESTCTP_H


#define MSG_SIZE 20

enum{
  LOG_INITIALIZED,
  LOG_ROOT,
  LOG_NOT_ROOT,
  LOG_FINISH,
  LOG_RECEIVED_COUNT,
  LOG_SENT_COUNT,
  LOG_SENDING,
  LOG_SET_RADIO_BIT,
  LOG_SEND_TIMER,
  LOG_SEND_FAILED,
  LOG_SEND_SUCCESS,
  LOG_IN_SEND,
  LOG_RADIO_START,
  LOG_RADIO_SEND,
  LOG_SEND_TASK,
  LOG_SEND_RADIO,
  LOG_POST_TASK,
  LOG_SEND_STEP,
  LOG_EBUSY,
  LOG_SEND_DONE,
  LOG_DROP_PACKET,
  LOG_ACKED,
  LOG_ROUTING_OFF,
  LOG_PACKET_TOO_BIG,
  LOG_NEED_RETRANSMISSION,
  LOG_FORWARD1_FROM,
  LOG_FORWARD2_FROM,
  LOG_RECEIVED_PACKET,
  LOG_RECEIVED_BEACON,
  LOG_SENT_BEACON,
  LOG_CURRENT_RADIO,
  LOG_CURRENT_DAD,
  LOG_DAD_CHANGE,
  LOG_SENDING_BEACON_RADIO,
  LOG_UPDATE_RADIO_TO,
  LOG_FROM,
  LOG_MIN_ETX_1,
  LOG_MIN_ETX_2,
  LOG_ETX_1,
  LOG_ETX_2,
  LOG_DUPLICATES,
  LOG_THROUGHPUT_TIME,
  LOG_TOTAL_BEACONS,
  LOG_AVERAGE_THL,
  LOG_MAX_THL,
  LOG_DUPLICATE_AT_ROOT,
  LOG_TOTAL_MESSAGES,
  LOG_OVERFLOW_COUNTER,
  LOG_FORCE_UPDATE,
  COLLECTION_ID = 0xee,
};

typedef nx_struct DataMsg{
	nx_uint8_t data[MSG_SIZE];
}DataMsg;




#endif

