interface MpdrStats {

  command void clear();
  command uint16_t getSentRadio1();
  command uint16_t getSentRadio2();
  command uint16_t getReceivedRadio1();
  command uint16_t getReceivedRadio2();
  command uint16_t getDropped1();
  command uint16_t getDropped2();
  command uint32_t getTimeRadio1();
  command uint32_t getTimeRadio2();
  command uint16_t getRetransmissions1();
  command uint16_t getRetransmissions2();
  command uint16_t getMaxQueueSize();
  command uint16_t getDuplicated1();
  command uint16_t getDuplicated2();
  command uint16_t getPoolErrors();
  command uint16_t getQueueErrors();
  command uint16_t getEbusyRadio1();
  command uint16_t getEbusyRadio2();

}
