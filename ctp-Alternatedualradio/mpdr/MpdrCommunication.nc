interface MpdrCommunication {
  command error_t send(uint16_t data, am_addr_t destination);
  command void setNumPaths(uint8_t num_paths);
  event void sendDone(uint16_t data, am_addr_t destination, error_t error);
  event void receive(uint16_t data, am_addr_t source);
}
