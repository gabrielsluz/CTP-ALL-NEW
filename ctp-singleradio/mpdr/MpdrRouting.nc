interface MpdrRouting {

  command am_addr_t getNextHop(am_addr_t destination);

  command error_t addRoutingItem(am_addr_t source, am_addr_t destination,
                                 am_addr_t next_hop, uint8_t radio,
                                 uint8_t channel);

  command error_t addSendRoute(am_addr_t source, am_addr_t destination,
                               am_addr_t next_hop, uint8_t radio,
                               uint8_t channel);

  command error_t setRadioChannel(uint8_t radio, uint8_t channel);

  command error_t sendRouteMsg(am_addr_t source, am_addr_t destination,
                               uint8_t path_id, uint8_t size, am_addr_t* items);

  command error_t getSendAddresses(am_addr_t destination, am_addr_t* addr1,
                                   am_addr_t* addr2);

  command void setNumPaths(uint8_t num_paths);

  command uint8_t getNumPaths();

  event void pathsReady(am_addr_t destination);

}
