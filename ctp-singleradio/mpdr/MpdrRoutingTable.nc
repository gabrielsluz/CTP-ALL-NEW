interface MpdrRoutingTable {

  command am_addr_t getNextHop(am_addr_t destination);

  command error_t addRoutingItem(am_addr_t source, am_addr_t destination,
                                 am_addr_t next_hop);

  command error_t sendRouteMsg(am_addr_t source, am_addr_t destination,
                               uint8_t size, am_addr_t* items);

  command error_t getSendAddresses(am_addr_t destination, am_addr_t* addr1,
                                   am_addr_t* addr2);

  event void pathsReady(am_addr_t destination);
}
