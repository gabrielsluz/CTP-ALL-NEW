interface UnicastNameFreeRouting1 {

  /**
   * Get the address of the best next hop set to the destination.
   * If there is not best next hop, the address is the local address.
   * @return : The next best hop, or the local address if there is no route.
   */
  command am_addr_t nextHop();
  command bool hasRoute();
  
  
event void routeFound();
  event void noRoute();

}

