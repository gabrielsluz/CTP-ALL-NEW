interface DualLinkEstimator {
  command uint16_t getLinkQuality(am_addr_t neighbor, uint8_t radio);
  command am_addr_t getNeighbor(uint8_t i, uint8_t radio);
  command uint8_t getNumOfNeighbors(uint8_t radio);
}
