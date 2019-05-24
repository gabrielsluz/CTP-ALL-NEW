#include "AM.h"

interface CtpInfoForward {

  command uint16_t totalDuplicates();
  command uint16_t maxTHL();
  command uint32_t averageTHL();
  command uint16_t totalMsgs();
}
