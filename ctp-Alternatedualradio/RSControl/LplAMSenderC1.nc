#include "AM.h"

generic configuration LplAMSenderC1(am_id_t AMId)
{
  provides {
    interface AMSend;
    interface Packet;
    interface AMPacket;
    interface PacketAcknowledgements as Acks;
  }
}

implementation
{
  components new DirectAMSenderC1(AMId) as DirectAMSenderC;
  components new LplAMSenderP1() as LplAMSenderP;
  components RF231ActiveMessageC as ActiveMessageC;
  components SystemLowPowerListeningC;

  AMSend = LplAMSenderP;
  Packet = DirectAMSenderC;
  AMPacket = DirectAMSenderC;
  Acks = DirectAMSenderC;

  LplAMSenderP.SubAMSend -> DirectAMSenderC;
  LplAMSenderP.Lpl -> ActiveMessageC;
  LplAMSenderP.SystemLowPowerListening -> SystemLowPowerListeningC;
}
