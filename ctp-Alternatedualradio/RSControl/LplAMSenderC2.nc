#include "AM.h"

generic configuration LplAMSenderC2(am_id_t AMId)
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
  components new DirectAMSenderC2(AMId) as DirectAMSenderC;
  components new LplAMSenderP2() as LplAMSenderP;
  components RF212ActiveMessageC as ActiveMessageC;
  components SystemLowPowerListeningC;

  AMSend = LplAMSenderP;
  Packet = DirectAMSenderC;
  AMPacket = DirectAMSenderC;
  Acks = DirectAMSenderC;

  LplAMSenderP.SubAMSend -> DirectAMSenderC;
  LplAMSenderP.Lpl -> ActiveMessageC;
  LplAMSenderP.SystemLowPowerListening -> SystemLowPowerListeningC;
}
