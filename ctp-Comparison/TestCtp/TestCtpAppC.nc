#include "TestCtp.h"
#include "Ctp.h"

configuration TestCtpAppC {}

implementation {
//Boot e Aplicacao
components MainC, TestCtpC as App;
App.Boot -> MainC;

//Ctp
components CollectionC as Ctp;
App.CtpRouteControl -> Ctp;
App.RootControl -> Ctp;
App.CtpInfo -> Ctp;
App.CtpInfoForward -> Ctp;
App.Receive -> Ctp.Receive[COLLECTION_ID];
App.Intercept -> Ctp.Intercept[COLLECTION_ID];
App.CollectionPacket -> Ctp.CollectionPacket;

components new CollectionSenderC(COLLECTION_ID);
App.Send -> CollectionSenderC;

//Timers
components new TimerMilliC() as InitTimerC;
App.InitTimer -> InitTimerC;
components new TimerMilliC() as FinishTimerC;
App.FinishTimer -> FinishTimerC;
components new TimerMilliC() as SendTimerC;
App.SendTimer -> SendTimerC;

//Logger
components SerialLoggerC;
App.SerialControl -> SerialLoggerC;
App.SerialLogger -> SerialLoggerC;

//Dual Radio Control
components RF231ActiveMessageC;
components RF212ActiveMessageC;
components DualRadioControlC;
DualRadioControlC.Radio1Control -> RF231ActiveMessageC;
DualRadioControlC.Radio2Control -> RF212ActiveMessageC;
App.RadiosControl -> DualRadioControlC;

components new LruCtpMsgCacheC(16) as ReceivedCacheP;
App.ReceivedCache -> ReceivedCacheP;

}

