module DualRadioControlC {
  provides {
    interface SplitControl;
  }
  uses {
    interface SplitControl as Radio1Control;
    interface SplitControl as Radio2Control;
  }
}

implementation {
  bool radio1_started = FALSE;
  bool radio2_started = FALSE;

  command error_t SplitControl.start() {
    call Radio1Control.start();
    call Radio2Control.start();
    return SUCCESS;
  }

  event void Radio1Control.startDone(error_t err) {
    if (err != SUCCESS) {
      dbg("Control", "ERROR starting radio 1\n");
      call Radio1Control.start();
    } else {
      dbg("Control", "Radio 1 started\n");
      radio1_started = TRUE;
      if (radio1_started && radio2_started) {
        signal SplitControl.startDone(SUCCESS);
      }
    }
  }

  event void Radio2Control.startDone(error_t err) {
    if (err != SUCCESS) {
      dbg("Control", "ERROR starting radio 2\n");
      call Radio2Control.start();
    } else {
      dbg("Control", "Radio 2 started\n");
      radio2_started = TRUE;
      if (radio1_started && radio2_started) {
        signal SplitControl.startDone(SUCCESS);
      }
    }
  }

  command error_t SplitControl.stop() {
    call Radio1Control.stop();
    call Radio2Control.stop();
    return SUCCESS;
  }

  event void Radio1Control.stopDone(error_t err) {
    if (err != SUCCESS) {
      dbg("Control", "ERROR stopping radio 1\n");
      call Radio1Control.stop();
    } else {
      radio1_started = FALSE;
      if (!radio1_started && !radio2_started) {
        signal SplitControl.stopDone(SUCCESS);
      }
    }
  }

  event void Radio2Control.stopDone(error_t err) {
    if (err != SUCCESS) {
      dbg("Control", "ERROR stopping radio 2\n");
      call Radio2Control.stop();
    } else {
      radio2_started = FALSE;
      if (!radio1_started && !radio2_started) {
        signal SplitControl.stopDone(SUCCESS);
      }
    }
  }
}
