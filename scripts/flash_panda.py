#!/usr/bin/env python3
import time
from panda.python.spi_dfu import PandaSpiDFU
from system.hardware import HARDWARE

if __name__ == "__main__":
  HARDWARE.recover_internal_panda()
  time.sleep(0.2)

  p = PandaSpiDFU('')
  p.global_erase()
  p.program_bootstub()
  p.program_app()
  p.reset()
