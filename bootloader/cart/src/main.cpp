#include "macros.h"

#include "gbabusrom_sram.h"
#include "DMAWriteWhatWhere.h"
#include <bootloader_gba.hex.h>


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

#define USEC_PER_SEC (1000000ULL)

#define GBABUS_PIN_IRQ 28

static uint32_t testbuf[0x20] = {};


int main() {
  int err = 0;
  int res = 0;

  set_sys_clock_khz(266e3, false); //gba doesn't boot if we go lower than 210e3

  cassure(!(res = gbabus_sram_serve(bootloader_gba, sizeof(bootloader_gba))));
  cassure(!(res = gbabusrom_sram_init()));

  // p/x ((dma_hw_t *)0x50000000)->ch[gDMA_channel_data_writer].write_addr

  while (1){
    tight_loop_contents();
  } 
error:
  debug("error %d",err);
  while (1){
   tight_loop_contents();
  }
  return 0;

}