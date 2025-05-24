#include "macros.h"

#include "DMAWriteWhatWhere.h"
#include "gbabus.h"


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hardware/clocks.h>
#include <hardware/pio.h>
#include <hardware/dma.h>
#include <pico/multicore.h>
#include <pico/stdlib.h>

const
#include <bootloader_gba.hex.h> //this should be in const section, so that it doesn't get copied to sram by bootloader!
__attribute__((section(".GBARAM.data.sram"))) uint8_t gba_sram[0x10000];


#define USEC_PER_SEC (1000000ULL)

#define GBABUS_PIN_IRQ 28

static uint32_t testbuf[0x20] = {};


int main() {
  int err = 0;
  int res = 0;

  // set_sys_clock_khz(266e3, false); //boots even without overclocking :D

  {
    uint32_t tmpDmaChannel = NUM_DMA_CHANNELS-1;
    dma_channel_config channel_config = dma_channel_get_default_config(tmpDmaChannel);
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, true);
    dma_channel_configure(tmpDmaChannel, 
                          &channel_config,
                          gba_sram,                       //write target
                          bootloader_gba,                 //read source 
                          (sizeof(bootloader_gba)/4) +1,  //going a bit oob is fine
                          true                            //start
    );
  }

  gbabus_rom_serve(gba_sram);
  cassure(!(res = gbabus_init()));


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