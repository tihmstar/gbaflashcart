#include "gbabus.h"

#include "macros.h"
#include "gbabus.pio.h"

#include <hardware/dma.h>
#include <pico/stdlib.h>

#define GBABUS_PIO          pio0
#define GBABUS_CS_SM        0
#define GBABUS_RD_SM        1
#define GBABUS_CS3_SM       2
#define GBABUS_WR_SM        3


static int gRomBus_cs_pio_pc = -1;
static int gRomBus_rd_pio_pc = -1;
static int gRomBus_cs2_pio_pc = -1;
static int gRomBus_wr_pio_pc = -1;

static int gDMA_channel_base_address_loader = -1;
static int gDMA_channel_cart_address_loader = -1;
static int gDMA_channel_fetch_trigger = -1;
static int gDMA_channel_data_fetcher = -1;

static int gDMA_channel_cs_high_waiter  = -1;
static int gDMA_channel_cs_high_dma_abort = -1;
static int gDMA_channel_cs_high_dma_clearerr = -1;
static int gDMA_channel_cs_high_fifo_reset = -1;


static const uint8_t *gGBAROMAddress = 0;

#pragma mark private
static int gbabus_init_pio(){
  int err = 0;

  if (gRomBus_cs_pio_pc == -1){
    pio_sm_set_enabled(GBABUS_PIO, GBABUS_CS_SM, false);
    pio_set_gpio_base(GBABUS_PIO, 0);

    gRomBus_cs_pio_pc = pio_add_program(GBABUS_PIO, &gbabus_cs_program);
    pio_sm_config c = gbabus_cs_program_get_default_config(gRomBus_cs_pio_pc);
    sm_config_set_clkdiv(&c, 1);
    
    sm_config_set_jmp_pin(&c, GBABUS_CS);
    sm_config_set_in_pins(&c, GBABUS_BUS_START);
    sm_config_set_in_shift(&c,  
                            false, //shift_right
                            true,  //autopush
                            GBABUS_BUS_SIZE + 1
                          );
    sm_config_set_out_shift(&c,  
                            false, //shift_right
                            false,  //autopull
                            32
                          );

    pio_sm_init(GBABUS_PIO, GBABUS_CS_SM, gRomBus_cs_pio_pc, &c);

    //fill fifo
    pio_sm_put(GBABUS_PIO, GBABUS_CS_SM, 0);
    pio_sm_put(GBABUS_PIO, GBABUS_CS_SM, 0);
    pio_sm_put(GBABUS_PIO, GBABUS_CS_SM, 0);
    pio_sm_put(GBABUS_PIO, GBABUS_CS_SM, 0);
  }
  
  if (gRomBus_rd_pio_pc == -1){
    pio_sm_set_enabled(GBABUS_PIO, GBABUS_RD_SM, false);

    gRomBus_rd_pio_pc = pio_add_program(GBABUS_PIO, &gbabus_rd_program);
    pio_sm_config c = gbabus_rd_program_get_default_config(gRomBus_rd_pio_pc);
    sm_config_set_clkdiv(&c, 1);

    sm_config_set_in_pins(&c, GBABUS_BUS_START);
    sm_config_set_out_pins(&c, GBABUS_BUS_START, 16);
    sm_config_set_jmp_pin(&c, GBABUS_CS);
    sm_config_set_out_shift(&c,  
                            false, //shift_right
                            false,  //autopull
                            16
                          );
                   
    pio_sm_init(GBABUS_PIO, GBABUS_RD_SM, gRomBus_rd_pio_pc, &c);
    pio_sm_set_enabled(GBABUS_PIO, GBABUS_RD_SM, true);
  }

error:
  return err;
}

static int gbabus_init_dma(){
  int err = 0;


  if (gDMA_channel_cart_address_loader == -1) gDMA_channel_cart_address_loader = dma_claim_unused_channel(true);
  if (gDMA_channel_base_address_loader == -1) gDMA_channel_base_address_loader = dma_claim_unused_channel(true);
  if (gDMA_channel_fetch_trigger == -1) gDMA_channel_fetch_trigger = dma_claim_unused_channel(true);
  if (gDMA_channel_data_fetcher == -1) gDMA_channel_data_fetcher = dma_claim_unused_channel(true);
  if (gDMA_channel_cs_high_waiter == -1) gDMA_channel_cs_high_waiter = dma_claim_unused_channel(true);
  if (gDMA_channel_cs_high_dma_abort == -1) gDMA_channel_cs_high_dma_abort = dma_claim_unused_channel(true);
  if (gDMA_channel_cs_high_dma_clearerr == -1) gDMA_channel_cs_high_dma_clearerr = dma_claim_unused_channel(true);
  if (gDMA_channel_cs_high_fifo_reset == -1) gDMA_channel_cs_high_fifo_reset = dma_claim_unused_channel(true);

  {
    /*
      1) Read rom address from GBA Bus and write it to sniff_data
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_cart_address_loader); /* get default configuration */
    channel_config_set_dreq(&channel_config, pio_get_dreq(GBABUS_PIO, GBABUS_CS_SM, false)); /* configure data request. true: sending data to the PIO state machine */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMA_channel_base_address_loader);
    dma_channel_configure(gDMA_channel_cart_address_loader,
                          &channel_config,
                          (&dma_hw->sniff_data),    //write address
                          &GBABUS_PIO->rxf[GBABUS_CS_SM],  //read address
                          1 | (1u << 28),   //trigger self
                          true); /* start */
  }

  {
    /*
      2) Perform addition using DMA to get correct ROM source address  
    */
    static uint32_t sDevNull;
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_base_address_loader); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_sniff_enable(&channel_config, true);
    channel_config_set_chain_to(&channel_config, gDMA_channel_fetch_trigger);
    dma_channel_configure(gDMA_channel_base_address_loader, 
                          &channel_config,
                          &sDevNull,                 //write target
                          &gGBAROMAddress,           //read source
                          1,    // always transfer one word (pointer)
                          false // do not trigger yet, will be done after all the
                                // other DMAs are setup
    );
    dma_sniffer_enable(gDMA_channel_base_address_loader, DMA_SNIFF_CTRL_CALC_VALUE_SUM, true);
  }

  {
    /*
      3) Tell data fetcher where to fetch data from and start it
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_fetch_trigger); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    dma_channel_configure(gDMA_channel_fetch_trigger, 
                          &channel_config,
                          &dma_hw->ch[gDMA_channel_data_fetcher].al3_read_addr_trig, //write target
                          &(dma_hw->sniff_data),                                     //read source
                          1,    //only one word to transfer pointer
                          false // do not trigger yet, will be done after all the
                                // other DMAs are setup
    );
  }

  {
    /*
      4) Finally fetch data and give it to handler PIO
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_data_fetcher); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_16);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_dreq(&channel_config, pio_get_dreq(GBABUS_PIO, GBABUS_RD_SM, true)); /* configure data request. true: sending data to the PIO state machine */
    dma_channel_configure(gDMA_channel_data_fetcher, 
                          &channel_config,
                          &(GBABUS_PIO->txf[GBABUS_RD_SM]), //write target
                          NULL, //read source will be set by other DMA
                          -1,   //infinite trigger
                          false // do not trigger yet, will be done after all the
                                // other DMAs are setup
    );
  }

  {
    /*
      5) Wait until CS goes high and trigger next DMA
    */
    static uint32_t sDevNull2;
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_cs_high_waiter); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_dreq(&channel_config, pio_get_dreq(GBABUS_PIO, GBABUS_CS_SM, true)); /* configure data request. true: sending data to the PIO state machine */
    channel_config_set_chain_to(&channel_config, gDMA_channel_cs_high_dma_abort);
    dma_channel_configure(gDMA_channel_cs_high_waiter, 
                          &channel_config,
                          &(GBABUS_PIO->txf[GBABUS_CS_SM]), //write target
                          &sDevNull2,       //read source
                          1,   //trigger self
                          true              //start
    );
  }

  {
    /*
      6) CS got high, abort read dma!
    */
    static uint32_t sAbortBits;
    sAbortBits =  (1u << gDMA_channel_data_fetcher);
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_cs_high_dma_abort); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMA_channel_cs_high_dma_clearerr);
    dma_channel_configure(gDMA_channel_cs_high_dma_abort, 
                          &channel_config,
                          &dma_hw->abort,   //write target
                          &sAbortBits,      //read source
                          1,                //one word
                          false             // do not trigger yet, will be done after all the
                                            // other DMAs are setup
    );
  }

  {
    /*
      7) CS got high, clear DMA errors!

      We need to do the following sequence:
        hw_set_bits(&dma_hw->ch[gDMA_channel_data_fetcher].al1_ctrl, DMA_CH0_CTRL_TRIG_READ_ERROR_BITS);
      which equals to the following 1 writes:
        *(uint32_t*)hw_set_alias_untyped((volatile void *) &dma_hw->ch[gDMA_channel_data_fetcher].al1_ctrl) = DMA_CH0_CTRL_TRIG_READ_ERROR_BITS;
    */
    static uint32_t sCntrlBitsDMA = DMA_CH0_CTRL_TRIG_READ_ERROR_BITS;
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_cs_high_dma_clearerr); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMA_channel_cs_high_fifo_reset);
    dma_channel_configure(gDMA_channel_cs_high_dma_clearerr, 
                          &channel_config,
                          hw_set_alias_untyped((volatile void *) &dma_hw->ch[gDMA_channel_data_fetcher].al1_ctrl),   //write target
                          &sCntrlBitsDMA,   //read source
                          1,                //do transfer once
                          false             // do not trigger yet, will be done after all the
                                            // other DMAs are setup
    );
  }

  {
    /*
      8) CS got high, reset RD PIO SM FIFO!

      We need to do the following sequence to reset fifo:
          hw_xor_bits(&GBABUS_PIO->sm[GBABUS_RD_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
          hw_xor_bits(&GBABUS_PIO->sm[GBABUSM_RD_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
      which equals to the following 2 writes:
        *(uint32_t*)hw_xor_alias_untyped((volatile void *) &GBABUS_PIO->sm[GBABUSM_RD_SM].shiftctrl) = PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS;
        *(uint32_t*)hw_xor_alias_untyped((volatile void *) &GBABUS_PIO->sm[GBABUSM_RD_SM].shiftctrl) = PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS;
    */
    static uint32_t sCntrlBitsPIO = PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS;
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_cs_high_fifo_reset); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMA_channel_cs_high_waiter);
    dma_channel_configure(gDMA_channel_cs_high_fifo_reset, 
                          &channel_config,
                          hw_xor_alias_untyped((volatile void *) &GBABUS_PIO->sm[GBABUS_RD_SM].shiftctrl),   //write target
                          &sCntrlBitsPIO,   //read source
                          2,                //do transfer twice
                          false             // do not trigger yet, will be done after all the
                                            // other DMAs are setup
    );
  }

error:
  return err;
}

#pragma mark public
int gbabus_init(){
  int err = 0;

  /*
    Configure PINS
  */
  pio_gpio_init(GBABUS_PIO, GBABUS_RD);
  pio_gpio_init(GBABUS_PIO, GBABUS_CS);
  pio_gpio_init(GBABUS_PIO, GBABUS_CS2);
  for (uint i = GBABUS_BUS_START; i < GBABUS_BUS_START + GBABUS_BUS_SIZE; i++) {
    pio_gpio_init(GBABUS_PIO, i);
  }
  uint32_t mask = ((1 << GBABUS_BUS_SIZE) - 1) | 1 << GBABUS_CS | 1 << GBABUS_RD;
  pio_sm_set_pins(GBABUS_PIO, GBABUS_CS_SM, 0);
  pio_sm_set_consecutive_pindirs(GBABUS_PIO, GBABUS_CS_SM, 0, 32, false);
  pio_sm_set_pindirs_with_mask(GBABUS_PIO, GBABUS_CS_SM, 0, mask);
  // bypass synchroniser
  hw_set_bits(&GBABUS_PIO->input_sync_bypass, mask);

  cassure(!gbabus_init_pio());
  cassure(!gbabus_init_dma());

  pio_sm_set_enabled(GBABUS_PIO, GBABUS_CS_SM, true);

error:
  return err;
}

void gbabus_deinit(){
  assert(0); //todo
}


void gbabus_rom_serve(const void *buf){
  gGBAROMAddress = (const uint8_t*)buf;
}