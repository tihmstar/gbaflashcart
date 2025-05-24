#include "gbabusrom_sram.h"

#include "macros.h"
#include "gbabusrom.pio.h"

#include <hardware/dma.h>
#include <pico/stdlib.h>

static int gRomBus_cs_pio_pc = -1;
static int gRomBus_rd_pio_pc = -1;
static int gRomBus_wr_pio_pc = -1;

static int gDMA_channel_cart_address_loader = -1;
static int gDMA_channel_base_address_loader = -1;
static int gDMA_channel_fetch_trigger = -1;
static int gDMA_channel_write_trigger = -1;
static int gDMA_channel_data_fetcher = -1;
static int gDMA_channel_data_writer = -1;

static const uint8_t *gGBAROMAddress = 0;

#pragma mark private
// static void __not_in_flash_func(reset_irq_handler)(){
//   dma_hw->abort = (1u << gDMA_channel_data_fetcher);
//   hw_set_bits(&dma_hw->ch[gDMA_channel_data_fetcher].al1_ctrl, DMA_CH0_CTRL_TRIG_READ_ERROR_BITS);
//   {
//     // changing the FIFO join state clears the fifo
//     hw_xor_bits(&GBABUSROM_SRAM_PIO->sm[GBABUSROM_SRAM_RD_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
//     hw_xor_bits(&GBABUSROM_SRAM_PIO->sm[GBABUSROM_SRAM_RD_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
//   }
//   dma_hw->multi_channel_trigger = 1u << gDMA_channel_cart_address_loader;
//   GBABUSROM_SRAM_PIO->irq = (1u << 0);
// }

// static void __not_in_flash_func(reset_irq_handler)(){
//   dma_hw->abort = (1u << 0);
//   hw_set_bits(&dma_hw->ch[0].al1_ctrl, DMA_CH0_CTRL_TRIG_READ_ERROR_BITS);
//   {
//     // changing the FIFO join state clears the fifo
//     hw_xor_bits(&GBABUSROM_SRAM_PIO->sm[GBABUSROM_SRAM_RD_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
//     hw_xor_bits(&GBABUSROM_SRAM_PIO->sm[GBABUSROM_SRAM_RD_SM].shiftctrl, PIO_SM0_SHIFTCTRL_FJOIN_RX_BITS);
//   }
//   dma_hw->multi_channel_trigger = 1u << 2;
//   GBABUSROM_SRAM_PIO->irq = (1u << 0);
// }

static void __attribute__((naked)) __not_in_flash_func(reset_irq_handler)(){
  __asm__(
"         mov.w      r2, #0x50000000\n"
"         movs       r3, #0x7\n"
"         str.w      r3, [r2, #0x464]\n"
"         add        r1, r2, 0x2000\n"
"         mov.w      r0, #0x40000000\n"
"         str        r0, [r1, #0x10]\n"
"         add.w      r1, r1, #0x1fe000\n"
"         add.w      r1, r1, #0x1000\n"
"         mov.w      r0, #0x80000000\n"
"         str.w      r0, [r1, #0xe8]\n"
"         str.w      r0, [r1, #0xe8]\n"
"         movs       r1, #0x8\n"
"         str.w      r1, [r2, #0x450]\n"
"         add.w      r2, r2, #0x200000\n"
"         str        r3, [r2, #0x30]\n"
"         bx         lr\n"
  );
}

#pragma mark public
int gbabusrom_sram_init(){
  int err = 0;

  if (gDMA_channel_data_fetcher == -1){
    gDMA_channel_data_fetcher = 0;
    dma_claim_mask(1<<gDMA_channel_data_fetcher);
  }
  if (gDMA_channel_data_writer == -1){
    gDMA_channel_data_writer = 1;
    dma_claim_mask(1<<gDMA_channel_data_writer);
  }
  if (gDMA_channel_write_trigger == -1){
    gDMA_channel_write_trigger = 2;
    dma_claim_mask(1<<gDMA_channel_write_trigger);
  }
  if (gDMA_channel_cart_address_loader == -1){
    gDMA_channel_cart_address_loader = 3;
    dma_claim_mask(1<<gDMA_channel_cart_address_loader);
  }

  if (gDMA_channel_fetch_trigger == -1){
    gDMA_channel_fetch_trigger = dma_claim_unused_channel(true);
  }
  if (gDMA_channel_base_address_loader == -1){
    gDMA_channel_base_address_loader = dma_claim_unused_channel(true);
  }

  // {
  //   /*
  //     6) Write data back to rom
  //   */
  //   dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_data_writer); /* get default configuration */
  //   channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_16);
  //   channel_config_set_read_increment(&channel_config, false);
  //   channel_config_set_write_increment(&channel_config, true);
  //   channel_config_set_dreq(&channel_config, pio_get_dreq(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_WR_SM, false)); /* configure data request. true: sending data to the PIO state machine */
  //   dma_channel_configure(gDMA_channel_data_writer, 
  //                         &channel_config,
  //                         gGBAROMAddress, //write target will be set by other DMA
  //                         &(GBABUSROM_SRAM_PIO->rxf[GBABUSROM_SRAM_WR_SM]), //read source
  //                         0x100, //
  //                         false // do not trigger yet, will be done after all the
  //                               // other DMAs are setup
  //   );
  // }

  // {
  //   /*
  //     5) Tell data writer where to fetch write to and start it
  //   */
  //   dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_write_trigger); /* get default configuration */
  //   channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
  //   channel_config_set_read_increment(&channel_config, false);
  //   channel_config_set_write_increment(&channel_config, false);
  //   dma_channel_configure(gDMA_channel_write_trigger, 
  //                         &channel_config,
  //                         &dma_hw->ch[gDMA_channel_data_writer].al2_write_addr_trig, //write target
  //                         &(dma_hw->sniff_data),                                     //read source
  //                         1,   //single transfer
  //                         false // do not trigger yet, will be done after all the
  //                               // other DMAs are setup
  //   );
  // }

  {
    /*
      4) Finally fetch data and give it to handler PIO
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_data_fetcher); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_16);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_dreq(&channel_config, pio_get_dreq(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_RD_SM, true)); /* configure data request. true: sending data to the PIO state machine */
    channel_config_set_high_priority(&channel_config, true); //this has priority over 5 / 6
    dma_channel_configure(gDMA_channel_data_fetcher, 
                          &channel_config,
                          &(GBABUSROM_SRAM_PIO->txf[GBABUSROM_SRAM_RD_SM]), //write target
                          NULL, //read source will be set by other DMA
                          -1,   //infinite trigger
                          false // do not trigger yet, will be done after all the
                                // other DMAs are setup
    );
  }

  {
    /*
      3) Tell data fetcher where to fetch data from and start it
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_fetch_trigger); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_high_priority(&channel_config, true); //this has priority over 5 / 6
    // channel_config_set_chain_to(&channel_config, gDMA_channel_data_writer);
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
    static uint32_t sDevNull = 0;
    /*
      2) Perform addition using DMA to get correct ROM source address  
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_base_address_loader); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_sniff_enable(&channel_config, true);
    channel_config_set_chain_to(&channel_config, gDMA_channel_fetch_trigger);
    channel_config_set_high_priority(&channel_config, true); //this has priority over 5 / 6
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
      1) Read rom address from GBA Bus and write it to sniff_data
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMA_channel_cart_address_loader); /* get default configuration */
    channel_config_set_dreq(&channel_config, pio_get_dreq(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, false)); /* configure data request. true: sending data to the PIO state machine */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMA_channel_base_address_loader);
    channel_config_set_high_priority(&channel_config, true); //this has priority over 5 / 6
    dma_channel_configure(gDMA_channel_cart_address_loader,
                          &channel_config,
                          (&dma_hw->sniff_data),    //write address
                          &GBABUSROM_SRAM_PIO->rxf[GBABUSROM_SRAM_CS_SM],  //read address
                          1,
                          false); /* start */
  }

if (gRomBus_cs_pio_pc == -1){
    pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, false);
    pio_set_gpio_base(GBABUSROM_SRAM_PIO, 0);

    /*
      Configure PINS
    */
    pio_gpio_init(GBABUSROM_SRAM_PIO, GBABUS_RD);
    pio_gpio_init(GBABUSROM_SRAM_PIO, GBABUS_CS);
    for (uint i = GBABUS_BUS_START; i < GBABUS_BUS_START + GBABUS_BUS_SIZE; i++) {
      pio_gpio_init(GBABUSROM_SRAM_PIO, i);
    }
    uint32_t mask = ((1 << GBABUS_BUS_SIZE) - 1) | 1 << GBABUS_CS | 1 << GBABUS_RD;
    pio_sm_set_pins(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, 0);
    pio_sm_set_consecutive_pindirs(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, 0, 32, false);
    pio_sm_set_pindirs_with_mask(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, 0, mask);
    // bypass synchroniser
    hw_set_bits(&GBABUSROM_SRAM_PIO->input_sync_bypass, mask);


    gRomBus_cs_pio_pc = pio_add_program(GBABUSROM_SRAM_PIO, &gbabusrom_cs_program);
    pio_sm_config c = gbabusrom_cs_program_get_default_config(gRomBus_cs_pio_pc);
    sm_config_set_clkdiv(&c, 1);
    
    sm_config_set_jmp_pin(&c, GBABUS_CS);
    sm_config_set_in_pins(&c, GBABUS_BUS_START);
    sm_config_set_in_shift(&c,  
                            false, //shift_right
                            true,  //autopush
                            GBABUS_BUS_SIZE + 1
                          );

    pio_sm_init(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, gRomBus_cs_pio_pc, &c);

    pio_set_irq0_source_enabled(GBABUSROM_SRAM_PIO, pis_interrupt0, true);
    irq_set_exclusive_handler(PIO0_IRQ_0, reset_irq_handler);
    irq_set_enabled(PIO0_IRQ_0, true);                

    pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, true);
  }

  if (gRomBus_rd_pio_pc == -1){
    pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_RD_SM, false);

    gRomBus_rd_pio_pc = pio_add_program(GBABUSROM_SRAM_PIO, &gbabusrom_rd_program);
    pio_sm_config c = gbabusrom_rd_program_get_default_config(gRomBus_rd_pio_pc);
    sm_config_set_clkdiv(&c, 1);

    // sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_in_pins(&c, GBABUS_BUS_START);
    sm_config_set_out_pins(&c, GBABUS_BUS_START, 16);
    sm_config_set_jmp_pin(&c, GBABUS_CS);
    sm_config_set_out_shift(&c,  
                            false, //shift_right
                            false,  //autopull
                            16
                          );
                   
    pio_sm_init(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_RD_SM, gRomBus_rd_pio_pc, &c);
    pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_RD_SM, true);
  }

  // if (gRomBus_wr_pio_pc == -1){
  //   pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_WR_SM, false);

  //   gRomBus_wr_pio_pc = pio_add_program(GBABUSROM_SRAM_PIO, &gbabusrom_wr_program);
  //   pio_sm_config c = gbabusrom_wr_program_get_default_config(gRomBus_wr_pio_pc);
  //   sm_config_set_clkdiv(&c, 1);

  //   sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_RX);
  //   sm_config_set_in_pins(&c, GBABUS_BUS_START);
  //   sm_config_set_out_pins(&c, GBABUS_BUS_START, 16);
  //   sm_config_set_jmp_pin(&c, GBABUS_CS);
  //   sm_config_set_in_shift(&c,  
  //                           false, //shift_right
  //                           true,  //autopush
  //                           16
  //                         );
                   
  //   pio_sm_init(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_WR_SM, gRomBus_wr_pio_pc, &c);
  //   pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_WR_SM, true);
  // }

  dma_channel_start(gDMA_channel_cart_address_loader);
  return err;
}

void gbabusrom_sram_deinit(){
  if (gRomBus_cs_pio_pc != -1){
    irq_set_enabled(PIO0_IRQ_0, false);         
    pio_set_irq0_source_enabled(GBABUSROM_SRAM_PIO, pis_interrupt0, false);
    irq_remove_handler(PIO0_IRQ_0, reset_irq_handler);

    pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_CS_SM, false);
    pio_remove_program(GBABUSROM_SRAM_PIO, &gbabusrom_cs_program, gRomBus_cs_pio_pc);
    gRomBus_cs_pio_pc = -1;
  }
  if (gRomBus_rd_pio_pc != -1){
    pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_RD_SM, false);
    pio_remove_program(GBABUSROM_SRAM_PIO, &gbabusrom_rd_program, gRomBus_rd_pio_pc);
    gRomBus_rd_pio_pc = -1;
  }
  {
    uint32_t abrtMask = 0;
    if (gDMA_channel_cart_address_loader != -1) abrtMask |= 1u<<gDMA_channel_cart_address_loader;
    if (gDMA_channel_base_address_loader != -1) abrtMask |= 1u<<gDMA_channel_base_address_loader;
    if (gDMA_channel_fetch_trigger != -1)       abrtMask |= 1u<<gDMA_channel_fetch_trigger;
    if (gDMA_channel_data_fetcher != -1)        abrtMask |= 1u<<gDMA_channel_data_fetcher;

    dma_hw->abort = abrtMask; //abort all at once!
    /*
      Wait for each channel to retire
    */
    if (gDMA_channel_cart_address_loader != -1) {
      dma_channel_abort(gDMA_channel_cart_address_loader);
      gDMA_channel_cart_address_loader = -1;
    }
    if (gDMA_channel_base_address_loader != -1) {
      dma_channel_abort(gDMA_channel_base_address_loader);
      gDMA_channel_base_address_loader = -1;
    }
    if (gDMA_channel_fetch_trigger != -1) {
      dma_channel_abort(gDMA_channel_fetch_trigger);
      gDMA_channel_fetch_trigger = -1;
    }
    if (gDMA_channel_data_fetcher != -1) {
      dma_channel_abort(gDMA_channel_data_fetcher);
      gDMA_channel_data_fetcher = -1;
    }
    dma_unclaim_mask(abrtMask);
  }
}

int gbabus_sram_serve(const void *buf, size_t size){
  gGBAROMAddress = (const uint8_t*)buf;
  return 0;
}

void gbabusrom_sram_enableGBAWrite(){
  pio_sm_set_enabled(GBABUSROM_SRAM_PIO, GBABUSROM_SRAM_WR_SM, true);
}