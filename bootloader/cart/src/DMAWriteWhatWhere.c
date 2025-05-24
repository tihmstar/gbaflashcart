#include "../include/DMAWriteWhatWhere.h"

#include "macros.h"

#include <hardware/dma.h>

#include <stdlib.h>

#define DWW_STOP_MARKER_ADDR NULL
#define DWW_STOP_MARKER_DATA 0

static uint32_t *gDataArr = NULL;
static uint32_t **gDataArrPtrs = NULL;
static uint32_t **gDstArr = NULL;
static uint32_t gArrSize = 0;


static int gDMAChannelSetupData = -1;
static int gDMAChannelSetupAddress = -1;
static int gDMAChannelPerformer = -1;


#pragma mark private
static void initGLobals(){
  if (gArrSize == 0){
    safeFree(gDstArr);
    safeFree(gDataArr);
    safeFree(gDataArrPtrs);
    gArrSize = 3;
    gDataArr =     (uint32_t *)calloc(gArrSize,sizeof(uint32_t));
    gDataArrPtrs = (uint32_t **)calloc(gArrSize,sizeof(uint32_t));
    gDstArr =      (uint32_t **)calloc(gArrSize,sizeof(uint32_t*));
  }
}

static void updateGlobals(){
  for (size_t i = 0; i < gArrSize; i++){
    gDataArrPtrs[i] = &gDataArr[i];
  }
  gDataArr[0] = DWW_STOP_MARKER_DATA;
  gDataArr[gArrSize-2] = (uint32_t)&gDstArr[0];
  gDataArr[gArrSize-1] = (uint32_t)&gDataArrPtrs[0];

  gDstArr[0] = (uint32_t*)&dma_hw->ch[gDMAChannelSetupData].al2_read_addr;
  gDstArr[1] = DWW_STOP_MARKER_ADDR;
  gDstArr[gArrSize-1] = (uint32_t*)&dma_hw->ch[gDMAChannelSetupAddress].al2_read_addr;

  {
    /*
      1) Setup Data
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMAChannelSetupData); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMAChannelSetupAddress);
    dma_channel_configure(gDMAChannelSetupData, 
                          &channel_config,
                          &dma_hw->ch[gDMAChannelPerformer].al2_read_addr, //write target
                          &gDataArrPtrs[1], //read source 
                          1,                //single transfer
                          false             // do not trigger yet
    );
  }

  {
    /*
      2) Setup Address and trigger performer
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMAChannelSetupAddress); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, true);
    channel_config_set_write_increment(&channel_config, false);
    dma_channel_configure(gDMAChannelSetupAddress, 
                          &channel_config,
                          &dma_hw->ch[gDMAChannelPerformer].al2_write_addr_trig, //write target
                          &gDstArr[2],  //read source 
                          1,            //single transfer
                          false         // do not trigger yet
    );
  }

  {
    /*
      3) Finally perform what-where write
    */
    dma_channel_config channel_config = dma_channel_get_default_config(gDMAChannelPerformer); /* get default configuration */
    channel_config_set_transfer_data_size(&channel_config, DMA_SIZE_32);
    channel_config_set_read_increment(&channel_config, false);
    channel_config_set_write_increment(&channel_config, false);
    channel_config_set_chain_to(&channel_config, gDMAChannelSetupData);
    dma_channel_configure(gDMAChannelPerformer, 
                          &channel_config,
                          NULL, //write target (will be set by other DMA)
                          NULL, //read source (will be set by other DMA)
                          1,    //single transfer
                          false // do not trigger yet,
    );
  }
}


#pragma mark public
int DMAWriteWhatWhere_init(){
  int err = 0;

  if (gDMAChannelSetupData == -1) gDMAChannelSetupData = dma_claim_unused_channel(true);
  if (gDMAChannelSetupAddress == -1) gDMAChannelSetupAddress = dma_claim_unused_channel(true);
  if (gDMAChannelPerformer == -1) gDMAChannelPerformer = dma_claim_unused_channel(true);

  initGLobals();
  updateGlobals();

error:
  return err;
}

void DMAWriteWhatWhere_deinit(){
  uint32_t abrtMask = 0;
  if (gDMAChannelSetupData != -1)     abrtMask |= 1u<<gDMAChannelSetupData;
  if (gDMAChannelSetupAddress != -1)  abrtMask |= 1u<<gDMAChannelSetupAddress;
  if (gDMAChannelPerformer != -1)     abrtMask |= 1u<<gDMAChannelPerformer;

  dma_hw->abort = abrtMask; //abort all at once!
  /*
    Wait for each channel to retire
  */
  if (gDMAChannelSetupData != -1) {
    dma_channel_abort(gDMAChannelSetupData);
    gDMAChannelSetupData = -1;
  }
  if (gDMAChannelSetupAddress != -1) {
    dma_channel_abort(gDMAChannelSetupAddress);
    gDMAChannelSetupAddress = -1;
  }
  if (gDMAChannelPerformer != -1) {
    dma_channel_abort(gDMAChannelPerformer);
    gDMAChannelPerformer = -1;
  }
  dma_unclaim_mask(abrtMask);
  safeFree(gDstArr);
  safeFree(gDataArr);
  safeFree(gDataArrPtrs);
  gArrSize = 0;
}

int DMAWriteWhatWhere_push(uint32_t *dst, uint32_t data){
  int err = 0;
  cassure(gDMAChannelSetupData != -1);

  gArrSize++;
  gDataArr = (uint32_t *)realloc(gDataArr, sizeof(uint32_t)*gArrSize);
  gDataArrPtrs = (uint32_t **)realloc(gDataArrPtrs, sizeof(uint32_t*)*gArrSize);
  gDstArr = (uint32_t **)realloc(gDstArr, sizeof(uint32_t*)*gArrSize);

  gDataArr[gArrSize-3] = data;
  gDstArr[gArrSize-2] = dst;

  updateGlobals();
  
error:
  return err;
}

int DMAWriteWhatWhere_trigger(){
  int err = 0;
  cassure(gDMAChannelSetupData != -1);
  dma_channel_start(gDMAChannelSetupData);
error:
  return err;
}