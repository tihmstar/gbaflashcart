#include <gba_console.h>
#include <gba_video.h>
#include <gba_interrupt.h>
#include <gba_systemcalls.h>
#include <gba_input.h>
#include <gba_sio.h>
#include <stdio.h>
#include <stdlib.h>

#define MEM_ROM ((volatile uint16_t*)0x0C000000)
#define MEM_ROM_MSG_OFFSET    0xd2
#define MEM_ROM_STATUS_OFFSET 0xd0
#define STATUS_OK 0x6969

#define BOOTED_MAGIC "doneboot"

char msg[0x100] = {};

int main(void){
	
	// the vblank interrupt must be enabled for VBlankIntrWait() to work
	// since the default dispatcher handles the bios flags no vblank handler
	// is required
	irqInit();
	irqEnable(IRQ_VBLANK);

	consoleDemoInit();

	// ansi escape sequence to set print co-ordinates
	// /x1b[line;columnH
	iprintf("\x1b[10;10HBooting\n");

  while (MEM_ROM[MEM_ROM_STATUS_OFFSET/2] != 0x6969){
    //notify cartridge that we booted successfully!
    volatile uint8_t *dst = (volatile uint8_t*)(SRAM);
    for (int i=0; i<8; i++){
      dst[i] = BOOTED_MAGIC[i];
    }
  }

  for (size_t i = 0; i < sizeof(msg)/2; i++){
    uint16_t *dst = (uint16_t*)msg;
    uint16_t v = MEM_ROM[MEM_ROM_MSG_OFFSET/2 + i];
    dst[i] = v;
    if (!v) break;
  }
  
  iprintf(CON_CLS() "\x1b[0;0H%s\n",msg);

	while (1) {
		VBlankIntrWait();
	}
  
  while (1);
}