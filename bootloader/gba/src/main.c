#include <gba_console.h>
#include <gba_video.h>
#include <gba_interrupt.h>
#include <gba_systemcalls.h>
#include <gba_input.h>
#include <gba_sio.h>
#include <stdio.h>
#include <stdlib.h>

#define MEM_ROM ((uint16_t*)0x0C000000)

#define BOOTED_MAGIC "doneboot"
char *dbgmsg = ((char*)MEM_ROM)+2;

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

	while (1) {
    uint16_t *dst = (uint16_t*)MEM_ROM;
    for (int i=0; i<4; i++){
      dst[i] = 0x4142;
    }
		VBlankIntrWait();
	}
  
  while (1);
}