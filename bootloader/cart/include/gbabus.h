#ifndef GBABUS_H
#define GBABUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

extern volatile uint8_t gba_sram[0x10000];

int gbabus_init();
void gbabus_deinit();

void gbabus_rom_serve(const void *buf);


#ifdef __cplusplus
}
#endif

#endif /* GBABUS_H */
