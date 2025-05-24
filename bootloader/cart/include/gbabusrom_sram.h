#ifndef GBABUSROM_SRAM_H
#define GBABUSROM_SRAM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define GBABUSROM_SRAM_PIO          pio0
#define GBABUSROM_SRAM_CS_SM        0
#define GBABUSROM_SRAM_RD_SM        1
#define GBABUSROM_SRAM_WR_SM        2

int gbabusrom_sram_init();
void gbabusrom_sram_deinit();

int gbabus_sram_serve(const void *buf, size_t size);
void gbabusrom_sram_enableGBAWrite();

#ifdef __cplusplus
}
#endif

#endif /* GBABUSROM_SRAM_H */
