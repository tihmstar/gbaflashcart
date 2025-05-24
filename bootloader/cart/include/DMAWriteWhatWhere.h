#ifndef DMAWRITEWHATWHERE_H
#define DMAWRITEWHATWHERE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>


int DMAWriteWhatWhere_init();
void DMAWriteWhatWhere_deinit();

int DMAWriteWhatWhere_push(uint32_t *dst, uint32_t data);
int DMAWriteWhatWhere_trigger();


#ifdef __cplusplus
}
#endif

#endif /* DMAWRITEWHATWHERE_H */
