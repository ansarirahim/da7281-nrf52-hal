/* DA7281 register model for host tests (see da7281_model.c). */
#ifndef DA7281_MODEL_H
#define DA7281_MODEL_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    unsigned writes;             /* data bytes written */
    unsigned ignored_writes;     /* to unmapped or read-only addresses */
    unsigned unmapped_reads;     /* reads of addresses not in the register map */
    unsigned unlocked_transfers; /* transfers made without the bus mutex held */
    unsigned split_reads;        /* a transfer came between a register pointer and its read */
    unsigned stop_before_read;   /* register pointer sent with STOP instead of repeated START */
    unsigned nacks_booting;      /* accesses during the boot window */
    unsigned null_instance;      /* TWI calls with a NULL instance */
    unsigned mutexes_created;
} model_stats_t;

void model_reset(void);
void model_apply_power(void);
void model_remove_power(void);
void model_set_addr_pins(uint8_t addr1, uint8_t addr0);
uint8_t model_reg(uint8_t addr);
double model_now_us(void);
const model_stats_t *model_stats(void);

#endif
