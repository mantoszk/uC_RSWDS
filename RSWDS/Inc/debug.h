#ifndef DEBUG_H_
#define DEBUG_H_
#include <stdint.h>

#define CRITICAL_MESSAGE 0
#define POSITIVE_MESSAGE 1
#define ONLY_OUTPUT 	 2

#define OUTPUT_HUMAN 	10
#define OUTPUT_MACHINE 	11

#define PSD_RAW			20
#define PSD_SI			21

static uint8_t trace_level = ONLY_OUTPUT;
static uint8_t output_type = OUTPUT_MACHINE;
static uint8_t psd_output = PSD_SI;

void trace(uint8_t message_trace_level, char *message);

void set_trace_level_critical();
void set_trace_level_positive();
void set_trace_level_output_only();

void set_output_type_human();
void set_output_type_machine();

void set_psd_value_raw();
void set_psd_value_si();

#endif
