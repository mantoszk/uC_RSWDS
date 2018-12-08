#include "debug.h"

void trace(uint8_t message_trace_level, char *message) {
	if (message_trace_level <= trace_level)
		printf("%s\r\n", message);
}

void set_trace_level_critical() {
	trace_level = CRITICAL_MESSAGE;
}

void set_trace_level_positive() {
	trace_level = POSITIVE_MESSAGE;
}

void set_trace_level_output_only() {
	trace_level = ONLY_OUTPUT;
}

void set_output_type_human()
{
	output_type = OUTPUT_HUMAN;
}

void set_output_type_machine()
{
	output_type = OUTPUT_MACHINE;
}

void set_psd_value_raw()
{
	psd_output = PSD_RAW;
}

void set_psd_value_si()
{
	psd_output = PSD_SI;
}
