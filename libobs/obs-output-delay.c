/******************************************************************************
    Copyright (C) 2015 by Hugh Bailey <obs.jim@gmail.com>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#include <inttypes.h>
#include "obs-internal.h"

static inline bool delay_active(const struct obs_output *output)
{
	return os_atomic_load_bool(&output->delay_active);
}

static inline bool delay_capturing(const struct obs_output *output)
{
	return os_atomic_load_bool(&output->delay_capturing);
}

static inline void push_packet(struct obs_output *output,
			       struct encoder_packet *packet, uint64_t t)
{
	struct delay_data dd = {0};

	dd.msg = DELAY_MSG_PACKET;
	dd.ts = t;
	dd.copy = false;
	obs_encoder_packet_create_instance(&dd.packet, packet);

	pthread_mutex_lock(&output->delay_mutex);
	circlebuf_push_back(&output->delay_data, &dd, sizeof(dd));
	pthread_mutex_unlock(&output->delay_mutex);
}

static inline void process_delay_data(struct obs_output *output,
				      struct delay_data *dd)
{
	switch (dd->msg) {
	case DELAY_MSG_PACKET:
		if (!delay_active(output) || !delay_capturing(output))
			obs_encoder_packet_release(&dd->packet);
		else
			output->delay_callback(output, &dd->packet);
		break;
	case DELAY_MSG_START:
		obs_output_actual_start(output);
		break;
	case DELAY_MSG_STOP:
		obs_output_actual_stop(output, false, dd->ts);
		break;
	}
}

void obs_output_cleanup_delay(obs_output_t *output)
{
	struct delay_data dd;

	while (output->delay_data.size) {
		circlebuf_pop_front(&output->delay_data, &dd, sizeof(dd));
		if (dd.msg == DELAY_MSG_PACKET) {
			obs_encoder_packet_release(&dd.packet);
		}
	}

	output->active_delay_ns = 0;
	os_atomic_set_long(&output->delay_restart_refs, 0);
}

static inline bool pop_packet(struct obs_output *output, uint64_t t)
{
	uint64_t elapsed_time;
	struct delay_data dd;
	bool popped = false;
	bool del = false;
	bool preserve;

	/* ------------------------------------------------ */

	preserve = (output->delay_cur_flags & OBS_OUTPUT_DELAY_PRESERVE) != 0;

	pthread_mutex_lock(&output->delay_mutex);

	if (output->delay_data.size) {
		circlebuf_peek_front(&output->delay_data, &dd, sizeof(dd));
		elapsed_time = (t - dd.ts);

		output->delay_sec = (uint32_t)config_get_int(
			output->config, "Output", "DelaySec");
		output->active_delay_ns =
			(uint64_t)output->delay_sec * SEC_TO_NSEC;

		if (dd.msg == DELAY_MSG_PACKET && !dd.copy) {
			dd.packet.dts += output->diff_dts[dd.packet.type];
			dd.packet.pts += output->diff_dts[dd.packet.type];
		}

		if (preserve && output->reconnecting) {
			output->active_delay_ns = elapsed_time;
		} else if (elapsed_time > output->active_delay_ns) {
			circlebuf_pop_front(&output->delay_data, NULL,
					    sizeof(dd));
			save_packet(output, dd);
			if (elapsed_time >
				    output->active_delay_ns + SEC_TO_NSEC &&
			    dd.msg == DELAY_MSG_PACKET) {
				del = true;
				obs_encoder_packet_release(&dd.packet);
			} else {
				popped = true;
			}
		} else if (elapsed_time <
			   output->active_delay_ns - SEC_TO_NSEC) {
			load_packet(output, dd, &del, t);
		}
	}

	pthread_mutex_unlock(&output->delay_mutex);

	/* ------------------------------------------------ */

	if (popped)
		process_delay_data(output, &dd);

	return popped || del;
}

void save_packet(struct obs_output *output, struct delay_data dd)
{
	if (dd.msg != DELAY_MSG_PACKET || dd.copy)
		return;
	if (config_get_bool(output->config, "Output", "BufferEnable")) {
		if (output->record_first[0] || output->record_first[1]) {
			output->last_record = dd.ts;
			output->last_record_dts[dd.packet.type] = dd.packet.dts;
			output->record_first[dd.packet.type] = false;

			struct delay_data dd2 = {0};
			while (output->delay_data2.size) {
				circlebuf_pop_front(&output->delay_data2, &dd2,
						    sizeof(dd2));
				obs_encoder_packet_release(&dd2.packet);
			}
			while (output->delay_data3.size) {
				circlebuf_pop_front(&output->delay_data3, &dd2,
						    sizeof(dd2));
				obs_encoder_packet_release(&dd2.packet);
			}

			output->read_first[0] = true;
			output->read_first[1] = true;
		}
		if (!output->record_first[0] && !output->record_first[1]) {
			struct delay_data dd2 = {0};

			dd2.msg = dd.msg;
			dd2.ts = dd.ts - output->last_record;
			dd2.copy = true;
			obs_encoder_packet_create_instance(&dd2.packet,
							   &dd.packet);
			dd2.packet.dts_usec -= output->last_record / 1000;
			dd2.packet.dts -=
				output->last_record_dts[dd2.packet.type];
			dd2.packet.pts -=
				output->last_record_dts[dd2.packet.type];
			dd2.packet.sys_dts_usec -= output->last_record;
			circlebuf_push_back(&output->delay_data2, &dd2,
					    sizeof(dd));
			if (dd2.ts > 60 * 5 * SEC_TO_NSEC)
				config_set_bool(output->config, "Output",
						"BufferEnable", false);
		}
	} else {
		output->record_first[0] = true;
		output->record_first[1] = true;
	}
}

void load_packet(struct obs_output *output, struct delay_data dd, bool *del,
		 uint64_t t)
{
	if (dd.msg != DELAY_MSG_PACKET || dd.copy)
		return;
	if (!output->delay_data2.size) {
		if (!output->delay_data3.size)
			return;
		while (output->delay_data3.size) {
			struct delay_data dd2 = {0};
			circlebuf_pop_front(&output->delay_data3, &dd2,
					    sizeof(dd2));
			circlebuf_push_back(&output->delay_data2, &dd2,
					    sizeof dd2);
		}
		output->read_first[0] = true;
		output->read_first[1] = true;
	}
	if (output->read_first[0] || output->read_first[1]) {
		output->last_read = t - output->active_delay_ns;
		output->last_read_dts[dd.packet.type] = dd.packet.dts;
		output->last_diff_dts[dd.packet.type] =
			output->diff_dts[dd.packet.type];
		output->read_first[dd.packet.type] = false;
		circlebuf_pop_front(&output->delay_data, NULL, sizeof(dd));
		obs_encoder_packet_release(&dd.packet);
		*del = true;
	}
	if (!output->read_first[0] && !output->read_first[1]) {
		struct delay_data dd2 = {0};
		circlebuf_pop_front(&output->delay_data2, &dd2, sizeof(dd2));

		struct delay_data dd3 = dd2;
		obs_encoder_packet_create_instance(&dd3.packet, &dd2.packet);
		circlebuf_push_back(&output->delay_data3, &dd3, sizeof dd3);

		dd2.ts += output->last_read;

		output->diff_dts[dd2.packet.type] =
			dd2.packet.dts + output->last_diff_dts[dd2.packet.type];
		dd2.packet.dts += output->last_read_dts[dd2.packet.type];
		dd2.packet.pts += output->last_read_dts[dd2.packet.type];

		dd2.packet.dts_usec += output->last_read / 1000;
		dd2.packet.sys_dts_usec += output->last_read / 1000;
		circlebuf_push_front(&output->delay_data, &dd2, sizeof dd2);
		*del = true;
	}
}

void process_delay(void *data, struct encoder_packet *packet)
{
	struct obs_output *output = data;
	uint64_t t = os_gettime_ns();
	push_packet(output, packet, t);
	while (pop_packet(output, t))
		;
}

void obs_output_signal_delay(obs_output_t *output, const char *signal)
{
	struct calldata params;
	uint8_t stack[128];

	calldata_init_fixed(&params, stack, sizeof(stack));
	calldata_set_ptr(&params, "output", output);
	calldata_set_int(&params, "sec", output->active_delay_ns / 1000000000);
	signal_handler_signal(output->context.signals, signal, &params);
}

bool obs_output_delay_start(obs_output_t *output)
{
	struct delay_data dd = {
		.msg = DELAY_MSG_START,
		.ts = os_gettime_ns(),
		.copy = false,
	};

	if (!delay_active(output)) {
		bool can_begin = obs_output_can_begin_data_capture(output, 0);
		if (!can_begin)
			return false;
		if (!obs_output_initialize_encoders(output, 0))
			return false;
	}

	pthread_mutex_lock(&output->delay_mutex);
	circlebuf_push_back(&output->delay_data, &dd, sizeof(dd));
	pthread_mutex_unlock(&output->delay_mutex);

	os_atomic_inc_long(&output->delay_restart_refs);

	if (delay_active(output)) {
		do_output_signal(output, "starting");
		return true;
	}

	if (!obs_output_begin_data_capture(output, 0)) {
		obs_output_cleanup_delay(output);
		return false;
	}

	return true;
}

void obs_output_delay_stop(obs_output_t *output)
{
	struct delay_data dd = {
		.msg = DELAY_MSG_STOP,
		.ts = os_gettime_ns(),
		.copy = false,
	};

	pthread_mutex_lock(&output->delay_mutex);
	circlebuf_push_back(&output->delay_data, &dd, sizeof(dd));
	pthread_mutex_unlock(&output->delay_mutex);

	do_output_signal(output, "stopping");
}

void obs_output_set_delay(obs_output_t *output, uint32_t delay_sec,
			  uint32_t flags)
{
	if (!obs_output_valid(output, "obs_output_set_delay"))
		return;

	if ((output->info.flags & OBS_OUTPUT_ENCODED) == 0) {
		blog(LOG_WARNING,
		     "Output '%s': Tried to set a delay "
		     "value on a non-encoded output",
		     output->context.name);
		return;
	}

	output->delay_sec = delay_sec;
	output->delay_flags = flags;
}

uint32_t obs_output_get_delay(const obs_output_t *output)
{
	return obs_output_valid(output, "obs_output_set_delay")
		       ? output->delay_sec
		       : 0;
}

uint32_t obs_output_get_active_delay(const obs_output_t *output)
{
	return obs_output_valid(output, "obs_output_set_delay")
		       ? (uint32_t)(output->active_delay_ns / 1000000000ULL)
		       : 0;
}
