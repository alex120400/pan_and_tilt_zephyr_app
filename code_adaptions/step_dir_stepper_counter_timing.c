/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/counter.h>
#include "step_dir_stepper_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(step_dir_stepper);

// original
// static void step_counter_top_interrupt(const struct device *dev, void *user_data)
// {
// 	ARG_UNUSED(dev);
// 	struct step_dir_stepper_common_data *data = user_data;
	
// 	LOG_INF("Entered Top callback"); // ALEX change

// 	stepper_handle_timing_signal(data->dev);
// }

// ALEX change
static void step_counter_alarm_interrupt(const struct device *counter_dev,
                                         uint8_t chan_id,
                                         uint32_t ticks,
                                         void *user_data)
{
    ARG_UNUSED(counter_dev);
    ARG_UNUSED(chan_id);
    ARG_UNUSED(ticks);
	LOG_INF("Entered Alarm callback");
    struct step_dir_stepper_common_data *data = user_data;
    const struct device *dev = data->dev;
    const struct step_dir_stepper_common_config *config = dev->config;

    /* generate step pulse */
    stepper_handle_timing_signal(dev);

    /* schedule next alarm */
    struct counter_alarm_cfg alarm_cfg = {
        .callback = step_counter_alarm_interrupt,
        .user_data = data,
        .ticks = data->counter_interval_ticks,
        .flags = 0
    };
	
    counter_set_channel_alarm(config->counter, 0, &alarm_cfg);
}
// change end

int step_counter_timing_source_update(const struct device *dev,
				      const uint64_t microstep_interval_ns)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	
	if (microstep_interval_ns == 0) {
		return -EINVAL;
	}

	// original:
	// int ret;
	// data->counter_top_cfg.ticks = DIV_ROUND_UP(
	// 	counter_get_frequency(config->counter) * microstep_interval_ns, NSEC_PER_SEC);

	// /* Lock interrupts while modifying counter settings */
	// int key = irq_lock();

	// ret = counter_set_top_value(config->counter, &data->counter_top_cfg);

	// irq_unlock(key);

	// if (ret != 0) {
	// 	LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
	// 	return ret;
	// }

	// ALEX change:
    uint64_t freq = counter_get_frequency(config->counter);

    data->counter_interval_ticks =
        DIV_ROUND_UP(freq * microstep_interval_ns, NSEC_PER_SEC);
	LOG_INF("Updated ticks to: %llu", data->counter_interval_ticks);
	// change end

	return 0;
}

int step_counter_timing_source_start(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	ret = counter_start(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to start counter: %d", ret);
		return ret;
	}
	
	// ALEX change
	LOG_INF("Started timer");
	struct counter_alarm_cfg alarm_cfg = {
        .callback = step_counter_alarm_interrupt,
        .user_data = data,
        .ticks = data->counter_interval_ticks,
        .flags = 0
    };

    ret = counter_set_channel_alarm(config->counter, 0, &alarm_cfg);
    if (ret) {
        LOG_ERR("Failed to set alarm: %d", ret);
        return ret;
    }
	// change end

	data->counter_running = true;

	return 0;
}

int step_counter_timing_source_stop(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	// ALEX change
	ret = counter_cancel_channel_alarm(config->counter, 0);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to cancel the alarm: %d", ret);
		return ret;
	}
	// change end

	ret = counter_stop(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to stop counter: %d", ret);
		return ret;
	}

	data->counter_running = false;

	return 0;
}

bool step_counter_timing_source_needs_reschedule(const struct device *dev)
{
	ARG_UNUSED(dev);
	return false;
}

bool step_counter_timing_source_is_running(const struct device *dev)
{
	struct step_dir_stepper_common_data *data = dev->data;

	return data->counter_running;
}

int step_counter_timing_source_init(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;

	if (!device_is_ready(config->counter)) {
		LOG_ERR("Counter device is not ready");
		return -ENODEV;
	}

	// original
	// data->counter_top_cfg.callback = step_counter_top_interrupt;
	// data->counter_top_cfg.user_data = data;
	// data->counter_top_cfg.flags = 0;
	// data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);

	// ALEX change:
	data->counter_interval_ticks = counter_us_to_ticks(config->counter, 1000000);
	// change end

	return 0;
}

const struct stepper_timing_source_api step_counter_timing_source_api = {
	.init = step_counter_timing_source_init,
	.update = step_counter_timing_source_update,
	.start = step_counter_timing_source_start,
	.needs_reschedule = step_counter_timing_source_needs_reschedule,
	.stop = step_counter_timing_source_stop,
	.is_running = step_counter_timing_source_is_running,
};
