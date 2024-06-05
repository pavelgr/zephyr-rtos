/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 * Copyright (c) 2021, pck
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/logging/log.h>

#include "mpu9150.h"

LOG_MODULE_DECLARE(MPU9150, CONFIG_SENSOR_LOG_LEVEL);

static void mpu9150_handle_interrupts(const struct device *dev)
{
	struct mpu9150_data *data = dev->data;
	//const struct mpu9150_config *cfg = dev->config;
	//int ret;

	if (data->drdy_handler != NULL) {
		data->drdy_handler(dev, data->drdy_trigger);
	}

    // Re-enable interrupt pin
	//ret = gpio_pin_interrupt_configure_dt(&cfg->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
	//if (ret < 0) {
		//LOG_ERR("Enabling gpio interrupt failed with err: %d", ret);
	//}
}

#ifdef CONFIG_MPU9150_TRIGGER_OWN_THREAD
K_KERNEL_STACK_MEMBER(mpu9150_thread_stack, CONFIG_MPU9150_THREAD_STACK_SIZE);
struct k_thread mpu9150_thread;

static void mpu9150_thread_main(void *arg1, void *unused1, void *unused2)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
    
	const struct device *dev = (const struct device *)arg1;
	struct mpu9150_data *data = &dev->data;

	while (1) {
		k_sem_take(&data->drdy_sem, K_FOREVER);
		mpu9150_handle_interrupts(dev);
	}
}
#endif

#ifdef CONFIG_MPU9150_TRIGGER_GLOBAL_THREAD
static void mpu9150_work_handler(struct k_work *work)
{
	struct mpu9150_data *data =
		CONTAINER_OF(work, struct mpu9150_data, drdy_work);

	mpu9150_handle_interrupts(data->dev);    
}
#endif

static void mpu9150_gpio_callback(const struct device *port,
				  struct gpio_callback *cb, 
                  uint32_t pin)
{
    
    ARG_UNUSED(port);
	ARG_UNUSED(pin);

	//int ret;
	struct mpu9150_data *data = CONTAINER_OF(cb, struct mpu9150_data, drdy_callback);
	//const struct mpu9150_config *cfg = data->dev->config;

    // Disable interrupt pin
	//ret = gpio_pin_interrupt_configure_dt(&cfg->drdy_gpio, GPIO_INT_DISABLE);
	//if (ret < 0) {
		//LOG_ERR("Disabling gpio interrupt failed with err: %d", ret);
		//return;
	//}

#if defined(CONFIG_MPU9150_TRIGGER_OWN_THREAD)
	k_sem_give(&data->drdy_sem);
#elif defined(CONFIG_MPU9150_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&data->drdy_work);
#elif defined(CONFIG_MPU9150_TRIGGER_DIRECT)
	mpu9150_handle_interrupts(data->dev);
#endif
}

int mpu9150_trigger_set(
            const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct mpu9150_data *data = dev->data;
	const struct mpu9150_config *cfg = dev->config;
	int ret;

#ifdef CONFIG_PM_DEVICE
	enum pm_device_state state;

	(void)pm_device_state_get(dev, &state);
	if (state != PM_DEVICE_STATE_ACTIVE) {
		return -EBUSY;
	}
#endif

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	data->drdy_handler = handler;
	data->drdy_trigger = trig;

	// Enable data ready interrupt
    ret = mpu9150_reg_field_update(
            dev,
		    MPU9150_REG_INT_ENABLE,
		    MPU9150_INT_ENABLE_DATA_RDY_EN_MASK,
		    (handler != NULL) << MPU9150_INT_ENABLE_DATA_RDY_EN_POS);
    if (ret < 0) {
		LOG_ERR("Failed to enable data ready interrupt.");
		return ret;
	}

    // Enable interrupt pin
	ret = gpio_pin_interrupt_configure(
                cfg->drdy_gpio.port,
                cfg->drdy_gpio.pin,
                GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to enable interrupt");
		return ret;
	}

	return 0;
}

int mpu9150_trigger_init(const struct device *dev)
{
	struct mpu9150_data *data = dev->data;
	const struct mpu9150_config *cfg = dev->config;
	int ret;

	if (!gpio_is_ready_dt(&cfg->drdy_gpio)) {
		LOG_ERR("Interrupt pin is not ready.");
		return -EIO;
	}

#if defined(CONFIG_MPU9150_TRIGGER_OWN_THREAD)
	ret = k_sem_init(&data->drdy_sem, 0, K_SEM_MAX_LIMIT);
	if (ret < 0) {
		LOG_ERR("Failed to enable semaphore");
		return ret;
	}

	k_thread_create(
        &mpu9150_thread, 
        mpu9150_thread_stack,
        CONFIG_MPU9150_THREAD_STACK_SIZE,
        mpu9150_thread_main, 
        (void *)dev,
        NULL,
        NULL,
        K_PRIO_COOP(CONFIG_MPU9150_THREAD_PRIORITY),
        0,
        K_NO_WAIT);
#elif defined(CONFIG_MPU9150_TRIGGER_GLOBAL_THREAD)
	data->drdy_work.handler = mpu9150_work_handler;
#endif

#if defined(CONFIG_MPU9150_TRIGGER_GLOBAL_THREAD) || \
	defined(CONFIG_MPU9150_TRIGGER_DIRECT)
	data->dev = dev;
#endif

	//gpio_pin_interrupt_configure(
                //cfg->drdy_gpio.port,
                //cfg->drdy_gpio.pin,
                //GPIO_INT_DISABLE);

	ret = gpio_pin_configure(
                cfg->drdy_gpio.port,
                cfg->drdy_gpio.pin,
                GPIO_INPUT | cfg->drdy_gpio.dt_flags);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt pin.");
		return ret;
	}

	gpio_init_callback(
                &data->drdy_callback,
                mpu9150_gpio_callback,
                BIT(cfg->drdy_gpio.pin));

	ret = gpio_add_callback(cfg->drdy_gpio.port, &data->drdy_callback);
	if (ret < 0) {
		LOG_ERR("Failed to set gpio callback.");
		return ret;
	}

	return 0;
}
