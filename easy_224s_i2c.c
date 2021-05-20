// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  easy_mxt224s.c - Atmel AT42easy_mxt224s Touch Sense Controller
 *
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/delay.h>

#define EASY_MXT224S_NAME        "easy_mXT224S"

/* Polling Rate */
static int scan_rate = 50;
module_param(scan_rate,
int, 0644);
MODULE_PARM_DESC(scan_rate,
"Polling rate in times/sec. Default = 100");
static unsigned long delay;

struct easy_mxt224s_data {
    struct i2c_client *client;
    struct input_dev *input;
    struct delayed_work dwork;
};

static int easy_mxt224s_read_touch_coordinats(struct easy_mxt224s_data *easy_mxt224s) {
    u8 buf[16];
    int ret;
    int x, y, f, p;
    ret = i2c_master_recv(easy_mxt224s->client, buf, 16);
    if (ret == 16) {
        //pr_debug("read easy_mxt224s touch_coordinat array: %u %u %u %u\n", buf[0], buf[1], buf[2], buf[3]);
        if (buf[0] != 0xff) {
            x = ((buf[1] << 2) + ((buf[3] >> 6) & 3));
            y = ((buf[2] << 2) + ((buf[3] >> 2) & 3));
            f = ((0xf0 & buf[0]) >> 4);
            p = (0x01 & buf[0]);
            pr_debug("read easy_mxt224s touch_coordinat x:%u y:%u f:%u p:%u\n", x, y, f, p);

            input_report_abs(easy_mxt224s->input, ABS_X, x);
            input_report_abs(easy_mxt224s->input, ABS_Y, y);
            input_report_key(easy_mxt224s->input, BTN_TOUCH, p);
            input_sync(easy_mxt224s->input);
        }
        ret = 0;
    }
    return ret;
}

static void easy_mxt224s_schedule_read(struct easy_mxt224s_data *easy_mxt224s) {
    schedule_delayed_work(&easy_mxt224s->dwork, delay);
}

static void easy_mxt224s_worker(struct work_struct *work) {
    struct easy_mxt224s_data *easy_mxt224s = container_of(work,
    struct easy_mxt224s_data, dwork.work);
    //dev_dbg(&easy_mxt224s->client->dev, "worker\n");
    easy_mxt224s_read_touch_coordinats(easy_mxt224s);
    easy_mxt224s_schedule_read(easy_mxt224s);
}

static int easy_mxt224s_create_input_device(struct easy_mxt224s_data *easy_mxt224s) {
    struct device *dev = &easy_mxt224s->client->dev;
    struct input_dev *input;
    int error;

    input = devm_input_allocate_device(dev);
    if (!input) {
        dev_err(dev, "failed to allocate input device\n");
        return -ENOMEM;
    }
    easy_mxt224s->input = input;

    input->name = "easy_mXT224S Touchscreen";
    input->id.bustype = BUS_I2C;
    input_set_abs_params(input, ABS_MT_POSITION_X, 0,
                         4711, 0, 0);
    input_set_abs_params(input, ABS_MT_POSITION_Y, 0,
                         4711, 0, 0);
    input_abs_set_res(input, ABS_MT_POSITION_X, 4711);
    input_abs_set_res(input, ABS_MT_POSITION_Y, 4711);

    input_set_abs_params(input, ABS_MT_TOUCH_MAJOR,
                         0, 4711, 0, 0);
    input_set_abs_params(input, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);
    input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
    __set_bit(EV_ABS, input->evbit);
    __set_bit(ABS_X, input->absbit);
    __set_bit(ABS_Y, input->absbit);
    __set_bit(ABS_PRESSURE, input->absbit);
    __set_bit(EV_SYN, input->evbit);
    __set_bit(EV_KEY, input->evbit);
    __set_bit(BTN_TOUCH, input->keybit);

    error = input_register_device(input);
    if (error) {
        dev_err(dev, "failed to register input device: %d\n", error);
        return error;
    }
    return 0;
}

static int easy_mxt224s_read_information(struct easy_mxt224s_data *easy_mxt224s) {
    u8 reg = 0x01; // mem address
    u8 buf[16];
    int ret;
    struct i2c_msg msg[2] = {
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg,
            },
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = I2C_M_RD,
                    .len = 16,
                    .buf = buf,
            }
    };
    ret = i2c_transfer(easy_mxt224s->client->adapter, msg, 2);
    if (ret == 2) {
        pr_debug("read easy_mxt224s firmware version: %u %u %u %u\n", buf[0], buf[1], buf[2], buf[3]);
        ret = 0;
    }
    return ret;
}

static int easy_mxt224s_set_mode(struct easy_mxt224s_data *easy_mxt224s, u8 mode) {
    u8 reg = 0x00; // mem address
    int ret;
    struct i2c_msg msg[2] = {
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg,
            },
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &mode,
            }
    };
    ret = i2c_transfer(easy_mxt224s->client->adapter, msg, 2);
    if (ret == 2) {
        pr_debug("set easy_mxt224s mode: 0x%02x set \n", mode);
        ret = 0;

    }
    return ret;
}

static int easy_mxt224s_reset(struct easy_mxt224s_data *easy_mxt224s) {
    u8 reg = 0x00; // mem address
    u8 rst = 0x80; // reset
    int ret;
    struct i2c_msg msg[2] = {
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg,
            },
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &rst,
            }
    };
    ret = i2c_transfer(easy_mxt224s->client->adapter, msg, 2);
    if (ret == 2) {
        pr_debug("reset easy_mxt224s\n");
        ret = 0;

    }
    return ret;
}

static int easy_mxt224s_probe(struct i2c_client *client,
                              const struct i2c_device_id *id) {
    struct easy_mxt224s_data *easy_mxt224s;
    struct input_dev *input;
    int error;

    dev_dbg(&client->dev, "adapter=%d, client irq: %d\n",
            client->adapter->nr, client->irq);

    /* Check if the I2C function is ok in this adaptor */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
        return -ENXIO;

    easy_mxt224s = kzalloc(sizeof(struct easy_mxt224s_data), GFP_KERNEL);
    input = input_allocate_device();
    if (!easy_mxt224s || !input) {
        dev_err(&client->dev, "insufficient memory\n");
        error = -ENOMEM;
        return error;
    }

    easy_mxt224s->client = client;
    i2c_set_clientdata(client, easy_mxt224s);
    easy_mxt224s->input = input;


    error = easy_mxt224s_create_input_device(easy_mxt224s);
    if (error) {
        dev_err(&client->dev, "failed to crate input device: %d\n", error);
        return error;
    }

    /* reset the controller */
    error = easy_mxt224s_reset(easy_mxt224s);
    if (error) {
        dev_err(&client->dev, "failed to reset the controller: %d\n", error);
        return error;
    }

    /* set Mode to 0x01 */
    error = easy_mxt224s_set_mode(easy_mxt224s, 0x01);
    if (error) {
        dev_err(&client->dev, "failed to read information: %d\n", error);
        return error;
    }

    /* read Information Register (Mode 0x01) */
    msleep(5);
    error = easy_mxt224s_read_information(easy_mxt224s);
    if (error) {
        dev_err(&client->dev, "failed to read information: %d\n", error);
        return error;
    }

    /* set Mode to 0x00 */
    error = easy_mxt224s_set_mode(easy_mxt224s, 0x00);
    if (error) {
        dev_err(&client->dev, "failed to read information: %d\n", error);
        return error;
    }

    INIT_DELAYED_WORK(&easy_mxt224s->dwork, easy_mxt224s_worker);
    delay = msecs_to_jiffies(MSEC_PER_SEC / scan_rate);
    easy_mxt224s_schedule_read(easy_mxt224s);
    return 0;
}

static int easy_mxt224s_remove(struct i2c_client *client) {
    struct easy_mxt224s_data *easy_mxt224s = i2c_get_clientdata(client);
    cancel_delayed_work_sync(&easy_mxt224s->dwork);
    input_unregister_device(easy_mxt224s->input);
    kfree(easy_mxt224s);
    return 0;
}

static const struct i2c_device_id easy_mxt224s_dev_id[] = {
        {EASY_MXT224S_NAME, 0,},
        {}
};

MODULE_DEVICE_TABLE(i2c, easy_mxt224s_dev_id
);

static struct i2c_driver easy_mxt224s_driver = {
        .driver = {
                .name    = EASY_MXT224S_NAME,
        },
        .id_table    = easy_mxt224s_dev_id,
        .probe       = easy_mxt224s_probe,
        .remove      = easy_mxt224s_remove,
};

module_i2c_driver(easy_mxt224s_driver);

MODULE_AUTHOR("Andreas Lüthi <andreas.luethi@gmx.net>");
MODULE_DESCRIPTION("easy I2C mXT224S Driver (from DATA MODUL)");
MODULE_LICENSE("GPL");
