/*
 *  easy_mxt224s.c - easy I2C mXT224S Driver (from DATA MODUL)
 *
 */

#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/delay.h>

#define EASY_MXT224S_NAME      "easy_mXT224S"
#define MODE_COORDINATES        0x0000
#define MODE_REGISTER_RW        0x0001
#define MODE_CONFIGURATION      0x0002

#define MEM_RESET               0x0000
#define MEM_MODE                0x0000
#define MEM_ATMEL_FW_VERSION    0x0001 // 4 bytes
#define MEM_EASYI2C_FW_VERSION  0x0005 // 2 bytes
#define MEM_SENSOR_ART_NUM      0x0007 // 10 bytes
#define MEM_SENSITIVITY         0x0080
#define MEM_ORIENTATION         0x0081
#define MEM_INIT_MOVE_HISTER    0x0082
#define MEM_NEXT_MOVE_HISTER    0x0083
#define MEM_TOUCH_DEDECT_INTEGR 0x0084
#define MEM_NUM_REPORT_TOUCH    0x0085
#define MEM_TOUCH_AUTOM_CALIB   0x0086
#define MEM_X_RESOL_LOW         0x0087
#define MEM_X_RESOL_HIGH        0x0088
#define MEM_Y_RESOL_LOW         0x0089
#define MEM_Y_RESOL_HIGH        0x008A

#define RESET                   0x0080 // in MEM_MODE

/* Polling Rate */
static int scan_rate = 30;
module_param(scan_rate,
int, 0644);
MODULE_PARM_DESC(scan_rate,
"Polling rate in times/sec. Default = 30");
static unsigned long delay;

/* Geometry */
static int width = 480;
module_param(width,
int, 0644);
MODULE_PARM_DESC(width,
"Window width in pixel. Default = 480");
static int height = 640;
module_param(height,
int, 0644);
MODULE_PARM_DESC(height,
"Window height in pixel. Default = 640");
static int orientation = 3;
module_param(orientation,
int, 0644);
MODULE_PARM_DESC(orientation,
"Orientation of x/y axis. Default = 3, invert x, switch x and y");

/* Sensitivity threshold */
static int sensitivity = 25;
module_param(sensitivity,
int, 0644);
MODULE_PARM_DESC(sensitivity,
"Touch sensitivity, less is more sensitive. Default = 25");

struct easy_mxt224s_data {
    struct i2c_client *client;
    struct input_dev *input;
    struct delayed_work dwork;
};

static int easy_mxt224s_read_touch_coordinates(struct easy_mxt224s_data *easy_mxt224s) {
    // assume in mode MODE_COORDINATES
    u8 buf[16];
    int ret;
    int x, y, f, p;
    ret = i2c_master_recv(easy_mxt224s->client, buf, 16);
    if (ret == 16) {
        if (buf[0] != 0xff) {
            x = ((buf[1] << 2) + ((buf[3] >> 6) & 3));
            y = ((buf[2] << 2) + ((buf[3] >> 2) & 3));
            f = ((0xf0 & buf[0]) >> 4);
            p = (0x01 & buf[0]);
            //pr_debug("read easy_mxt224s touch_coordinates x:%u y:%u f:%u p:%u\n", x, y, f, p);
            input_report_key(easy_mxt224s->input, BTN_TOUCH, p);
            input_report_abs(easy_mxt224s->input, ABS_X, x);
            input_report_abs(easy_mxt224s->input, ABS_Y, y);
            //input_report_abs(easy_mxt224s->input, ABS_PRESSURE, 200);
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

    easy_mxt224s_read_touch_coordinates(easy_mxt224s);
    easy_mxt224s_schedule_read(easy_mxt224s);
}

static int easy_mxt224s_create_input_device(struct easy_mxt224s_data *easy_mxt224s) {
    struct device *dev = &easy_mxt224s->client->dev;
    struct input_dev *input;
    int error;

    input = input_allocate_device();
    if (!input) {
        dev_err(dev, "failed to allocate input device\n");
        return -ENOMEM;
    }
    easy_mxt224s->input = input;

    input->name = "easy_mXT224S Touchscreen";
    input->id.bustype = BUS_I2C;
    input_set_abs_params(input, ABS_X, 0, width, 0, 0);
    input_set_abs_params(input, ABS_Y, 0, height, 0, 0);
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
    // assume in mode MODE_REGISTER_RW
    u8 reg = MEM_ATMEL_FW_VERSION;
    u8 buf[16];
    int ret;
    struct i2c_msg msg[] = {
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg,
            },
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = I2C_M_RD,
                    .len = ARRAY_SIZE(buf),
                    .buf = buf,
            }
    };

    ret = i2c_transfer(easy_mxt224s->client->adapter, msg, ARRAY_SIZE(msg));
    if (ret == ARRAY_SIZE(msg)) {
        pr_debug("read easy_mxt224s info: Atmel fw version: %u %u %u %u\n", buf[0], buf[1], buf[2], buf[3]);
        pr_debug("read easy_mxt224s info: easyI2C fw version: %u %u\n", buf[4], buf[5]);
        pr_debug("read easy_mxt224s info: sensor article nr: %u %u %u %u %u %u %u %u %u %u\n", buf[6], buf[7],
                 buf[8], buf[9],
                 buf[10], buf[11],
                 buf[12], buf[13],
                 buf[14], buf[15]);
        ret = 0;
        msleep(5);
    }
    return ret;
}

static int easy_mxt224s_read_registers(struct easy_mxt224s_data *easy_mxt224s) {
    // assume in mode MODE_REGISTER_RW
    u8 reg = MEM_SENSITIVITY;
    u8 buf[11];
    int ret;
    struct i2c_msg msg[] = {
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &reg,
            },
            {
                    .addr = easy_mxt224s->client->addr,
                    .flags = I2C_M_RD,
                    .len = ARRAY_SIZE(buf),
                    .buf = buf,
            }
    };

    ret = i2c_transfer(easy_mxt224s->client->adapter, msg, ARRAY_SIZE(msg));
    if (ret == ARRAY_SIZE(msg)) {
        pr_debug("read easy_mxt224s reg: Sensitivity threshold: %u\n", buf[0]);
        pr_debug("read easy_mxt224s reg: Orientation: %u\n", buf[1]);
        pr_debug("read easy_mxt224s reg: Initial movement hysteresis: %u\n", buf[2]);
        pr_debug("read easy_mxt224s reg: Next movement hysteresis: %u\n", buf[3]);
        pr_debug("read easy_mxt224s reg: Touch detection integration: %u\n", buf[4]);
        pr_debug("read easy_mxt224s reg: Number of reported touches: %u\n", buf[5]);
        pr_debug("read easy_mxt224s reg: Touch automatic calibration: %u\n", buf[6]);
        pr_debug("read easy_mxt224s reg: X-resolution LOW: %u\n", buf[7]);
        pr_debug("read easy_mxt224s reg: X-resolution HIGH: %u\n", buf[8]);
        pr_debug("read easy_mxt224s reg: Y-resolution LOW: %u\n", buf[9]);
        pr_debug("read easy_mxt224s reg: Y-resolution HIGH: %u\n", buf[10]);
        ret = 0;
        msleep(5);
    }
    return ret;
}

static int
easy_mxt224s_set_touch_coordinates(struct easy_mxt224s_data *easy_mxt224s, int width, int height, int orientation) {
    // assume in mode MODE_REGISTER_RW
    int x_res_high, x_res_low, y_res_low, y_res_high, ret;
    char buf1[5];
    char buf2[] = {
            MEM_ORIENTATION, orientation
    };
    char buf3[] = {
            MEM_SENSITIVITY, sensitivity
    };
    if (orientation % 2) {
        x_res_high = (height - 1) / 256;
        x_res_low = (height - 1) - (x_res_high * 256);
        y_res_high = (width - 1) / 256;
        y_res_low = (width - 1) - (y_res_high * 256);
    } else {
        x_res_high = (width - 1) / 256;
        x_res_low = (width - 1) - (x_res_high * 256);
        y_res_high = (height - 1) / 256;
        y_res_low = (height - 1) - (y_res_high * 256);
    }
    buf1[0] = MEM_X_RESOL_LOW;
    buf1[1] = x_res_low;
    buf1[2] = x_res_high;
    buf1[3] = y_res_low;
    buf1[4] = y_res_high;

    ret = i2c_master_send(easy_mxt224s->client, (char *) buf1, ARRAY_SIZE(buf1));
    if (ret == ARRAY_SIZE(buf1)) {
        pr_debug("easy_mxt224s set touch coordinates: x_res_high=%u x_res_low=%u y_res_high=%u y_res_low=%u\n",
                 x_res_high, x_res_low, y_res_high, y_res_low);
    }
    msleep(5);
    ret = i2c_master_send(easy_mxt224s->client, (char *) buf2, ARRAY_SIZE(buf2));
    if (ret == ARRAY_SIZE(buf2)) {
        pr_debug("easy_mxt224s set touch orientation: orientation=%u\n",
                 orientation);
        ret = 0;
    }
    msleep(5);
    ret = i2c_master_send(easy_mxt224s->client, (char *) buf3, ARRAY_SIZE(buf3));
    if (ret == ARRAY_SIZE(buf3)) {
        pr_debug("easy_mxt224s set sensitivity threshold: sensitivity=%u\n",
                 orientation);
        ret = 0;
    }
    return ret;
}

static int easy_mxt224s_set_mode(struct easy_mxt224s_data *easy_mxt224s, u8 mode) {
    char buf[] = {
            MEM_MODE, mode
    };
    int ret;

    ret = i2c_master_send(easy_mxt224s->client, (char *) buf, 2);
    if (ret == ARRAY_SIZE(buf)) {
        pr_debug("set easy_mxt224s mode: 0x%02x set \n", mode);
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
    easy_mxt224s->input = input;
    easy_mxt224s->client = client;
    i2c_set_clientdata(client, easy_mxt224s);

    error = easy_mxt224s_set_mode(easy_mxt224s, MODE_REGISTER_RW);
    if (error) {
        dev_err(&client->dev, "failed to set mode: %d\n", error);
        return error;
    }
    error = easy_mxt224s_read_information(easy_mxt224s);
    if (error) {
        dev_err(&client->dev, "failed to read information: %d\n", error);
        return error;
    }

    error = easy_mxt224s_set_touch_coordinates(easy_mxt224s, width, height, orientation);
    if (error) {
        dev_err(&client->dev, "failed to set touch coordinates: %d\n", error);
        return error;
    }

    error = easy_mxt224s_read_registers(easy_mxt224s);
    if (error) {
        dev_err(&client->dev, "failed to read registers: %d\n", error);
        return error;
    }

    error = easy_mxt224s_set_mode(easy_mxt224s, MODE_COORDINATES);
    if (error) {
        dev_err(&client->dev, "failed to set mode: %d\n", error);
        return error;
    }

    error = easy_mxt224s_create_input_device(easy_mxt224s);
    if (error) {
        dev_err(&client->dev, "failed to crate input device: %d\n", error);
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
MODULE_ALIAS("easy_mxt224s_i2c-driver");
