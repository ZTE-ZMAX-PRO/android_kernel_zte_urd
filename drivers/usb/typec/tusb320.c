/*
 * tusb320.c -- TUSB320 USB TYPE-C Controller device driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Revised by Bright Yang < bright.yang@farichildsemi.com>
 *
 */


#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/timer.h>
#include <linux/wakelock.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include "linux/i2c/tusb320.h"
#include "linux/i2c/fusb301.h"

/******************************************************************************
* Register addresses
******************************************************************************/
#define REG_ID		                    0x00
#define REG_ID_LENGTH	             6

#define REG_MOD	                    0x08
#define REG_INT			             0x09
#define REG_SET			             0x0A
#define REG_CTL                            0X45
/******************************************************************************
* Register bits
******************************************************************************/
/*    REG_MOD (0x08)    */
#define MOD_ACTIVE_CABLE_DETECTION	0X01
#define MOD_ACCESSORY_CONNECTED_SHIFT 1
#define MOD_ACCESSORY_CONNECTED        (0x07 << MOD_ACCESSORY_CONNECTED_SHIFT)
#define MOD_CURRENT_MODE_DETECT_SHIFT 4
#define MOD_CURRENT_MODE_DETECT        (0x03 << MOD_CURRENT_MODE_DETECT_SHIFT)
#define MOD_CURRENT_MODE_ADVERTISE_SHIFT 6
#define MOD_CURRENT_MODE_ADVERTISE   (0x03 << MOD_CURRENT_MODE_ADVERTISE_SHIFT)

/*    REG_INT (0x09)    */
#define INT_DRP_DUTY_CYCLE_SHIFT	       1
#define INT_DRP_DUTY_CYCLE			(0x03 << INT_DRP_DUTY_CYCLE_SHIFT)
#define INT_INTERRUPT_STATUS_SHIFT	4
#define INT_INTERRUPT_STATUS 			(0x01 << INT_INTERRUPT_STATUS_SHIFT)
#define INT_CABLE_DIR_SHIFT		       5
#define INT_CABLE_DIR 				       (0x01 << INT_CABLE_DIR_SHIFT)
#define INT_ATTACHED_STATE_SHIFT		6
#define INT_ATTACHED_STATE                    (0x03 << INT_ATTACHED_STATE_SHIFT)

/*    REG_SET (0x0A)    */
#define SET_I2C_SOFT_RESET_SHIFT	       3
#define SET_I2C_SOFT_RESET			       (0x01 << SET_I2C_SOFT_RESET_SHIFT)
#define SET_MODE_SELECT_SHIFT               4
#define SET_MODE_SELECT                          (0x03 << SET_MODE_SELECT_SHIFT)
#define SET_DEBOUNCE_SHIFT                     6
#define SET_DEBOUNCE                                (0x03 << SET_DEBOUNCE_SHIFT)

/*    REG_CTR (0x45)    */
#define CTR_DISABLE_RD_RP_SHIFT	       2
#define CTR_DISABLE_RD_RP			       (0x01 << CTR_DISABLE_RD_RP_SHIFT)

/******************************************************************************
* Constants
******************************************************************************/
/* TYPE */
#define ATTACHED_TYPE_NONE			0x00
#define ATTACHED_TYPE_SNK			       0x01
#define ATTACHED_TYPE_SRC   		       0x02
#define ATTACHED_TYPE_ACC			       0x03

/* REG_INT (0x09) */
#define INT_DRP_DUTY_30	                     0x00
#define INT_DRP_DUTY_40	                     0x02
#define INT_DRP_DUTY_50	                     0x04
#define INT_DRP_DUTY_60	                     0x06

/******************************************************************************/
enum tusb320_drp_toggle{
	TUSB320_TOGGLE_SNK35_SRC15 = 0,  // default
	TUSB320_TOGGLE_SNK30_SRC20,
	TUSB320_TOGGLE_SNK25_SRC25,
	TUSB320_TOGGLE_SNK20_SRC30,
};

enum tusb320_host_cur{
	TUSB320_HOST_CUR_NO = 0,  // no current
	TUSB320_HOST_CUR_80,  // default USB
	TUSB320_HOST_CUR_180,  // 1.5A
	TUSB320_HOST_CUR_330,  // 3A
};

enum tusb320_orient{
	TUSB320_ORIENT_CC1_CC = 0,
	TUSB320_ORIENT_CC2_CC
};

enum tusb320_config_modes{
	TUSB320_MODE_SRC = 0,
	TUSB320_MODE_SRC_ACC,
	TUSB320_MODE_SNK,
	TUSB320_MODE_SNK_ACC,
	TUSB320_MODE_DRP,
	TUSB320_MODE_DRP_ACC
};

struct tusb320_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct mutex		mutex;
	struct class *fusb_class;
	int irq_gpio;
	int en_gpio;
	int switch_sel_gpio;
	enum tusb320_type fusb_type;
	enum tusb320_orient fusb_orient;

	struct pinctrl *pinctrl;
	struct pinctrl_state *typec_int_cfg;

       struct timer_list prime_timer;
};

/* begin - Implement Typec TRY SNK function */
u8 Try_Snk_Attempt = 0x00; //Global Variable for Try.Snk attempt.

/* end */

/*begin - added by zhenghuan 2015-08-07 */
static int typec_pinctrl_init(struct tusb320_info *info)
{
       struct pinctrl_state *set_state;
	int ret = 0;

	info->pinctrl = devm_pinctrl_get(&info->i2c->dev);
	if (IS_ERR_OR_NULL(info->pinctrl)) {
		pr_err("%s(): Pinctrl not defined", __func__);
		ret = PTR_ERR(info->pinctrl);
             return ret;
	} else {
		printk("%s(): Using Pinctrl", __func__);

		set_state = pinctrl_lookup_state(info->pinctrl,"typec_int_cfg");
		if (IS_ERR_OR_NULL(set_state)) {
			pr_err("pinctrl lookup failed for typec_int_cfg");
			ret = PTR_ERR(set_state);
			goto pinctrl_fail;
		}

		printk("%s(): Pinctrl typec_int_cfg %p\n", __func__, set_state);
		info->typec_int_cfg = set_state;

		return ret;
	}
pinctrl_fail:
	info->pinctrl = NULL;
	return ret;
}
/*end - added by zhenghuan 2015-08-07 */

static int tusb320_read_reg(struct i2c_client *i2c, u8 reg, u8 *dest)
{
	struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_read_byte_data(i2c, reg);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		printk("%s:reg(0x%x), ret(%d)\n", __func__, reg, ret);
		return ret;
	}

	ret &= 0xff;
	*dest = ret;
	return 0;
}

static int tusb320_write_reg(struct i2c_client *i2c, u8 reg, u8 value)
{
	struct tusb320_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
	ret = i2c_smbus_write_byte_data(i2c, reg, value);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		printk("%s:reg(0x%x), ret(%d)\n",
				__func__, reg, ret);

	return ret;
}

static ssize_t show_current_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct tusb320_info *info = dev_get_drvdata(dev);

	switch(info->fusb_type){
		case TUSB320_TYPE_ACC:
			return sprintf(buf, "TUSB320_TYPE_AUDIO\n");
		case TUSB320_TYPE_SOURCE:
			return sprintf(buf, "TUSB320_SOURCE\n");
		case TUSB320_TYPE_SINK:
			return sprintf(buf, "TUSB320_TYPE_SINK\n");
		default:
			return sprintf(buf, "Not Connected\n");
	}
}

static DEVICE_ATTR(type, S_IRUGO, show_current_type, NULL);

/* Add read typec chip info node for Emode */
static ssize_t show_chip_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct tusb320_info *info = dev_get_drvdata(dev);
    char intr[REG_ID_LENGTH + 1];
    int i;

    for (i = 0; i <= REG_ID_LENGTH; i++)
    {
        tusb320_read_reg(info->i2c, i, &intr[REG_ID_LENGTH - i]);
    }

    return sprintf(buf, "Manufacture: TI \n"
                               "Chip Name   : %s \n", intr);
}

static DEVICE_ATTR(chip_info, S_IRUGO, show_chip_info, NULL);
/* end */

static void tusb320_check_type(struct tusb320_info *info, u8 type)
{
       switch ((type & INT_ATTACHED_STATE) >> INT_ATTACHED_STATE_SHIFT)
       {
           case ATTACHED_TYPE_NONE:
               info->fusb_type = TUSB320_TYPE_NONE;
	    break;
           case ATTACHED_TYPE_SNK:
               info->fusb_type = TUSB320_TYPE_SINK;
	    break;
           case ATTACHED_TYPE_SRC:
               info->fusb_type = TUSB320_TYPE_SOURCE;
	    break;
           case ATTACHED_TYPE_ACC:
               info->fusb_type = TUSB320_TYPE_ACC;
	    break;
	    default:
	        printk("%s: error type occured !\n", __func__);
       }
}

static void tusb320_check_orient(struct tusb320_info *info, u8 status)
{
	u8 orient = ((status & INT_CABLE_DIR)>>INT_CABLE_DIR_SHIFT);
	info->fusb_orient = orient;
}

static irqreturn_t tusb320_irq_thread(int irq, void *handle)
{
       struct tusb320_info *info = (struct tusb320_info *)handle;
       u8 intr;

	tusb320_write_reg(info->i2c, REG_INT, INT_INTERRUPT_STATUS); //Clear Interrupt status and set DRP duty cycle to default
       del_timer_sync(&info->prime_timer); //Clear_Stop_Watchdog_timer
       printk("%s: Clear_Stop_Watchdog_timer ! \n", __func__);
	tusb320_read_reg(info->i2c, REG_INT, &intr);
	tusb320_check_type(info, intr);//Read TUSB320 Attached state register
       printk("\n%s: type<%d> int(%u)\n", __func__,info->fusb_type, intr);

       if (info->fusb_type && Try_Snk_Attempt) //Try.Snk has been attempted.
       {
             Try_Snk_Attempt = 0x00; //Clear flag.
             tusb320_write_reg(info->i2c, REG_INT, INT_DRP_DUTY_30);//Change DRP duty Cycle
		tusb320_check_orient(info, intr);
		printk("%s: After TrySNK, orient is %d\n", __func__, info->fusb_orient);
		if (info->fusb_orient == TUSB320_ORIENT_CC1_CC)
		{
		       /* TypeC USB2.0 do not need mux */
			printk("%s: After TrySNK, orient is TUSB320_ORIENT_CC1_CC\n", __func__);
		}
		else 
             {
		       /*TypeC USB2.0 do not need mux */
			printk("%s: After TrySNK, orient is TUSB320_ORIENT_CC2_CC\n", __func__);
		}

	      if(info->fusb_type == TUSB320_TYPE_SOURCE)
	      {
		      //SOURCE
			printk("%s: After TrySNK, SOURCE (host or DFP) is attached \n", __func__);
		}
		else if(info->fusb_type == TUSB320_TYPE_SINK)
		{
		       // SINK
		       printk("%s: After TrySNK, SINK (device or UFP) is attached \n", __func__);
		}
		else if (info->fusb_type == TUSB320_TYPE_ACC)
		{
		       // ACC
		       printk("%s: After TrySNK, ACC is attached \n", __func__);
		}
       }
       else if (info->fusb_type && !Try_Snk_Attempt) //Try.Snk has NOT been attempted
       {
             if (info->fusb_type == TUSB320_TYPE_SINK) //DFP. Perform Try.SNK.
             {
             		// SINK
		       printk("%s: In Try.SNK, SINK (device or UFP) is attached \n", __func__);
             	       //Try.SINK
		       printk("%s: Perform Try.SNK. \n", __func__);
		       Try_Snk_Attempt = 0x01; //Set Try_Snk flag.
		       tusb320_write_reg(info->i2c, REG_INT, INT_DRP_DUTY_60);//Change DRP duty Cycle
		       tusb320_write_reg(info->i2c, REG_SET, SET_I2C_SOFT_RESET);//Set Soft Reset
		       msleep(25);//Wait 25ms.
		       tusb320_write_reg(info->i2c, REG_SET, SET_I2C_SOFT_RESET);//Set Soft Reset
                    mod_timer(&info->prime_timer, jiffies + msecs_to_jiffies(400)); //Start_WatchDog_Timer,400ms;
                     printk("%s: Start_WatchDog_Timer!mod_timer \n", __func__);
              }
	       else if (info->fusb_type == TUSB320_TYPE_SOURCE)
	       {
		        //SOURCE
			 printk("%s: In Try.SNK, SOURCE (host or DFP) is attached \n", __func__);
		 }
           	 else if (info->fusb_type == TUSB320_TYPE_ACC)
		 {
		        // ACC
		        printk("%s: In Try.SNK, ACC is attached \n", __func__);
		 }
       }
       else
            printk("%s:  disconnect !\n", __func__);

       return IRQ_HANDLED;
}

void tusb320_watchdog_handler (unsigned long  data)
{
    Try_Snk_Attempt = 0x00; //Clear flag
}

static int tusb320_initialization(struct tusb320_info *info)
{
      int ret;
	info->fusb_type = TUSB320_TYPE_NONE;

	/* clear Interrupt status modified by zhenghuan */
       ret = tusb320_write_reg(info->i2c, REG_INT, INT_INTERRUPT_STATUS);
	return ret;
}

static int tusb320_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int irq;
	struct tusb320_info *info;
	struct device_node *np = client->dev.of_node;

       if (TypecChipSel != TYPEC_CHIP_SEL_NONE) {
	   	printk("%s():  typec chip has been registered!!!\n", __func__);
		goto error_device_register;
	}

       printk("tusb320_probe():  start\n");
	info = kzalloc(sizeof(struct tusb320_info), GFP_KERNEL);
	info->i2c = client;

	ret = of_get_named_gpio(np, "cc,irq_gpio", 0);
	if (ret < 0) {
		pr_err("%s(): error invalid irq_gpio number  %d\n", __func__, ret);
		goto err_device_create_failed;
	}
	else {
		info->irq_gpio = ret;
		printk("%s(): valid irq_gpio number  %d\n", __func__, info->irq_gpio);
	}

       ret = of_get_named_gpio(np, "cc,en_gpio", 0);
       if (ret < 0) {
		pr_err("%s(): error invalid en_gpio number  %d\n", __func__, ret);
		goto err_device_create_failed;
	}
	else {
		info->en_gpio = ret;
		printk("%s(): valid en_gpio number  %d\n", __func__, info->en_gpio);
	}

       ret = gpio_direction_output(info->en_gpio, 1);
	if (ret < 0) {
	      printk("%s: fail to set the gpio %d\n", __func__, ret);
	      goto err_device_create_failed;
	}
	msleep(60);

	ret = typec_pinctrl_init(info);
	if (ret)
		printk("%s: error typec_pinctrl_init returned %d\n", __func__, ret);
	else
		printk("typec_pinctrl_init success\n");

	if (info->pinctrl) {
           ret = pinctrl_select_state(info->pinctrl,info->typec_int_cfg);
	    if (ret)
	        pr_err("%s(): error set typec_int_cfg \n", __func__);
           else
		 pr_err("%s(): set typec_int_cfg success \n", __func__);
	}

       irq = gpio_to_irq(info->irq_gpio);
	if (irq < 0) {
		pr_err("%s: error gpio_to_irq returned %d\n", __func__, irq);
	      goto	request_irq_failed;
	}
	else
             printk("%s : requesting IRQ %d\n", __func__, irq);

	client->irq = irq;
	i2c_set_clientdata(client, info);
	mutex_init(&info->mutex);

	info->fusb_class = class_create(THIS_MODULE, "typec");
	info->dev_t = device_create(info->fusb_class, NULL, 0, NULL, "typec_chip");
	device_create_file(info->dev_t, &dev_attr_type);
       device_create_file(info->dev_t, &dev_attr_chip_info);
	dev_set_drvdata(info->dev_t, info);

	ret = tusb320_initialization(info);
       if (ret < 0) {
              pr_err("%s: error tusb320_initialization returned %d\n", __func__, ret);
		goto request_irq_failed;
       }

	ret = request_threaded_irq(client->irq, NULL, tusb320_irq_thread,
			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, "tusb320_irq", info);

	if (ret){
	    dev_err(&client->dev, "error failed to reqeust IRQ\n");
	    goto request_irq_failed;
	}

       /* setup 400ms timer */
      	setup_timer(&info->prime_timer, tusb320_watchdog_handler, (unsigned long)info);

	ret = enable_irq_wake(client->irq);
	if (ret < 0){
	    dev_err(&client->dev, "failed to enable wakeup src %d\n", ret);
		goto enable_irq_failed;
	}
       TypecChipSel = TYPEC_CHIP_SEL_ONE;
	printk(" tusb320_probe() success \n");
	return 0;

enable_irq_failed:
	free_irq(client->irq,NULL);
request_irq_failed:
	device_remove_file(info->dev_t, &dev_attr_type);
	device_destroy(info->fusb_class, 0);
	class_destroy(info->fusb_class);
	mutex_destroy(&info->mutex);
	i2c_set_clientdata(client, NULL);
err_device_create_failed:
       kfree(info);
       info = NULL;
error_device_register:
	return ret;
}

static int tusb320_remove(struct i2c_client *client)
{
    struct tusb320_info *info = i2c_get_clientdata(client);

    if (client->irq) {
        disable_irq_wake(client->irq);
        free_irq(client->irq, info);
    }
    device_remove_file(info->dev_t, &dev_attr_type);
    device_destroy(info->fusb_class, 0);
    class_destroy(info->fusb_class);
    mutex_destroy(&info->mutex);
    i2c_set_clientdata(client, NULL);

    kfree(info);
    return 0;
}


static int  tusb320_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  tusb320_resume(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id tusb320_id[] = {
		{.compatible = "tusb320"},
		{},
};
MODULE_DEVICE_TABLE(of, tusb320_id);


static const struct i2c_device_id tusb320_i2c_id[] = {
	{ "tusb320", 0 },
	{ }
};

static struct i2c_driver tusb320_i2c_driver = {
	.driver = {
		.name = "tusb320",
		.of_match_table = of_match_ptr(tusb320_id),
	},
	.probe    = tusb320_probe,
	.remove   = tusb320_remove,
	.suspend  = tusb320_suspend,
	.resume	  = tusb320_resume,
	.id_table = tusb320_i2c_id,
};

static __init int tusb320_i2c_init(void)
{
	return i2c_add_driver(&tusb320_i2c_driver);
}

static __exit void tusb320_i2c_exit(void)
{
	i2c_del_driver(&tusb320_i2c_driver);
}

module_init(tusb320_i2c_init);
module_exit(tusb320_i2c_exit);

MODULE_AUTHOR("zheng.huan35@zte.com.cn");
MODULE_DESCRIPTION("I2C bus driver for TUSB320 USB Type-C");
MODULE_LICENSE("GPL v2");
