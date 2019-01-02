/*
 * pi5usb30216.c -- PI5USB30216 USB TYPE-C Controller device driver
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
#include "linux/i2c/pi5usb.h"

/******************************************************************************
* Register addresses
******************************************************************************/
/*In datasheet REG_DEVICE register is 1, define it to be 0 just for convenient to process.
*/
#define REG_DEVICE			       0
#define REG_CTR			              1
#define REG_INT			              2
#define REG_CC		                     3
/******************************************************************************
* Register bits
******************************************************************************/
/*    REG_CTR Control  (0x02h)    */
#define CTR_PORT_SETTING_SHIFT	       1
#define CTR_PORT_SETTING			       (0x03 << CTR_PORT_SETTING_SHIFT)

/*    REG_CC_STATUS (0x04)    */
#define CC_ORIENTATION                        0x03
#define CC_PORT_STATUS_SHIFT             2
#define CC_PORT_STATUS                        (0x07 << CC_PORT_STATUS_SHIFT)
#define CC_CHARGE_CUR_DETECT_SHIFT 5
#define CC_CHARGE_CUR_DETECT            (0x03 << CC_PORT_STATUS_SHIFT)
#define CC_VBUS_DETECT_SHIFT              7
#define CC_VBUS_DETECT                        (0x01 << CC_PORT_STATUS_SHIFT)

/******************************************************************************
* value assignment
******************************************************************************/
/*    REG_CTR Control  (0x02h)    */
#define CTR_INTERRUPT_MASK                   0x01
#define CTR_INTERRUPT_NOMASK               0x00

#define CTR_PORT_SETTING_SHIFT	       1
#define CTR_PORT_SETTING_DEVICE		(0x00 << CTR_PORT_SETTING_SHIFT)
#define CTR_PORT_SETTING_HOST		(0x01 << CTR_PORT_SETTING_SHIFT)
#define CTR_PORT_SETTING_DRP   		(0x02 << CTR_PORT_SETTING_SHIFT)

/*    REG_INT (0x03)    */
#define INT_ATTACH_EVENT                    0x01
#define INT_DETACH_EVENT                    0x02
#define ATTACH_DETACH_MASK               0x03

/*    REG_CC_STATUS (0x04)    */
#define CC_VBUS_DETECTED                    0x80
#define CC_VBUS_DETECT_MASK              0x80
#define CC_PORT_DEVICE_SHIFT             2
#define CC_PORT_DEVICE                        (0x01 << CC_PORT_STATUS_SHIFT)
#define CC_PORT_HOST                           (0x02 << CC_PORT_STATUS_SHIFT)
/******************************************************************************/
/* add try snk function for pericom */
u8 trySNK_flag = 0;

/* end */

enum pi5usb_orient{
	PI5USB_ORIENT_STANDBY = 0,
	PI5USB_ORIENT_CC1,
	PI5USB_ORIENT_CC2
};

struct pi5usb_info {
	struct i2c_client		*i2c;
	struct device *dev_t;
	struct mutex		mutex;
	struct class *fusb_class;
	int irq_gpio;
	int en_gpio;
	int switch_sel_gpio;
	enum pi5usb_type fusb_type;
	enum pi5usb_orient fusb_orient;

	struct pinctrl *pinctrl;
	struct pinctrl_state *typec_int_cfg;
};

/******************************************************************************/

/*begin - added by zhenghuan 2015-08-07 */
static int typec_pinctrl_init(struct pi5usb_info *info)
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

/* begin - For cc chips which does not support offset .
** Please use ¡°I2C Transport¡± API instead of ¡°I2C SMBus¡±
** API to communicate with PI5USB30216A if needed.
*/
static int cchip_read_block_data(struct i2c_client *i2c, int count,char* buf)
{
	struct pi5usb_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
       ret = i2c_master_recv(i2c, buf, count);
	mutex_unlock(&info->mutex);
	if (ret < 0) {
		printk("%s: ret(%d)\n", __func__, ret);
		return ret;
	}

	return 0;
}

static int cchip_write_block_data(struct i2c_client *i2c, int count,char* buf)
{
	struct pi5usb_info *info = i2c_get_clientdata(i2c);
	int ret;

	mutex_lock(&info->mutex);
       ret = i2c_master_send(i2c, buf, count);
	mutex_unlock(&info->mutex);
	if (ret < 0)
		printk("%s , ret(%d)\n", __func__, ret);

	return ret;
}
/* end  */

static ssize_t show_current_type(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct pi5usb_info *info = dev_get_drvdata(dev);

	switch(info->fusb_type){
		case PI5USB_TYPE_STANDBY:
			return sprintf(buf, "PI5USB_TYPE_STANDBY\n");
		case PI5USB_TYPE_DEVICE:
			return sprintf(buf, "PI5USB_TYPE_DEVICE\n");
		case PI5USB_TYPE_HOST:
			return sprintf(buf, "PI5USB_TYPE_HOST\n");
		case PI5USB_TYPE_VBUS:
			return sprintf(buf, "PI5USB_TYPE_VBUS\n");
		default:
			return sprintf(buf, "error type occured !\n");
	}
}

static DEVICE_ATTR(type, S_IRUGO, show_current_type, NULL);

/* Add read typec chip info node for Emode */
static ssize_t show_chip_info(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct pi5usb_info *info = dev_get_drvdata(dev);
    u8 valueRead[4] = {0x00};

    cchip_read_block_data(info->i2c, 4, valueRead);

    return sprintf(buf,  "Manufacture: Pericom \n"
                                "Chip Name  : PI5USB30216 \n"
                                "Device ID    : 0x%x \n",
                                valueRead[REG_DEVICE]);
}

static DEVICE_ATTR(chip_info, S_IRUGO, show_chip_info, NULL);
/* end */

static void pi5usb_check_type(struct pi5usb_info *info, u8 type)
{
       switch ((type & CC_PORT_STATUS) >> CC_PORT_STATUS_SHIFT)
       {
           case PI5USB_TYPE_STANDBY:
               info->fusb_type = PI5USB_TYPE_STANDBY;
	    break;
           case PI5USB_TYPE_DEVICE:
               info->fusb_type = PI5USB_TYPE_DEVICE;
	    break;
           case PI5USB_TYPE_HOST:
               info->fusb_type = PI5USB_TYPE_HOST;
	    break;
	    default:
	        printk("%s: error type occured !\n", __func__);
       }
}

static void pi5usb_check_orient(struct pi5usb_info *info, u8 status)
{
	u8 orient = (status & CC_ORIENTATION);
	info->fusb_orient = orient;
}

static irqreturn_t pi5usb_irq_thread(int irq, void *handle)
{
      char  valueWrite[2];
      u8 valueRead[4] = {0x00};
	struct pi5usb_info *info = (struct pi5usb_info *)handle;

      /* begin - the interrupt process is  recommend by pericom */
      valueWrite[REG_CTR] = CTR_PORT_SETTING_DRP | CTR_INTERRUPT_MASK;
      cchip_write_block_data(info->i2c, 2, valueWrite);//Mask interrupt
      msleep(30);//delay 30ms
      cchip_read_block_data(info->i2c, 4, valueRead);//read register data

      printk("%s: begin to enter irq fusb read value[0] = %u; value[1] = %u; value[2] = %u; value[3] = %u \n", 
	   	__func__, valueRead[0],valueRead[1],valueRead[2],valueRead[3]);
       /* end - the interrupt process is  recommend by pericom */

	if ((valueRead[REG_CC] & CC_PORT_STATUS) >> CC_PORT_STATUS_SHIFT == CC_PORT_DEVICE)
	{
             // VBUS
	      printk("%s: Special action, mask interrupt !\n", __func__);
             valueWrite[REG_CTR] = CTR_PORT_SETTING_DEVICE | CTR_INTERRUPT_MASK;
             cchip_write_block_data(info->i2c, 2, valueWrite);//Mask interrupt
	}
    	else if ((valueRead[REG_CC] & CC_PORT_STATUS) >> CC_PORT_STATUS_SHIFT == CC_PORT_HOST)
	{
             // VBUS
	      printk("%s: Special action, mask interrupt !\n", __func__);
             valueWrite[REG_CTR] = CTR_PORT_SETTING_HOST | CTR_INTERRUPT_MASK;
             cchip_write_block_data(info->i2c, 2, valueWrite);//Mask interrupt
	}

	if ((valueRead[REG_INT] & ATTACH_DETACH_MASK) == INT_ATTACH_EVENT)
	{
	       /* begin - check orient */
		pi5usb_check_orient(info, valueRead[REG_CC]);
		if (info->fusb_orient == PI5USB_ORIENT_CC1)
		{
		       /* TypeC USB2.0 do not need mux */
			//gpio_set_value(info->switch_sel_gpio, 1);
			printk("%s: Orient is PI5USB_ORIENT_CC1_CC\n", __func__);
		}
		else if (info->fusb_orient == PI5USB_ORIENT_CC2)
		{
		       /*TypeC USB2.0 do not need mux */
			//gpio_set_value(info->switch_sel_gpio, 0);
			printk("%s: Orient is PI5USB_ORIENT_CC2_CC\n", __func__);
		}
	       else
	       {
		   	printk("%s: Orient is PI5USB_ORIENT_STANDBY\n", __func__);
	       }
              /* end - check orient */

             /* begin - check attached type */
             pi5usb_check_type(info, valueRead[REG_CC]);
		if(info->fusb_type == PI5USB_TYPE_HOST)
		{
		      //SOURCE
			printk("%s: SOURCE (host or DFP) is attached \n", __func__);
		}
		else if(info->fusb_type == PI5USB_TYPE_DEVICE)
		{
                    if (trySNK_flag == 1)
                    {
                        // SINK
		           printk("%s: SINK (device or UFP) is attached! trySNK is finished! \n", __func__);
                    }
                    else//begin to trySNK
                    {
                         printk("%s: trySNK begin ! \n", __func__);
                         trySNK_flag = 1;
                         valueWrite[REG_CTR] = CTR_PORT_SETTING_DEVICE | CTR_INTERRUPT_MASK;
                         cchip_write_block_data(info->i2c, 2, valueWrite);//into UFP
                         msleep(500);
                         cchip_read_block_data(info->i2c, 4, valueRead);//read register data
                         printk(KERN_ERR "%s: In trySNK fusb read value[0] = %u; value[1] = %u; value[2] = %u; value[3] = %u \n", 
	   	            __func__, valueRead[0],valueRead[1],valueRead[2],valueRead[3]);
                         pi5usb_check_type(info, valueRead[REG_CC]);
                         if (info->fusb_type != PI5USB_TYPE_HOST)
                         {
                             valueWrite[REG_CTR] = CTR_PORT_SETTING_DRP | CTR_INTERRUPT_MASK;//For DRP
                             cchip_write_block_data(info->i2c, 2, valueWrite);
                             msleep(50);

                             valueWrite[REG_CTR] = CTR_PORT_SETTING_DEVICE | CTR_INTERRUPT_MASK;//For UFP
                             cchip_write_block_data(info->i2c, 2, valueWrite);
                             msleep(50);

                             valueWrite[REG_CTR] = CTR_PORT_SETTING_DRP | CTR_INTERRUPT_MASK;//For DRP
                             cchip_write_block_data(info->i2c, 2, valueWrite);
                             msleep(30);

                             //Read registers to clear read reg[0x03]
                             cchip_read_block_data(info->i2c, 4, valueRead);

                             // SINK
		                printk("%s: In trySNK SINK (device or UFP) is attached ! \n", __func__);
                         }
                         else if (info->fusb_type == PI5USB_TYPE_HOST)
                         {
                         	   //SOURCE
			          printk("%s: In trySNK SOURCE (host or DFP) is attached \n", __func__);
                         }
                    }
		}
             else
			pr_err("%s: Unexpected type occurred! \n", __func__);
		/* end - check attached type */

		if ((valueRead[REG_CC] & CC_VBUS_DETECT_MASK) == CC_VBUS_DETECTED)
	      {
                   // VBUS
	            printk("%s: After detect attached, then detect VBUS connect !\n", __func__);
		}
	}
	else if (((valueRead[REG_INT] & ATTACH_DETACH_MASK)  == INT_DETACH_EVENT)
                  && (valueRead[REG_CC] == 0x00))
	{
	       trySNK_flag = 0;
              valueWrite[REG_CTR] = CTR_PORT_SETTING_DRP | CTR_INTERRUPT_MASK;//For DRP
		pr_err("%s:  disconnect !\n", __func__);
	}
	else if ((valueRead[REG_CC] & CC_VBUS_DETECT_MASK) == CC_VBUS_DETECTED)
	{
             // VBUS
	      printk("%s: No attach and detach interrupt, but detect VBUS connect !\n", __func__);
	}
	else {
             pr_err("%s:  Wrong interrupt occur !!!\n", __func__);
       }

      /* begin - the interrupt process is  recommend by pericom */
      msleep(20);
      valueWrite[REG_CTR] = valueWrite[REG_CTR] & 0xfe;
      cchip_write_block_data(info->i2c, 2, valueWrite);
      /* end - the interrupt process is  recommend by pericom */

       return IRQ_HANDLED;
}

static int pi5usb_initialization(struct pi5usb_info *info)
{
      int ret;
      char  valueWrite[2];
      u8 valueRead[4];

      info->fusb_type = PI5USB_TYPE_STANDBY;

      /* begin - the chip initialization process is  recommend by pericom */
      msleep(10);
      valueWrite[REG_CTR] = CTR_INTERRUPT_MASK;
      ret = cchip_write_block_data(info->i2c, 2, valueWrite);
	if (ret < 0)
	   	return ret;

       msleep(30);

       valueWrite[REG_CTR] = CTR_PORT_SETTING_DRP | CTR_INTERRUPT_NOMASK;
       cchip_write_block_data(info->i2c, 2, valueWrite);
       msleep(10);

	ret = cchip_read_block_data(info->i2c, 4, valueRead);
	if (ret < 0)
	   	return ret;

       printk(KERN_ERR "%s: fusb read value[0] = %u; value[1] = %u; value[2] = %u; value[3] = %u \n", 
	   	__func__, valueRead[0],valueRead[1],valueRead[2],valueRead[3]);
       /* end - the chip initialization process is  recommend by pericom */
	return ret;
}

static int pi5usb_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
	int irq;
	struct pi5usb_info *info;
	struct device_node *np = client->dev.of_node;

       if (TypecChipSel != TYPEC_CHIP_SEL_NONE) {
	   	printk("%s():  typec chip has been registered!!!\n", __func__);
		goto error_device_register;
	}

       printk("pi5usb_probe():  start\n");
	info = kzalloc(sizeof(struct pi5usb_info), GFP_KERNEL);
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
	   	printk("%s: Fail to set chip enable gpio !\n", __func__);
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

	ret = pi5usb_initialization(info);
       if (ret < 0) {
              pr_err("%s: error pi5usb_initialization returned %d\n", __func__, ret);
		goto request_irq_failed;
       }

	ret = request_threaded_irq(client->irq, NULL, pi5usb_irq_thread,
			  IRQF_TRIGGER_LOW | IRQF_ONESHOT, "pi5usb_irq", info);

	if (ret){
	    dev_err(&client->dev, "error failed to reqeust IRQ\n");
	    goto request_irq_failed;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0){
	    dev_err(&client->dev, "failed to enable wakeup src %d\n", ret);
		goto enable_irq_failed;
	}

       TypecChipSel = TYPEC_CHIP_SEL_ONE;
	printk(" pi5usb_probe() success \n");
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

static int pi5usb_remove(struct i2c_client *client)
{
    struct pi5usb_info *info = i2c_get_clientdata(client);

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


static int  pi5usb_suspend(struct i2c_client *client, pm_message_t message)
{
	return 0;
}

static int  pi5usb_resume(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id pi5usb_id[] = {
		{.compatible = "pi5usb"},
		{},
};
MODULE_DEVICE_TABLE(of, pi5usb_id);


static const struct i2c_device_id pi5usb_i2c_id[] = {
	{ "pi5usb", 0 },
	{ }
};

static struct i2c_driver pi5usb_i2c_driver = {
	.driver = {
		.name = "pi5usb",
		.of_match_table = of_match_ptr(pi5usb_id),
	},
	.probe    = pi5usb_probe,
	.remove   = pi5usb_remove,
	.suspend  = pi5usb_suspend,
	.resume	  = pi5usb_resume,
	.id_table = pi5usb_i2c_id,
};

static __init int pi5usb_i2c_init(void)
{
	return i2c_add_driver(&pi5usb_i2c_driver);
}

static __exit void pi5usb_i2c_exit(void)
{
	i2c_del_driver(&pi5usb_i2c_driver);
}

module_init(pi5usb_i2c_init);
module_exit(pi5usb_i2c_exit);

MODULE_AUTHOR("zheng.huan35@zte.com.cn");
MODULE_DESCRIPTION("I2C bus driver for PI5USB USB Type-C");
MODULE_LICENSE("GPL v2");
