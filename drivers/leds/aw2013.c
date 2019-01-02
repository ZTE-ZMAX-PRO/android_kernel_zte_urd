/*
 * leds-aw2013
 */

#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/i2c.h>
#include	<linux/input.h>
#include	<linux/uaccess.h>
#include	<linux/workqueue.h>
#include	<linux/irq.h>
#include	<linux/gpio.h>
#include	<linux/interrupt.h>
#include	<linux/slab.h>
#include	<linux/pm.h>
#include	<linux/module.h>
#include	<linux/regulator/consumer.h>
#include	<linux/of_gpio.h>
#include	<linux/sensors.h>
#include    <linux/leds.h>
#include    <linux/ctype.h>

#define    RSTR         0x00
#define    AW_GCR       0x01
#define    LCTR         0x30
#define    LCFG0_EN     0x31
#define    LCFG1_EN     0x32
#define    LCFG2_EN     0x33
#define    AW_PWM0      0x34
#define    AW_PWM1      0x35
#define    AW_PWM2      0x36
#define    LED0T0       0x37
#define    LED0T1       0x38
#define    LED0T2       0x39

#define    LED1T0       0x3A
#define    LED1T1       0x3B
#define    LED1T2       0x3C

#define    LED2T0       0x3D
#define    LED2T1       0x3E
#define    LED2T2       0x3F

#define    AW_IADR      0x77

#define	   I2C_RETRY_DELAY		5
#define	   I2C_RETRIES          5

#define	   AW2013_REG_MAX       14




enum aw2013_mode {
	AW2013_MODE_ALL_ON = 0,	         /* on */
	AW2013_MODE_BLINK_1,		     /* Breath slow. For charger,  T1=T3=T4=4.16s..T2=0.26s */
	AW2013_MODE_BLINK_2,             /* Breath fast. For power on£¬ T1=T2=T4=T3=0.52s.  */
	AW2013_MODE_BLINK_3,             /* Breath between BLINK_1 and BLINK_2. For missed message . T1=T3=1.04s,  T2=T4=0.52s,  */
	AW2013_MODE_BLINK_4,             /* T0=T2=0s,       T1=0.5s, T3=0.5s.  */
	AW2013_MODE_BLINK_5,             /* for notification 6s led baseline requirement in 201505 ,T1=T2=T3=1.04s T4=2.08 period=5.2s  */
	AW2013_MODE_BLINK_6,             /*Blue led breath for p852a01*/
	AW2013_MODE_BLINK_7,
	AW2013_MODE_BREATH_RED,          /*T0=T2=1s,       T1=2s, T3=2s. RED*/
	AW2013_MODE_BREATH_GREEN, 		/*T0=T2=1s,       T1=2s, T3=2s. GREEN*/
	AW2013_MODE_BREATH_BLUE,		/*T0=T2=1s,       T1=2s, T3=2s. BLUE*/
	AW2013_MODE_BLINK_RED,			/*T0=0,T1=2s,       T2=0s, T3=2s. RED*/
	AW2013_MODE_BLINK_GREEN,		/*T0=0,T1=2s,       T2=0s, T3=2s. GREEN*/
	AW2013_MODE_BLINK_BLUE,			/*T0=0,T1=2s,       T2=0s, T3=2s. BLUE*/
	AW2013_MODE_RED_ON ,            /*red led on for p852a01*/
	AW2013_MODE_BLUE_ON ,           /*blue led on for p852a01*/
	AW2013_MODE_GREEN_ON ,          /*green led on for p852a01*/
	AW2013_MODE_MID_ON ,
	AW2013_MODE_SIDE_ON ,	
	AW2013_MODE_ALL_OFF,             /* all off */
	AW2013_MODE_BLINK_RED_3p5,
	AW2013_MODE_BLINK_GREEN_3p5
};

struct aw2013_mode_map {
	const char *mode;
	enum aw2013_mode mode_val;
};
static struct aw2013_mode_map mode_map[] = {
	{ "all_on",  AW2013_MODE_ALL_ON },
	{ "blink_1", AW2013_MODE_BLINK_1 },
	{ "blink_2", AW2013_MODE_BLINK_2 },
	{ "blink_3", AW2013_MODE_BLINK_3 },
	{ "blink_4", AW2013_MODE_BLINK_4 },
	{ "blink_5", AW2013_MODE_BLINK_5 },
	{ "blink_6", AW2013_MODE_BLINK_6 },
	{ "blink_7", AW2013_MODE_BLINK_7 },
	{ "breath_red", AW2013_MODE_BREATH_RED },
	{ "breath_green", AW2013_MODE_BREATH_GREEN },
	{ "breath_blue", AW2013_MODE_BREATH_BLUE },
	{ "blink_red", AW2013_MODE_BLINK_RED },
	{ "blink_green", AW2013_MODE_BLINK_GREEN },
	{ "blink_blue", AW2013_MODE_BLINK_BLUE },
	{ "red",     AW2013_MODE_RED_ON },
	{ "blue",    AW2013_MODE_BLUE_ON },
	{ "green",   AW2013_MODE_GREEN_ON },
	{ "mid_on",  AW2013_MODE_MID_ON },
	{ "side_on", AW2013_MODE_SIDE_ON },	
	{ "all_off", AW2013_MODE_ALL_OFF },
	{ "blink_red_3p5", AW2013_MODE_BLINK_RED_3p5 },
	{ "blink_green_3p5", AW2013_MODE_BLINK_GREEN_3p5 },
};

enum aw2013_pause {
	AW2013_PAUSE = 0,
	AW2013_PAUSE_0 = 130,
	AW2013_PAUSE_1 = 260,
	AW2013_PAUSE_2 = 520,
	AW2013_PAUSE_3 = 1024,
	AW2013_PAUSE_4 = 2080,
	AW2013_PAUSE_5 = 4160,
	AW2013_PAUSE_6 = 8320,
	AW2013_PAUSE_7 = 16640,
};

struct aw2013_pause_map {
	const char *pause;
	enum aw2013_pause pause_val;
};
static struct aw2013_pause_map pause_map[] = {
	{ "0", AW2013_PAUSE_0 },
	{ "1", AW2013_PAUSE_1 },
	{ "2", AW2013_PAUSE_2 },
	{ "3", AW2013_PAUSE_3 },
	{ "4", AW2013_PAUSE_4 },
	{ "5", AW2013_PAUSE_5 },
	{ "6", AW2013_PAUSE_6 },
	{ "7", AW2013_PAUSE_7 },

};


struct aw_2013_platform_data  {
	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);

	int gpio_shdn;
	u8 brt_val;
};
struct aw2013_data {
	struct i2c_client *client;
	struct led_classdev cdev;
	struct regulator *regulator;
	atomic_t enabled;
	enum aw2013_mode mode;
	enum aw2013_pause pause_rising;
	enum aw2013_pause pause_falling;
	enum aw2013_pause pause_hi;
	enum aw2013_pause pause_lo;
	int  red_brightness;
	int  green_brightness;
	int  blue_brightness;
	int gpio_rst;
#ifdef DEBUG
	u8 reg_addr;
#endif
};
static int aw2013_get_mode_from_str(const char *str)
{
	int i;
	
	for (i = 0; i < ARRAY_SIZE(mode_map); i++)
		if (sysfs_streq(str, mode_map[i].mode))
			return mode_map[i].mode_val;

	return -EINVAL;
}
static int aw2013_i2c_write(struct aw2013_data *drvdata, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = drvdata->client->addr,
			.flags = drvdata->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};
	//printk(":===shuangdan i2c write: drvdata %p , client is %p, adapter %p\n", drvdata, drvdata->client, drvdata->client->adapter);

	do {
		err = i2c_transfer(drvdata->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&drvdata->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int aw2013_i2c_read(struct i2c_client *client, u8 reg, u8 *value)
{
	int tmp;

	tmp = i2c_smbus_read_byte_data(client, reg);
	if (tmp < 0)
		return tmp;

	*value = tmp;

	return 0;
}

static int aw2013_register_write(struct aw2013_data *drvdata, u8 *buf,
		u8 reg_address, u8 new_value)
{
	int err = -1;
	u8 buf_temp[7];
	/* Sets configuration register at reg_address
	 *  NOTE: this is a straight overwrite  */
		buf_temp[0] = reg_address;
		buf_temp[1] = new_value;
		err = aw2013_i2c_write(drvdata, buf_temp, 1);
		if (err < 0)
			return err;
	return err;
}

static int aw2013_i2c_test(struct aw2013_data *drvdata)
{
	int err = 0;
	u8 val = 0;
	err = aw2013_i2c_read(drvdata->client, AW_IADR, &val);
	//printk("%s: == shuangdan  AW_IADR 0x77 is 0x%x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, RSTR, &val);
	//printk("%s: == shuangdan   RSTR 0x00 is 0x%x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, AW_PWM0, &val);
	//printk("%s: == shuangdan   AW_PWM0 0x34 is 0x%x \n",__func__,val);


	err = aw2013_i2c_read(drvdata->client, AW_PWM1, &val);
	//printk("%s: == shuangdan  AW_PWM1 0x35 is 0x%x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, LCTR, &val);//LCTR
	//printk("%s: == shuangdan  LCTR 0x30 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, AW_GCR, &val);//GCR
	//printk("%s: == shuangdan  GCR 0x01 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, LCFG0_EN, &val);
	//printk("%s: == shuangdan  LCFG0_EN 0x31 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, LCFG1_EN, &val);
	//printk("%s: == shuangdan  LCFG1_EN 0x32 is %x \n",__func__,val);
 	err = aw2013_i2c_read(drvdata->client, LED0T0, &val);
        printk("%s: == shuangdan  LED0T0 0x37 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, LED0T1, &val);
	//printk("%s: == shuangdan  LED0T1 0x38 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, LED0T2, &val);
	//printk("%s: == shuangdan  LED0T2 0x39 is %x \n",__func__,val);

	return err;
}

static int aw2013_hw_init(struct aw2013_data *drvdata)
{
	int 	err = 0;
        u8 buf[7];
    
        aw2013_register_write(drvdata,buf,RSTR,0x55);
	msleep(8);
	
	return err;
}
static int aw2013_write_init(struct aw2013_data *drvdata)
{
    u8      buf[7];
    int 	err = 0;
    aw2013_register_write(drvdata,buf,RSTR,0x55);
    aw2013_register_write(drvdata,buf,AW_GCR,0x01);//GCR
    aw2013_register_write(drvdata,buf,LCTR,0x07);//LED Control register
    return err;
}
static int aw2013_set_registers(struct aw2013_data *drvdata)
{
	int err = 0;
	u8 buf[7];
	
	switch (drvdata->mode) {
	case AW2013_MODE_ALL_ON:
		printk("%s: entry  AW2013_MODE_ALL_ON\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x61);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x61);// enable the register of LED1
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x61);// enable the register of LED1
		aw2013_register_write(drvdata,buf,AW_PWM0,0x80);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM1,0x80);
		aw2013_register_write(drvdata,buf,AW_PWM2,0x80);
	    break;
	case AW2013_MODE_BLINK_1:    //Breath slow. For charger
	    printk("%s: entry  AW2013_MODE_BLINK_1\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x32);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 0.52s
		aw2013_register_write(drvdata,buf,LED0T1,0x34);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0xff);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_2:	//Breath fast. For power on.
		printk("%s: entry  AW2013_MODE_BLINK_2\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x33);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 1.04s
		aw2013_register_write(drvdata,buf,LED0T1,0x35);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 4.16s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0xff);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_3:  //Breath between BLINK_1 and BLINK_2. For missed message .
		printk("%s: entry  AW2013_MODE_BLINK_3\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x32);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 0.52s
		aw2013_register_write(drvdata,buf,LED0T1,0x32);//LED0 T3:falling 011 = 1.04s	 T4:001 brightness keeper 0.52s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0xff);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_4:
		 printk("%s: entry  AW2013_MODE_BLINK_4\n",__func__);
		 aw2013_write_init(drvdata);
		 aw2013_register_write(drvdata,buf,LCFG0_EN,0x11);// enable the register of LEDO
		 aw2013_register_write(drvdata,buf,AW_PWM0,0xff);//bringhtness=256
		 break;
    case AW2013_MODE_BLINK_5:
		printk("%s: entry  AW2013_MODE_BLINK_5\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x33);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 1.04s
		aw2013_register_write(drvdata,buf,LED0T1,0x35);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0xff);//bringhtness=256
		break;
    case AW2013_MODE_BLINK_6:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_6\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED2T0,0x32);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 0.52s
		aw2013_register_write(drvdata,buf,LED2T1,0x34);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED2T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM2,0x80);//bringhtness=256
		break;
    case AW2013_MODE_BLINK_7:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_7\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED2T0,0x02);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 0.52s
		aw2013_register_write(drvdata,buf,LED2T1,0x01);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED2T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM2,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BREATH_RED:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BREATH_RED\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x33);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper1.04s
		aw2013_register_write(drvdata,buf,LED0T1,0x34);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BREATH_GREEN:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BREATH_GREEN\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED1T0,0x33);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 1.04s
		aw2013_register_write(drvdata,buf,LED1T1,0x34);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED1T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM1,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BREATH_BLUE:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BREATH_BLUE\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED2T0,0x33);//LED0 T1:rising 011 = 1.04s	 T2:011 brightness keeper 1.04s
		aw2013_register_write(drvdata,buf,LED2T1,0x34);//LED0 T3:falling 011 = 1.04s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED2T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM2,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_RED:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_RED\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x03);//LED0 T1:rising 011 = 0.13s	 T2:011 brightness keeper1.04s
		aw2013_register_write(drvdata,buf,LED0T1,0x04);//LED0 T3:falling 011 = 0.13s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_GREEN:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_7\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED1T0,0x03);//LED0 T1:rising 011 = 0.13s	 T2:011 brightness keeper 1.04s
		aw2013_register_write(drvdata,buf,LED1T1,0x04);//LED0 T3:falling 011 = 0.13s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED1T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM1,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_BLUE:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_BLUE\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED2T0,0x03);//LED0 T1:rising 011 = 0.13s	 T2:011 brightness keeper 1.04s
		aw2013_register_write(drvdata,buf,LED2T1,0x04);//LED0 T3:falling 011 = 0.13s	 T4:001 dark keeper 2.08s 
		aw2013_register_write(drvdata,buf,LED2T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM2,0x80);//bringhtness=256
		break;
    case AW2013_MODE_RED_ON:
		printk("%s: entry  AW2013_MODE_RED_ON\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x61);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,AW_PWM0,0x5F);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM1,0x00);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM2,0x00);//bringhtness=128
	    break;
    case AW2013_MODE_GREEN_ON:
		printk("%s: entry  AW2013_MODE_GREEN_ON\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x61);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,AW_PWM0,0x00);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM1,0x1F);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM2,0x00);//bringhtness=128
	    break;
	case AW2013_MODE_BLUE_ON:
		printk("%s: entry  AW2013_MODE_BLUE_ON\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x61);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,AW_PWM0,0x00);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM1,0x00);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM2,0x80);//bringhtness=128
	    break;
    case AW2013_MODE_MID_ON:
		printk("%s: entry  AW2013_MODE_MID_ON\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x61);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x61);// enable the register of LED1
		aw2013_register_write(drvdata,buf,AW_PWM0,0xff);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM1,0x00);
		break;
	case AW2013_MODE_SIDE_ON:
		printk("%s: entry  AW2013_MODE_SIDE_ON\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x61);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x61);// enable the register of LED1
		aw2013_register_write(drvdata,buf,AW_PWM0,0x00);//bringhtness=128
		aw2013_register_write(drvdata,buf,AW_PWM1,0xff);
		aw2013_register_write(drvdata,buf,AW_PWM2,0x00);
	    break;
	case AW2013_MODE_ALL_OFF:
		pr_debug("%s: entry  AW2013_MODE_ALL_OFF\n",__func__);
		aw2013_register_write(drvdata,buf,RSTR,0x55);
		aw2013_register_write(drvdata,buf,AW_GCR,0x00);//GCR 

				
		aw2013_register_write(drvdata,buf,AW_PWM0,0x00);//bringhtness=0
		aw2013_register_write(drvdata,buf,AW_PWM1,0x00); 
		aw2013_register_write(drvdata,buf,AW_PWM2,0x00); 
		break;
	case AW2013_MODE_BLINK_RED_3p5://Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_RED_3p5\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED0T0,0x02);//LED0 T1:rising 011 = 0.13s	 T2:011 brightness keeper0.52s
		aw2013_register_write(drvdata,buf,LED0T1,0x05);//LED0 T3:falling 011 = 0.13s	 T4:001 dark keeper 4.16s 
		aw2013_register_write(drvdata,buf,LED0T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM0,0x80);//bringhtness=256
		break;
	case AW2013_MODE_BLINK_GREEN_3p5:    //Breath slow. For charger
		printk("%s: entry  AW2013_MODE_BLINK_GREEN_3p5\n",__func__);
		aw2013_write_init(drvdata);
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x71);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LED1T0,0x02);//LED0 T1:rising 011 = 0.13s	 T2:011 brightness keeper 0.52s
		aw2013_register_write(drvdata,buf,LED1T1,0x05);//LED0 T3:falling 011 = 0.13s	 T4:001 dark keeper 4.16s 
		aw2013_register_write(drvdata,buf,LED1T2,0x40);//blink always
		aw2013_register_write(drvdata,buf,AW_PWM1,0x80);//bringhtness=256
		break;
	default:
		printk("%s: entry  default\n",__func__);
		//shuangan begin
		aw2013_register_write(drvdata,buf,RSTR,0x55);
		aw2013_register_write(drvdata,buf,LED0T0,0x33);
		aw2013_register_write(drvdata,buf,LED0T1,0x33);
		aw2013_register_write(drvdata,buf,LED0T2,0x42);
		aw2013_register_write(drvdata,buf,LED1T0,0x33);
		aw2013_register_write(drvdata,buf,LED1T1,0x33);
		aw2013_register_write(drvdata,buf,LED1T2,0x4f);
		//shuangan end
        break;
	}
	//err = aw2013_i2c_test(drvdata);
	return err;
}
static int aw2013_power_Off(struct aw2013_data *drvdata)
{
    u8 buf[7];
    int err = -1;

    printk("%s: entry  \n",__func__);
	  
    aw2013_register_write(drvdata,buf,RSTR,0x55); 
    msleep(200);

    return err;
}

static ssize_t attr_aw2013_set_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
    struct aw2013_data *drvdata = dev_get_drvdata(dev);
    unsigned long val;

    if (strict_strtoul(buf, 10, &val))
		return -EINVAL;
	
    drvdata->mode = AW2013_MODE_ALL_OFF;
    aw2013_set_registers(drvdata);
    if (val)
	{
		//enable control reset pin
		atomic_set(&drvdata->enabled, 1);		
	}
	else
	{
		//disable control reset pin	
		atomic_set(&drvdata->enabled, 0);		
	}

	return size;
}

static ssize_t attr_aw2013_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata = dev_get_drvdata(dev);
	int val = atomic_read(&drvdata->enabled);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);

}


static ssize_t attr_aw2013_get_mode(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->mode;

	pr_debug("%s: entry drvdata->mode =%d \n",__func__,drvdata->mode);
		
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
			
}
static ssize_t attr_aw2013_set_mode(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	//struct aw2013_data *drvdata = i2c_get_clientdata(aw2013_client);	
	int err = -1;
	int mode = AW2013_MODE_ALL_ON;
	int val = atomic_read(&drvdata->enabled);
	pr_debug("%s:attr_aw2013_set_mode +++++ \n",__func__);
	
	if (val == 0)
	{
	    printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
		return AW2013_MODE_ALL_OFF;
	}
	mode = aw2013_get_mode_from_str(buf);
	if (mode < 0) {
		dev_err(dev, "Invalid mode\n");
		return mode;
	}
	
	drvdata->mode = mode;
	
	err =aw2013_set_registers(drvdata);
	if (err) {
		dev_err(dev, "Setting %s Mode failed :%d\n", buf, err);
		return err;
	}

	return sizeof(drvdata->mode);

}
static ssize_t aw2013_get_pause_value(const char *str)
{
	int i;
	
	for (i = 0; i < ARRAY_SIZE(pause_map); i++)
		if (sysfs_streq(str, pause_map[i].pause))
			return pause_map[i].pause_val;

	return -EINVAL;
}
static ssize_t aw2013_set_pause_value(int value)
{
    int a;
		
    switch (value){
	case AW2013_PAUSE_0:
		a = 0; break;
	case AW2013_PAUSE_1:
		a = 1; break;
	case AW2013_PAUSE_2:
		a = 2; break;
	case AW2013_PAUSE_3:
		a = 3; break;
	case AW2013_PAUSE_4:
		a = 4; break;
	case AW2013_PAUSE_5:
		a = 5; break;
	case AW2013_PAUSE_6:
		a = 6; break;
	case AW2013_PAUSE_7:
	default:
		a = 7; break;
	}
	return a;
}

static ssize_t aw2013_pause_rising_get(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	
	int val = drvdata->pause_rising;

        pr_debug("%s: entry drvdata->pause_rising =%d \n",__func__,drvdata->pause_rising);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}


static ssize_t aw2013_pause_rising_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
    	int err = -1 ; 
    	struct aw2013_data *drvdata= dev_get_drvdata(dev);
    	u8 a;
   	int enable = atomic_read(&drvdata->enabled);
	u8 val = 0;
	u8 buf2[7];
	int pause_rising;
	
	if (enable == 0)
	{
	    printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
	    return -1;
	}
	pause_rising = aw2013_get_pause_value(buf);
	err = aw2013_i2c_read(drvdata->client, LED0T0, &val);
	drvdata->pause_rising = pause_rising;
	pr_debug("%s: shuangdan++++ LED0T0 0x37 value = %x ,pause_rising = %d \n",__func__,val,drvdata->pause_rising);
	
	a = aw2013_set_pause_value(drvdata->pause_rising);
	val = (a<<4) | (val & 0x7);
	aw2013_register_write(drvdata,buf2,LED0T0,val);
	
	err = aw2013_i2c_read(drvdata->client, LED0T0, &val);
	printk("%s: shuangdan  LED0T0 0x37 value = %x ,pause_rising T1= %d.%02d s,\n",__func__,val,13*(1<<a)/100,13*(1<<a)%100);
    	return size;
}


static ssize_t aw2013_pause_falling_get(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->pause_falling;
	pr_debug("%s: entry drvdata->pause_falling =%d \n",__func__,drvdata->pause_falling);

	return snprintf(buf, sizeof(val) + 2, "falling_get = %d\n", val);
}

static ssize_t aw2013_pause_falling_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
    int err = -1 ; 
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	u8 a;
	int enable = atomic_read(&drvdata->enabled);
	u8 val = 0;
	u8 buf2[7];
	int pause_falling;
	
	if (enable == 0)
	{
		printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
		return -1;
	}
	pause_falling = aw2013_get_pause_value(buf);
	err = aw2013_i2c_read(drvdata->client, LED0T1, &val);
	drvdata->pause_falling = pause_falling;
	pr_debug("%s: shuangdan++++ LED0T1 0x38 value = %x ,pause_falling = %d \n",__func__,val,drvdata->pause_falling);
	
	a = aw2013_set_pause_value(drvdata->pause_falling);
	val = (a<<4) | (val & 0x7);
	aw2013_register_write(drvdata,buf2,LED0T1,val);
	
	err = aw2013_i2c_read(drvdata->client, LED0T1, &val);
    	printk("%s: shuangdan LED0T0 0x38 value = %x ,pause_falling T1= %d.%02d s,\n",__func__,val,13*(1<<a)/100,13*(1<<a)%100);
    	return size;
}

static ssize_t aw2013_pause_hi_get(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->pause_hi;

    	pr_debug("%s: entry drvdata->pause_hi =%d \n",__func__,drvdata->pause_hi);
	
    	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}


static ssize_t aw2013_pause_hi_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
    	int err = -1 ; 
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	u8 a;
	int enable = atomic_read(&drvdata->enabled);
	u8 val = 0;
	u8 buf2[7];
	int pause_hi;
	
	if (enable == 0)
	{
		printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
		return -1;
	}
	pause_hi = aw2013_get_pause_value(buf);
	err = aw2013_i2c_read(drvdata->client, LED0T0, &val);
	drvdata->pause_hi = pause_hi;
	pr_debug("%s: shuangdan++++ LED0T0 0x37 value = %x ,pause_hi = %d \n",__func__,val,drvdata->pause_hi);
	
	switch (drvdata->pause_hi){
	case AW2013_PAUSE_0:
		a = 0; break;
	case AW2013_PAUSE_1:
		a = 1; break;
	case AW2013_PAUSE_2:
		a = 2; break;
	case AW2013_PAUSE_3:
		a = 3; break;
	case AW2013_PAUSE_4:
		a = 4; break;
	case AW2013_PAUSE_5:		
	case AW2013_PAUSE_6:
	case AW2013_PAUSE_7:
	default:
		a = 5; break;
	}
	val = (val & 0xf0) | a;
	aw2013_register_write(drvdata,buf2,LED0T0,val);
	
	err = aw2013_i2c_read(drvdata->client, LED0T0, &val);
	printk("%s: shuangdan  LED0T0 0x37 value = %x ,pause_hi T2= %d.%02d s,\n",__func__,val,13*(1<<a)/100,13*(1<<a)%100);
    return size;
}

static ssize_t aw2013_pause_lo_get(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->pause_lo;

    	pr_debug("%s: entry drvdata->pause_lo =%d \n",__func__,drvdata->pause_lo);

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}


static ssize_t aw2013_pause_lo_set(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
    	int err = -1 ; 
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	u8 a;
	int enable = atomic_read(&drvdata->enabled);
	u8 val = 0;
	u8 buf2[7];
	int pause_lo;
	
	if (enable == 0)
		{
			printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
			return -1;
		}
	pause_lo = aw2013_get_pause_value(buf);
	err = aw2013_i2c_read(drvdata->client, LED0T1, &val);
	drvdata->pause_lo = pause_lo;
	pr_debug("%s: shuangdan++++ LED0T0 0x38 value = %x ,pause_lo = %d \n",__func__,val,drvdata->pause_lo);
	
	a = aw2013_set_pause_value(drvdata->pause_lo);
	val = (val & 0xf0) | a;
	aw2013_register_write(drvdata,buf2,LED0T1,val);
	err = aw2013_i2c_read(drvdata->client, LED0T1, &val);
    printk("%s: shuangdan LED0T1 0x38 value = %x ,pause_lo T4= %d.%02d s,\n",__func__,val,13*(1<<a)/100,13*(1<<a)%100);
	
    return size;
}

//extern int atoi(const char *num);
static int hexval(char c)
{
	if (c >= '0' && c <= '9')
		return c - '0';
	else if (c >= 'a' && c <= 'f')
		return c - 'a' + 10;
	else if (c >= 'A' && c <= 'F')
		return c - 'A' + 10;
		
	return 0;
}
long atol(const char *num)
{
	long value = 0;
	int neg = 0;

	if (num[0] == '0' && num[1] == 'x') {
		// hex
		num += 2;
		while (*num && isxdigit(*num))
			value = value * 16 + hexval(*num++);
	} else {
		// decimal
		if (num[0] == '-') {
			neg = 1;
			num++;
		}
		while (*num && isdigit(*num))
			value = value * 10 + *num++  - '0';
	}

	if (neg)
		value = -value;

	return value;
}

int atoi(const char *num)
{
	return atol(num);
}
#define RED_LED_OFFSET 0
#define GREEN_LED_OFFSET 1
#define BLUE_LED_OFFSET 2


static void enable_led(struct aw2013_data *drvdata,int value,int led_offset)
{
	u8 buf_data[7];
    
	aw2013_register_write(drvdata,buf_data,AW_GCR,0x01);//GCR
    aw2013_register_write(drvdata,buf_data,LCTR,0x07);//LED Control register
	aw2013_register_write(drvdata,buf_data,LCFG0_EN+led_offset,0x01);// enable the register of LEDO
	aw2013_register_write(drvdata,buf_data,AW_PWM0+led_offset,value);//bringhtness=value	
   
}
static ssize_t attr_aw2013_get_red_brightness(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->red_brightness;
	pr_debug("%s: entry drvdata->red_brightness =%d \n",__func__,drvdata->red_brightness);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_aw2013_set_red_brightness(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int enable = atomic_read(&drvdata->enabled);
	if (enable == 0)
	{
		printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
		return -1;
	}
	drvdata->red_brightness = atoi(buf);
	enable_led(drvdata,drvdata->red_brightness,RED_LED_OFFSET);	
	return size;
}
static ssize_t attr_aw2013_get_green_brightness(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->green_brightness;
	pr_debug("%s: entry drvdata->green_brightness =%d \n",__func__,drvdata->green_brightness);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_aw2013_set_green_brightness(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int enable = atomic_read(&drvdata->enabled);
	if (enable == 0)
	{
		printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
		return -1;
	}
	drvdata->green_brightness = atoi(buf);
	enable_led(drvdata,drvdata->green_brightness,GREEN_LED_OFFSET);	
	return size;
}
static ssize_t attr_aw2013_get_blue_brightness(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	int val = drvdata->blue_brightness;
	pr_debug("%s: entry drvdata->blue_brightness =%d \n",__func__,drvdata->blue_brightness);
	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
}

static ssize_t attr_aw2013_set_blue_brightness(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
    
	int enable = atomic_read(&drvdata->enabled);

	if (enable == 0)
	{
		printk("%s:aw2013 : atomic_cmpxchg is disabled \n",__func__);
		return -1;
	}
	drvdata->blue_brightness = atoi(buf);
	enable_led(drvdata,drvdata->blue_brightness,BLUE_LED_OFFSET);	
	return size;
}



static DEVICE_ATTR(enable, 0644, attr_aw2013_get_enable, attr_aw2013_set_enable);
static DEVICE_ATTR(mode, 0644, attr_aw2013_get_mode, attr_aw2013_set_mode);
static DEVICE_ATTR(pause_rising, 0644, aw2013_pause_rising_get, aw2013_pause_rising_set);
static DEVICE_ATTR(pause_hi, 0644, aw2013_pause_hi_get, aw2013_pause_hi_set);
static DEVICE_ATTR(pause_falling, 0644, aw2013_pause_falling_get, aw2013_pause_falling_set);
static DEVICE_ATTR(pause_lo, 0644, aw2013_pause_lo_get, aw2013_pause_lo_set);
static DEVICE_ATTR(red_brightness, 0644, attr_aw2013_get_red_brightness, attr_aw2013_set_red_brightness);
static DEVICE_ATTR(green_brightness, 0644, attr_aw2013_get_green_brightness, attr_aw2013_set_green_brightness);
static DEVICE_ATTR(blue_brightness, 0644, attr_aw2013_get_blue_brightness, attr_aw2013_set_blue_brightness);


//static DEVICE_ATTR(green, 0644, attr_aw2013_get_green, attr_aw2013_set_green);
//static DEVICE_ATTR(blue, 0644, attr_aw2013_get_blue, attr_aw2013_set_blue);



//static DEVICE_ATTR(aw_register, 0644, NULL, aw2013_store_register);//aw2013_show_registers



static void aw2013_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{
	struct aw2013_data *drvdata;
	u8 buf[7];
	drvdata = container_of(led_cdev, struct aw2013_data, cdev);
	pr_debug("%s: entry and brightness will be %d\n",__func__,value);
	if (value)
	{  
		aw2013_register_write(drvdata,buf,AW_GCR,0x01);//GCR
		aw2013_register_write(drvdata,buf,LCTR,0x07);//LED Control register
		aw2013_register_write(drvdata,buf,LCFG0_EN,0x01);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LCFG1_EN,0x01);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,LCFG2_EN,0x01);// enable the register of LEDO
		aw2013_register_write(drvdata,buf,AW_PWM0,value);//bringhtness=value		
		aw2013_register_write(drvdata,buf,AW_PWM1,value);
		aw2013_register_write(drvdata,buf,AW_PWM2,value);
	}
	else
	{
        drvdata->mode =  AW2013_MODE_ALL_OFF;
		aw2013_set_registers(drvdata);
	}

}

int notifyLight_parse_dt(struct device *dev,
			struct aw2013_data *drvdata)
{

	struct device_node *np = dev->of_node;

	int rc;
	u32 temp_val;

	//printk("%s:aw2013 entry parse_dt \n",__func__);
	//rc = of_get_named_gpio(np, "aw,gpio_shdn", 0);
	//printk("%s:aw2013 entry parse_dt %d\n",__func__,rc);

	//if (rc < 0)
		//goto parse_error;
	drvdata->gpio_rst = -1;//rc;

	rc  = of_property_read_u32(np, "aw2013_mode", &temp_val);
	printk("%s:aw2013 temp_val is %d \n",__func__,temp_val);

	if (rc < 0)
		goto parse_error;
	drvdata->mode = temp_val;//(int)temp_val;
	
	return 0;
  parse_error:
	dev_err(dev,"parse property is failed, rc = %d\n", rc);
	return -EINVAL;
}
static int aw2013_power_on(struct i2c_client *i2c_client)
{
    struct regulator *aw2013_power;
    int retval;
    aw2013_power = regulator_get(&i2c_client->dev,"aw2013_power");
    if(IS_ERR(aw2013_power)) {
        retval = PTR_ERR( aw2013_power);
    }else{
        retval = regulator_enable(aw2013_power);
        if (retval) {
        }
    }
    return retval;
}
static int aw2013_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct aw2013_data *drvdata;
	int err = -1;
	int rc = 0;

	printk("%s:aw2013 entry \n",__func__);
	

	if (client == NULL)
	{
		printk("%s:===aw2013 probe: i2c client error \n",__func__);
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENXIO;
		printk("%s:===aw2013 error1: i2c check error \n",__func__);
		goto exit_check_functionality_failed;
	}
	printk("%s:aw2013 : i2c check success \n",__func__);
	drvdata = kzalloc(sizeof(struct aw2013_data),GFP_KERNEL);

	if (drvdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
				"failed to allocate memory for module data: "
					"%d\n", err);
	    printk("%s:===aw2013 error2: drvdata is null \n",__func__);
		goto exit_check_functionality_failed;
	}

	if (client->dev.of_node) {
		err = notifyLight_parse_dt(&client->dev, drvdata);
		if (err) {
			dev_err(&client->dev, "Failed to parse device tree\n");
			err = -EINVAL;
			return err;
			//goto exit_kfree_pdata;
		}
	} 
	else {
		dev_err(&client->dev, "No valid platform data. exiting.\n");
		err = -ENODEV;
		return err;
		//goto exit_kfree_pdata;
	}


	drvdata->client = client;	
	
	i2c_set_clientdata(client, drvdata);	
	aw2013_power_on(drvdata->client);
	aw2013_i2c_test(drvdata);
	err = aw2013_hw_init(drvdata);;
	atomic_set(&drvdata->enabled, 1);
	drvdata->mode = AW2013_MODE_ALL_OFF;
    err =aw2013_set_registers(drvdata);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		//goto exit_kfree_pdata;
	}

/*class node*/	   	
    drvdata->cdev.brightness = LED_OFF;
	drvdata->cdev.name = "aw2013_led";
	drvdata->cdev.brightness_set = aw2013_brightness_set;
	drvdata->cdev.max_brightness = LED_FULL;

	
	rc = led_classdev_register(&client->dev, &drvdata->cdev);
	if (rc) {
		dev_err(&client->dev,"STATUS_LED: led_classdev_register failed\n");
		goto err_led_classdev_register_failed;
	}
	printk("%s:led_classdev_register --- \n",__func__);

/*device node*/
	rc = device_create_file(&client->dev, &dev_attr_enable);
	if (rc) {
		dev_err(&client->dev,"STATUS_LED: create dev_attr_enable failed\n");
		goto err_out_attr_enable;
	}
	
	rc = device_create_file(&client->dev, &dev_attr_mode);
	if (rc) {
		dev_err(&client->dev,"STATUS_LED: create dev_attr_mode failed\n");
		goto err_out_attr_mode;
		}
	rc = device_create_file(&client->dev, &dev_attr_pause_rising);
	if (rc) {
		dev_err(&client->dev,"STATUS_LED: create dev_attr_pause_rising failed\n");
		goto err_out_attr_pause_rising;
	}
	rc = device_create_file(&client->dev, &dev_attr_pause_falling);
	rc = device_create_file(&client->dev, &dev_attr_pause_hi);
	
	rc = device_create_file(&client->dev, &dev_attr_pause_lo);
	rc = device_create_file(&client->dev, &dev_attr_red_brightness);
	rc = device_create_file(&client->dev, &dev_attr_green_brightness);
	rc = device_create_file(&client->dev, &dev_attr_blue_brightness);
	//rc = device_create_file(&client->dev, &dev_attr_aw_register);

    //err = aw2013_power_Off(drvdata); 
	return 0;

err_out_attr_enable:
	device_remove_file(&client->dev, &dev_attr_enable);
err_out_attr_mode:
	device_remove_file(&client->dev, &dev_attr_mode);
err_led_classdev_register_failed:
	led_classdev_unregister(&drvdata->cdev);
err_out_attr_pause_rising:
	device_remove_file(&client->dev, &dev_attr_pause_rising);

exit_check_functionality_failed:
	dev_err(&client->dev, "%s: Driver Init failed\n", "aw2013B");
	return err;

}

static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013_data *drvdata = i2c_get_clientdata(client);
	int err;

	if (gpio_is_valid(drvdata->gpio_rst)) {
		gpio_free(drvdata->gpio_rst);
	}
	err = aw2013_power_Off(drvdata);

	kfree(drvdata);

	return err;
}

//#define aw2013_suspend	NULL
//#define aw2013_resume	NULL

static int aw2013_resume(struct i2c_client *client)
{
	int err = 0;
	//u8 buf[7];
	//struct i2c_client *client = to_i2c_client(dev);
	//struct aw2013_data *drvdata = i2c_get_clientdata(client);
	//printk("%s:aw2013 3\n",__func__);

	//err = aw2013_power_On(drvdata);
	//drvdata->mode = 3;
    //err = aw2013_set_registers(drvdata);	

    return err;
}
  
static int aw2013_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	
	return err;
}

//static SIMPLE_DEV_PM_OPS(aw2013_pm, aw2013_suspend, aw2013_resume);
static const struct i2c_device_id aw2013_id[] = { 
		{ "aw2013", 0 }, 
		{ }, 
};
MODULE_DEVICE_TABLE(i2c, aw2013_id);


static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "aw2013", },
	{ },
};

static struct i2c_driver aw2013_driver = {
	.driver = {
			.name = "aw2013",
			.owner = THIS_MODULE,
			//.pm	= &aw2013_pm,
			.of_match_table = aw2013_match_table,
		  },
	.probe     = aw2013_probe,
	.remove  = aw2013_remove,
	.resume = aw2013_resume,
	.suspend = aw2013_suspend,
	.id_table  = aw2013_id,
};

static int __init aw2013_init(void)
{
	return i2c_add_driver(&aw2013_driver);
}

static void __exit aw2013_exit(void)
{
	i2c_del_driver(&aw2013_driver);
	return;
}

module_init(aw2013_init);
module_exit(aw2013_exit);

MODULE_DESCRIPTION("LED Class Interface");
MODULE_AUTHOR("shuang.dan@zte.com.cn");
MODULE_LICENSE("GPL");

