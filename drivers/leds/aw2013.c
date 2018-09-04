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


#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	DEBUG	1

/***************************************************************************/
// 寄存器
#define   RSTR      0x00   // 复位寄存器
#define   EN_GCR  0x01   // LED使能寄存器，正常工作为0x01，关断为0x00
#define   EN_BRE    0x30   // LED使能寄存器，低三位分别对应LED0 LED1 LED2，设为1使能，为0关断
#define   LCFG_0    0x31   // LED0 可配置LED最大电流0,5,10,15mA，淡进淡出效果
#define   LCFG_1    0x32   // LED1 可配置LED最大电流0,5,10,15mA，淡进淡出效果
#define   LCFG_2    0x33   // LED2 可配置LED最大电流0,5,10,15mA，淡进淡出效果
#define   PWM_0    0x34   // LED0闪烁模式下最大亮度配置0~255
#define   PWM_1    0x35   // LED1闪烁模式下最大亮度配置0~255
#define   PWM_2    0x36   // LED2闪烁模式下最大亮度配置0~255
#define   RISE_ON_TIME_0    0x37  // LED0  上升时间，常亮时间(前6级可选)设定，8级：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   RISE_ON_TIME_1    0x3A  // LED1  上升时间，常亮时间(前6级可选)设定，8级：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   RISE_ON_TIME_2   0x3D  // LED2  上升时间，常亮时间(前6级可选)设定，8级：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   FALL_OFF_TIME_0    0x38  // LED0  下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   FALL_OFF_TIME_1    0x3B  // LED1  下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   FALL_OFF_TIME_2   0x3E  // LED2  下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s

#define   DELAY_TIME_0    0x39 // LED0  下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   DELAY_TIME_1    0x3C  // LED1  下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s
#define   DELAY_TIME_2   0x3F  // LED2  下降时间设定，常灭时间设定8级可选：0.13 0.26 0.52 1.04 2.08 4.16 8.32 16.64s

/***************************************************************************/
//以下为调节呼吸效果的参数
#define Imax          0x01   //LED最大电流配置,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Imax1          0x01   //T3版本由10mA改为5mA  //LED最大电流配置,0x00=omA,0x01=5mA,0x02=10mA,0x03=15mA,
#define Rise_time   0x03   //LED呼吸上升时间,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Hold_time   0x00   //LED呼吸到最亮时的保持时间0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s
#define Fall_time     0x03   //LED呼吸下降时间,0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Off_time      0x03   //LED呼吸到灭时的保持时间0x00=0.13s,0x01=0.26s,0x02=0.52s,0x03=1.04s,0x04=2.08s,0x05=4.16s,0x06=8.32s,0x07=16.64s
#define Delay_time   0x00   //LED呼吸启动后的延迟时间0x00=0s,0x01=0.13s,0x02=0.26s,0x03=0.52s,0x04=1.04s,0x05=2.08s,0x06=4.16s,0x07=8.32s,0x08=16.64s
#define Period_Num  0x00   //LED呼吸次数0x00=无限次,0x01=1次,0x02=2次.....0x0f=15次

// T60和V55项目结构设计不一样，导致不能使用同样的亮度
#ifdef CONFIG_LED_BRIGHTNESS_SETTING
#define brightness_setting  0xFF
#define charging_setting 0x7F
#define TPColor_white_brightness  0x14  // 0.4mA
#define TPColor_white_charging    0x0A
#else
#define brightness_setting  0x7F // 2.5mA
#define charging_setting  0x3F
#define TPColor_white_brightness  0x14  // 0.4mA
#define TPColor_white_charging    0x0A
#endif
enum aw2013_mode {
	AW2013_MODE_ALL_ON = 0,	     /* 全亮 */
	AW2013_MODE_MIDD,		     /* 中间灯亮 */
	AW2013_MODE_QUICK_BLINK,     /* 中间灯闪，频率较快一般用于指示消息. */
	AW2013_MODE_SLOW_BLINK,      /* 中间灯闪，频率较慢一般用于充电以及低点指示. */
	AW2013_MODE_ALL_ON_W,	     /* 全亮 */
	AW2013_MODE_MIDD_W,		     /* 中间灯亮 */
	AW2013_MODE_QUICK_BLINK_W,     /* 中间灯闪，频率较快一般用于指示消息. */
	AW2013_MODE_SLOW_BLINK_W,      /* 中间灯闪，频率较慢一般用于充电以及低点指示. */
	AW2013_MODE_ALL_OFF,             /* 全灭 */                     
};

/*added for vdd & vio regulator_begin*/
#ifdef CONFIG_OF
#define BREATHLIGHT_VDD_MIN_UV        2700000
#define BREATHLIGHT_VDD_MAX_UV        3300000
#define BREATHLIGHT_VIO_MIN_UV        1700000
#define BREATHLIGHT_VIO_MAX_UV        1950000
#endif
/*added for vdd & vio regulator_end*/

struct aw2013_mode_map {
	const char *mode;
	enum aw2013_mode mode_val;
};
static struct aw2013_mode_map mode_map[] = {
	{ "allon", AW2013_MODE_ALL_ON },
	{ "midd", AW2013_MODE_MIDD },
	{ "blink1", AW2013_MODE_QUICK_BLINK },
	{ "blink2", AW2013_MODE_SLOW_BLINK },
	{ "allon_w", AW2013_MODE_ALL_ON_W },
	{ "midd_w", AW2013_MODE_MIDD_W },
	{ "blink1_w", AW2013_MODE_QUICK_BLINK_W },
	{ "blink2_w", AW2013_MODE_SLOW_BLINK_W },
	{ "alloff", AW2013_MODE_ALL_OFF },
};

//static struct i2c_client *aw2013_client = NULL;

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

/*added  for vdd&vio regulator begin*/
#ifdef CONFIG_OF
	bool    vdd_enabled;
	bool    vio_enabled;
	struct regulator *vdd;
	struct regulator *vio;
#endif
/*added  for vdd&vio regulator end*/	
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

	//printk(":===cyf i2c write: drvdata %p , client is %p, adapter %p\n", drvdata, drvdata->client, drvdata->client->adapter);

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
/*
static int aw2013_i2c_test(struct aw2013_data *drvdata)
{
	int err = 0;
	u8 val = 0;
	err = aw2013_i2c_read(drvdata->client, RISE_ON_TIME_1, &val);
	printk("%s: == cyf  RISE_ON_TIME_1 is %x \n",__func__,val);
	
	err = aw2013_i2c_read(drvdata->client, PWM_1, &val);
	printk("%s: == cyf  PWM_1 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, FALL_OFF_TIME_1, &val);
	printk("%s: == cyf  FALL_OFF_TIME_1 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, 0x14, &val);
	printk("%s: == cyf  0x14 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, 0x15, &val);
	printk("%s: == cyf  0x15 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, 0x16, &val);
	printk("%s: == cyf  0x16 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, 0x04, &val);
	printk("%s: == cyf  0x04 is %x \n",__func__,val);

	err = aw2013_i2c_read(drvdata->client, 0x05, &val);
	printk("%s: == cyf  0x05 is %x \n",__func__,val);

	return err;
}
*/
static int aw2013_hw_init(struct aw2013_data *drvdata)
{
	int 	err = 0;
        u8 buf[7];

      // 软件复位
      aw2013_register_write(drvdata,buf,RSTR, 0x55);		       // Reset 
      //aw2013_register_write(drvdata,buf,EN_GCR, 0x01);		// enable LED 不使用中断

	return err;
}

static int aw2013_set_registers(struct aw2013_data *drvdata)
{
	int err = 0;
    u8 buf[7];
 
    //printk("%s: entry  AW2013_MODE_MIDD\n",__func__);
	aw2013_register_write(drvdata,buf,EN_GCR, 0x01);

	switch (drvdata->mode) {
	case AW2013_MODE_MIDD:
	case AW2013_MODE_MIDD_W:
		aw2013_register_write(drvdata,buf,LCFG_1,Imax);  // LED1, IMAX = 5mA	
		if (drvdata->mode == AW2013_MODE_MIDD_W)
		{
			aw2013_register_write(drvdata,buf,PWM_1,TPColor_white_charging);  // LED1, 闪烁模式下的最大亮度	
		}
		else
		{
			aw2013_register_write(drvdata,buf,PWM_1,charging_setting);  // LED1, 闪烁模式下的最大亮度		
		}	
        aw2013_register_write(drvdata,buf,RISE_ON_TIME_1,Rise_time<<4 | Hold_time); //LED1淡进时间，常亮时间设定		
		aw2013_register_write(drvdata,buf,FALL_OFF_TIME_1,Rise_time<<4 | Hold_time); //LED1淡出时间，常灭时间设定			 								
		aw2013_register_write(drvdata,buf,EN_BRE,0x02);  // LED1，即中间等使能，其余两灯去使能
		break;

	case AW2013_MODE_SLOW_BLINK:  //中间呼吸，呼吸频率慢 用于充电
	case AW2013_MODE_SLOW_BLINK_W:
		aw2013_register_write(drvdata,buf,LCFG_1,0x70|Imax);  // LED1, IMAX = 5mA	
		if (drvdata->mode == AW2013_MODE_SLOW_BLINK_W)
		{
			aw2013_register_write(drvdata,buf,PWM_1,TPColor_white_brightness);  // LED1, 闪烁模式下的最大亮度		
		}
		else
		{
			aw2013_register_write(drvdata,buf,PWM_1,brightness_setting);  // LED1, 闪烁模式下的最大亮度	
		}	
        aw2013_register_write(drvdata,buf,RISE_ON_TIME_1,Rise_time<<4 | Hold_time); //LED1淡进时间，常亮时间设定		
		aw2013_register_write(drvdata,buf,FALL_OFF_TIME_1,Rise_time<<4 | Off_time); //LED1淡出时间，常灭时间设定			 
		aw2013_register_write(drvdata,buf,DELAY_TIME_1,Rise_time<<4 | Period_Num);									
		aw2013_register_write(drvdata,buf,EN_BRE,0x02);  // LED1，即中间等使能，其余两灯去使能
		break;

	case AW2013_MODE_QUICK_BLINK:  //中间两路呼吸， 用于消息
	case AW2013_MODE_QUICK_BLINK_W:
		aw2013_register_write(drvdata,buf,LCFG_1,0x70|Imax);  // LED1, IMAX = 5mA	
		if (drvdata->mode == AW2013_MODE_QUICK_BLINK_W)
		{
			aw2013_register_write(drvdata,buf,PWM_1,TPColor_white_brightness);  // LED1, 闪烁模式下的最大亮度		
		}
		else
		{
			aw2013_register_write(drvdata,buf,PWM_1,brightness_setting);  // LED1, 闪烁模式下的最大亮度	
		}
	
        aw2013_register_write(drvdata,buf,RISE_ON_TIME_1,Rise_time<<4 | Hold_time); //LED1淡进时间，常亮时间设定		
		aw2013_register_write(drvdata,buf,FALL_OFF_TIME_1,Rise_time<<4 | Off_time); //LED1淡出时间，常灭时间设定			 
		aw2013_register_write(drvdata,buf,DELAY_TIME_1,Rise_time<<4 | Period_Num);						
		aw2013_register_write(drvdata,buf,EN_BRE,0x02);  // LED1，即中间等使能，其余两灯去使能
		break;

	case AW2013_MODE_ALL_OFF:
		aw2013_register_write(drvdata,buf,EN_BRE,0x00);  // LED1，即中间等使能，其余两灯去使能	
		break;
	case AW2013_MODE_ALL_ON:
	case AW2013_MODE_ALL_ON_W:
	default:
		aw2013_register_write(drvdata,buf,LCFG_0,Imax1);  // LED0, IMAX = 10mA
		aw2013_register_write(drvdata,buf,LCFG_1,Imax);  // LED1, IMAX = 5mA	
		aw2013_register_write(drvdata,buf,LCFG_2,Imax1);  // LED2, IMAX = 10mA
		if (drvdata->mode == AW2013_MODE_ALL_ON_W)
		{
			aw2013_register_write(drvdata,buf,PWM_0,TPColor_white_brightness);  // LED0, 闪烁模式下的最大亮度
			aw2013_register_write(drvdata,buf,PWM_1,TPColor_white_brightness);  // LED1, 闪烁模式下的最大亮度	
			aw2013_register_write(drvdata,buf,PWM_2,TPColor_white_brightness);  // LED2, 闪烁模式下的最大亮度		
		}
		else
		{
			aw2013_register_write(drvdata,buf,PWM_0,brightness_setting);  // LED0, 闪烁模式下的最大亮度
			aw2013_register_write(drvdata,buf,PWM_1,brightness_setting);  // LED1, 闪烁模式下的最大亮度	
			aw2013_register_write(drvdata,buf,PWM_2,brightness_setting);  // LED2, 闪烁模式下的最大亮度	
		}			
		aw2013_register_write(drvdata,buf,EN_BRE,0x07);  // LED1，即中间等使能，其余两灯去使能	
		break;
	}  
	//aw2013_i2c_read(drvdata->client, EN_BRE, &val);
    udelay(8);    
	//err = aw2013_i2c_test(drvdata);

	return err;
}
/*
static int aw2013_power_Off(struct aw2013_data *drvdata)
{
     u8 buf[7];
	 int err = -1;

	 //printk("%s: entry  \n",__func__);
       // 软件复位
       aw2013_register_write(drvdata,buf,RSTR, 0x55);		 // Reset 
       aw2013_register_write(drvdata,buf,EN_BRE,0x00);  // LED1，即中间等使能，其余两灯去使能	
       aw2013_register_write(drvdata,buf,EN_GCR,0x00);
 

	return err;
}

static int aw2013_i2c_read(struct aw2013_data *drvdata,
				u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg	msgs[] = {
		{
			.addr = drvdata->client->addr,
			.flags = drvdata->client->flags & I2C_M_TEN,
			.len = 1,
			.buf = buf,
		},
		{
			.addr = drvdata->client->addr,
			.flags = (drvdata->client->flags & I2C_M_TEN) | I2C_M_RD,
			.len = len,
			.buf = buf,
		},
	};

	do {
		err = i2c_transfer(drvdata->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&drvdata->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}
*/

static ssize_t attr_aw2013_set_enable(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t size)
{
    u8 buf_reg[7];
    struct aw2013_data *drvdata = dev_get_drvdata(dev);
    unsigned long val;
	
    //int err;

    if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

    aw2013_register_write(drvdata,buf_reg,EN_GCR,0x00);   //关断

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

/*
static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct aw2013_data *drvdata = dev_get_drvdata(dev);
	int err;

	u8 data = reg;
	err = aw2013_i2c_read(drvdata, &data, 1);
	if (err < 0)
		return err;
	ret = snprintf(buf, 4, "0x%02x\n", data);
	return ret;

}
*/

static ssize_t attr_aw2013_get_mode(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);

	int val = drvdata->mode;

	return snprintf(buf, sizeof(val) + 2, "%d\n", val);
			
}
static ssize_t attr_aw2013_set_mode(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct aw2013_data *drvdata= dev_get_drvdata(dev);
	
	int err = -1;
	int mode = AW2013_MODE_ALL_ON;
	int val = atomic_read(&drvdata->enabled);
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

	//atomic_set(&drvdata->mode, mode);
	
	//printk("%s: cyf drvdata->mode is %d, the set mode %d \n",__func__,drvdata->mode, mode);
	//printk(":===cyf set mode: drvdata %p , client is %p, adapter %p, \n", drvdata, drvdata->client, drvdata->client->adapter);
	err =aw2013_set_registers(drvdata);
	if (err) {
		dev_err(dev, "Setting %s Mode failed :%d\n", buf, err);
		return err;
	}

	return sizeof(drvdata->mode);

}

static DEVICE_ATTR(enable, 0644, attr_aw2013_get_enable, attr_aw2013_set_enable);
static DEVICE_ATTR(mode, 0644, attr_aw2013_get_mode, attr_aw2013_set_mode);

/*
static u8 Transform10to16(int nbrightness)
{
	int a,b;
	u8 brightness_value = 0x00;
	if (nbrightness <1 && nbrightness >255)
	{
		return brightness_value;
	}
	a = nbrightness/16;
	b = nbrightness % 16;
        if (a>9)
        {
		brightness_value = (0xA + (u8)(a - 10))*10;
	}
	else
	{
		brightness_value =  (u8)a;
	}

	 if(b>9)
        {
		brightness_value = (u8)brightness_value + (0xA + (u8)(a - 10));
	}
	else
	{
		brightness_value = (u8)brightness_value + (u8)a;
	}
	return brightness_value;
	
}
*/
static void aw2013_brightness_set(struct led_classdev *led_cdev,
		enum led_brightness value)
{

	struct aw2013_data *drvdata;
	u8 buf[7];
	u8 nbrightness = 0x00;
	drvdata = container_of(led_cdev, struct aw2013_data, cdev);
	
	//printk("%s: cyf led_brightness value is %d \n",__func__,value);
#ifdef CONFIG_LED_BRIGHTNESS_SETTING
	// T60
    nbrightness = brightness_setting;
#else
	// V55
	if (value == 1)
	{
		// 表示TP是白色后盖
		nbrightness = TPColor_white_brightness;

	}
	else 
	{
		// 表示TP是黑色后盖
		nbrightness = brightness_setting;
	}
#endif

	
	if (value)
	{
		// 亮
		aw2013_register_write(drvdata,buf,LCFG_0,Imax1);  // LED0, IMAX = 10mA	
		aw2013_register_write(drvdata,buf,PWM_0,nbrightness);  // LED0, 闪烁模式下的最大亮度
		aw2013_register_write(drvdata,buf,LCFG_1,Imax);  // LED1, IMAX = 5mA	
		aw2013_register_write(drvdata,buf,PWM_1,nbrightness);  // LED1, 闪烁模式下的最大亮度	
		aw2013_register_write(drvdata,buf,LCFG_2,Imax1);  // LED2, IMAX = 10mA	
		aw2013_register_write(drvdata,buf,PWM_2,nbrightness);  // LED2, 闪烁模式下的最大亮度	
								
		aw2013_register_write(drvdata,buf,EN_BRE,0x07);  // LED0/LED1/LED2，所有灯使能
	}
	else
	{
		// 灭
		aw2013_register_write(drvdata,buf,EN_BRE,0x00);  // LED0/LED1/LED2，所有灯去使能

		// 恢复之前面的状态
		aw2013_set_registers(drvdata);
	}

     
}

int NotifyLight_parse_dt(struct device *dev,
			struct aw2013_data *drvdata)
{
#ifdef CONFIG_OF
	struct device_node *np = dev->of_node;

	int rc;
	u32 temp_val;

	rc  = of_property_read_u32(np, "aw,aw2013_mode", &temp_val);
	//printk("%s:aw2013 temp_val is %d \n",__func__,temp_val);

	if (rc < 0)
		goto parse_error;
	drvdata->mode = (int)temp_val;
	
	return 0;
parse_error:
	dev_err(dev,"parse property is failed, rc = %d\n", rc);
	return -EINVAL;
#else
	return 0;
#endif
}

/*added for vdd&vio regulator begin*/
#ifdef CONFIG_OF
static int aw2013_power_init(struct aw2013_data *data, bool on)
{
        int rc;

        if (!on) {
                if (data->vdd_enabled) {
                        regulator_disable(data->vdd);
                        dev_info(&data->client->dev,"power_init:vdd disabled");						
                        data->vdd_enabled = false;
                }    
                if (data->vio_enabled) {
                        regulator_disable(data->vio);
                        dev_info(&data->client->dev,"power_init:vio disabled");						
                        data->vio_enabled = false;
                }    

                if (regulator_count_voltages(data->vdd) > 0) 
                        regulator_set_voltage(data->vdd, 0, BREATHLIGHT_VDD_MAX_UV);

                regulator_put(data->vdd);

                if (regulator_count_voltages(data->vio) > 0) 
                        regulator_set_voltage(data->vio, 0, BREATHLIGHT_VIO_MAX_UV);

                regulator_put(data->vio);
        } else {
                data->vdd = regulator_get(&data->client->dev, "vdd");
                if (IS_ERR(data->vdd)) {
                        rc = PTR_ERR(data->vdd);
                        dev_err(&data->client->dev,
                                        "Regulator get failed vdd rc=%d\n", rc); 
                        return rc;
                }    
                if (regulator_count_voltages(data->vdd) > 0) { 
                        rc = regulator_set_voltage(data->vdd, BREATHLIGHT_VDD_MIN_UV,
                                       BREATHLIGHT_VDD_MAX_UV);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator set failed vdd rc=%d\n",
                                                rc); 
                                goto reg_vdd_put;
                        }    
                }    
 
                data->vio = regulator_get(&data->client->dev, "vio");
                if (IS_ERR(data->vio)) {
                        rc = PTR_ERR(data->vio);
                        dev_err(&data->client->dev,
                                        "Regulator get failed vio rc=%d\n", rc);
                        return rc;
                }

                if (regulator_count_voltages(data->vio) > 0) {
                        rc = regulator_set_voltage(data->vio, BREATHLIGHT_VIO_MIN_UV,
                                       BREATHLIGHT_VIO_MAX_UV);
                        if (rc) {
                                dev_err(&data->client->dev,
                                                "Regulator set failed vio rc=%d\n", rc);
                                goto reg_vio_put;
                        }
                }
 
        }
        return 0;

reg_vio_put:
        regulator_put(data->vio);
reg_vdd_put:
        regulator_put(data->vdd);
        return rc;
}

#else

static int aw2013_power_init(struct aw2013_data*data, bool on)
{
	return 0;
}
#endif

#if 0
static void aw2013_device_power_off(struct aw2013_data *data)
{
	int err;

	if (data->vdd_enabled) {
		err = regulator_disable(data->vdd);
		if (err) {
			dev_err(&data->client->dev,
			"Regulator vdd disable failed err=%d\n", err);
		}
		dev_info(&data->client->dev,"power_off:vdd disabled");	
		data->vdd_enabled = false;				
	}

}
#endif
static int aw2013_device_power_on(struct aw2013_data *data)
{
	int err = -1;
	
	if(!data->vdd_enabled) {
		err = regulator_enable(data->vdd);
		if (err) {
			dev_err(&data->client->dev,
			"Regulator vdd enable failed err=%d\n", err);
			return err;
		}
		dev_info(&data->client->dev,"power_on:vdd enabled");			
		data->vdd_enabled = true;			
	} 
	if (!data->vio_enabled) {
		err = regulator_enable(data->vio);
		if (err) {
			dev_err(&data->client->dev,
			"Regulator vio enable failed err=%d\n", err);
			return err;
		}
		dev_info(&data->client->dev,"power_on:vio enabled");			
		data->vio_enabled = true;			
	}
    return 0;
}
/*added for vdd&vio regulator begin*/


static int aw2013_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct aw2013_data *drvdata;
	u8  val = 0;
	int err = -1;
	int rc = 0;

	printk("%s:aw2013 begin \n",__func__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENXIO;
		printk("%s:===aw2013 error1: i2c check error \n",__func__);
		goto exit_check_functionality_failed;
	}

	drvdata = kzalloc(sizeof(struct aw2013_data),GFP_KERNEL);

	if (drvdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,"failed to allocate memory for module data:%d\n", err);
	       printk("%s:===aw2013 error2: drvdata is null \n",__func__);
		goto exit_check_functionality_failed;
	}

	if (client->dev.of_node) {
		err = NotifyLight_parse_dt(&client->dev, drvdata);
		if (err) {
			err = -EINVAL;
			dev_err(&client->dev, "Failed to parse device tree: %d\n", err);
			goto exit_kfree_pdata;
		}
	} 
	else {
		err = -ENODEV;
		dev_err(&client->dev, "No valid platform data. exiting.: %d\n", err);
		goto exit_kfree_pdata;
	}


	drvdata->client = client;	
	//printk(":===cyf probe: drvdata %p , client is %p, adapter %p \n", drvdata, drvdata->client, drvdata->client->adapter);
	
	i2c_set_clientdata(client, drvdata);	

/*added for vdd and vio regulator begin*/
       drvdata->vdd_enabled = false;
       drvdata->vio_enabled = false;
       err = aw2013_power_init(drvdata, true);
	if (err < 0) {
		dev_err(&client->dev, "power init failed: %d\n", err);
		goto err_power_deinit;
	}		

	err = aw2013_device_power_on(drvdata);
	if(err < 0)
	{
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err_power_deinit;
	}
/*added for vdd and vio regulator end*/
	
	aw2013_i2c_read(drvdata->client, RSTR, &val);
	//printk("%s: == cyf===  RSTR is %x \n",__func__,val);
	if (val != 0x33)
	{
		err = -EINVAL;
		dev_err(&client->dev, "unsupported chip id: %d\n", err);
		goto err_chip_id;
	}
	
	err = aw2013_hw_init(drvdata);
	atomic_set(&drvdata->enabled, 1);
       err =aw2013_set_registers(drvdata);
	if (err < 0) {
		dev_err(&client->dev, "Register read failure: %d\n", err);
		goto exit_kfree_pdata;
	}

/*class node*/	   	
	drvdata->cdev.brightness = LED_OFF;
	drvdata->cdev.name = "bln";
	drvdata->cdev.brightness_set = aw2013_brightness_set;
	drvdata->cdev.max_brightness = LED_FULL;
	
       // 创建节点
	rc = led_classdev_register(&client->dev, &drvdata->cdev);
	if (rc) {
		dev_err(&client->dev,"STATUS_LED: led_classdev_register failed\n");
		goto err_led_classdev_register_failed;
	}

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

	return 0;

err_out_attr_enable:
	device_remove_file(&client->dev, &dev_attr_enable);
err_out_attr_mode:
	device_remove_file(&client->dev, &dev_attr_mode);
err_led_classdev_register_failed:
	led_classdev_unregister(&drvdata->cdev);
err_power_deinit:
	aw2013_power_init(drvdata, false);	
err_chip_id:
exit_kfree_pdata:
	kfree(drvdata);
exit_check_functionality_failed:
	dev_err(&client->dev, "%s: Driver Init failed\n", "aw2013");
	return err;

}

static int aw2013_remove(struct i2c_client *client)
{
	struct aw2013_data *drvdata = i2c_get_clientdata(client);

	u8 buf[7];

	// 软件复位
	aw2013_register_write(drvdata,buf,RSTR, 0x55);		       // Reset 
	aw2013_register_write(drvdata,buf,EN_BRE,0x00);  // LED0\LED1\LED2 去使能
	aw2013_register_write(drvdata,buf,EN_GCR,0x00);  // 整体去使能
	aw2013_power_init(drvdata, false);		
	kfree(drvdata);

	return 0;
}

//#define aw2013_suspend	NULL
//#define aw2013_resume	NULL

static int aw2013_resume(struct i2c_client *client)
{
	int err = 0;	
	return err;
}
  
static int aw2013_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int err = 0;
	return err;
}

static const struct i2c_device_id aw2013_id[] = { 
		{ "aw2013", 0 }, 
		{ }, 
};
MODULE_DEVICE_TABLE(i2c, aw2013_id);


static struct of_device_id aw2013_match_table[] = {
	{ .compatible = "aw,aw2013", },
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

MODULE_DESCRIPTION("sssss");
MODULE_AUTHOR("gggg");
MODULE_LICENSE("hhhh");

