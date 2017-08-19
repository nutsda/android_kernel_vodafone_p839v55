/*
 * Copyright (C) 2010 Trusted Logic S.A.
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
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <pn544.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <asm-generic/gpio.h>
#include <linux/mfd/pm8xxx/pm8921.h>

#include <linux/string.h>
#include <linux/of_gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>//FIXED Screen off transcation proformance issue
#include <linux/clk.h>

//#include <linux/clk.h>

//#define pr_err printk
//#define pr_debug printk
//#define pr_warning printk
#define NFC_RF_CLK_FREQ			(19200000)

#define MAX_BUFFER_SIZE	512

#define PN544_DRIVER_NAME         "pn544"



struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	unsigned int        clkreq_gpio;
	struct	clk		   *s_clk;
	bool                clk_run;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
	struct wake_lock   wl;//FIXED Screen off transcation proformance issue
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};
///////////////////////////////////////////////////////////////////////////////
/* enable LDO */
struct vregs_info {
	const char * const name;
	struct regulator *regulator;
};
struct vregs_info regulators = {"vdd", NULL};
/////////////////////////////////////////////////////////////////////////
static struct pn544_dev    *pn544_dev = NULL;
static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	printk("%s : pn544_disable_irq\n", __func__);
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;
	
	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		return IRQ_HANDLED;
	}
	pn544_disable_irq(pn544_dev);
	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);
    printk("%s : IRQ trigger!\n", __func__);
	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret, i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	printk("%s : reading %zu bytes.\n", __func__, count);
	
	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}
		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}
	wake_lock(&pn544_dev->wl);//FIXED Screen off transcation proformance issue
	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	wake_unlock(&pn544_dev->wl);//FIXED Screen off transcation proformance issue
	mutex_unlock(&pn544_dev->read_mutex);
	
	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	printk("NFCC->DH:");
	for(i = 0; i < ret; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");
	
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	wake_unlock(&pn544_dev->wl);//FIXED Screen off transcation proformance issue
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret, i;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	printk("%s : writing %zu bytes.\n", __func__, count);
	printk("DH->NFCC:");
	for(i = 0; i < count; i++){
		printk(" %02X", tmp[i]);
	}
	printk("\n");
//	mutex_lock(&pn544_dev->read_mutex);
	/* Write data */
 //   for (i = 0; i < 3; i++)
    //{
        ret = i2c_master_send(pn544_dev->client, tmp, count);
	//	if (ret > 0) 
		//	break;
	//	else 
	//	{
	//		printk("%s : delay 5 ms and  retry.\n", __func__);
	//		mdelay(5);
	//	}
    //}
	//mutex_unlock(&pn544_dev->read_mutex);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;
	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static int pn544_clock_enable(struct pn544_dev *dev,bool enable)
{
	
	int rc = 0;
	
	if (enable && dev->clk_run == false) {
		rc = clk_prepare_enable(dev->s_clk);
		if (rc) 
		{
			dev_err(&dev->client->dev,"clk_prepare_enable failed\n");
	        return -1;
		}
		dev->clk_run = true;
		dev_dbg(&dev->client->dev,"ref_clk enabled\n");
		return 0;
    }
    else if (!enable && dev->clk_run == true) {
		clk_disable_unprepare(dev->s_clk);
		dev->clk_run = false;
		dev_dbg(&dev->client->dev,"ref_clk disabled\n");
		return 0;
    }
	else
	{
		dev_err(&dev->client->dev,"Invalid req\n");
		return 0;
	}
		
}


static int pn544_pinctrl_init(struct pn544_dev *dev)
{
	struct i2c_client *client = dev->client;
    dev_dbg(&client->dev, "%s\n",__func__);
	dev->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(dev->pinctrl)) {
		dev_err(&client->dev, "Failed to get pinctrl\n");
		return PTR_ERR(dev->pinctrl);
	}

	dev->gpio_state_active =
		pinctrl_lookup_state(dev->pinctrl, "nfc_active");
	if (IS_ERR_OR_NULL(dev->gpio_state_active))
		dev_err(&client->dev, "Failed to look up active state\n");

	dev->gpio_state_suspend =
		pinctrl_lookup_state(dev->pinctrl, "nfc_suspend");
	if (IS_ERR_OR_NULL(dev->gpio_state_suspend))
		dev_err(&client->dev, "Failed to look up sleep state\n");

	return 0;
}

static void pn544_pinctrl_state(struct pn544_dev *dev,
			bool active)
{
	struct i2c_client *client = dev->client;
	int ret;

	dev_dbg(&client->dev, "%s en=%d\n", __func__,active);

	if (active) {
		if (!IS_ERR_OR_NULL(dev->pinctrl)) {
			ret = pinctrl_select_state(dev->pinctrl,
				dev->gpio_state_active);
			if (ret)
				dev_err(&client->dev,
					"Error pinctrl_select_state(%s) err:%d\n",
					"nfc_active", ret);
		}
	} else {
		if (!IS_ERR_OR_NULL(dev->gpio_state_suspend)) {
			ret = pinctrl_select_state(dev->pinctrl,
				dev->gpio_state_suspend);
			if (ret)
				dev_err(&client->dev,
					"Error pinctrl_select_state(%s) err:%d\n",
					"nfc_suspend", ret);
		}
	}
	return;
}




static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			printk("%s power on with firmware download\n", __func__);
		
			gpio_set_value(pn544_dev->ven_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(10);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(50);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(10);
			pn544_clock_enable(pn544_dev,true);
		} else if (arg == 1) {
			/* power on */
			printk("%s power on\n", __func__);
			
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			irq_set_irq_wake(pn544_dev->client->irq, 1);
			msleep(10);
			pn544_clock_enable(pn544_dev,true);

		} else  if (arg == 0) {
			/* power off */
			printk("%s power off\n", __func__);
			pn544_clock_enable(pn544_dev,false);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			irq_set_irq_wake(pn544_dev->client->irq, 0);

			msleep(10);

		} else {
			//printk("%s bad arg %x\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		printk("%s bad ioctl %d\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static int nxp_pn544_reset(struct i2c_client *client)
{

	int rc = 0;
	
	if (pn544_dev->s_clk == NULL) {
        pn544_dev->s_clk	= clk_get(&pn544_dev->client->dev, "ref_clk");
		if (pn544_dev->s_clk == NULL)
		{
			dev_err(&client->dev, "%s: unable to get bbclk2 rc =%d \n",
					__func__, rc);
	        return -EIO;
		}
		rc = clk_set_rate(pn544_dev->s_clk,NFC_RF_CLK_FREQ);
		if (rc)
		{
			dev_err(&client->dev, "%s failed to set clk rate:rc=%d\n",
					__func__, rc);
		}
	} 
	
    pn544_pinctrl_state(pn544_dev,true);
	 
	/********************IRQ**************************/
	rc = gpio_request(pn544_dev->irq_gpio, "nxp_pn544_IRQ");
	if (rc) {
	  dev_err(&client->dev, "%s: unable to request nfc gpio %d (%d)\n",
					__func__, pn544_dev->irq_gpio, rc);
	  return -EIO;
	}
	rc = gpio_direction_input(pn544_dev->irq_gpio);
	if (rc) {
	  dev_err(&client->dev, "%s: unable to config nfc irq_gpio %d (%d)\n",
					__func__, pn544_dev->irq_gpio, rc);
      return -EIO;
	}
	
	client->irq = gpio_to_irq(pn544_dev->irq_gpio);


	/********************DOWNLOAD************************/
	rc = gpio_request(pn544_dev->firm_gpio, "nxp_pn544_download");
	if (rc) {
	  dev_err(&client->dev,"%s: unable to request firm_gpio %d (%d)\n",
					__func__, pn544_dev->firm_gpio, rc);
	  return -EIO;
	}
	rc = gpio_direction_output(pn544_dev->firm_gpio, 0);
	if (rc) {
	  dev_err(&client->dev, "%s: unable to config firm_gpio %d (%d)\n",
					__func__, pn544_dev->firm_gpio, rc);
	  return -EIO;
    }
	gpio_set_value(pn544_dev->firm_gpio, 0);
	/********************VEN***************************/
	rc = gpio_request(pn544_dev->ven_gpio, "nxp_pn544_en");
	if (rc) {
		dev_err(&client->dev, "%s: unable to request ven_gpio %d (%d)\n",
					__func__, pn544_dev->ven_gpio, rc);
			 return -EIO;
	}
	rc = gpio_direction_output(pn544_dev->ven_gpio, 1);
	if (rc) {
	  dev_err(&client->dev, "%s: unable to config ven_gpio %d (%d)\n",
					__func__, pn544_dev->ven_gpio, rc);
	  return -EIO;
	}
	gpio_set_value(pn544_dev->ven_gpio, 0);
	/********************CLK_REQ***************************/
    rc = gpio_request(pn544_dev->clkreq_gpio,"nxp_pn544_clkreq");
	if (rc) {
	  dev_err(&client->dev, "unable to request clkreq_gpio %d (%d)\n",
						pn544_dev->clkreq_gpio,rc);
	  return -EIO;
	}
	rc = gpio_direction_input(pn544_dev->clkreq_gpio);
	if (rc) {
	  dev_err(&client->dev,
						"unable to set direction for clkreq_gpio %d (%d)\n",
						pn544_dev->clkreq_gpio,rc);
	  return -EIO;
	}

    

	dev_dbg(&client->dev,"%s Success",__func__);
  
#if 0
	int rc;

	rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->irq_gpio, 0,
				GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	  if (rc) {
			printk( "%s: Could not configure nfc gpio %d\n",
					__func__, pn544_dev->irq_gpio);
			 return -EIO;
		   }

	 rc = gpio_request(pn544_dev->irq_gpio, "nxp_pn544_IRQ");
	 if (rc) {
			printk( "%s: unable to request nfc gpio %d (%d)\n",
					__func__, pn544_dev->irq_gpio, rc);
			 return -EIO;
		    }


	  rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->firm_gpio, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	  //printk("pn544 config firmgpio pull down\n");
	  if (rc) {
			printk( "%s: Could not configure nfc gpio %d\n",
					__func__, pn544_dev->firm_gpio);
			 return -EIO;
		   }
    
	 rc = gpio_request(pn544_dev->firm_gpio, "nxp_pn544_download");
	 if (rc) {
			printk( "%s: unable to request nfc gpio %d (%d)\n",
					__func__, pn544_dev->firm_gpio, rc);
			 return -EIO;
	 }
     gpio_direction_output(pn544_dev->firm_gpio, 0);
     /*ven gpio out*/
	 rc = gpio_tlmm_config(GPIO_CFG(pn544_dev->ven_gpio, 0,
	                                   GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
	                                   GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    //printk("pn544 config vengpio out put no pull\n");
    if (rc) {
        printk( "%s: Could not configure nfc gpio %d\n",
                __func__, pn544_dev->ven_gpio);
        return -EIO;
    }

	rc = gpio_request(pn544_dev->ven_gpio, "nxp_pn544_en");
	if (rc) {
			printk( "%s: unable to request nfc gpio %d (%d)\n",
					__func__,pn544_dev->ven_gpio, rc);
			 return -EIO;
	}
	gpio_direction_output(pn544_dev->ven_gpio, 0);
	
#endif
	return 0;
}
static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl  = pn544_dev_ioctl,
};


static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;
   struct device_node *of_node = NULL;
   //static struct clk *bb_clk2;  /*added bb_clk2 for nfc , start */ 

   printk("pn544_probe(): start\n");   
   if (pn544_dev != NULL) {
      printk("pn544_probe: multiple devices NOT supported\n");
      ret = -ENODEV;
      goto err_single_device;
   }
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}

	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}

	pn544_dev->client   = client;	
	/////////////////////////////////////////////////////////////////////////////
	regulators.regulator = regulator_get(&client->dev, regulators.name);
	   if (IS_ERR(regulators.regulator)) {
		   ret = PTR_ERR(regulators.regulator);
		   dev_err(&client->dev,"regulator get of %s failed (%d)\n", regulators.name, ret);
	   } else {
		  // if (regulator_count_voltages(regulators.regulator) > 0) {   
		//	   ret = regulator_set_voltage(regulators.regulator,
		//		   1800000, 1800000);
		//	   if (ret) {		   
		//		   dev_err(&client->dev,
		//			   "regulator set_vtg failed retval =%d\n",
		//			   ret);
				   //goto err_set_vtg_vdd;
		//	   }
		//   }
		   
	       ret = regulator_set_voltage(regulators.regulator,
				1800000, 1800000);
		   if (ret) {
		   	    dev_err(&client->dev,"vreg %s set voltage failed (%d)\n",
				   regulators.name, ret);
		   }
		   /* Enable the regulator */
		   ret = regulator_enable(regulators.regulator);
		   if (ret) {
			   dev_err(&client->dev,"vreg %s enable failed (%d)\n",
				   regulators.name, ret);
		   }
	
	   }
	   ////////////////////////////////////////////////
   if (client->dev.of_node) {
   	
   	of_node = client->dev.of_node;

	ret = of_get_named_gpio(of_node, "nxp,irq_gpio", 0);
	if (ret>0 && gpio_is_valid(ret)) {
		pn544_dev->irq_gpio=ret;
		dev_dbg(&client->dev,"%s: irq_gpio:%d\n",__func__,ret);
	} else{
		dev_err(&client->dev,"%s: irq_gpio fail:%d\n",__func__,ret);
		goto err_device_create_failed;
	}

	ret = of_get_named_gpio(of_node, "nxp,firm_gpio", 0);
	if (ret>0 && gpio_is_valid(ret)) {
		pn544_dev->firm_gpio=ret;
		dev_dbg(&client->dev,"%s: firm_gpio:%d\n",__func__,ret);
	} else{
		dev_err(&client->dev,"%s: firm_gpio fail:%d\n",__func__,ret);
		goto err_device_create_failed;
	}
	
	ret = of_get_named_gpio(of_node, "nxp,ven_gpio", 0);	//8974 mpp7
	if (ret>0 && gpio_is_valid(ret)) {
		pn544_dev->ven_gpio=ret;
		dev_dbg(&client->dev,"%s: ven_gpio:%d\n",__func__,ret);
	} else{
		dev_err(&client->dev,"%s: ven_gpio fail:%d\n",__func__,ret);
		goto err_device_create_failed;
	}

	ret = of_get_named_gpio(of_node, "nxp,clk-gpio", 0);
	if (ret>0 && gpio_is_valid(ret)) {
		pn544_dev->clkreq_gpio = ret;
		dev_dbg(&client->dev,"%s: clkreq_gpio:%d\n",__func__,ret);
	} else{
		dev_err(&client->dev,"%s: clkreq_gpio fail:%d\n",__func__,ret);
		goto err_device_create_failed;
	}
	
    ret = pn544_pinctrl_init(pn544_dev);
	if (ret) {
       dev_err(&client->dev, "Can't initialize pinctrl\n");
	}
	
	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	/*Initialise wake lock*/
	wake_lock_init(&pn544_dev->wl,WAKE_LOCK_SUSPEND,"nfc_locker");//FIXED Screen off transcation proformance issue

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}
   
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
    ret =nxp_pn544_reset(client);
    if (ret < 0) {
        dev_err(&client->dev,"can't reset device\n");
        goto err_device_create_file_failed;
    }

    /*bb_clk2 = clk_get(&client->dev, "bb_clk2");
	if (IS_ERR(bb_clk2)) {
		printk(KERN_ERR "%s: Error getting bb_clk2\n", __func__);
		bb_clk2 = NULL;
		//return -ENOENT;
	}
	else{
		printk("%s: start prepare bb_clk2\n", __func__);
        ret = clk_prepare_enable(bb_clk2);
		if(ret){
		    printk(KERN_ERR "%s: prepare bb_clk2 failed ret:%d\n", __func__, ret);
	    }
	}*/

	pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
	pn544_dev->irq_enabled = true;

	ret = request_irq(client->irq, pn544_dev_irq_handler,
			  IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}
	
	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);	
	printk("nfc probe is ok\n");

	return 0;
   }
err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_device_create_failed:
   kfree(pn544_dev);
   pn544_dev = NULL;
err_device_create_file_failed:
    //device_destroy(pn544_dev_class, MKDEV(pn544_major, pn544_minor));
err_exit:
    if (pn544_dev->firm_gpio)
	  gpio_free(pn544_dev->firm_gpio);
	if (pn544_dev->ven_gpio)
	  gpio_free(pn544_dev->ven_gpio);
	if (pn544_dev->irq_gpio)
	  gpio_free(pn544_dev->irq_gpio);
	if (pn544_dev->clkreq_gpio)
	  gpio_free(pn544_dev->clkreq_gpio);
err_single_device:
	return ret;

}

static int pn544_remove(struct i2c_client *client)
{
	printk("pn544_remove start\n");
	pn544_dev = i2c_get_clientdata(client);
	pn544_disable_irq(pn544_dev);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	wake_lock_destroy(&pn544_dev->wl);//FIXED Screen off transcation proformance issue
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);
	pn544_dev = NULL;
	printk("pn544_remove end\n");

	return 0;
}

static int pn544_suspend(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(client->irq);
	printk("pn544_suspend\n");
	return 0;
}

static int pn544_resume(struct device *device)
{
	struct i2c_client *client = to_i2c_client(device);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(client->irq);
	printk("pn544_resume\n");
	return 0;
}


static const struct i2c_device_id pn544_id[] = {
   	{ PN544_DRIVER_NAME, 0 },
   	{ }
};

static struct of_device_id nfc_match_table[] = {
	{.compatible = "nxp,pn544",},
	{ },
};

static const struct dev_pm_ops nfc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pn544_suspend, pn544_resume)
};

static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= {
		.name   = "pn544",
		.owner = THIS_MODULE,
		.of_match_table = nfc_match_table,
		.pm = &nfc_pm_ops,
	},
};

/*
 * module load/unload record keeping
 */
static int __init pn544_dev_init(void)
{
	pr_info("Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_info("Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
