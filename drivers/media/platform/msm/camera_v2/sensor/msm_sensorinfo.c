/*
 * drivers/media/video/msm/msm_sensorinfo.c
 *
 * For sensor cfg test
 *
 * Copyright (C) 2009-2010 ZFG Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Created by li.jing19@zfg.com.cn
 */
/*-----------------------------------------------------------------------------------------
  when         who      what, where, why                        comment tag
  --------     ----     -------------------------------------   ---------------------------
  2010-06-13   lijing   modify file permission                  LIJING_CAM_20100613
  2010-06-10   lijing   create file                             
------------------------------------------------------------------------------------------*/
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include "msm_sensor.h"
/*-----------------------------------------------------------------------------------------
 *
 * MACRO DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
#define MAX_NAME_LENGTH     32


#define SENSOR_INFO_OV5648_MODEL_ID     0x5648
#define SENSOR_INFO_IMX214_MODEL_ID     0x0214
#define SENSOR_INFO_S5K3L2_MODEL_ID     0x30c2
#define SENSOR_INFO_S5K3M2XM_MODEL_ID     0x30D2
#define SENSOR_INFO_S5K5E2_MODEL_ID     0x5e20



/*-----------------------------------------------------------------------------------------
 *
 * GLOBAL VARIABLE DEFINITION
 *
 *----------------------------------------------------------------------------------------*/
static uint16_t g_back_sensor_id = 0;
static uint16_t g_front_sensor_id = 0;
static uint16_t module_integrator_id = 0;
static uint16_t module_integrator_3m2_id = 0;

/*-----------------------------------------------------------------------------------------
 *
 * FUNCTION DECLARATION
 *
 *----------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------
 *
 * FUNCTION DEFINITION
 *
 *----------------------------------------------------------------------------------------*/

DEFINE_SEMAPHORE(set_back_sensor_id_sem);
void msm_sensorinfo_set_back_sensor_id(uint16_t id)
{
    down(&set_back_sensor_id_sem);
    g_back_sensor_id = id;
    up(&set_back_sensor_id_sem);
}
EXPORT_SYMBOL(msm_sensorinfo_set_back_sensor_id);


DEFINE_SEMAPHORE(set_back_sensor_module_id_sem);
void msm_sensorinfo_set_back_sensor_module_id(uint8_t module_id, uint8_t eeprom_id)
{
    down(&set_back_sensor_module_id_sem);
	if(eeprom_id == 8){
	module_integrator_id = module_id;
	}else{
	  module_integrator_3m2_id = module_id;
	}
    up(&set_back_sensor_module_id_sem);
}
EXPORT_SYMBOL(msm_sensorinfo_set_back_sensor_module_id);


DEFINE_SEMAPHORE(set_front_sensor_id_sem);
void msm_sensorinfo_set_front_sensor_id(uint16_t id)
{
    down(&set_front_sensor_id_sem);
    g_front_sensor_id = id;
    up(&set_front_sensor_id_sem);   
}
EXPORT_SYMBOL(msm_sensorinfo_set_front_sensor_id);


static int rear_camera_id_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_back_sensor_id);
	return 0;
}
static int rear_camera_name_proc_show(struct seq_file *m, void *v)
{
	char sensor_name[MAX_NAME_LENGTH] = {0x00};	
	switch(g_back_sensor_id)
	{
		case SENSOR_INFO_IMX214_MODEL_ID:			
			if(module_integrator_id==0x6){ 
				sprintf(sensor_name, "IMX214-qtech-13M-AF");
			}else if(module_integrator_id==0x31){ 
				sprintf(sensor_name, "IMX214-mcnex-13M-AF");
			}else if(module_integrator_id==0x15){ 
				sprintf(sensor_name, "IMX214-Liteon-13M-AF");
			}else{ 
			  sprintf(sensor_name, "IMX214-13M-AF");
			}
			break;
		case SENSOR_INFO_S5K3L2_MODEL_ID:
			if(module_integrator_id==0x6){ 
				sprintf(sensor_name, "S5K3L2-qtech-13M-AF");
			}else if(module_integrator_id==0x31){ 
				sprintf(sensor_name, "S5K3L2-mcnex-13M-AF");
			}else if(module_integrator_id==0x15){ 
				sprintf(sensor_name, "S5K3L2-Liteon-13M-AF");
			}else{ 
			  sprintf(sensor_name, "S5K3L2-13M-AF");
			}
			break;	
		case SENSOR_INFO_S5K3M2XM_MODEL_ID:
			if(module_integrator_3m2_id==0x6){ 
				sprintf(sensor_name, "S5K3M2XM-qtech-13M-AF");
			}else if(module_integrator_3m2_id==0x31){ 
				sprintf(sensor_name, "S5K3M2XM-mcnex-13M-AF");
			}else if(module_integrator_3m2_id==0x15){ 
				sprintf(sensor_name, "S5K3M2XM-Liteon-13M-AF");
			}else{ 
			  sprintf(sensor_name, "S5K3M2XM-13M-AF");
			}
			break;	
		default:
			sprintf(sensor_name, "No sensor or error ID!");
			break;
	}
	seq_printf(m, "%s\n", sensor_name);
	return 0;
}

static int front_camera_id_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "0x%x\n", g_front_sensor_id);
	return 0;
}
static int front_camera_name_proc_show(struct seq_file *m, void *v)
{
	char sensor_name[MAX_NAME_LENGTH] = {0x00};	
	switch(g_front_sensor_id)
	{
		case SENSOR_INFO_OV5648_MODEL_ID:
			sprintf(sensor_name, "OV5648-5Mp-FF");
			break;
		case SENSOR_INFO_S5K5E2_MODEL_ID:
			sprintf(sensor_name, "S5K5E2-5Mp-FF");
			break;
		default:
			sprintf(sensor_name, "No sensor or error ID!");
			break;
	}
	seq_printf(m, "%s\n", sensor_name);
	return 0;
}


static int rear_camera_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rear_camera_id_proc_show, NULL);
}
static int rear_camera_name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rear_camera_name_proc_show, NULL);
}

static int front_camera_id_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, front_camera_id_proc_show, NULL);
}
static int front_camera_name_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, front_camera_name_proc_show, NULL);
}


static const struct file_operations rear_camera_id_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rear_camera_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations rear_camera_name_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rear_camera_name_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations front_camera_id_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= front_camera_id_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
static const struct file_operations front_camera_name_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= front_camera_name_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
extern const struct file_operations flash_proc_fops;
/*
 * Attention:
 *
 * Path of camera's sysdev: /sys/devices/system/camera/camera0
 */
static int __init sensorinfo_init(void)
{	
	struct proc_dir_entry *dir, *res;

	dir = proc_mkdir("camera", NULL);
	if (!dir)
		return -ENOMEM;

	res = proc_create_data("rear_camera_id", 0, dir,
			       &rear_camera_id_proc_fops, NULL);
	if (!res)
		return -ENOMEM;

	res = proc_create_data("rear_camera_name", 0, dir,
			       &rear_camera_name_proc_fops, NULL);
	if (!res)
		return -ENOMEM;

	res = proc_create_data("front_camera_id", 0, dir,
			       &front_camera_id_proc_fops, NULL);
	if (!res)
		return -ENOMEM;

	res = proc_create_data("front_camera_name", 0, dir,
			       &front_camera_name_proc_fops, NULL);
	if (!res)
		return -ENOMEM;
  return 0;
}
module_init(sensorinfo_init);

