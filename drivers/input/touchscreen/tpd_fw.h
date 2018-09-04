/*
 * FILE:__TSP_FW_CLASS_H_INCLUDED
 *
 */
#ifndef __TPD_FW_H_INCLUDED
#define __TPD_FW_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/string.h>
#include <linux/mutex.h>

#define CMD_TP_INFO 	0
#define CMD_FW_UPG	1
#define CMD_CLEAN_BUF	2
#define CMD_WRITE_FW	3
#define CMD_READ_FW 	4
#define CMD_WRITE_REG	5
#define CMD_READ_REG	6
#define CMD_TP_CALIB	7
#define CMD_SDCARD_UPG	8
#define CMD_FORCE_UPG	9
#define CMD_CHECK_CHANNEL	10
#define CMD_CHECK_FW	11

#define STATUS_BUF_NULL 	0
#define STATUS_BUF_RDY		1
#define STATUS_UPG_ING		2
#define STATUS_UPG_SUCC 	4
#define STATUS_UPG_FAIL 	8
#define STATUS_UPG_NONEED	16
#define STATUS_FILE_FAIL	32
#define STATUS_NULL   STATUS_BUF_NULL
#define STATUS_OK	STATUS_UPG_SUCC

//#define TPD_DEBUG
#define TPD_DMESG
#define TPD_DEVICE			  "tpd"

#ifdef TPD_DMESG
#undef TPD_DMESG
#define TPD_DMESG(a,arg...) printk(TPD_DEVICE ": " a,##arg)
#else
#define TPD_DMESG(arg...) 
#endif

#ifdef TPD_DEBUG
#undef TPD_DEBUG
#define TPD_DEBUG(a,arg...) printk(TPD_DEVICE ": " a,##arg)
#else
#define TPD_DEBUG(arg...) 
#endif

struct tpd_tpinfo_t {
	unsigned int chip_id;
	unsigned int vendor_id;
	unsigned int chip_ver;
	unsigned int firmware_ver;
	unsigned int config_ver;
	unsigned int i2c_addr;
	unsigned int i2c_type;
	char tp_name[20];
	char vendor_name[20];
};

struct tpvendor_t {
	int vendor_id;
	char * vendor_name;
};

struct firmware_tpinfo {
	unsigned int chip_id;
	unsigned int vendor_id;
	unsigned int firmware_ver;
	char compare[8];
};

struct firmware_t{
	unsigned int size;
	unsigned char* data;
};

struct tpd_classdev_t {
	const char		*name;
	int 		 cmd;
	int 		 size;
	int 		 status;
	int 		 b_fwloader;
	int 		 b_cmd_done;
	int            b_gesture_enable;
	int b_force_upgrade;
	int		fw_compare_result;

	int (*tpd_calib)(struct tpd_classdev_t *cdev);
	int (*read_block)(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len);
	int (*write_block)(struct tpd_classdev_t *cdev, u16 addr, u8 *buf, int len);
	int (*flash_fw)(struct tpd_classdev_t *cdev, unsigned char * data, unsigned int size, int force_upg);
	int (*check_tp)(int type, unsigned char* buf, int max_buf_num);
	int (*compare_tp)(struct tpd_classdev_t *cdev, unsigned char *data);
	int (*get_gesture)(struct tpd_classdev_t *cdev);
	int (*wake_gesture)(struct tpd_classdev_t *cdev, int enable);
	int (*get_tpinfo)(struct tpd_classdev_t *cdev);
	void *private;

	struct mutex	flash_mutex;
	wait_queue_head_t wait;
	
	struct tpd_tpinfo_t tp_info;
	struct tpd_tpinfo_t sd_info;
	struct firmware_t tp_fw;
	struct work_struct	sdcard_upgrade_work;
	struct work_struct	check_fwinfo_work;
	struct firmware_tpinfo fw_info;
	struct device		*dev;
	struct list_head	 node;
};

extern struct tpd_classdev_t tpd_fw_cdev;
extern int tpd_classdev_register(struct device *parent, struct tpd_classdev_t *tsp_fw_cdev);
#endif	/* __TSP_FW_CLASS_H_INCLUDED */

