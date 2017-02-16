#include	<linux/err.h>
#include	<linux/errno.h>
#include	<linux/delay.h>
#include	<linux/fs.h>
#include	<linux/uaccess.h>
#include	<linux/slab.h>
#include	<linux/platform_device.h>
#include	<linux/module.h>
#include	<linux/regulator/consumer.h>

struct power_ctl {
	struct platform_device *pdev;
        struct regulator *vdd;
        struct regulator *vio;
        int    vdd_enabled;
        int    vio_enabled;
};
static struct power_ctl *pc;

#define SENSOR_POWER_DRIVER	"sensor_power_ctrl"
#define SENSOR_VDD_MIN_UV	2700000
#define SENSOR_VDD_MAX_UV	3300000
#define SENSOR_VIO_MIN_UV	1700000
#define SENSOR_VIO_MAX_UV	2000000

static int sensor_power_init(struct power_ctl *data, bool init)
{
        int rc;

        if (!init) {
                regulator_put(data->vdd);
                regulator_put(data->vio);
                data->vdd = NULL;
                data->vio = NULL;
        } else {
                data->vdd = regulator_get(&data->pdev->dev, "vdd");
                if (IS_ERR(data->vdd)) {
                        rc = PTR_ERR(data->vdd);
                        dev_err(&data->pdev->dev,
                                        "Regulator get failed vdd rc=%d", rc);
                        return rc;
                }

                data->vio = regulator_get(&data->pdev->dev, "vio");
                if (IS_ERR(data->vio)) {
                        rc = PTR_ERR(data->vio);
                        dev_err(&data->pdev->dev,
                                        "Regulator get failed vio rc=%d", rc);
			regulator_put(data->vdd);
			data->vdd = NULL;
			return rc;
                }
        }
        return 0;
}

int sensor_vdd_onoff(bool on)
{
        int rc;
	
	if ((!pc) || (!pc->vdd) || (!pc->vio)){
		dev_err(&pc->pdev->dev, "sensor power driver failed to load in %s", __func__);
		return -ENODEV;
	}

        if (!on) {
		if (pc->vdd_enabled <= 0) {
			dev_err(&pc->pdev->dev, "vdd off not balance with on");
			return -EBADF;
		}
                if (pc->vdd_enabled == 1){
                        regulator_set_voltage(pc->vdd, 0, SENSOR_VDD_MAX_UV);
                        regulator_disable(pc->vdd);
                        pc->vdd_enabled = 0;
                        dev_info(&pc->pdev->dev,"set vdd disable in %s, now = %d", __func__, regulator_is_enabled(pc->vdd));
			return 0;
		}
		pc->vdd_enabled -= 1;
        } else {
                if (regulator_count_voltages(pc->vdd) > 0) {
                        rc = regulator_set_voltage(pc->vdd, SENSOR_VDD_MIN_UV,
                                        SENSOR_VDD_MAX_UV);
                        if (rc) {
                                dev_err(&pc->pdev->dev,
                                                "Regulator set failed  rc=%d in %s",rc, __func__);
				return rc;
                        }
                }
                if (!pc->vdd_enabled) {
                        rc = regulator_enable(pc->vdd);
                        if (rc) {
                                dev_err(&pc->pdev->dev,
                                                "Regulator vdd enable failed rc=%d", rc);
				regulator_set_voltage(pc->vdd, 0, SENSOR_VDD_MAX_UV);
				return rc;
                        }
                        dev_info(&pc->pdev->dev,"set vdd enable in %s, now = %d", __func__, regulator_is_enabled(pc->vdd));
                }
                pc->vdd_enabled += 1;
        }
	return 0;
}
EXPORT_SYMBOL(sensor_vdd_onoff);

int sensor_vio_onoff(bool on)
{
        int rc;
	
	if ((!pc) || (!pc->vdd) || (!pc->vio)){
		dev_err(&pc->pdev->dev, "power driver failed to load in %s", __func__);
		return -ENODEV;
	}

        if (!on) {
		if (pc->vio_enabled <= 0) {
			dev_err(&pc->pdev->dev, "vio off not balance with on");
			return -EBADF;
		}
                if (pc->vio_enabled == 1){
                        regulator_set_voltage(pc->vio, 0, SENSOR_VIO_MAX_UV);
                        regulator_disable(pc->vio);
                        pc->vio_enabled = 0;
                        dev_info(&pc->pdev->dev,"set vio disable in %s, now = %d", __func__, regulator_is_enabled(pc->vio));
			return 0;
		}
		pc->vio_enabled -= 1;
        } else {
                if (regulator_count_voltages(pc->vio) > 0) {
                        rc = regulator_set_voltage(pc->vio, SENSOR_VIO_MIN_UV,
                                        SENSOR_VIO_MAX_UV);
                        if (rc) {
                                dev_err(&pc->pdev->dev,
                                                "Regulator set vio failed rc=%d in %s",rc, __func__);
				return rc;
                        }
                }
                if (!pc->vio_enabled) {
                        rc = regulator_enable(pc->vio);
                        if (rc) {
                                dev_err(&pc->pdev->dev,
                                                "Regulator vio enable failed rc=%d", rc);
				regulator_set_voltage(pc->vio, 0, SENSOR_VIO_MAX_UV);
				return rc;
                        }
                        dev_info(&pc->pdev->dev,"set vio enable in %s, now = %d", __func__, regulator_is_enabled(pc->vio));
                }
                pc->vio_enabled += 1;
        }
	return 0;
}
EXPORT_SYMBOL(sensor_vio_onoff);

int sensor_power_onoff(bool on)
{
	sensor_vdd_onoff(on);
	sensor_vio_onoff(on);
	return 0;
}
EXPORT_SYMBOL(sensor_power_onoff);

static int sensor_power_probe(struct platform_device *pdev)
{
	int ret;

	pc = kzalloc(sizeof(struct power_ctl), GFP_KERNEL);
	if (!pc){
		dev_err(&pdev->dev, "failed to allocate memory for module data");
		return -ENOMEM;
	}
	pc->pdev = pdev;
	ret = sensor_power_init(pc, true);
	if (ret) {
		printk("ERROR****sensor power probe failed, no such device*****\n");
		return ret;
	}
	printk("sensor_power probe success\n");
	return 0;
}

static int sensor_power_remove(struct platform_device *pdev)
{
	sensor_power_onoff(false);
	sensor_power_init(pc, false);
	kfree(pc);
	return 0;
}

static struct of_device_id sensor_power_match_table[] = {
         { .compatible = "sensor,power", },
         { }, 
};

static struct platform_driver sensor_power_ctrl = {
	.driver = {
			.owner = THIS_MODULE,
			.name = SENSOR_POWER_DRIVER,
			.of_match_table = sensor_power_match_table,
		  },
	.probe = sensor_power_probe,
	.remove = sensor_power_remove,
};
module_platform_driver(sensor_power_ctrl);

MODULE_DESCRIPTION("sensor power driver");
MODULE_AUTHOR("Haifei Wang <wang.haifei@zte.com.cn>");
MODULE_LICENSE("GPL");

