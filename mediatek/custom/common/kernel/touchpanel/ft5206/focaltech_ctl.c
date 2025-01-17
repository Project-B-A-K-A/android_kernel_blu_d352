#include <linux/slab.h>
#include <linux/i2c.h>
//#include <linux/semaphore.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>

///#include <linux/i2c/ft5x06_ts.h>
#include "focaltech_ctl.h"


#include "tpd.h"
#include <linux/interrupt.h>
#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include "tpd_custom_ft5206.h"

#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>

#include "cust_gpio_usage.h"


static int ft_rw_iic_drv_major = FT_RW_IIC_DRV_MAJOR;
struct ft_rw_i2c_dev {
	struct cdev cdev;
	struct mutex ft_rw_i2c_mutex;
	struct i2c_client *client;
};
struct ft_rw_i2c_dev *ft_rw_i2c_dev_tt;
static struct class *fts_class;


/////////////////////////
int ft5x0x_i2c_Read(struct i2c_client *client, char *writebuf,
		    int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "f%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
			 .addr = client->addr,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}
/*write data by i2c*/
int ft5x0x_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msg[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = writelen,
		 .buf = writebuf,
		 },
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);

	return ret;
}
/////////////////////////

static int ft_rw_iic_drv_myread(struct i2c_client *client, u8 *buf, int length)
{
	int ret = 0;	
	ret = ft5x0x_i2c_Read(client, NULL, 0, buf, length);

	if(ret<0)
		dev_err(&client->dev, "%s:IIC Read failed\n",
				__func__);
    	return ret;
}

static int ft_rw_iic_drv_mywrite(struct i2c_client *client, u8 *buf, int length)
{
	int ret = 0;
	ret = ft5x0x_i2c_Write(client, buf, length);
	if(ret<0)
		dev_err(&client->dev, "%s:IIC Write failed\n",
				__func__);
	return ret;
}

static int ft_rw_iic_drv_RDWR(struct i2c_client *client, unsigned long arg)
{
	struct ft_rw_i2c_queue i2c_rw_queue;
	u8 __user **data_ptrs;
	struct ft_rw_i2c * i2c_rw_msg;
	int ret = 0;
	int i;

	if (!access_ok(VERIFY_READ, (struct ft_rw_i2c_queue *)arg, sizeof(struct ft_rw_i2c_queue)))
		return -EFAULT;

	if (copy_from_user(&i2c_rw_queue,
		(struct ft_rw_i2c_queue *)arg, 
		sizeof(struct ft_rw_i2c_queue)))
		return -EFAULT;

	if (i2c_rw_queue.queuenum > FT_I2C_RDWR_MAX_QUEUE)
		return -EINVAL;


	i2c_rw_msg = (struct ft_rw_i2c*)
		kmalloc(i2c_rw_queue.queuenum *sizeof(struct ft_rw_i2c),
		GFP_KERNEL);
	if (!i2c_rw_msg)
		return -ENOMEM;

	if (copy_from_user(i2c_rw_msg, i2c_rw_queue.i2c_queue,
			i2c_rw_queue.queuenum*sizeof(struct ft_rw_i2c))) {
		kfree(i2c_rw_msg);
		return -EFAULT;
	}

	data_ptrs = kmalloc(i2c_rw_queue.queuenum * sizeof(u8 __user *), GFP_KERNEL);
	if (data_ptrs == NULL) {
		kfree(i2c_rw_msg);
		return -ENOMEM;
	}
	
	ret = 0;
	for (i=0; i< i2c_rw_queue.queuenum; i++) {
		if ((i2c_rw_msg[i].length > 8192)||
			(i2c_rw_msg[i].flag & I2C_M_RECV_LEN)) {
			ret = -EINVAL;
			break;
		}
		data_ptrs[i] = (u8 __user *)i2c_rw_msg[i].buf;
		i2c_rw_msg[i].buf = kmalloc(i2c_rw_msg[i].length, GFP_KERNEL);
		if (i2c_rw_msg[i].buf == NULL) {
			ret = -ENOMEM;
			break;
		}

		if (copy_from_user(i2c_rw_msg[i].buf, data_ptrs[i], i2c_rw_msg[i].length)) {
			++i;
			ret = -EFAULT;
			break;
		}
	}

	if (ret < 0) {
		int j;
		for (j=0; j<i; ++j)
			kfree(i2c_rw_msg[j].buf);
		kfree(data_ptrs);
		kfree(i2c_rw_msg);
		return ret;
	}

	for (i=0; i< i2c_rw_queue.queuenum; i++) {
		if (i2c_rw_msg[i].flag) {
   	   		ret = ft_rw_iic_drv_myread(client,
					i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
			if (ret>=0)
   	   			ret = copy_to_user(data_ptrs[i], i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
   	   	}
		else
			ret = ft_rw_iic_drv_mywrite(client,
					i2c_rw_msg[i].buf, i2c_rw_msg[i].length);
	}

	return ret;
	
}

/*
*char device open function interface 
*/
static int ft_rw_iic_drv_open(struct inode *inode, struct file *filp)
{
	filp->private_data = ft_rw_i2c_dev_tt;
	return 0;
}

/*
*char device close function interface 
*/
static int ft_rw_iic_drv_release(struct inode *inode, struct file *filp)
{

	return 0;
}

void ft5x0x_reset_tp(int HighOrLow)
{
	pr_info("set tp reset pin to %d\n", HighOrLow);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, HighOrLow);
}


static int ft_rw_iic_drv_ioctl(struct file *filp, unsigned
  int cmd, unsigned long arg)
{
	int ret = 0;
	struct ft_rw_i2c_dev *ftdev = filp->private_data;
	ftdev = filp->private_data;
	
	mutex_lock(&ft_rw_i2c_dev_tt->ft_rw_i2c_mutex);
	switch (cmd)
	{
	case FT_I2C_RW:
		ret = ft_rw_iic_drv_RDWR(ftdev->client, arg);	
		break;
	case FT_RESET_TP:
		ft5x0x_reset_tp((int)arg);
		break;
	default:
		ret =  -ENOTTY;
		break;
	}
	mutex_unlock(&ft_rw_i2c_dev_tt->ft_rw_i2c_mutex);
	return ret;	
}


/*    
*char device file operation which will be put to register the char device
*/
static const struct file_operations ft_rw_iic_drv_fops = {
	.owner			= THIS_MODULE,
	.open			= ft_rw_iic_drv_open,
	.release			= ft_rw_iic_drv_release,
	.unlocked_ioctl	= ft_rw_iic_drv_ioctl,
};


static void ft_rw_iic_drv_setup_cdev(struct ft_rw_i2c_dev *dev, int index)
{
	int err, devno = MKDEV(ft_rw_iic_drv_major, index);

	cdev_init(&dev->cdev, &ft_rw_iic_drv_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &ft_rw_iic_drv_fops;
	err = cdev_add(&dev->cdev, devno, 1);
	if (err)
		printk(KERN_NOTICE "Error %d adding LED%d", err, index);
}

static int ft_rw_iic_drv_myinitdev(struct i2c_client *client)
{
	int err = 0;
	dev_t devno = MKDEV(ft_rw_iic_drv_major, 0);

	if (ft_rw_iic_drv_major)
		err = register_chrdev_region(devno, 1, FT_RW_IIC_DRV);
	else {
		err = alloc_chrdev_region(&devno, 0, 1, FT_RW_IIC_DRV);
		ft_rw_iic_drv_major = MAJOR(devno);
	}
	if (err < 0) {
		dev_err(&client->dev, "%s:ft_rw_iic_drv failed  error code=%d---\n",
				__func__, err);
		return err;
	}

	ft_rw_i2c_dev_tt = kmalloc(sizeof(struct ft_rw_i2c_dev), GFP_KERNEL);
	if (!ft_rw_i2c_dev_tt){
		err = -ENOMEM;
		unregister_chrdev_region(devno, 1);
		dev_err(&client->dev, "%s:ft_rw_iic_drv failed\n",
				__func__);
		return err;
	}
	ft_rw_i2c_dev_tt->client = client;
	
	mutex_init(&ft_rw_i2c_dev_tt->ft_rw_i2c_mutex);
	ft_rw_iic_drv_setup_cdev(ft_rw_i2c_dev_tt, 0); 

	fts_class = class_create(THIS_MODULE, "fts_class");
	if (IS_ERR(fts_class)) {
		dev_err(&client->dev, "%s:failed in creating class.\n",
				__func__);
		return -1; 
	} 
	/*create device node*/
	device_create(fts_class, NULL, MKDEV(ft_rw_iic_drv_major, 0), 
			NULL, FT_RW_IIC_DRV);

	return 0;
}

int ft_rw_iic_drv_init(struct i2c_client *client)
{
    dev_dbg(&client->dev, "[FTS]----ft_rw_iic_drv init ---\n");
	return ft_rw_iic_drv_myinitdev(client);
}

void  ft_rw_iic_drv_exit(void)
{
	device_destroy(fts_class, MKDEV(ft_rw_iic_drv_major, 0)); 
	/*delete class created by us*/
	class_destroy(fts_class); 
	/*delet the cdev*/
	cdev_del(&ft_rw_i2c_dev_tt->cdev);
	kfree(ft_rw_i2c_dev_tt);
	unregister_chrdev_region(MKDEV(ft_rw_iic_drv_major, 0), 1); 
}

