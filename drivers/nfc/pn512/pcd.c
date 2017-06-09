#include <linux/of.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/slab.h> 
#include <linux/err.h>     //IS_ERR()
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/poll.h>
#include <linux/sched.h>   //wake_up_process()
#include "spi.h"

#include "common.h"
#include "picc.h"
#include "debug.h"
#include "pn512.h"
#include "ccid_picc.c"


struct pcd_param
{
    u8 *p_iBuf;
    u8 *p_oBuf;
    u32  iDataLen;
    u32  oDataLen;
    u32 statusCode;
};


#define  Card_PowerOn     0x01
#define  Card_PowerOff    0x02
#define  Card_XfrAPDU     0x03



void run_picc_poll(struct work_struct *work);
DECLARE_DELAYED_WORK(card_Poll, run_picc_poll);


/* struct spi_device *access_spi; */


static long pcd_ioctl(struct file *filp, u32 cmd, unsigned long arg) 
{
	struct pcd_common *common = filp->private_data;
    u8 pcd_cmd = (cmd >> 4) & 0xFF;
    struct pcd_param KerParam;
    struct pcd_param *UsrParam = (struct pcd_param *)arg;
    u8 *p_iData;
    u8 *p_oData;
    u32  ret = 0;
    u8 level = 0;
  printk("%s-%s-%d\n", __FILE__, __FUNCTION__, __LINE__);  

    if(down_interruptible(&common->mutex))    // acquire the semaphore
    {
        ret = -ERESTARTSYS;
        goto err;
    }

    if((!UsrParam) || (copy_from_user(&KerParam, UsrParam, sizeof(KerParam))))
    {
        ret = -EFAULT;          // bad address
        goto err;
    }

    switch(pcd_cmd)
    {		
        case Card_PowerOn:
            {
               // printk("line=%d\n", __LINE__);
            if(!KerParam.p_oBuf) 
			{
                ret = -EFAULT;       // bad address
                goto err;
            }
			//printk("line=%d\n", __LINE__);
            p_oData = kmalloc(KerParam.oDataLen, GFP_KERNEL);
			
            if(!p_oData) 
			{
                ret = -EFAULT;       // bad address
                goto err;                
            }
/* printk("line=%d\n", __LINE__); */
			if((ret = picc_power_on(&common->picc, p_oData, &KerParam.oDataLen)) != 0)	
				goto err2;

			if(copy_to_user(KerParam.p_oBuf, p_oData, KerParam.oDataLen)) 
			{
                ret = -EFAULT;       // bad address
                goto err2;
            }

			if(copy_to_user(&UsrParam->oDataLen, &KerParam.oDataLen, sizeof(KerParam.oDataLen)))
			{
                ret = -EFAULT;       // bad address
                goto err2;
            }

			kfree(p_oData);
			
            break; 
        }

        case Card_PowerOff:
        {
            picc_power_off(&common->picc);
			ret = 0; 
            break;
        }

        case Card_XfrAPDU:
        {
            if((KerParam.iDataLen <= 0) || (KerParam.oDataLen <= 0) || (!KerParam.p_iBuf) || (!KerParam.p_oBuf)) 
			{
                ret = -EFAULT;       // bad address
                goto err;
            }
			
            p_iData = kmalloc(KerParam.iDataLen, GFP_KERNEL);
            p_oData = kmalloc(KerParam.oDataLen, GFP_KERNEL);
			
            if((!p_iData) || (!p_oData) || (copy_from_user(p_iData, KerParam.p_iBuf, KerParam.iDataLen)))
            {
                ret = -EFAULT;       // bad address
                goto err1;
            }
		//printk("line=%d\n", __LINE__);				
            if((ret = picc_command_exchange(&common->picc, p_iData, KerParam.iDataLen, p_oData, &KerParam.oDataLen, &level)) != 0)	
				goto err;
		//printk("line=%d\n", __LINE__);	
            if((KerParam.oDataLen <= 0) || (copy_to_user(KerParam.p_oBuf, p_oData, (unsigned long)KerParam.oDataLen)))
            {
                ret = -EFAULT;       // bad address
                goto err1;
            }
					//printk("line=%d\n", __LINE__);	
            if(copy_to_user(&UsrParam->oDataLen, &KerParam.oDataLen, sizeof(KerParam.oDataLen)))
            {
                ret = -EFAULT;       // bad address
                goto err1;
            }
		//printk("line=%d\n", __LINE__);	
			kfree(p_iData);
			kfree(p_oData);
			
            break;
        }

        default:
            break;
    }

	up(&common->mutex); 
	return(0);


err1:
	if(p_iData)		kfree(p_iData);
err2:
	if(p_oData)		kfree(p_oData);
err:
    up(&common->mutex);                    // release the semaphore
    UsrParam->statusCode = ret;
    return(ret);
}


static int pcd_open(struct inode *inode, struct file *filp)
{
    if(common->sem_inc > 0)    return(-ERESTARTSYS);
    common->sem_inc++;

    filp->private_data = common;

    return(0);
}
static int pcd_release(struct inode *inode, struct file *filp)
{
	struct pcd_common *common = filp->private_data;

	
	common->sem_inc--;
	
    return(0);
}

extern int picc_interrput_in(u8 slot_status);

void run_picc_poll(struct work_struct *work)
{

    if(down_trylock(&common->mutex))    
    {
        goto done;
    }

    if(BITISSET(common->pcd.flags_polling, AUTO_POLLING) && BITISSET(common->pcd.flags_polling, POLLING_CARD_ENABLE))
    {	
        picc_polling_tags(&common->picc);
		if(BITISSET(common->picc.status, SLOT_CHANGE))
		{
			if(!picc_interrput_in(common->picc.status & PRESENT))
				CLEAR_BIT(common->picc.status, SLOT_CHANGE);
		}
    }



    up(&common->mutex);

done:
	
	queue_delayed_work(common->polling, &card_Poll, (common->pcd.poll_interval * HZ) / 1000);


}


static struct file_operations pcd_fops=
{
    .owner = THIS_MODULE,
    .open = pcd_open,
    .unlocked_ioctl = pcd_ioctl,
    .release = pcd_release
};


int spidev_init_open(struct spidev_data *spidev)
{
	/* struct spidev_data	*spidev; */
	int			status = -ENXIO;
	struct spi_device	*spi;
	/* mutex_lock(&device_list_lock); */

	spin_lock_irq(&spidev->spi_lock);
	spi = spi_dev_get(spidev->spi);
	spin_unlock_irq(&spidev->spi_lock);
	if (spi == NULL)
		return -ESHUTDOWN;

	if (!spidev->tx_buffer) {
            spidev->tx_buffer = kmalloc(4096, GFP_KERNEL);
            		/* spidev->tx_buffer = kmalloc(bufsiz, GFP_KERNEL); */
		if (!spidev->tx_buffer) {
				dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
				status = -ENOMEM;
			goto err_find_dev;
			}
		}

	if (!spidev->rx_buffer) {
            spidev->rx_buffer = kmalloc(4096, GFP_KERNEL);
            		/* spidev->rx_buffer = kmalloc(bufsiz, GFP_KERNEL); */
		if (!spidev->rx_buffer) {
			dev_dbg(&spidev->spi->dev, "open/ENOMEM\n");
			status = -ENOMEM;
			goto err_alloc_rx_buf;
		}
	}

	spidev->users++;
        spi->mode &= ~(0x03);//SPI_MODE_0
	/* filp->private_data = spidev; */
	/* nonseekable_open(inode, filp); */

	/* mutex_unlock(&device_list_lock); */
	return 0;

err_alloc_rx_buf:
	kfree(spidev->tx_buffer);
	spidev->tx_buffer = NULL;
err_find_dev:
	/* mutex_unlock(&device_list_lock); */
	return status;
}
void spidev_uninit(struct spidev_data *spidev)
{
    --spidev->users;
    if (!spidev->users) {
        int		dofree;

        kfree(spidev->tx_buffer);
        spidev->tx_buffer = NULL;

        kfree(spidev->rx_buffer);
        spidev->rx_buffer = NULL;

        spidev->speed_hz = spidev->spi->max_speed_hz;

        /* ... after we unbound from the underlying device? */
        spin_lock_irq(&spidev->spi_lock);
        dofree = (spidev->spi == NULL);
        spin_unlock_irq(&spidev->spi_lock);

        if (dofree)
            kfree(spidev);
    }

}
int nfc_major = 0;
int nfc_minor = 0;
dev_t nfcdev;
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
static struct class *spidev_class;
extern struct pn512_common *pn512;
struct spi_device *spi_device_for_pn512;
static int pcd_init(void);
static void pcd_exit(void);
static int spidev_probe(struct spi_device *spi)
{
	struct spidev_data	*spidev;
	int			status;
        struct cdev *nfc_cdev;       
	/* Allocate driver data */
                spidev = kzalloc(sizeof(*spidev), GFP_KERNEL);        
	if (!spidev)
            return -ENOMEM;
        /* pn512->spi_device = spi;//not reasonable */
        spi_device_for_pn512 = spi;
              
	spidev->spi = spi;
	spin_lock_init(&spidev->spi_lock);
	mutex_init(&spidev->buf_lock);
	INIT_LIST_HEAD(&spidev->device_entry);
               
	mutex_lock(&device_list_lock);
	/* Initialize the driver data */
        /**************************************/
               
        if(nfc_major){
            nfcdev = MKDEV(nfc_major, nfc_minor);
            status = register_chrdev_region(nfcdev, 1, "nfc");
        }
        else{
            status = alloc_chrdev_region(&nfcdev, nfc_minor, 1, "nfc");
            nfc_major = MAJOR(nfcdev);
        }      
        if(status < 0)
            {
               // printk(KERN_WARNING "NFC: can't get major %d\n", nfc_major);
                return status;
            }
        nfc_cdev = cdev_alloc();
        if(NULL==nfc_cdev){
            //printk(KERN_WARNING "NFC: can't alloc cdev!\n");
            return -1;
        }
        cdev_init(nfc_cdev, &pcd_fops);
        nfc_cdev->ops = &pcd_fops;
        status = cdev_add(nfc_cdev, nfcdev, 1);

        if(status){
            //printk(KERN_NOTICE "Error %d add NFC.\n", status);
        }
	spidev_class = class_create(THIS_MODULE, "nfcdev");
	if (IS_ERR(spidev_class)) {
		unregister_chrdev_region(nfc_major, 1);
		return PTR_ERR(spidev_class);
	}

	if (nfc_minor < 2) {
		struct device *dev;
		spidev->devt = nfcdev;
		dev = device_create(spidev_class, &spi->dev, spidev->devt, spidev, "nfcdev");
		status = PTR_ERR_OR_ZERO(dev);
	} else {
		dev_dbg(&spi->dev, "no minor number available!\n");
		status = -ENODEV;
	}
	if (status == 0) {
		list_add(&spidev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);

	spidev->speed_hz = spi->max_speed_hz;
	if (status == 0)
            {
		spi_set_drvdata(spi, spidev);
                pcd_init();
            }
	else
            {
                device_destroy(spidev_class, spidev->devt);
                class_destroy(spidev_class);
                kfree(spidev);

            }
        /* pcd_init(); */
        printk("%s-%s-%d,spidev_probe success!!!\n", __FILE__, __FUNCTION__, __LINE__);
	return status;
}

static int spidev_remove(struct spi_device *spi)
{
	struct spidev_data	*spidev = spi_get_drvdata(spi);

	/* make sure ops on existing fds can abort cleanly */
	spin_lock_irq(&spidev->spi_lock);
	spidev->spi = NULL;
	spin_unlock_irq(&spidev->spi_lock);

	/* prevent new opens */
	mutex_lock(&device_list_lock);                                           
	list_del(&spidev->device_entry);
	device_destroy(spidev_class, spidev->devt);
	/* clear_bit(MINOR(spidev->devt), minors); */
	if (spidev->users == 0)
		kfree(spidev);
	mutex_unlock(&device_list_lock);
        
        pcd_exit();
	return 0;
}

static const struct of_device_id spidev_dt_ids[] = {
	{ .compatible = "pn512" },
	{},
};

static struct spi_driver spidev_spi_driver = {
	.driver = {
		.name =		"mynfcdev",
		.owner =	THIS_MODULE,
		.of_match_table = of_match_ptr(spidev_dt_ids),
	},
	.probe =	spidev_probe,
	.remove =	spidev_remove,

	/* NOTE:  suspend/resume methods are not necessary here.
	 * We don't do anything except pass the requests to/from
	 * the underlying controller.  The refrigerator handles
	 * most issues; the controller driver handles the rest.
	 */
};
static int pcd_init(void)
{
	int ret;
	
	common = kzalloc(sizeof *common, GFP_KERNEL);
	if (!common)
	{
		ret = -ENOMEM;
		goto err1;
	}        
    sema_init(&common->mutex, 0);    // initial a semaphore, and lock it       
    ret = picc_init(common);//initial spi,etc
	if(ret)
		goto err2;       
    /*     ret = misc_register(&pcd_misc); */
    /* if(ret) */
    /* { */
    /*     ERROR_TO("fail to register device\n"); */
    /*     goto err3; */
    /* } */        
    common->polling = create_singlethread_workqueue("polling picc");
    if(!common->polling)
    {
        ERROR_TO("can't create work queue 'pcdPoll'\n");
		ret = -EFAULT;
        goto err4;
    }      
    run_picc_poll(0);        
    up(&common->mutex);                 
    return (0);

err4:
    /* misc_deregister(&pcd_misc); */
/* err3: */
	picc_uninit();
err2:	
    up(&common->mutex);
	kfree(common);
err1:	
    return ret;
}

static int __init spidev_init(void)
{
	int status;


	/* Claim our 256 reserved device numbers.  Then register a class
	 * that will key udev/mdev to add/remove /dev nodes.  Last, register
	 * the driver which manages those device numbers.
	 */

 
        status = spi_register_driver(&spidev_spi_driver);
	if (status < 0) {
		class_destroy(spidev_class);
		unregister_chrdev(nfc_major, spidev_spi_driver.driver.name);
	}
	return status;
}

static void pcd_exit(void)
{
	
    if (down_interruptible(&common->mutex)) 
    {
        return;
    }
   
    if(!cancel_delayed_work(&card_Poll)) 
    {
        flush_workqueue(common->polling);
    }
    destroy_workqueue(common->polling);

    picc_uninit();
	
    /* misc_deregister(&pcd_misc); */
	
    up(&common->mutex);

	kfree(common);
	
    return;
}

static void __exit spidev_exit(void)
{
	spi_unregister_driver(&spidev_spi_driver);
	class_destroy(spidev_class);
	unregister_chrdev(nfc_major, spidev_spi_driver.driver.name);
        pcd_exit();
}

module_init(spidev_init);
module_exit(spidev_exit);

MODULE_DESCRIPTION("Contactless Card Driver");
MODULE_AUTHOR("taoxianchong@gooagoo.com");
MODULE_LICENSE("GPL");


