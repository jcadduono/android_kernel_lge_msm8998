/*
 * stub_dev.c
 * - This file is derived form /drivers/usb/usbip/stub_dev.c
 *
 * Copyright (C) 2003-2008 Takahiro Hirofuchi
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307,
 * USA.
 */

#include <linux/device.h>
#include <linux/file.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/wakelock.h>
#include "mausb_common.h"
#include "stub.h"
#include "mausb_util.h"



struct wake_lock mausb_wl;
//void mausb_bind(void);
//void mausb_unbind(void);
/*
 * Define device IDs here if you want to explicitly limit exportable devices.
 * In most cases, wildcard matching will be okay because driver binding can be
 * changed dynamically by a userland program.
 */
static struct usb_device_id stub_table[] = {
	/* magic for wild card */
	{ .driver_info = 1 },
	{ 0, }                                     /* Terminating entry */
};

MODULE_DEVICE_TABLE(usb, stub_table);

/*
 * mausb_status shows the status of mausb-host as long as this driver is bound
 * to the target device.
 */
static ssize_t mausb_status_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct stub_device *sdev = dev_get_drvdata(dev);
	int status;
	unsigned long flags;
	if (!sdev) {
		dev_err(dev, "sdev is null\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&sdev->ud.lock,flags);
	status = sdev->ud.status;
	spin_unlock_irqrestore(&sdev->ud.lock,flags);

	return snprintf(buf, PAGE_SIZE, "%d\n", status);
}
static DEVICE_ATTR_RO(mausb_status);

/*
 * mausb_sockfd gets a socket descriptor of an established TCP connection that
 * is used to transfer mausb requests by kernel threads. -1 is a magic number
 * by which mausb connection is finished.
 */
static ssize_t store_sockfd(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct stub_device *sdev = dev_get_drvdata(dev);
	int sockfd = 0;
	struct socket *socket;
	int rv;
	unsigned long flags;
	//struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };//sandeep
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"\n---> %s",__func__);

	if (!sdev) {
		dev_err(dev, "sdev is null\n");
		return -ENODEV;
	}

	rv = sscanf(buf, "%d", &sockfd);
	if (rv != 1)
		return -EINVAL;

	if (sockfd != -1) {
		int err;
		dev_info(dev, "stub up\n");

		spin_lock_irqsave(&sdev->ud.lock,flags);

		if (sdev->ud.status != MAUSB_SDEV_ST_AVAILABLE) {
			dev_err(dev, "not ready\n");
			goto err;
		}

		socket = sockfd_lookup(sockfd, &err);
		if (!socket)
			goto err;
		sdev->ud.tcp_socket = socket;
		wake_lock_init(&mausb_wl, WAKE_LOCK_SUSPEND, "mausb_wakelock");
		wake_lock(&mausb_wl);
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN," Init wakelock");
		spin_unlock_irqrestore(&sdev->ud.lock,flags);

		sdev->ud.tcp_rx = kthread_get_run(stub_rx_loop, &sdev->ud,
						  "stub_rx");
		sdev->ud.tcp_tx = kthread_get_run(stub_tx_loop, &sdev->ud,
						  "stub_tx");

		//sched_setscheduler(sdev->ud.tcp_rx, SCHED_FIFO, &param);//sandeep

		spin_lock_irqsave(&sdev->ud.lock,flags);
		sdev->ud.status = MAUSB_SDEV_ST_USED;
		spin_unlock_irqrestore(&sdev->ud.lock,flags);
	} else {
		dev_info(dev, "stub down\n");

		spin_lock_irqsave(&sdev->ud.lock,flags);
		if (sdev->ud.status != MAUSB_SDEV_ST_USED)
			goto err;

		spin_unlock_irqrestore(&sdev->ud.lock,flags);

		mausb_event_add(&sdev->ud, SDEV_EVENT_DOWN);
	}

	return count;

err:
	spin_unlock_irqrestore(&sdev->ud.lock,flags);
	return -EINVAL;
}
static DEVICE_ATTR(mausb_sockfd, S_IWUSR, NULL, store_sockfd);



static int stub_add_files(struct device *dev)
{
	int err = 0;
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"---> %s",__func__);

	err = device_create_file(dev, &dev_attr_mausb_status);
	if (err)
		goto err_status;

	err = device_create_file(dev, &dev_attr_mausb_sockfd);
	if (err)
		goto err_sockfd;

	err = device_create_file(dev, &dev_attr_mausb_debug);
	if (err)
		goto err_debug;
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"<-- %s",__func__);

	return 0;

err_debug:
	device_remove_file(dev, &dev_attr_mausb_sockfd);
err_sockfd:
	device_remove_file(dev, &dev_attr_mausb_status);

err_status:
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,
			"\n Error in creating attributes");
	return err;
}

static void stub_remove_files(struct device *dev)
{
	device_remove_file(dev, &dev_attr_mausb_status);
	device_remove_file(dev, &dev_attr_mausb_sockfd);
	device_remove_file(dev, &dev_attr_mausb_debug);
}

static void stub_shutdown_connection(struct mausb_device *ud)
{
	struct stub_device *sdev = container_of(ud, struct stub_device, ud);
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN," ---> stub_shutdown_connection");
	/*
	 * When removing an exported device, kernel panic sometimes occurred
	 * and then EIP was sk_wait_data of stub_rx thread. Is this because
	 * sk_wait_data returned though stub_rx thread was already finished by
	 * step 1?
	 */
	//mausb_unbind();
	if (ud->tcp_socket) {
		dev_dbg(&sdev->udev->dev, "shutdown tcp_socket %p\n",
			ud->tcp_socket);
		kernel_sock_shutdown(ud->tcp_socket, SHUT_RDWR);
	}

	/* 1. stop threads */
	if (ud->tcp_rx) {
		kthread_stop_put(ud->tcp_rx);
		ud->tcp_rx = NULL;
	}
	if (ud->tcp_tx) {
		kthread_stop_put(ud->tcp_tx);
		ud->tcp_tx = NULL;
	}

	/*
	 * 2. close the socket
	 *
	 * tcp_socket is freed after threads are killed so that mausb_xmit does
	 * not touch NULL socket.
	 */
	if (ud->tcp_socket) {
		sockfd_put(ud->tcp_socket);
		ud->tcp_socket = NULL;
	}

	/* 3. free used data */
	stub_device_cleanup_urbs(sdev);

	/* 4. free stub_unlink */
	{
		unsigned long flags;
		struct stub_mausb_pal *mausb_unlink, *mausb_tmp;

		spin_lock_irqsave(&sdev->mausb_pal_lock, flags);
		list_for_each_entry_safe(mausb_unlink, mausb_tmp, &sdev->mausb_unlink_tx, list) {
			list_del(&mausb_unlink->list);
			kfree(mausb_unlink->pdu);
			kfree(mausb_unlink);
		}
		list_for_each_entry_safe(mausb_unlink, mausb_tmp, &sdev->mausb_unlink_free,
					 list) {
			list_del(&mausb_unlink->list);
			kfree(mausb_unlink->pdu);
			kfree(mausb_unlink);
		}
		spin_unlock_irqrestore(&sdev->mausb_pal_lock, flags);

	}
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"<-- stub_shutdown_connection");
}

static void stub_device_reset(struct mausb_device *ud)
{
	struct stub_device *sdev = container_of(ud, struct stub_device, ud);
	struct usb_device *udev = sdev->udev;
	int ret = 0;

	dev_dbg(&udev->dev, "device reset");
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN," ---> stub_device_reset");
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"-->stub_device_reset lock for reset ret=%d\n",ret);


	spin_lock_irq(&ud->lock);
	if (ret) {
		dev_err(&udev->dev, "device reset\n");
		ud->status = MAUSB_SDEV_ST_ERROR;
	} else {
		dev_info(&udev->dev, "device reset\n");
		ud->status = MAUSB_SDEV_ST_AVAILABLE;
	}
	spin_unlock_irq(&ud->lock);
	if(wake_lock_active(&mausb_wl))
	{
	wake_unlock(&mausb_wl);
	wake_lock_destroy(&mausb_wl);
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN," Removing wakelock");
	}
//	mausb_start_upnp();
	//mausb_bind();

	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN," <-- stub_device_reset");
}

static void stub_device_unusable(struct mausb_device *ud)
{
	spin_lock_irq(&ud->lock);
	ud->status = MAUSB_SDEV_ST_ERROR;
	spin_unlock_irq(&ud->lock);
}

/**
 * stub_device_alloc - allocate a new stub_device struct
 * @interface: usb_interface of a new device
 *
 * Allocates and initializes a new stub_device struct.
 */
static struct stub_device *stub_device_alloc(struct usb_device *udev,
					     struct usb_interface *interface)
{
	struct stub_device *sdev;
	int busnum = interface_to_busnum(interface);
	int devnum = interface_to_devnum(interface);

	dev_dbg(&interface->dev, "allocating stub device");

	/* yes, it's a new device */
	sdev = kzalloc(sizeof(struct stub_device), GFP_KERNEL);
	if (!sdev)
		return NULL;

	sdev->interface = usb_get_intf(interface);
	sdev->udev = usb_get_dev(udev);

	/*
	 * devid is defined with devnum when this driver is first allocated.
	 * devnum may change later if a device is reset. However, devid never
	 * changes during a mausb connection.
	 */
	sdev->devid		= (busnum << 16) | devnum;
	sdev->ud.side		= MAUSB_STUB;
	sdev->ud.status		= MAUSB_SDEV_ST_AVAILABLE;
	spin_lock_init(&sdev->ud.lock);
	sdev->ud.tcp_socket	= NULL;

	INIT_LIST_HEAD(&sdev->mausb_pal_mgmt_init);
	INIT_LIST_HEAD(&sdev->mausb_pal_in_init);
	INIT_LIST_HEAD(&sdev->mausb_pal_out_init);
	INIT_LIST_HEAD(&sdev->mausb_pal_submit);
	INIT_LIST_HEAD(&sdev->mausb_pal_tx);
	INIT_LIST_HEAD(&sdev->mausb_pal_free);

	INIT_LIST_HEAD(&sdev->mausb_unlink_free);
	INIT_LIST_HEAD(&sdev->mausb_unlink_tx);

	spin_lock_init(&sdev->mausb_pal_lock);

	init_waitqueue_head(&sdev->tx_waitq);

	sdev->ud.mausb_eh_ops.shutdown = stub_shutdown_connection;
	sdev->ud.mausb_eh_ops.reset    = stub_device_reset;
	sdev->ud.mausb_eh_ops.unusable = stub_device_unusable;
	mausb_start_eh(&sdev->ud);

	dev_dbg(&interface->dev, "register new interface\n");

	return sdev;
}

static void stub_device_free(struct stub_device *sdev)
{
	kfree(sdev);
}

/*
 * If a usb device has multiple active interfaces, this driver is bound to all
 * the active interfaces. However, mausb exports *a* usb device (i.e., not *an*
 * active interface). Currently, a userland program must ensure that it
 * looks at the mausb's sysfs entries of only the first active interface.
 *
 * TODO: use "struct usb_device_driver" to bind a usb device.
 * However, it seems it is not fully supported in mainline kernel yet
 * (2.6.19.2).
 */
static int stub_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct stub_device *sdev = NULL;
	const char *udev_busid = dev_name(interface->dev.parent);
	int err = 0;
	struct bus_id_priv *busid_priv=NULL;

	//WARN_ON(1);

	dev_dbg(&interface->dev, "Enter\n");
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"---> %s",__func__);
	/* check we should claim or not by busid_table */
	busid_priv = get_busid_priv(udev_busid);
	if (!busid_priv || (busid_priv->status == MAUSB_STUB_BUSID_REMOV) ||
	    (busid_priv->status == MAUSB_STUB_BUSID_OTHER)) {
		dev_info(&interface->dev, "%s is not in match_busid table... "
			 "skip!\n", udev_busid);

		/*
		 * Return value should be ENODEV or ENOXIO to continue trying
		 * other matched drivers by the driver core.
		 * See driver_probe_device() in driver/base/dd.c
		 */
		 if(!busid_priv)
			 printk(KERN_INFO"stub_probe busid_priv is NULL \n");
		 if(busid_priv)
		 {
		  if(busid_priv->status == MAUSB_STUB_BUSID_REMOV)
			  printk(KERN_INFO"stub_probe MAUSB_STUB_BUSID_REMOV\n");
		  if(busid_priv->status == MAUSB_STUB_BUSID_OTHER)
			  printk(KERN_INFO"stub_probe MAUSB_STUB_BUSID_OTHER\n");
		 }
		return -ENODEV;
	}

	if (udev->descriptor.bDeviceClass == USB_CLASS_HUB) {
		dev_info(&udev->dev, "%s is a usb hub device... skip!\n",
			 udev_busid);
		return -ENODEV;
	}

	if (!strcmp(udev->bus->bus_name, "vhci_hcd")) {
		dev_info(&udev->dev, "%s is attached on vhci_hcd... skip!\n",
			 udev_busid);
		return -ENODEV;
	}

	if (busid_priv->status == MAUSB_STUB_BUSID_ALLOC) {
		sdev = busid_priv->sdev;
		if (!sdev)
			return -ENODEV;

		busid_priv->interf_count++;
		dev_info(&interface->dev, "mausb-host: register new interface "
			 "(bus %u dev %u ifn %u)\n",
			 udev->bus->busnum, udev->devnum,
			 interface->cur_altsetting->desc.bInterfaceNumber);

		/* set private data to usb_interface */
		usb_set_intfdata(interface, sdev);

		err = stub_add_files(&interface->dev);
		if (err) {
			dev_err(&interface->dev, "stub_add_files for %s\n",
				udev_busid);
			usb_set_intfdata(interface, NULL);
			busid_priv->interf_count--;
			return err;
		}

		usb_get_intf(interface);
		return 0;
	}

	/* ok, this is my device */
	sdev = stub_device_alloc(udev, interface);
	if (!sdev)
		return -ENOMEM;

	dev_info(&interface->dev, "mausb-host: register new device "
		 "(bus %u dev %u ifn %u)\n", udev->bus->busnum, udev->devnum,
		 interface->cur_altsetting->desc.bInterfaceNumber);

	busid_priv->interf_count = 0;
	busid_priv->shutdown_busid = 0;

	/* set private data to usb_interface */
	usb_set_intfdata(interface, sdev);
	busid_priv->interf_count++;
	busid_priv->sdev = sdev;

	err = stub_add_files(&interface->dev);
	if (err) {
		dev_err(&interface->dev, "stub_add_files for %s\n", udev_busid);
		usb_set_intfdata(interface, NULL);
		usb_put_intf(interface);
		usb_put_dev(udev);
		kthread_stop_put(sdev->ud.eh);

		busid_priv->interf_count = 0;
		busid_priv->sdev = NULL;
		stub_device_free(sdev);
		return err;
	}
	busid_priv->status = MAUSB_STUB_BUSID_ALLOC;

	return 0;
}

static void shutdown_busid(struct bus_id_priv *busid_priv)
{
	if (busid_priv->sdev && !busid_priv->shutdown_busid) {
		busid_priv->shutdown_busid = 1;

		/*
		if (busid_priv->sdev->ud.tcp_rx) {
			kthread_stop_put(busid_priv->sdev->ud.tcp_rx);
			busid_priv->sdev->ud.tcp_rx = NULL;
		}
		if (busid_priv->sdev->ud.tcp_tx) {
			kthread_stop_put(busid_priv->sdev->ud.tcp_tx);
			busid_priv->sdev->ud.tcp_tx = NULL;
		}
		*/
		mausb_event_add(&busid_priv->sdev->ud, SDEV_EVENT_REMOVED);

		/* wait for the stop of the event handler */
		mausb_stop_eh(&busid_priv->sdev->ud);
	}
}

/*
 * called in usb_disconnect() or usb_deregister()
 * but only if actconfig(active configuration) exists
 */
static void stub_disconnect(struct usb_interface *interface)
{
	struct stub_device *sdev;
	const char *udev_busid = dev_name(interface->dev.parent);
	struct bus_id_priv *busid_priv;

	WARN_ON(1);

	dev_dbg(&interface->dev, "Enter\n");
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"---> %s",__func__);

	busid_priv = get_busid_priv(udev_busid);
	if (!busid_priv) {
		BUG();
		return;
	}

	sdev = usb_get_intfdata(interface);

	/* get stub_device */
	if (!sdev) {
		dev_err(&interface->dev, "could not get device");
		return;
	}

	usb_set_intfdata(interface, NULL);

	/*
	 * NOTE: rx/tx threads are invoked for each usb_device.
	 */
	stub_remove_files(&interface->dev);

	/* If usb reset is called from event handler */
	if (busid_priv->sdev->ud.eh == current) {
		busid_priv->interf_count--;
		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,
				"Event Handler called stub_disconnect");
		return;
	}

	if (busid_priv->interf_count > 1) {

		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,
			"---> %s busid_priv=%p busid_priv->interf_count=%d ",
			__func__,busid_priv,busid_priv->interf_count);
		busid_priv->interf_count--;
		shutdown_busid(busid_priv);
		usb_put_intf(interface);
		return;
	}

	busid_priv->interf_count = 0;
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,
			"---> %s busid_priv=%p busid_priv->interf_count=%d ",
			__func__,busid_priv,busid_priv->interf_count);
	/* shutdown the current connection */
	shutdown_busid(busid_priv);

	usb_put_dev(sdev->udev);
	usb_put_intf(interface);

	/* free sdev */
	busid_priv->sdev = NULL;
	stub_device_free(sdev);

	if (busid_priv->status == MAUSB_STUB_BUSID_ALLOC) {
		busid_priv->status = MAUSB_STUB_BUSID_ADDED;
	} else {
		busid_priv->status = MAUSB_STUB_BUSID_OTHER;
		del_match_busid((char *)udev_busid);
	}
}

/*
 * Presence of pre_reset and post_reset prevents the driver from being unbound
 * when the device is being reset
 */
static int stub_pre_reset(struct usb_interface *interface)
{
	dev_dbg(&interface->dev, "pre_reset\n");
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"---> %s",__func__);
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"<-- %s",__func__);
	return 0;
}

static int stub_post_reset(struct usb_interface *interface)
{

	const char *udev_busid = dev_name(interface->dev.parent);
	struct bus_id_priv *busid_priv;
	struct stub_device *sdev = NULL;
	dev_dbg(&interface->dev, "post_reset\n");
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"---> %s",__func__);

	busid_priv = get_busid_priv(udev_busid);

	if(busid_priv) {
		sdev = busid_priv->sdev;
		/* 3. free used data */
		//stub_device_cleanup_urbs(sdev);
	}
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MAIN,"<-- %s",__func__);
	return 0;
}

/*static int stub_suspend(struct usb_interface *interface, pm_message_t message)
{
         pr_info("%s\n", __func__);

         return 0;
}

static int stub_resume(struct usb_interface *interface)
{
         pr_info("%s\n", __func__);

         return 0;
}*/



struct usb_driver stub_driver = {
	.name		= "mausb-host",
	.probe		= stub_probe,
	.disconnect	= stub_disconnect,
	//.suspend    = stub_suspend,
	//.resume     = stub_resume,
	.id_table	= stub_table,
	.pre_reset	= stub_pre_reset,
	.post_reset	= stub_post_reset,
};
