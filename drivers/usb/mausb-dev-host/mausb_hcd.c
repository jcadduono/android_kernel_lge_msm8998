/*
 * mausb_hcd.c
 *
 * This file is derived from dummy_hcd.c
 *
 * Copyright (C) 2003 David Brownell
 * Copyright (C) 2003-2005 Alan Stern
 * Copyright (C) 2016 LGE Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */


/*
 * This exposes a device side "USB gadget" API, driven by requests to a
 * Linux-USB host controller driver.  USB traffic is simulated; there's
 * no need for USB hardware.  Use this with two other drivers:
 *
 *  - Gadget driver, responding to requests (slave);
 *  - Host-side device driver, as already familiar in Linux.
 *
 * Having this all in one kernel can help some stages of development,
 * bypassing some hardware (and driver) issues.  UML could help too.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/scatterlist.h>

#include <asm/byteorder.h>
#include <linux/io.h>
#include <asm/irq.h>
#include <asm/unaligned.h>

#define DRIVER_DESC	"USB MAUSB Host+Gadget Emulator"
#define DRIVER_VERSION	"02 May 2005"

#define POWER_BUDGET	500	/* in mA; use 8 for low-power port testing */

static int enable_mausb;
module_param(enable_mausb, int, S_IRUGO | S_IWUSR);

static const char	driver_name[] = "mausb_hcd";
static const char	driver_desc[] = DRIVER_DESC;

static const char	gadget_name[] = "mausb_udc";

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("David Brownell");
MODULE_LICENSE("GPL");

struct mausb_hcd_module_parameters {
	bool is_super_speed;
	bool is_high_speed;
	unsigned int num;
};

static struct mausb_hcd_module_parameters mod_data = {
	.is_super_speed = false,
	.is_high_speed = true,
	.num = 1,
};
module_param_named(is_super_speed, mod_data.is_super_speed, bool, S_IRUGO);
MODULE_PARM_DESC(is_super_speed, "true to simulate SuperSpeed connection");
module_param_named(is_high_speed, mod_data.is_high_speed, bool, S_IRUGO);
MODULE_PARM_DESC(is_high_speed, "true to simulate HighSpeed connection");
module_param_named(num, mod_data.num, uint, S_IRUGO);
MODULE_PARM_DESC(num, "number of emulated controllers");
/*-------------------------------------------------------------------------*/

/* gadget side driver data structres */
struct mausb_ep {
	struct list_head		queue;
	unsigned long			last_io;	/* jiffies timestamp */
	struct usb_gadget		*gadget;
	const struct usb_endpoint_descriptor *desc;
	struct usb_ep			ep;
	unsigned			halted:1;
	unsigned			wedged:1;
	unsigned			already_seen:1;
	unsigned			setup_stage:1;
	unsigned			stream_en:1;
};

struct mausb_request {
	struct list_head		queue;		/* ep's requests */
	struct usb_request		req;
};

static inline struct mausb_ep *usb_ep_to_mausb_ep(struct usb_ep *_ep)
{
	return container_of(_ep, struct mausb_ep, ep);
}

static inline struct mausb_request *usb_request_to_mausb_request
		(struct usb_request *_req)
{
	return container_of(_req, struct mausb_request, req);
}

/*-------------------------------------------------------------------------*/

/*
 * Every device has ep0 for control requests, plus up to 30 more endpoints,
 * in one of two types:
 *
 *   - Configurable:  direction (in/out), type (bulk, iso, etc), and endpoint
 *     number can be changed.  Names like "ep-a" are used for this type.
 *
 *   - Fixed Function:  in other cases.  some characteristics may be mutable;
 *     that'd be hardware-specific.  Names like "ep12out-bulk" are used.
 *
 * Gadget drivers are responsible for not setting up conflicting endpoint
 * configurations, illegal or unsupported packet lengths, and so on.
 */

static const char ep0name[] = "ep0";

static const char *const ep_name[] = {
	ep0name,				/* everyone has ep0 */

	/* act like a pxa250: fifteen fixed function endpoints */
	"ep1in-bulk", "ep2out-bulk", "ep3in-iso", "ep4out-iso", "ep5in-int",
	"ep6in-bulk", "ep7out-bulk", "ep8in-iso", "ep9out-iso", "ep10in-int",
	"ep11in-bulk", "ep12out-bulk", "ep13in-iso", "ep14out-iso",
		"ep15in-int",

	/* or like sa1100: two fixed function endpoints */
	"ep1out-bulk", "ep2in-bulk",

	/* and now some generic EPs so we have enough in multi config */
	"ep3out", "ep4in", "ep5out", "ep6out", "ep7in", "ep8out", "ep9in",
	"ep10out", "ep11out", "ep12in", "ep13out", "ep14in", "ep15out",
};
#define MAUSB_ENDPOINTS	ARRAY_SIZE(ep_name)

/*-------------------------------------------------------------------------*/

#define FIFO_SIZE		64

struct urbp {
	struct urb		*urb;
	struct list_head	urbp_list;
	struct sg_mapping_iter	miter;
	u32			miter_started;
};


enum mausb_rh_state {
	MAUSB_RH_RESET,
	MAUSB_RH_SUSPENDED,
	MAUSB_RH_RUNNING
};

struct mausb_hcd {
	struct mausb			*mu;
	enum mausb_rh_state		rh_state;
	struct tasklet_hrtimer		ttimer;
	u32				port_status;
	u32				old_status;
	unsigned long			re_timeout;

	struct usb_device		*udev;
	struct list_head		urbp_list;
	u32				stream_en_ep;
	u8				num_stream[30 / 2];

	unsigned			active:1;
	unsigned			old_active:1;
	unsigned			resuming:1;
};

struct mausb {
	spinlock_t			lock;

	/*
	 * SLAVE/GADGET side support
	 */
	struct mausb_ep			ep[MAUSB_ENDPOINTS];
	int				address;
	struct usb_gadget		gadget;
	struct usb_gadget_driver	*driver;
	struct mausb_request		fifo_req;
	u8				fifo_buf[FIFO_SIZE];
	u16				devstatus;
	unsigned			udc_suspended:1;
	unsigned			pullup:1;

	/*
	 * MASTER/HOST side support
	 */
	struct mausb_hcd		*hs_hcd;
	struct mausb_hcd		*ss_hcd;
};

static inline struct mausb_hcd *hcd_to_mausb_hcd(struct usb_hcd *hcd)
{
	return (struct mausb_hcd *) (hcd->hcd_priv);
}

static inline struct usb_hcd *mausb_hcd_to_hcd(struct mausb_hcd *mausb)
{
	return container_of((void *) mausb, struct usb_hcd, hcd_priv);
}

static inline struct device *mausb_dev(struct mausb_hcd *mu)
{
	return mausb_hcd_to_hcd(mu)->self.controller;
}

static inline struct device *udc_dev(struct mausb *mu)
{
	return mu->gadget.dev.parent;
}

static inline struct mausb *ep_to_mausb(struct mausb_ep *ep)
{
	return container_of(ep->gadget, struct mausb, gadget);
}

static inline struct mausb_hcd *gadget_to_mausb_hcd(struct usb_gadget *gadget)
{
	struct mausb *mu = container_of(gadget, struct mausb, gadget);
	if (mu->gadget.speed == USB_SPEED_SUPER)
		return mu->ss_hcd;
	else
		return mu->hs_hcd;
}

static inline struct mausb *gadget_dev_to_mausb(struct device *dev)
{
	return container_of(dev, struct mausb, gadget.dev);
}

/*-------------------------------------------------------------------------*/

/* SLAVE/GADGET SIDE UTILITY ROUTINES */

/* called with spinlock held */
static void nuke(struct mausb *mu, struct mausb_ep *ep)
{
	while (!list_empty(&ep->queue)) {
		struct mausb_request	*req;

		req = list_entry(ep->queue.next, struct mausb_request, queue);
		list_del_init(&req->queue);
		req->req.status = -ESHUTDOWN;

		spin_unlock(&mu->lock);
		usb_gadget_giveback_request(&ep->ep, &req->req);
		spin_lock(&mu->lock);
	}
}

/* caller must hold lock */
static void stop_activity(struct mausb *mu)
{
	struct mausb_ep	*ep;

	/* prevent any more requests */
	mu->address = 0;

	/* The timer is left running so that outstanding URBs can fail */

	/* nuke any pending requests first, so driver i/o is quiesced */
	list_for_each_entry(ep, &mu->gadget.ep_list, ep.ep_list)
		nuke(mu, ep);

	/* driver now does any non-usb quiescing necessary */
}

/**
 * set_link_state_by_speed() - Sets the current state of the link according to
 *	the hcd speed
 * @mu_hcd: pointer to the mausb_hcd structure to update the link state for
 *
 * This function updates the port_status according to the link state and the
 * speed of the hcd.
 */
static void set_link_state_by_speed(struct mausb_hcd *mu_hcd)
{
	struct mausb *mu = mu_hcd->mu;

	if (mausb_hcd_to_hcd(mu_hcd)->speed == HCD_USB3) {
		if ((mu_hcd->port_status & USB_SS_PORT_STAT_POWER) == 0) {
			mu_hcd->port_status = 0;
		} else if (!mu->pullup || mu->udc_suspended) {
			/* UDC suspend must cause a disconnect */
			mu_hcd->port_status &= ~(USB_PORT_STAT_CONNECTION |
						USB_PORT_STAT_ENABLE);
			if ((mu_hcd->old_status &
			     USB_PORT_STAT_CONNECTION) != 0)
				mu_hcd->port_status |=
					(USB_PORT_STAT_C_CONNECTION << 16);
		} else {
			/* device is connected and not suspended */
			mu_hcd->port_status |= (USB_PORT_STAT_CONNECTION |
						 USB_PORT_STAT_SPEED_5GBPS) ;
			if ((mu_hcd->old_status &
			     USB_PORT_STAT_CONNECTION) == 0)
				mu_hcd->port_status |=
					(USB_PORT_STAT_C_CONNECTION << 16);
			if ((mu_hcd->port_status &
			     USB_PORT_STAT_ENABLE) == 1 &&
				(mu_hcd->port_status &
				 USB_SS_PORT_LS_U0) == 1 &&
				mu_hcd->rh_state != MAUSB_RH_SUSPENDED)
				mu_hcd->active = 1;
		}
	} else {
		if ((mu_hcd->port_status & USB_PORT_STAT_POWER) == 0) {
			mu_hcd->port_status = 0;
		} else if (!mu->pullup || mu->udc_suspended) {
			/* UDC suspend must cause a disconnect */
			mu_hcd->port_status &= ~(USB_PORT_STAT_CONNECTION |
						USB_PORT_STAT_ENABLE |
						USB_PORT_STAT_LOW_SPEED |
						USB_PORT_STAT_HIGH_SPEED |
						USB_PORT_STAT_SUSPEND);
			if ((mu_hcd->old_status &
			     USB_PORT_STAT_CONNECTION) != 0)
				mu_hcd->port_status |=
					(USB_PORT_STAT_C_CONNECTION << 16);
		} else {
			mu_hcd->port_status |= USB_PORT_STAT_CONNECTION;
			if ((mu_hcd->old_status &
			     USB_PORT_STAT_CONNECTION) == 0)
				mu_hcd->port_status |=
					(USB_PORT_STAT_C_CONNECTION << 16);
			if ((mu_hcd->port_status & USB_PORT_STAT_ENABLE) == 0)
				mu_hcd->port_status &= ~USB_PORT_STAT_SUSPEND;
			else if ((mu_hcd->port_status &
				  USB_PORT_STAT_SUSPEND) == 0 &&
					mu_hcd->rh_state != MAUSB_RH_SUSPENDED)
				mu_hcd->active = 1;
		}
	}
}

/* caller must hold lock */
static void set_link_state(struct mausb_hcd *mu_hcd)
{
	struct mausb *mu = mu_hcd->mu;

	mu_hcd->active = 0;
	if (mu->pullup)
		if ((mausb_hcd_to_hcd(mu_hcd)->speed == HCD_USB3 &&
		     mu->gadget.speed != USB_SPEED_SUPER) ||
		    (mausb_hcd_to_hcd(mu_hcd)->speed != HCD_USB3 &&
		     mu->gadget.speed == USB_SPEED_SUPER))
			return;

	set_link_state_by_speed(mu_hcd);

	if ((mu_hcd->port_status & USB_PORT_STAT_ENABLE) == 0 ||
	     mu_hcd->active)
		mu_hcd->resuming = 0;

	/* if !connected or reset */
	if ((mu_hcd->port_status & USB_PORT_STAT_CONNECTION) == 0 ||
			(mu_hcd->port_status & USB_PORT_STAT_RESET) != 0) {
		/*
		 * We're connected and not reset (reset occurred now),
		 * and driver attached - disconnect!
		 */
		if ((mu_hcd->old_status & USB_PORT_STAT_CONNECTION) != 0 &&
		    (mu_hcd->old_status & USB_PORT_STAT_RESET) == 0 &&
		    mu->driver) {
			stop_activity(mu);
			spin_unlock(&mu->lock);
			mu->driver->disconnect(&mu->gadget);
			spin_lock(&mu->lock);
		}
	} else if (mu_hcd->active != mu_hcd->old_active) {
		if (mu_hcd->old_active && mu->driver->suspend) {
			spin_unlock(&mu->lock);
			mu->driver->suspend(&mu->gadget);
			spin_lock(&mu->lock);
		} else if (!mu_hcd->old_active &&  mu->driver->resume) {
			spin_unlock(&mu->lock);
			mu->driver->resume(&mu->gadget);
			spin_lock(&mu->lock);
		}
	}

	mu_hcd->old_status = mu_hcd->port_status;
	mu_hcd->old_active = mu_hcd->active;
}

/*-------------------------------------------------------------------------*/

/* SLAVE/GADGET SIDE DRIVER
 *
 * This only tracks gadget state.  All the work is done when the host
 * side tries some (emulated) i/o operation.  Real device controller
 * drivers would do real i/o using dma, fifos, irqs, timers, etc.
 */

#define is_enabled(mu) \
	(mu->port_status & USB_PORT_STAT_ENABLE)

static int mausb_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct mausb		*mu;
	struct mausb_hcd	*mu_hcd;
	struct mausb_ep		*ep;
	unsigned		max;
	int			retval;

	ep = usb_ep_to_mausb_ep(_ep);
	if (!_ep || !desc || ep->desc || _ep->name == ep0name
			|| desc->bDescriptorType != USB_DT_ENDPOINT)
		return -EINVAL;
	mu = ep_to_mausb(ep);
	if (!mu->driver)
		return -ESHUTDOWN;

	mu_hcd = gadget_to_mausb_hcd(&mu->gadget);
	if (!is_enabled(mu_hcd))
		return -ESHUTDOWN;

	/*
	 * For HS/FS devices only bits 0..10 of the wMaxPacketSize represent the
	 * maximum packet size.
	 * For SS devices the wMaxPacketSize is limited by 1024.
	 */
	max = usb_endpoint_maxp(desc) & 0x7ff;

	/* drivers must not request bad settings, since lower levels
	 * (hardware or its drivers) may not check.  some endpoints
	 * can't do iso, many have maxpacket limitations, etc.
	 *
	 * since this "hardware" driver is here to help debugging, we
	 * have some extra sanity checks.  (there could be more though,
	 * especially for "ep9out" style fixed function ones.)
	 */
	retval = -EINVAL;
	switch (usb_endpoint_type(desc)) {
	case USB_ENDPOINT_XFER_BULK:
		if (strstr(ep->ep.name, "-iso")
				|| strstr(ep->ep.name, "-int")) {
			goto done;
		}
		switch (mu->gadget.speed) {
		case USB_SPEED_SUPER:
			if (max == 1024)
				break;
			goto done;
		case USB_SPEED_HIGH:
			if (max == 512)
				break;
			goto done;
		case USB_SPEED_FULL:
			if (max == 8 || max == 16 || max == 32 || max == 64)
				/* we'll fake any legal size */
				break;
			/* save a return statement */
		default:
			goto done;
		}
		break;
	case USB_ENDPOINT_XFER_INT:
		if (strstr(ep->ep.name, "-iso")) /* bulk is ok */
			goto done;
		/* real hardware might not handle all packet sizes */
		switch (mu->gadget.speed) {
		case USB_SPEED_SUPER:
		case USB_SPEED_HIGH:
			if (max <= 1024)
				break;
			/* save a return statement */
		case USB_SPEED_FULL:
			if (max <= 64)
				break;
			/* save a return statement */
		default:
			if (max <= 8)
				break;
			goto done;
		}
		break;
	case USB_ENDPOINT_XFER_ISOC:
		if (strstr(ep->ep.name, "-bulk")
				|| strstr(ep->ep.name, "-int"))
			goto done;
		/* real hardware might not handle all packet sizes */
		switch (mu->gadget.speed) {
		case USB_SPEED_SUPER:
		case USB_SPEED_HIGH:
			if (max <= 1024)
				break;
			/* save a return statement */
		case USB_SPEED_FULL:
			if (max <= 1023)
				break;
			/* save a return statement */
		default:
			goto done;
		}
		break;
	default:
		/* few chips support control except on ep0 */
		goto done;
	}

	_ep->maxpacket = max;
	if (usb_ss_max_streams(_ep->comp_desc)) {
		if (!usb_endpoint_xfer_bulk(desc)) {
			dev_err(udc_dev(mu), "Can't enable stream support on "
					"non-bulk ep %s\n", _ep->name);
			return -EINVAL;
		}
		ep->stream_en = 1;
	}
	ep->desc = desc;

	dev_dbg(udc_dev(mu), "enabled %s (ep%d%s-%s) maxpacket %d stream %s\n",
		_ep->name,
		desc->bEndpointAddress & 0x0f,
		(desc->bEndpointAddress & USB_DIR_IN) ? "in" : "out",
		({ char *val;
		 switch (usb_endpoint_type(desc)) {
		 case USB_ENDPOINT_XFER_BULK:
			 val = "bulk";
			 break;
		 case USB_ENDPOINT_XFER_ISOC:
			 val = "iso";
			 break;
		 case USB_ENDPOINT_XFER_INT:
			 val = "intr";
			 break;
		 default:
			 val = "ctrl";
			 break;
		 } val; }),
		max, ep->stream_en ? "enabled" : "disabled");

	/* at this point real hardware should be NAKing transfers
	 * to that endpoint, until a buffer is queued to it.
	 */
	ep->halted = ep->wedged = 0;
	retval = 0;
done:
	return retval;
}

static int mausb_disable(struct usb_ep *_ep)
{
	struct mausb_ep		*ep;
	struct mausb		*mu;
	unsigned long		flags;

	ep = usb_ep_to_mausb_ep(_ep);
	if (!_ep || !ep->desc || _ep->name == ep0name)
		return -EINVAL;
	mu = ep_to_mausb(ep);

	spin_lock_irqsave(&mu->lock, flags);
	ep->desc = NULL;
	ep->stream_en = 0;
	nuke(mu, ep);
	spin_unlock_irqrestore(&mu->lock, flags);

	dev_dbg(udc_dev(mu), "disabled %s\n", _ep->name);
	return 0;
}

static struct usb_request *mausb_alloc_request(struct usb_ep *_ep,
		gfp_t mem_flags)
{
	struct mausb_ep		*ep;
	struct mausb_request	*req;

	if (!_ep)
		return NULL;
	ep = usb_ep_to_mausb_ep(_ep);

	req = kzalloc(sizeof(*req), mem_flags);
	if (!req)
		return NULL;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

static void mausb_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct mausb_request	*req;

	if (!_ep || !_req) {
		WARN_ON(1);
		return;
	}

	req = usb_request_to_mausb_request(_req);
	WARN_ON(!list_empty(&req->queue));
	kfree(req);
}

/*static void fifo_complete(struct usb_ep *ep, struct usb_request *req)
{
}*/

static int mausb_queue(struct usb_ep *_ep, struct usb_request *_req,
		gfp_t mem_flags)
{
	struct mausb_ep		*ep;
	struct mausb_request	*req;
	struct mausb		*mu;
	struct mausb_hcd	*mu_hcd;
	unsigned long		flags;

	req = usb_request_to_mausb_request(_req);
	if (!_req || !list_empty(&req->queue) || !_req->complete)
		return -EINVAL;

	ep = usb_ep_to_mausb_ep(_ep);
	if (!_ep || (!ep->desc && _ep->name != ep0name))
		return -EINVAL;

	mu = ep_to_mausb(ep);
	mu_hcd = gadget_to_mausb_hcd(&mu->gadget);
	if (!mu->driver || !is_enabled(mu_hcd))
		return -ESHUTDOWN;

#if 0
	dev_dbg(udc_dev(mu), "ep %p queue req %p to %s, len %d buf %p\n",
			ep, _req, _ep->name, _req->length, _req->buf);
#endif
	_req->status = -EINPROGRESS;
	_req->actual = 0;
	spin_lock_irqsave(&mu->lock, flags);

	/* implement an emulated single-request FIFO */
	/*if (ep->desc && (ep->desc->bEndpointAddress & USB_DIR_IN) &&
			list_empty(&mu->fifo_req.queue) &&
			list_empty(&ep->queue) &&
			_req->length <= FIFO_SIZE) {
		req = &mu->fifo_req;
		req->req = *_req;
		req->req.buf = mu->fifo_buf;
		memcpy(mu->fifo_buf, _req->buf, _req->length);
		req->req.context = mu;
		req->req.complete = fifo_complete;

		list_add_tail(&req->queue, &ep->queue);
		spin_unlock(&mu->lock);
		_req->actual = _req->length;
		_req->status = 0;
		pr_info(">--%s calling usb_gadget_giveback_request ",__func__);
		//usb_gadget_giveback_request(_ep, _req);//sandeep
		spin_lock(&mu->lock);
	}  else*/
		list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&mu->lock, flags);

	/* real hardware would likely enable transfers here, in case
	 * it'd been left NAKing.
	 */
	return 0;
}

static int mausb_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct mausb_ep		*ep;
	struct mausb		*mu;
	int			retval = -EINVAL;
	unsigned long		flags;
	struct mausb_request	*req = NULL;

	if (!_ep || !_req)
		return retval;
	ep = usb_ep_to_mausb_ep(_ep);
	mu = ep_to_mausb(ep);

	if (!mu->driver)
		return -ESHUTDOWN;

	local_irq_save(flags);
	spin_lock(&mu->lock);
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req) {
			list_del_init(&req->queue);
			_req->status = -ECONNRESET;
			retval = 0;
			break;
		}
	}
	spin_unlock(&mu->lock);

	if (retval == 0) {
		dev_dbg(udc_dev(mu),
				"dequeued req %p from %s, len %d buf %p\n",
				req, _ep->name, _req->length, _req->buf);
		usb_gadget_giveback_request(_ep, _req);
	}
	local_irq_restore(flags);
	return retval;
}

static int
mausb_set_halt_and_wedge(struct usb_ep *_ep, int value, int wedged)
{
	struct mausb_ep		*ep;
	struct mausb		*mu;

	if (!_ep)
		return -EINVAL;
	ep = usb_ep_to_mausb_ep(_ep);
	mu = ep_to_mausb(ep);
	if (!mu->driver)
		return -ESHUTDOWN;
	if (!value)
		ep->halted = ep->wedged = 0;
	else if (ep->desc && (ep->desc->bEndpointAddress & USB_DIR_IN) &&
			!list_empty(&ep->queue))
		return -EAGAIN;
	else {
		ep->halted = 1;
		if (wedged)
			ep->wedged = 1;
	}
	/* FIXME clear emulated data toggle too */
	return 0;
}

static int
mausb_set_halt(struct usb_ep *_ep, int value)
{
	return mausb_set_halt_and_wedge(_ep, value, 0);
}

static int mausb_set_wedge(struct usb_ep *_ep)
{
	if (!_ep || _ep->name == ep0name)
		return -EINVAL;
	return mausb_set_halt_and_wedge(_ep, 1, 1);
}

static const struct usb_ep_ops mausb_ep_ops = {
	.enable		= mausb_enable,
	.disable	= mausb_disable,

	.alloc_request	= mausb_alloc_request,
	.free_request	= mausb_free_request,

	.queue		= mausb_queue,
	.dequeue	= mausb_dequeue,

	.set_halt	= mausb_set_halt,
	.set_wedge	= mausb_set_wedge,
};

/*-------------------------------------------------------------------------*/

/* there are both host and device side versions of this call ... */
static int mausb_g_get_frame(struct usb_gadget *_gadget)
{
	struct timeval	tv;

	do_gettimeofday(&tv);
	return tv.tv_usec / 1000;
}

static int mausb_wakeup(struct usb_gadget *_gadget)
{
	struct mausb_hcd *mu_hcd;

	mu_hcd = gadget_to_mausb_hcd(_gadget);
	if (!(mu_hcd->mu->devstatus & ((1 << USB_DEVICE_B_HNP_ENABLE)
				| (1 << USB_DEVICE_REMOTE_WAKEUP))))
		return -EINVAL;
	if ((mu_hcd->port_status & USB_PORT_STAT_CONNECTION) == 0)
		return -ENOLINK;
	if ((mu_hcd->port_status & USB_PORT_STAT_SUSPEND) == 0 &&
			 mu_hcd->rh_state != MAUSB_RH_SUSPENDED)
		return -EIO;

	/* FIXME: What if the root hub is suspended but the port isn't? */

	/* hub notices our request, issues downstream resume, etc */
	mu_hcd->resuming = 1;
	mu_hcd->re_timeout = jiffies + msecs_to_jiffies(20);
	mod_timer(&mausb_hcd_to_hcd(mu_hcd)->rh_timer, mu_hcd->re_timeout);
	return 0;
}

static int mausb_set_selfpowered(struct usb_gadget *_gadget, int value)
{
	struct mausb	*mu;

	mu = gadget_to_mausb_hcd(_gadget)->mu;
	if (value)
		mu->devstatus |= (1 << USB_DEVICE_SELF_POWERED);
	else
		mu->devstatus &= ~(1 << USB_DEVICE_SELF_POWERED);
	return 0;
}

static void mausb_udc_update_ep0(struct mausb *mu)
{
	if (mu->gadget.speed == USB_SPEED_SUPER)
		mu->ep[0].ep.maxpacket = 9;
	else
		mu->ep[0].ep.maxpacket = 64;
}

static int mausb_pullup(struct usb_gadget *_gadget, int value)
{
	struct mausb_hcd *mu_hcd;
	struct mausb	*mu;
	unsigned long	flags;

	pr_info("%s",__func__);
	//WARN_ON(1);


	mu = gadget_dev_to_mausb(&_gadget->dev);

	if (value && mu->driver) {
		if (mod_data.is_super_speed)
			mu->gadget.speed = mu->driver->max_speed;
		else if (mod_data.is_high_speed)
			mu->gadget.speed = min_t(u8, USB_SPEED_HIGH,
					mu->driver->max_speed);
		else
			mu->gadget.speed = USB_SPEED_FULL;
		mausb_udc_update_ep0(mu);

		if (mu->gadget.speed < mu->driver->max_speed)
			dev_dbg(udc_dev(mu), "This device can perform faster"
				" if you connect it to a %s port...\n",
				usb_speed_string(mu->driver->max_speed));
	}
	mu_hcd = gadget_to_mausb_hcd(_gadget);

	spin_lock_irqsave(&mu->lock, flags);
	mu->pullup = (value != 0);
	set_link_state(mu_hcd);
	spin_unlock_irqrestore(&mu->lock, flags);

	usb_hcd_poll_rh_status(mausb_hcd_to_hcd(mu_hcd));
	return 0;
}

static int mausb_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver);
static int mausb_udc_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver);

static const struct usb_gadget_ops mausb_ops = {
	.get_frame	= mausb_g_get_frame,
	.wakeup		= mausb_wakeup,
	.set_selfpowered = mausb_set_selfpowered,
	.pullup		= mausb_pullup,
	.udc_start	= mausb_udc_start,
	.udc_stop	= mausb_udc_stop,
};

/*-------------------------------------------------------------------------*/

/* "function" sysfs attribute */
static ssize_t function_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct mausb	*mu = gadget_dev_to_mausb(dev);

	if (!mu->driver || !mu->driver->function)
		return 0;
	return scnprintf(buf, PAGE_SIZE, "%s\n", mu->driver->function);
}
static DEVICE_ATTR_RO(function);

/*-------------------------------------------------------------------------*/

/*
 * Driver registration/unregistration.
 *
 * This is basically hardware-specific; there's usually only one real USB
 * device (not host) controller since that's how USB devices are intended
 * to work.  So most implementations of these api calls will rely on the
 * fact that only one driver will ever bind to the hardware.  But curious
 * hardware can be built with discrete components, so the gadget API doesn't
 * require that assumption.
 *
 * For this emulator, it might be convenient to create a usb slave device
 * for each driver that registers:  just add to a big root hub.
 */

static int mausb_udc_start(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct mausb_hcd	*mu_hcd = gadget_to_mausb_hcd(g);
	struct mausb		*mu = mu_hcd->mu;

	if (driver->max_speed == USB_SPEED_UNKNOWN)
		return -EINVAL;

	/*
	 * SLAVE side init ... the layer above hardware, which
	 * can't enumerate without help from the driver we're binding.
	 */

	mu->devstatus = 0;

	mu->driver = driver;
	dev_dbg(udc_dev(mu), "binding gadget driver '%s'\n",
			driver->driver.name);
	return 0;
}

static int mausb_udc_stop(struct usb_gadget *g,
		struct usb_gadget_driver *driver)
{
	struct mausb_hcd	*mu_hcd = gadget_to_mausb_hcd(g);
	struct mausb		*mu = mu_hcd->mu;

	if (driver)
		dev_dbg(udc_dev(mu), "unregister gadget driver '%s'\n",
				driver->driver.name);

	mu->driver = NULL;

	return 0;
}

#undef is_enabled

/* The gadget structure is stored inside the hcd structure and will be
 * released along with it. */
static void init_mausb_udc_hw(struct mausb *mu)
{
	int i;

	INIT_LIST_HEAD(&mu->gadget.ep_list);
	for (i = 0; i < MAUSB_ENDPOINTS; i++) {
		struct mausb_ep	*ep = &mu->ep[i];

		if (!ep_name[i])
			break;
		ep->ep.name = ep_name[i];
		ep->ep.ops = &mausb_ep_ops;
		list_add_tail(&ep->ep.ep_list, &mu->gadget.ep_list);
		ep->halted = ep->wedged = ep->already_seen =
				ep->setup_stage = 0;
		usb_ep_set_maxpacket_limit(&ep->ep, ~0);
		ep->ep.max_streams = 16;
		ep->last_io = jiffies;
		ep->gadget = &mu->gadget;
		ep->desc = NULL;
		INIT_LIST_HEAD(&ep->queue);
	}

	mu->gadget.ep0 = &mu->ep[0].ep;
	list_del_init(&mu->ep[0].ep.ep_list);
	INIT_LIST_HEAD(&mu->fifo_req.queue);

#ifdef CONFIG_USB_OTG
	mu->gadget.is_otg = 1;
#endif
}

static int mausb_udc_probe(struct platform_device *pdev)
{
	struct mausb	*mu;
	int		rc;

	mu = *((void **)dev_get_platdata(&pdev->dev));
	mu->gadget.name = gadget_name;
	mu->gadget.ops = &mausb_ops;
	mu->gadget.max_speed = USB_SPEED_SUPER;

	mu->gadget.dev.parent = &pdev->dev;
	init_mausb_udc_hw(mu);

	rc = usb_add_gadget_udc(&pdev->dev, &mu->gadget);
	if (rc < 0)
		goto err_udc;

	rc = device_create_file(&mu->gadget.dev, &dev_attr_function);
	if (rc < 0)
		goto err_dev;
	platform_set_drvdata(pdev, mu);
	return rc;

err_dev:
	usb_del_gadget_udc(&mu->gadget);
err_udc:
	return rc;
}

static int mausb_udc_remove(struct platform_device *pdev)
{
	struct mausb	*mu = platform_get_drvdata(pdev);

	device_remove_file(&mu->gadget.dev, &dev_attr_function);
	usb_del_gadget_udc(&mu->gadget);
	return 0;
}

/*static void mausb_udc_pm(struct mausb *mu, struct mausb_hcd *mu_hcd,
		int suspend)
{
	spin_lock_irq(&mu->lock);
	mu->udc_suspended = suspend;
	set_link_state(mu_hcd);
	spin_unlock_irq(&mu->lock);
}*/

static int mausb_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/*struct mausb		*mu = platform_get_drvdata(pdev);
	struct mausb_hcd	*mu_hcd = gadget_to_mausb_hcd(&mu->gadget);

	dev_dbg(&pdev->dev, "%s\n", __func__);
	mausb_udc_pm(mu, mu_hcd, 1);
	usb_hcd_poll_rh_status(mausb_hcd_to_hcd(mu_hcd));*/
	pr_info("%s\n", __func__);
	return 0;
}

static int mausb_udc_resume(struct platform_device *pdev)
{
	/*struct mausb		*mu = platform_get_drvdata(pdev);
	struct mausb_hcd	*mu_hcd = gadget_to_mausb_hcd(&mu->gadget);

	dev_dbg(&pdev->dev, "%s\n", __func__);
	mausb_udc_pm(mu, mu_hcd, 0);
	usb_hcd_poll_rh_status(mausb_hcd_to_hcd(mu_hcd));*/
	pr_info("%s\n", __func__);
	return 0;
}

static struct platform_driver mausb_udc_driver = {
	.probe		= mausb_udc_probe,
	.remove		= mausb_udc_remove,
	.suspend	= mausb_udc_suspend,
	.resume		= mausb_udc_resume,
	.driver		= {
		.name	= (char *) gadget_name,
		.owner	= THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/

static unsigned int mausb_get_ep_idx(const struct usb_endpoint_descriptor *desc)
{
	unsigned int index;

	index = usb_endpoint_num(desc) << 1;
	if (usb_endpoint_dir_in(desc))
		index |= 1;
	return index;
}

/* MASTER/HOST SIDE DRIVER
 *
 * this uses the hcd framework to hook up to host side drivers.
 * its root hub will only have one device, otherwise it acts like
 * a normal host controller.
 *
 * when urbs are queued, they're just stuck on a list that we
 * scan in a timer callback.  that callback connects writes from
 * the host with reads from the device, and so on, based on the
 * usb 2.0 rules.
 */

static int mausb_ep_stream_en(struct mausb_hcd *mu_hcd, struct urb *urb)
{
	const struct usb_endpoint_descriptor *desc = &urb->ep->desc;
	u32 index;

	if (!usb_endpoint_xfer_bulk(desc))
		return 0;

	index = mausb_get_ep_idx(desc);
	return (1 << index) & mu_hcd->stream_en_ep;
}

/*
 * The max stream number is saved as a nibble so for the 30 possible endpoints
 * we only 15 bytes of memory. Therefore we are limited to max 16 streams (0
 * means we use only 1 stream). The maximum according to the spec is 16bit so
 * if the 16 stream limit is about to go, the array size should be incremented
 * to 30 elements of type u16.
 */
static int get_max_streams_for_pipe(struct mausb_hcd *mu_hcd,
		unsigned int pipe)
{
	int max_streams;

	max_streams = mu_hcd->num_stream[usb_pipeendpoint(pipe)];
	if (usb_pipeout(pipe))
		max_streams >>= 4;
	else
		max_streams &= 0xf;
	max_streams++;
	return max_streams;
}

static void set_max_streams_for_pipe(struct mausb_hcd *mu_hcd,
		unsigned int pipe, unsigned int streams)
{
	int max_streams;

	streams--;
	max_streams = mu_hcd->num_stream[usb_pipeendpoint(pipe)];
	if (usb_pipeout(pipe)) {
		streams <<= 4;
		max_streams &= 0xf;
	} else {
		max_streams &= 0xf0;
	}
	max_streams |= streams;
	mu_hcd->num_stream[usb_pipeendpoint(pipe)] = max_streams;
}

static int mausb_validate_stream(struct mausb_hcd *mu_hcd, struct urb *urb)
{
	unsigned int max_streams;
	int enabled;

	enabled = mausb_ep_stream_en(mu_hcd, urb);
	if (!urb->stream_id) {
		if (enabled)
			return -EINVAL;
		return 0;
	}
	if (!enabled)
		return -EINVAL;

	max_streams = get_max_streams_for_pipe(mu_hcd,
			usb_pipeendpoint(urb->pipe));
	if (urb->stream_id > max_streams) {
		dev_err(mausb_dev(mu_hcd), "Stream id %d is out of range.\n",
				urb->stream_id);
		BUG();
		return -EINVAL;
	}
	return 0;
}

static int mausb_urb_enqueue(
	struct usb_hcd			*hcd,
	struct urb			*urb,
	gfp_t				mem_flags
) {
	struct mausb_hcd *mu_hcd;
	struct urbp	*urbp;
	unsigned long	flags;
	int		rc;

	urbp = kmalloc(sizeof *urbp, mem_flags);
	if (!urbp)
		return -ENOMEM;
	urbp->urb = urb;
	urbp->miter_started = 0;

	mu_hcd = hcd_to_mausb_hcd(hcd);
	spin_lock_irqsave(&mu_hcd->mu->lock, flags);

	rc = mausb_validate_stream(mu_hcd, urb);
	if (rc) {
		kfree(urbp);
		goto done;
	}

	rc = usb_hcd_link_urb_to_ep(hcd, urb);
	if (rc) {
		kfree(urbp);
		goto done;
	}

	if (!mu_hcd->udev) {
		mu_hcd->udev = urb->dev;
		usb_get_dev(mu_hcd->udev);
	} else if (unlikely(mu_hcd->udev != urb->dev))
		dev_err(mausb_dev(mu_hcd), "usb_device address has changed!\n");

	list_add_tail(&urbp->urbp_list, &mu_hcd->urbp_list);
	urb->hcpriv = urbp;
	if (usb_pipetype(urb->pipe) == PIPE_CONTROL)
		urb->error_count = 1;		/* mark as a new urb */

	/* kick the scheduler, it'll do the rest */
	if (!hrtimer_is_queued(&mu_hcd->ttimer.timer)) {
		tasklet_hrtimer_start(&mu_hcd->ttimer,
				ms_to_ktime(1),
				HRTIMER_MODE_REL);
	}
 done:
	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);
	return rc;
}

static int mausb_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
	struct mausb_hcd *mu_hcd;
	unsigned long	flags;
	int		rc;

	/* giveback happens automatically in timer callback,
	 * so make sure the callback happens */
	mu_hcd = hcd_to_mausb_hcd(hcd);
	spin_lock_irqsave(&mu_hcd->mu->lock, flags);

	rc = usb_hcd_check_unlink_urb(hcd, urb, status);
	if (!rc && mu_hcd->rh_state != MAUSB_RH_RUNNING &&
			!list_empty(&mu_hcd->urbp_list) &&
			!hrtimer_is_queued(&mu_hcd->ttimer.timer)) {
				tasklet_hrtimer_start(&mu_hcd->ttimer,
				ns_to_ktime(100),
				HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);
	return rc;
}

static int mausb_perform_transfer(struct urb *urb, struct mausb_request *req,
		u32 len)
{
	void *ubuf, *rbuf;
	struct urbp *urbp = urb->hcpriv;
	int to_host;
	struct sg_mapping_iter *miter = &urbp->miter;
	u32 trans = 0;
	u32 this_sg;
	bool next_sg;

	to_host = usb_pipein(urb->pipe);
	rbuf = req->req.buf + req->req.actual;

	if (!urb->num_sgs) {
		ubuf = urb->transfer_buffer + urb->actual_length;
		if (to_host)
			memcpy(ubuf, rbuf, len);
		else
			memcpy(rbuf, ubuf, len);
		return len;
	}

	if (!urbp->miter_started) {
		u32 flags = SG_MITER_ATOMIC;

		if (to_host)
			flags |= SG_MITER_TO_SG;
		else
			flags |= SG_MITER_FROM_SG;

		sg_miter_start(miter, urb->sg, urb->num_sgs, flags);
		urbp->miter_started = 1;
	}
	next_sg = sg_miter_next(miter);
	if (next_sg == false) {
		WARN_ON_ONCE(1);
		return -EINVAL;
	}
	do {
		ubuf = miter->addr;
		this_sg = min_t(u32, len, miter->length);
		miter->consumed = this_sg;
		trans += this_sg;

		if (to_host)
			memcpy(ubuf, rbuf, this_sg);
		else
			memcpy(rbuf, ubuf, this_sg);
		len -= this_sg;

		if (!len)
			break;
		next_sg = sg_miter_next(miter);
		if (next_sg == false) {
			WARN_ON_ONCE(1);
			return -EINVAL;
		}

		rbuf += this_sg;
	} while (1);

	sg_miter_stop(miter);
	return trans;
}

/* transfer up to a frame's worth; caller must own lock */
static int transfer(struct mausb_hcd *mu_hcd, struct urb *urb,
		struct mausb_ep *ep, int limit, int *status)
{
	struct mausb		*mu = mu_hcd->mu;
	struct mausb_request	*req;

top:
	/* if there's no request queued, the device is NAKing; return */
	list_for_each_entry(req, &ep->queue, queue) {
		unsigned	host_len, dev_len, len;
		int		is_short, to_host;
		int		rescan = 0;

		if (mausb_ep_stream_en(mu_hcd, urb)) {
			if ((urb->stream_id != req->req.stream_id))
				continue;
		}

		/* 1..N packets of ep->ep.maxpacket each ... the last one
		 * may be short (including zero length).
		 *
		 * writer can send a zlp explicitly (length 0) or implicitly
		 * (length mod maxpacket zero, and 'zero' flag); they always
		 * terminate reads.
		 */
		host_len = urb->transfer_buffer_length - urb->actual_length;
		dev_len = req->req.length - req->req.actual;
		len = min(host_len, dev_len);

		/* FIXME update emulated data toggle too */

		to_host = usb_pipein(urb->pipe);
		if (unlikely(len == 0))
			is_short = 1;
		else {
			/* not enough bandwidth left? */
			if (limit < ep->ep.maxpacket && limit < len)
				break;
			len = min_t(unsigned, len, limit);
			if (len == 0)
				break;

			/* use an extra pass for the final short packet */
			if (len > ep->ep.maxpacket) {
				rescan = 1;
				len -= (len % ep->ep.maxpacket);
			}
			is_short = (len % ep->ep.maxpacket) != 0;

			len = mausb_perform_transfer(urb, req, len);

			ep->last_io = jiffies;
			if ((int)len < 0) {
				req->req.status = len;
			} else {
				limit -= len;
				urb->actual_length += len;
				req->req.actual += len;
			}
		}

		/* short packets terminate, maybe with overflow/underflow.
		 * it's only really an error to write too much.
		 *
		 * partially filling a buffer optionally blocks queue advances
		 * (so completion handlers can clean up the queue) but we don't
		 * need to emulate such data-in-flight.
		 */
		if (is_short) {
			if (host_len == dev_len) {
				req->req.status = 0;
				*status = 0;
			} else if (to_host) {
				req->req.status = 0;
				if (dev_len > host_len)
					*status = -EOVERFLOW;
				else
					*status = 0;
			} else if (!to_host) {
				*status = 0;
				if (host_len > dev_len)
					req->req.status = -EOVERFLOW;
				else
					req->req.status = 0;
			}

		/* many requests terminate without a short packet */
		} else {
			if (req->req.length == req->req.actual
					&& !req->req.zero)
				req->req.status = 0;
			if (urb->transfer_buffer_length == urb->actual_length
					&& !(urb->transfer_flags
						& URB_ZERO_PACKET))
				*status = 0;
		}

		/* device side completion --> continuable */
		if (req->req.status != -EINPROGRESS) {
			list_del_init(&req->queue);

			spin_unlock(&mu->lock);
			usb_gadget_giveback_request(&ep->ep, &req->req);
			spin_lock(&mu->lock);

			/* requests might have been unlinked... */
			rescan = 1;
		}

		/* host side completion --> terminate */
		if (*status != -EINPROGRESS)
			break;

		/* rescan to continue with any other queued i/o */
		if (rescan)
			goto top;
	}
	return limit;
}

static int periodic_bytes(struct mausb *mu, struct mausb_ep *ep)
{
	int	limit = ep->ep.maxpacket;

	if (mu->gadget.speed == USB_SPEED_HIGH) {
		int	tmp;

		/* high bandwidth mode */
		tmp = usb_endpoint_maxp(ep->desc);
		tmp = (tmp >> 11) & 0x03;
		tmp *= 8 /* applies to entire frame */;
		limit += limit * tmp;
	}
	if (mu->gadget.speed == USB_SPEED_SUPER) {
		switch (usb_endpoint_type(ep->desc)) {
		case USB_ENDPOINT_XFER_ISOC:
			/* Sec. 4.4.8.2 USB3.0 Spec */
			limit = 3 * 16 * 1024 * 8;
			break;
		case USB_ENDPOINT_XFER_INT:
			/* Sec. 4.4.7.2 USB3.0 Spec */
			limit = 3 * 1024 * 8;
			break;
		case USB_ENDPOINT_XFER_BULK:
		default:
			break;
		}
	}
	return limit;
}

#define is_active(mu_hcd)	((mu_hcd->port_status & \
		(USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE | \
			USB_PORT_STAT_SUSPEND)) \
		== (USB_PORT_STAT_CONNECTION | USB_PORT_STAT_ENABLE))

static struct mausb_ep *find_endpoint(struct mausb *mu, u8 address)
{
	int		i;

	if (!is_active((mu->gadget.speed == USB_SPEED_SUPER ?
			mu->ss_hcd : mu->hs_hcd)))
		return NULL;
	if ((address & ~USB_DIR_IN) == 0)
		return &mu->ep[0];
	for (i = 1; i < MAUSB_ENDPOINTS; i++) {
		struct mausb_ep	*ep = &mu->ep[i];

		if (!ep->desc)
			continue;
		if (ep->desc->bEndpointAddress == address)
			return ep;
	}
	return NULL;
}

#undef is_active

#define Dev_Request	(USB_TYPE_STANDARD | USB_RECIP_DEVICE)
#define Dev_InRequest	(Dev_Request | USB_DIR_IN)
#define Intf_Request	(USB_TYPE_STANDARD | USB_RECIP_INTERFACE)
#define Intf_InRequest	(Intf_Request | USB_DIR_IN)
#define Ep_Request	(USB_TYPE_STANDARD | USB_RECIP_ENDPOINT)
#define Ep_InRequest	(Ep_Request | USB_DIR_IN)


/**
 * handle_control_request() - handles all control transfers
 * @mu: pointer to mausb (the_controller)
 * @urb: the urb request to handle
 * @setup: pointer to the setup data for a USB device control
 *	 request
 * @status: pointer to request handling status
 *
 * Return 0 - if the request was handled
 *	  1 - if the request wasn't handles
 *	  error code on error
 */
static int handle_control_request(struct mausb_hcd *mu_hcd, struct urb *urb,
				  struct usb_ctrlrequest *setup,
				  int *status)
{
	struct mausb_ep		*ep2;
	struct mausb		*mu = mu_hcd->mu;
	int			ret_val = 1;
	unsigned	w_index;
	unsigned	w_value;

	w_index = le16_to_cpu(setup->wIndex);
	w_value = le16_to_cpu(setup->wValue);
	switch (setup->bRequest) {
	case USB_REQ_SET_ADDRESS:
		if (setup->bRequestType != Dev_Request)
			break;
		mu->address = w_value;
		*status = 0;
		dev_dbg(udc_dev(mu), "set_address = %d\n",
				w_value);
		ret_val = 0;
		break;
	case USB_REQ_SET_FEATURE:
		if (setup->bRequestType == Dev_Request) {
			ret_val = 0;
			switch (w_value) {
			case USB_DEVICE_REMOTE_WAKEUP:
				break;
			case USB_DEVICE_B_HNP_ENABLE:
				mu->gadget.b_hnp_enable = 1;
				break;
			case USB_DEVICE_A_HNP_SUPPORT:
				mu->gadget.a_hnp_support = 1;
				break;
			case USB_DEVICE_A_ALT_HNP_SUPPORT:
				mu->gadget.a_alt_hnp_support = 1;
				break;
			case USB_DEVICE_U1_ENABLE:
				if (mausb_hcd_to_hcd(mu_hcd)->speed ==
				    HCD_USB3)
					w_value = USB_DEV_STAT_U1_ENABLED;
				else
					ret_val = -EOPNOTSUPP;
				break;
			case USB_DEVICE_U2_ENABLE:
				if (mausb_hcd_to_hcd(mu_hcd)->speed ==
				    HCD_USB3)
					w_value = USB_DEV_STAT_U2_ENABLED;
				else
					ret_val = -EOPNOTSUPP;
				break;
			case USB_DEVICE_LTM_ENABLE:
				if (mausb_hcd_to_hcd(mu_hcd)->speed ==
				    HCD_USB3)
					w_value = USB_DEV_STAT_LTM_ENABLED;
				else
					ret_val = -EOPNOTSUPP;
				break;
			default:
				ret_val = -EOPNOTSUPP;
			}
			if (ret_val == 0) {
				mu->devstatus |= (1 << w_value);
				*status = 0;
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = find_endpoint(mu, w_index);
			if (!ep2 || ep2->ep.name == ep0name) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			ep2->halted = 1;
			ret_val = 0;
			*status = 0;
		}
		break;
	case USB_REQ_CLEAR_FEATURE:
		if (setup->bRequestType == Dev_Request) {
			ret_val = 0;
			switch (w_value) {
			case USB_DEVICE_REMOTE_WAKEUP:
				w_value = USB_DEVICE_REMOTE_WAKEUP;
				break;
			case USB_DEVICE_U1_ENABLE:
				if (mausb_hcd_to_hcd(mu_hcd)->speed ==
				    HCD_USB3)
					w_value = USB_DEV_STAT_U1_ENABLED;
				else
					ret_val = -EOPNOTSUPP;
				break;
			case USB_DEVICE_U2_ENABLE:
				if (mausb_hcd_to_hcd(mu_hcd)->speed ==
				    HCD_USB3)
					w_value = USB_DEV_STAT_U2_ENABLED;
				else
					ret_val = -EOPNOTSUPP;
				break;
			case USB_DEVICE_LTM_ENABLE:
				if (mausb_hcd_to_hcd(mu_hcd)->speed ==
				    HCD_USB3)
					w_value = USB_DEV_STAT_LTM_ENABLED;
				else
					ret_val = -EOPNOTSUPP;
				break;
			default:
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (ret_val == 0) {
				mu->devstatus &= ~(1 << w_value);
				*status = 0;
			}
		} else if (setup->bRequestType == Ep_Request) {
			/* endpoint halt */
			ep2 = find_endpoint(mu, w_index);
			if (!ep2) {
				ret_val = -EOPNOTSUPP;
				break;
			}
			if (!ep2->wedged)
				ep2->halted = 0;
			ret_val = 0;
			*status = 0;
		}
		break;
	case USB_REQ_GET_STATUS:
		if (setup->bRequestType == Dev_InRequest
				|| setup->bRequestType == Intf_InRequest
				|| setup->bRequestType == Ep_InRequest) {
			char *buf;
			/*
			 * device: remote wakeup, selfpowered
			 * interface: nothing
			 * endpoint: halt
			 */
			buf = (char *)urb->transfer_buffer;
			if (urb->transfer_buffer_length > 0) {
				if (setup->bRequestType == Ep_InRequest) {
					ep2 = find_endpoint(mu, w_index);
					if (!ep2) {
						ret_val = -EOPNOTSUPP;
						break;
					}
					buf[0] = ep2->halted;
				} else if (setup->bRequestType ==
					   Dev_InRequest) {
					buf[0] = (u8)mu->devstatus;
				} else
					buf[0] = 0;
			}
			if (urb->transfer_buffer_length > 1)
				buf[1] = 0;
			urb->actual_length = min_t(u32, 2,
				urb->transfer_buffer_length);
			ret_val = 0;
			*status = 0;
		}
		break;
	}
	return ret_val;
}

/* drive both sides of the transfers; looks like irq handlers to
 * both drivers except the callbacks aren't in_irq().
 */
 static enum hrtimer_restart mausb_timer(struct hrtimer *timer)
{
	struct mausb_hcd	*mu_hcd = container_of(timer,
				struct mausb_hcd, ttimer.timer);
	struct mausb		*mu = mu_hcd->mu;
	struct urbp		*urbp, *tmp;
	unsigned long		flags;
	int			limit, total;
	int			i;

	/* simplistic model for one frame's bandwidth */
	switch (mu->gadget.speed) {
	case USB_SPEED_LOW:
		total = 8/*bytes*/ * 12/*packets*/;
		break;
	case USB_SPEED_FULL:
		total = 64/*bytes*/ * 19/*packets*/;
		break;
	case USB_SPEED_HIGH:
		total = 512/*bytes*/ * 13/*packets*/ * 8/*uframes*/;
		break;
	case USB_SPEED_SUPER:
		/* Bus speed is 500000 bytes/ms, so use a little less */
		total = 490000;
		break;
	default:
		dev_err(mausb_dev(mu_hcd), "bogus device speed\n");
		goto out;
	}

	/* FIXME if HZ != 1000 this will probably misbehave ... */

	/* look at each urb queued by the host side driver */
	spin_lock_irqsave(&mu->lock, flags);

	if (!mu_hcd->udev) {
		dev_err(mausb_dev(mu_hcd),
				"timer fired with no URBs pending?\n");
		spin_unlock_irqrestore(&mu->lock, flags);
		goto out;
	}

	for (i = 0; i < MAUSB_ENDPOINTS; i++) {
		if (!ep_name[i])
			break;
		mu->ep[i].already_seen = 0;
	}

restart:
	list_for_each_entry_safe(urbp, tmp, &mu_hcd->urbp_list, urbp_list) {
		struct urb		*urb;
		struct mausb_request	*req;
		u8			address;
		struct mausb_ep		*ep = NULL;
		int			type;
		int			status = -EINPROGRESS;

		urb = urbp->urb;
		if (urb->unlinked)
			goto return_urb;
		else if (mu_hcd->rh_state != MAUSB_RH_RUNNING)
			continue;
		type = usb_pipetype(urb->pipe);

		/* used up this frame's non-periodic bandwidth?
		 * FIXME there's infinite bandwidth for control and
		 * periodic transfers ... unrealistic.
		 */
		if (total <= 0 && type == PIPE_BULK)
			continue;

		/* find the gadget's ep for this request (if configured) */
		address = usb_pipeendpoint (urb->pipe);
		if (usb_pipein(urb->pipe))
			address |= USB_DIR_IN;
		ep = find_endpoint(mu, address);
		if (!ep) {
			/* set_configuration() disagreement */
			dev_dbg(mausb_dev(mu_hcd),
				"no ep configured for urb %p\n",
				urb);
			status = -EPROTO;
			goto return_urb;
		}

		if (ep->already_seen)
			continue;
		ep->already_seen = 1;
		if (ep == &mu->ep[0] && urb->error_count) {
			ep->setup_stage = 1;	/* a new urb */
			urb->error_count = 0;
		}
		if (ep->halted && !ep->setup_stage) {
			/* NOTE: must not be iso! */
			dev_dbg(mausb_dev(mu_hcd), "ep %s halted, urb %p\n",
					ep->ep.name, urb);
			status = -EPIPE;
			goto return_urb;
		}
		/* FIXME make sure both ends agree on maxpacket */

		/* handle control requests */
		if (ep == &mu->ep[0] && ep->setup_stage) {
			struct usb_ctrlrequest		setup;
			int				value = 1;

			setup = *(struct usb_ctrlrequest *) urb->setup_packet;
			/* paranoia, in case of stale queued data */
			list_for_each_entry(req, &ep->queue, queue) {
				list_del_init(&req->queue);
				req->req.status = -EOVERFLOW;
				dev_dbg(udc_dev(mu), "stale req = %p\n",
						req);

				spin_unlock(&mu->lock);
				usb_gadget_giveback_request(&ep->ep, &req->req);
				spin_lock(&mu->lock);
				ep->already_seen = 0;
				goto restart;
			}

			/* gadget driver never sees set_address or operations
			 * on standard feature flags.  some hardware doesn't
			 * even expose them.
			 */
			ep->last_io = jiffies;
			ep->setup_stage = 0;
			ep->halted = 0;

			value = handle_control_request(mu_hcd, urb, &setup,
						       &status);

			/* gadget driver handles all other requests.  block
			 * until setup() returns; no reentrancy issues etc.
			 */
			if (value > 0) {
				spin_unlock(&mu->lock);
				value = mu->driver->setup(&mu->gadget,
						&setup);
				spin_lock(&mu->lock);

				if (value >= 0) {
					/* no delays (max 64KB data stage) */
					limit = 64*1024;
					goto treat_control_like_bulk;
				}
				/* error, see below */
			}

			if (value < 0) {
				if (value != -EOPNOTSUPP)
					dev_dbg(udc_dev(mu),
						"setup --> %d\n",
						value);
				status = -EPIPE;
				urb->actual_length = 0;
			}

			goto return_urb;
		}

		/* non-control requests */
		limit = total;
		switch (usb_pipetype(urb->pipe)) {
		case PIPE_ISOCHRONOUS:
			/* FIXME is it urb->interval since the last xfer?
			 * use urb->iso_frame_desc[i].
			 * complete whether or not ep has requests queued.
			 * report random errors, to debug drivers.
			 */
			limit = max(limit, periodic_bytes(mu, ep));
			status = -ENOSYS;
			break;

		case PIPE_INTERRUPT:
			/* FIXME is it urb->interval since the last xfer?
			 * this almost certainly polls too fast.
			 */
			limit = max(limit, periodic_bytes(mu, ep));
			/* FALLTHROUGH */

		default:
treat_control_like_bulk:
			ep->last_io = jiffies;
			total = transfer(mu_hcd, urb, ep, limit, &status);
			break;
		}

		/* incomplete transfer? */
		if (status == -EINPROGRESS)
			continue;

return_urb:
		list_del(&urbp->urbp_list);
		kfree(urbp);
		if (ep)
			ep->already_seen = ep->setup_stage = 0;

		usb_hcd_unlink_urb_from_ep(mausb_hcd_to_hcd(mu_hcd), urb);
		spin_unlock(&mu->lock);
		usb_hcd_giveback_urb(mausb_hcd_to_hcd(mu_hcd), urb, status);
		spin_lock(&mu->lock);

		goto restart;
	}

	if (list_empty(&mu_hcd->urbp_list)) {
		usb_put_dev(mu_hcd->udev);
		mu_hcd->udev = NULL;
	} else if (mu_hcd->rh_state == MAUSB_RH_RUNNING) {
		/* want a 1 msec delay here */
		tasklet_hrtimer_start(&mu_hcd->ttimer, ms_to_ktime(1),
				HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&mu->lock, flags);
out:
	return HRTIMER_NORESTART;
}

/*-------------------------------------------------------------------------*/

#define PORT_C_MASK \
	((USB_PORT_STAT_C_CONNECTION \
	| USB_PORT_STAT_C_ENABLE \
	| USB_PORT_STAT_C_SUSPEND \
	| USB_PORT_STAT_C_OVERCURRENT \
	| USB_PORT_STAT_C_RESET) << 16)

static int mausb_hub_status(struct usb_hcd *hcd, char *buf)
{
	struct mausb_hcd	*mu_hcd;
	unsigned long		flags;
	int			retval = 0;

	mu_hcd = hcd_to_mausb_hcd(hcd);

	spin_lock_irqsave(&mu_hcd->mu->lock, flags);
	if (!HCD_HW_ACCESSIBLE(hcd))
		goto done;

	if (mu_hcd->resuming && time_after_eq(jiffies, mu_hcd->re_timeout)) {
		mu_hcd->port_status |= (USB_PORT_STAT_C_SUSPEND << 16);
		mu_hcd->port_status &= ~USB_PORT_STAT_SUSPEND;
		set_link_state(mu_hcd);
	}

	if ((mu_hcd->port_status & PORT_C_MASK) != 0) {
		*buf = (1 << 1);
		dev_dbg(mausb_dev(mu_hcd), "port status 0x%08x has changes\n",
				mu_hcd->port_status);
		retval = 1;
		if (mu_hcd->rh_state == MAUSB_RH_SUSPENDED)
			usb_hcd_resume_root_hub(hcd);
	}
done:
	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);
	return retval;
}

/* usb 3.0 root hub device descriptor */
static struct {
	struct usb_bos_descriptor bos;
	struct usb_ss_cap_descriptor ss_cap;
} __packed usb3_bos_desc = {

	.bos = {
		.bLength		= USB_DT_BOS_SIZE,
		.bDescriptorType	= USB_DT_BOS,
		.wTotalLength		= cpu_to_le16(sizeof(usb3_bos_desc)),
		.bNumDeviceCaps		= 1,
	},
	.ss_cap = {
		.bLength		= USB_DT_USB_SS_CAP_SIZE,
		.bDescriptorType	= USB_DT_DEVICE_CAPABILITY,
		.bDevCapabilityType	= USB_SS_CAP_TYPE,
		.wSpeedSupported	= cpu_to_le16(USB_5GBPS_OPERATION),
		.bFunctionalitySupport	= ilog2(USB_5GBPS_OPERATION),
	},
};

static inline void
ss_hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof *desc);
	desc->bDescriptorType = 0x2a;
	desc->bDescLength = 12;
	desc->wHubCharacteristics = cpu_to_le16(0x0001);
	desc->bNbrPorts = 1;
	desc->u.ss.bHubHdrDecLat = 0x04; /* Worst case: 0.4 micro sec*/
	desc->u.ss.DeviceRemovable = 0xffff;
}

static inline void hub_descriptor(struct usb_hub_descriptor *desc)
{
	memset(desc, 0, sizeof *desc);
	desc->bDescriptorType = 0x29;
	desc->bDescLength = 9;
	desc->wHubCharacteristics = cpu_to_le16(0x0001);
	desc->bNbrPorts = 1;
	desc->u.hs.DeviceRemovable[0] = 0xff;
	desc->u.hs.DeviceRemovable[1] = 0xff;
}

static int mausb_hub_control(
	struct usb_hcd	*hcd,
	u16		typeReq,
	u16		wValue,
	u16		wIndex,
	char		*buf,
	u16		wLength
) {
	struct mausb_hcd *mu_hcd;
	int		retval = 0;
	unsigned long	flags;

	if (!HCD_HW_ACCESSIBLE(hcd))
		return -ETIMEDOUT;

	mu_hcd = hcd_to_mausb_hcd(hcd);

	spin_lock_irqsave(&mu_hcd->mu->lock, flags);
	switch (typeReq) {
	case ClearHubFeature:
		break;
	case ClearPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			if (hcd->speed == HCD_USB3) {
				dev_dbg(mausb_dev(mu_hcd),
					 "USB_PORT_FEAT_SUSPEND req not "
					 "supported for USB 3.0 roothub\n");
				goto error;
			}
			if (mu_hcd->port_status & USB_PORT_STAT_SUSPEND) {
				/* 20msec resume signaling */
				mu_hcd->resuming = 1;
				mu_hcd->re_timeout = jiffies +
						msecs_to_jiffies(20);
			}
			break;
		case USB_PORT_FEAT_POWER:
			if (hcd->speed == HCD_USB3) {
				if (mu_hcd->port_status & USB_PORT_STAT_POWER)
					dev_dbg(mausb_dev(mu_hcd),
						"power-off\n");
			} else
				if (mu_hcd->port_status &
							USB_SS_PORT_STAT_POWER)
					dev_dbg(mausb_dev(mu_hcd),
						"power-off\n");
			/* FALLS THROUGH */
		default:
			mu_hcd->port_status &= ~(1 << wValue);
			set_link_state(mu_hcd);
		}
		break;
	case GetHubDescriptor:
		if (hcd->speed == HCD_USB3 &&
				(wLength < USB_DT_SS_HUB_SIZE ||
				 wValue != (USB_DT_SS_HUB << 8))) {
			dev_dbg(mausb_dev(mu_hcd),
				"Wrong hub descriptor type for "
				"USB 3.0 roothub.\n");
			goto error;
		}
		if (hcd->speed == HCD_USB3)
			ss_hub_descriptor((struct usb_hub_descriptor *) buf);
		else
			hub_descriptor((struct usb_hub_descriptor *) buf);
		break;

	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		if (hcd->speed != HCD_USB3)
			goto error;

		if ((wValue >> 8) != USB_DT_BOS)
			goto error;

		memcpy(buf, &usb3_bos_desc, sizeof(usb3_bos_desc));
		retval = sizeof(usb3_bos_desc);
		break;

	case GetHubStatus:
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case GetPortStatus:
		if (wIndex != 1)
			retval = -EPIPE;

		/* whoever resets or resumes must GetPortStatus to
		 * complete it!!
		 */
		if (mu_hcd->resuming &&
				time_after_eq(jiffies, mu_hcd->re_timeout)) {
			mu_hcd->port_status |= (USB_PORT_STAT_C_SUSPEND << 16);
			mu_hcd->port_status &= ~USB_PORT_STAT_SUSPEND;
		}
		if ((mu_hcd->port_status & USB_PORT_STAT_RESET) != 0 &&
				time_after_eq(jiffies, mu_hcd->re_timeout)) {
			mu_hcd->port_status |= (USB_PORT_STAT_C_RESET << 16);
			mu_hcd->port_status &= ~USB_PORT_STAT_RESET;
			if (mu_hcd->mu->pullup) {
				mu_hcd->port_status |= USB_PORT_STAT_ENABLE;

				if (hcd->speed < HCD_USB3) {
					switch (mu_hcd->mu->gadget.speed) {
					case USB_SPEED_HIGH:
						mu_hcd->port_status |=
						      USB_PORT_STAT_HIGH_SPEED;
						break;
					case USB_SPEED_LOW:
						mu_hcd->mu->gadget.ep0->
							maxpacket = 8;
						mu_hcd->port_status |=
							USB_PORT_STAT_LOW_SPEED;
						break;
					default:
						mu_hcd->mu->gadget.speed =
							USB_SPEED_FULL;
						break;
					}
				}
			}
		}
		set_link_state(mu_hcd);
		((__le16 *) buf)[0] = cpu_to_le16(mu_hcd->port_status);
		((__le16 *) buf)[1] = cpu_to_le16(mu_hcd->port_status >> 16);
		break;
	case SetHubFeature:
		retval = -EPIPE;
		break;
	case SetPortFeature:
		switch (wValue) {
		case USB_PORT_FEAT_LINK_STATE:
			if (hcd->speed != HCD_USB3) {
				dev_dbg(mausb_dev(mu_hcd),
					 "USB_PORT_FEAT_LINK_STATE req not "
					 "supported for USB 2.0 roothub\n");
				goto error;
			}
			/*
			 * Since this is dummy-virtual mausb- we don't have an actual link so
			 * there is nothing to do for the SET_LINK_STATE cmd
			 */
			break;
		case USB_PORT_FEAT_U1_TIMEOUT:
		case USB_PORT_FEAT_U2_TIMEOUT:
			/* TODO: add suspend/resume support! */
			if (hcd->speed != HCD_USB3) {
				dev_dbg(mausb_dev(mu_hcd),
					 "USB_PORT_FEAT_U1/2_TIMEOUT req not "
					 "supported for USB 2.0 roothub\n");
				goto error;
			}
			break;
		case USB_PORT_FEAT_SUSPEND:
			/* Applicable only for USB2.0 hub */
			if (hcd->speed == HCD_USB3) {
				dev_dbg(mausb_dev(mu_hcd),
					 "USB_PORT_FEAT_SUSPEND req not "
					 "supported for USB 3.0 roothub\n");
				goto error;
			}
			if (mu_hcd->active) {
				mu_hcd->port_status |= USB_PORT_STAT_SUSPEND;

				/* HNP would happen here; for now we
				 * assume b_bus_req is always true.
				 */
				set_link_state(mu_hcd);
				if (((1 << USB_DEVICE_B_HNP_ENABLE)
						& mu_hcd->mu->devstatus) != 0)
					dev_dbg(mausb_dev(mu_hcd),
							"no HNP yet!\n");
			}
			break;
		case USB_PORT_FEAT_POWER:
			if (hcd->speed == HCD_USB3)
				mu_hcd->port_status |= USB_SS_PORT_STAT_POWER;
			else
				mu_hcd->port_status |= USB_PORT_STAT_POWER;
			set_link_state(mu_hcd);
			break;
		case USB_PORT_FEAT_BH_PORT_RESET:
			/* Applicable only for USB3.0 hub */
			if (hcd->speed != HCD_USB3) {
				dev_dbg(mausb_dev(mu_hcd),
					 "USB_PORT_FEAT_BH_PORT_RESET req not "
					 "supported for USB 2.0 roothub\n");
				goto error;
			}
			/* FALLS THROUGH */
		case USB_PORT_FEAT_RESET:
			/* if it's already enabled, disable */
			if (hcd->speed == HCD_USB3) {
				mu_hcd->port_status = 0;
				mu_hcd->port_status =
					(USB_SS_PORT_STAT_POWER |
					 USB_PORT_STAT_CONNECTION |
					 USB_PORT_STAT_RESET);
			} else
				mu_hcd->port_status &= ~(USB_PORT_STAT_ENABLE
					| USB_PORT_STAT_LOW_SPEED
					| USB_PORT_STAT_HIGH_SPEED);
			/*
			 * We want to reset device status. All but the
			 * Self powered feature
			 */
			mu_hcd->mu->devstatus &=
				(1 << USB_DEVICE_SELF_POWERED);
			/*
			 * FIXME USB3.0: what is the correct reset signaling
			 * interval? Is it still 50msec as for HS?
			 */
			mu_hcd->re_timeout = jiffies + msecs_to_jiffies(50);
			/* FALLS THROUGH */
		default:
			if (hcd->speed == HCD_USB3) {
				if ((mu_hcd->port_status &
				     USB_SS_PORT_STAT_POWER) != 0) {
					mu_hcd->port_status |= (1 << wValue);
					set_link_state(mu_hcd);
				}
			} else
				if ((mu_hcd->port_status &
				     USB_PORT_STAT_POWER) != 0) {
					mu_hcd->port_status |= (1 << wValue);
					set_link_state(mu_hcd);
				}
		}
		break;
	case GetPortErrorCount:
		if (hcd->speed != HCD_USB3) {
			dev_dbg(mausb_dev(mu_hcd),
				 "GetPortErrorCount req not "
				 "supported for USB 2.0 roothub\n");
			goto error;
		}
		/* We'll always return 0 since this is a dummy hub */
		*(__le32 *) buf = cpu_to_le32(0);
		break;
	case SetHubDepth:
		if (hcd->speed != HCD_USB3) {
			dev_dbg(mausb_dev(mu_hcd),
				 "SetHubDepth req not supported for "
				 "USB 2.0 roothub\n");
			goto error;
		}
		break;
	default:
		dev_dbg(mausb_dev(mu_hcd),
			"hub control req%04x v%04x i%04x l%d\n",
			typeReq, wValue, wIndex, wLength);
error:
		/* "protocol stall" on error */
		retval = -EPIPE;
	}
	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);

	if ((mu_hcd->port_status & PORT_C_MASK) != 0)
		usb_hcd_poll_rh_status(hcd);
	return retval;
}

static int mausb_bus_suspend(struct usb_hcd *hcd)
{
	struct mausb_hcd *mu_hcd = hcd_to_mausb_hcd(hcd);

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock_irq(&mu_hcd->mu->lock);
	mu_hcd->rh_state = MAUSB_RH_SUSPENDED;
	set_link_state(mu_hcd);
	hcd->state = HC_STATE_SUSPENDED;
	spin_unlock_irq(&mu_hcd->mu->lock);
	return 0;
}

static int mausb_bus_resume(struct usb_hcd *hcd)
{
	struct mausb_hcd *mu_hcd = hcd_to_mausb_hcd(hcd);
	int rc = 0;

	dev_dbg(&hcd->self.root_hub->dev, "%s\n", __func__);

	spin_lock_irq(&mu_hcd->mu->lock);
	if (!HCD_HW_ACCESSIBLE(hcd)) {
		rc = -ESHUTDOWN;
	} else {
		mu_hcd->rh_state = MAUSB_RH_RUNNING;
		set_link_state(mu_hcd);
		if (!list_empty(&mu_hcd->urbp_list))
			tasklet_hrtimer_start(&mu_hcd->ttimer,
					ms_to_ktime(1),
					HRTIMER_MODE_REL);
		hcd->state = HC_STATE_RUNNING;
	}
	spin_unlock_irq(&mu_hcd->mu->lock);
	return rc;
}

/*-------------------------------------------------------------------------*/

static inline ssize_t show_urb(char *buf, size_t size, struct urb *urb)
{
	int ep = usb_pipeendpoint(urb->pipe);

	return snprintf(buf, size,
		"urb/%p %s ep%d%s%s len %d/%d\n",
		urb,
		({ char *s;
		switch (urb->dev->speed) {
		case USB_SPEED_LOW:
			s = "ls";
			break;
		case USB_SPEED_FULL:
			s = "fs";
			break;
		case USB_SPEED_HIGH:
			s = "hs";
			break;
		case USB_SPEED_SUPER:
			s = "ss";
			break;
		default:
			s = "?";
			break;
		 } s; }),
		ep, ep ? (usb_pipein(urb->pipe) ? "in" : "out") : "",
		({ char *s; \
		switch (usb_pipetype(urb->pipe)) { \
		case PIPE_CONTROL: \
			s = ""; \
			break; \
		case PIPE_BULK: \
			s = "-bulk"; \
			break; \
		case PIPE_INTERRUPT: \
			s = "-int"; \
			break; \
		default: \
			s = "-iso"; \
			break; \
		} s; }),
		urb->actual_length, urb->transfer_buffer_length);
}

static ssize_t urbs_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct usb_hcd		*hcd = dev_get_drvdata(dev);
	struct mausb_hcd	*mu_hcd = hcd_to_mausb_hcd(hcd);
	struct urbp		*urbp;
	size_t			size = 0;
	unsigned long		flags;

	spin_lock_irqsave(&mu_hcd->mu->lock, flags);
	list_for_each_entry(urbp, &mu_hcd->urbp_list, urbp_list) {
		size_t		temp;

		temp = show_urb(buf, PAGE_SIZE - size, urbp->urb);
		buf += temp;
		size += temp;
	}
	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);

	return size;
}
static DEVICE_ATTR_RO(urbs);

static int mausb_start_ss(struct mausb_hcd *mu_hcd)
{
	tasklet_hrtimer_init(&mu_hcd->ttimer, mausb_timer,
		CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	mu_hcd->rh_state = MAUSB_RH_RUNNING;
	mu_hcd->stream_en_ep = 0;
	INIT_LIST_HEAD(&mu_hcd->urbp_list);
	mausb_hcd_to_hcd(mu_hcd)->power_budget = POWER_BUDGET;
	mausb_hcd_to_hcd(mu_hcd)->state = HC_STATE_RUNNING;
	mausb_hcd_to_hcd(mu_hcd)->uses_new_polling = 1;
#ifdef CONFIG_USB_OTG
	mausb_hcd_to_hcd(mu_hcd)->self.otg_port = 1;
#endif
	return 0;

	/* FIXME 'urbs' should be a per-device thing, maybe in usbcore */
	return device_create_file(mausb_dev(mu_hcd), &dev_attr_urbs);
}

static int mausb_start(struct usb_hcd *hcd)
{
	struct mausb_hcd	*mu_hcd = hcd_to_mausb_hcd(hcd);

	/*
	 * MASTER side init ... we emulate a root hub that'll only ever
	 * talk to one device (the slave side).  Also appears in sysfs,
	 * just like more familiar pci-based HCDs.
	 */
	if (!usb_hcd_is_primary_hcd(hcd))
		return mausb_start_ss(mu_hcd);

	spin_lock_init(&mu_hcd->mu->lock);
	tasklet_hrtimer_init(&mu_hcd->ttimer, mausb_timer,
		CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	mu_hcd->rh_state = MAUSB_RH_RUNNING;

	INIT_LIST_HEAD(&mu_hcd->urbp_list);

	hcd->power_budget = POWER_BUDGET;
	hcd->state = HC_STATE_RUNNING;
	hcd->uses_new_polling = 1;

#ifdef CONFIG_USB_OTG
	hcd->self.otg_port = 1;
#endif

	/* FIXME 'urbs' should be a per-device thing, maybe in usbcore */
	return device_create_file(mausb_dev(mu_hcd), &dev_attr_urbs);
}

static void mausb_stop(struct usb_hcd *hcd)
{
	struct mausb		*mu;

	mu = hcd_to_mausb_hcd(hcd)->mu;
	device_remove_file(mausb_dev(hcd_to_mausb_hcd(hcd)), &dev_attr_urbs);
	usb_gadget_unregister_driver(mu->driver);
	dev_info(mausb_dev(hcd_to_mausb_hcd(hcd)), "stopped\n");
}

/*-------------------------------------------------------------------------*/

static int mausb_h_get_frame(struct usb_hcd *hcd)
{
	return mausb_g_get_frame(NULL);
}

static int mausb_setup(struct usb_hcd *hcd)
{
	struct mausb *mu;

	mu = *((void **)dev_get_platdata(hcd->self.controller));
	hcd->self.sg_tablesize = ~0;
	if (usb_hcd_is_primary_hcd(hcd)) {
		mu->hs_hcd = hcd_to_mausb_hcd(hcd);
		mu->hs_hcd->mu = mu;
		/*
		 * Mark the first roothub as being USB 2.0.
		 * The USB 3.0 roothub will be registered later by
		 * mausb_hcd_probe()
		 */
		hcd->speed = HCD_USB2;
		hcd->self.root_hub->speed = USB_SPEED_HIGH;
	} else {
		mu->ss_hcd = hcd_to_mausb_hcd(hcd);
		mu->ss_hcd->mu = mu;
		hcd->speed = HCD_USB3;
		hcd->self.root_hub->speed = USB_SPEED_SUPER;
	}
	return 0;
}

/* Change a group of bulk endpoints to support multiple stream IDs */
static int mausb_alloc_streams(struct usb_hcd *hcd, struct usb_device *udev,
	struct usb_host_endpoint **eps, unsigned int num_eps,
	unsigned int num_streams, gfp_t mem_flags)
{
	struct mausb_hcd *mu_hcd = hcd_to_mausb_hcd(hcd);
	unsigned long flags;
	int max_stream;
	int ret_streams = num_streams;
	unsigned int index;
	unsigned int i;

	if (!num_eps)
		return -EINVAL;

	spin_lock_irqsave(&mu_hcd->mu->lock, flags);
	for (i = 0; i < num_eps; i++) {
		index = mausb_get_ep_idx(&eps[i]->desc);
		if ((1 << index) & mu_hcd->stream_en_ep) {
			ret_streams = -EINVAL;
			goto out;
		}
		max_stream = usb_ss_max_streams(&eps[i]->ss_ep_comp);
		if (!max_stream) {
			ret_streams = -EINVAL;
			goto out;
		}
		if (max_stream < ret_streams) {
			dev_dbg(mausb_dev(mu_hcd), "Ep 0x%x only supports %u "
					"stream IDs.\n",
					eps[i]->desc.bEndpointAddress,
					max_stream);
			ret_streams = max_stream;
		}
	}

	for (i = 0; i < num_eps; i++) {
		index = mausb_get_ep_idx(&eps[i]->desc);
		mu_hcd->stream_en_ep |= 1 << index;
		set_max_streams_for_pipe(mu_hcd,
				usb_endpoint_num(&eps[i]->desc), ret_streams);
	}
out:
	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);
	return ret_streams;
}

/* Reverts a group of bulk endpoints back to not using stream IDs. */
static int mausb_free_streams(struct usb_hcd *hcd, struct usb_device *udev,
	struct usb_host_endpoint **eps, unsigned int num_eps,
	gfp_t mem_flags)
{
	struct mausb_hcd *mu_hcd = hcd_to_mausb_hcd(hcd);
	unsigned long flags;
	int ret;
	unsigned int index;
	unsigned int i;

	spin_lock_irqsave(&mu_hcd->mu->lock, flags);
	for (i = 0; i < num_eps; i++) {
		index = mausb_get_ep_idx(&eps[i]->desc);
		if (!((1 << index) & mu_hcd->stream_en_ep)) {
			ret = -EINVAL;
			goto out;
		}
	}

	for (i = 0; i < num_eps; i++) {
		index = mausb_get_ep_idx(&eps[i]->desc);
		mu_hcd->stream_en_ep &= ~(1 << index);
		set_max_streams_for_pipe(mu_hcd,
				usb_endpoint_num(&eps[i]->desc), 0);
	}
	ret = 0;
out:
	spin_unlock_irqrestore(&mu_hcd->mu->lock, flags);
	return ret;
}
#if 1
static int mausb_map_urb_for_dma(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)

{
//pr_info("%s mausb_hcd",__func__);
	return 0;
}
static void mausb_unmap_urb_for_dma(struct usb_hcd *hcd, struct urb *urb)
{

//pr_info("%s mausb_hcd",__func__);
}
#endif
static struct hc_driver mausb_hcd = {
	.description =		(char *) driver_name,
	.product_desc =		"MAUSB host controller",
	.hcd_priv_size =	sizeof(struct mausb_hcd),

	.flags =		HCD_USB3 | HCD_SHARED,

	.reset =		mausb_setup,
	.start =		mausb_start,
	.stop =			mausb_stop,

	.urb_enqueue =		mausb_urb_enqueue,
	.urb_dequeue =		mausb_urb_dequeue,

	.get_frame_number =	mausb_h_get_frame,

	.hub_status_data =	mausb_hub_status,
	.hub_control =		mausb_hub_control,
	.bus_suspend =		mausb_bus_suspend,
	.bus_resume =		mausb_bus_resume,
	.map_urb_for_dma=	mausb_map_urb_for_dma,
	.unmap_urb_for_dma= mausb_unmap_urb_for_dma,

	.alloc_streams =	mausb_alloc_streams,
	.free_streams =		mausb_free_streams,
};

static int mausb_hcd_probe(struct platform_device *pdev)
{
	struct mausb		*mu;
	struct usb_hcd		*hs_hcd;
	struct usb_hcd		*ss_hcd;
	int			retval;

    if (!enable_mausb) {
        printk("don't load mausb hcd in normal booting\n");
        return 0;
    }

	dev_info(&pdev->dev, "%s, driver " DRIVER_VERSION "\n", driver_desc);
	mu = *((void **)dev_get_platdata(&pdev->dev));

	if (!mod_data.is_super_speed)
		mausb_hcd.flags = HCD_USB2;
	hs_hcd = usb_create_hcd(&mausb_hcd, &pdev->dev, dev_name(&pdev->dev));
	if (!hs_hcd)
		return -ENOMEM;
	hs_hcd->has_tt = 1;

	retval = usb_add_hcd(hs_hcd, 0, 0);
	if (retval)
		goto put_usb2_hcd;

	if (mod_data.is_super_speed) {
		ss_hcd = usb_create_shared_hcd(&mausb_hcd, &pdev->dev,
					dev_name(&pdev->dev), hs_hcd);
		if (!ss_hcd) {
			retval = -ENOMEM;
			goto dealloc_usb2_hcd;
		}

		retval = usb_add_hcd(ss_hcd, 0, 0);
		if (retval)
			goto put_usb3_hcd;
	}
	return 0;

put_usb3_hcd:
	usb_put_hcd(ss_hcd);
dealloc_usb2_hcd:
	usb_remove_hcd(hs_hcd);
put_usb2_hcd:
	usb_put_hcd(hs_hcd);
	mu->hs_hcd = mu->ss_hcd = NULL;
	return retval;
}

static int mausb_hcd_remove(struct platform_device *pdev)
{
	struct mausb		*mu;

    if (!enable_mausb) {
        printk("don't load mausb hcd in normal booting\n");
        return 0;
    }

	mu = hcd_to_mausb_hcd(platform_get_drvdata(pdev))->mu;

	if (mu->ss_hcd) {
		usb_remove_hcd(mausb_hcd_to_hcd(mu->ss_hcd));
		usb_put_hcd(mausb_hcd_to_hcd(mu->ss_hcd));
	}

	usb_remove_hcd(mausb_hcd_to_hcd(mu->hs_hcd));
	usb_put_hcd(mausb_hcd_to_hcd(mu->hs_hcd));

	mu->hs_hcd = NULL;
	mu->ss_hcd = NULL;

	return 0;
}

static int mausb_hcd_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct usb_hcd		*hcd;
	struct mausb_hcd	*mu_hcd;
	int			rc = 0;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	hcd = platform_get_drvdata(pdev);
	mu_hcd = hcd_to_mausb_hcd(hcd);
	if (mu_hcd->rh_state == MAUSB_RH_RUNNING) {
		dev_warn(&pdev->dev, "Root hub isn't suspended!\n");
		rc = -EBUSY;
	} else
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	return rc;
}

static int mausb_hcd_resume(struct platform_device *pdev)
{
	struct usb_hcd		*hcd;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	hcd = platform_get_drvdata(pdev);
	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	usb_hcd_poll_rh_status(hcd);
	return 0;
}

static struct platform_driver mausb_hcd_driver = {
	.probe		= mausb_hcd_probe,
	.remove		= mausb_hcd_remove,
	.suspend	= mausb_hcd_suspend,
	.resume		= mausb_hcd_resume,
	.driver		= {
		.name	= (char *) driver_name,
		.owner	= THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/
#define MAX_NUM_UDC	2
static struct platform_device *the_udc_pdev[MAX_NUM_UDC];
static struct platform_device *the_hcd_pdev[MAX_NUM_UDC];

static int __init init(void)
{
	int	retval = -ENOMEM;
	int	i;
	struct	mausb *mu[MAX_NUM_UDC];

	if (usb_disabled())
		return -ENODEV;

	if (!mod_data.is_high_speed && mod_data.is_super_speed)
		return -EINVAL;

	if (mod_data.num < 1 || mod_data.num > MAX_NUM_UDC) {
		pr_err("Number of emulated UDC must be in range of 1…%d\n",
				MAX_NUM_UDC);
		return -EINVAL;
	}

	for (i = 0; i < mod_data.num; i++) {
		the_hcd_pdev[i] = platform_device_alloc(driver_name, i);
		if (!the_hcd_pdev[i]) {
			i--;
			while (i >= 0)
				platform_device_put(the_hcd_pdev[i--]);
			return retval;
		}
	}
	for (i = 0; i < mod_data.num; i++) {
		the_udc_pdev[i] = platform_device_alloc(gadget_name, i);
		if (!the_udc_pdev[i]) {
			i--;
			while (i >= 0)
				platform_device_put(the_udc_pdev[i--]);
			goto err_alloc_udc;
		}
	}
	for (i = 0; i < mod_data.num; i++) {
		mu[i] = kzalloc(sizeof(struct mausb), GFP_KERNEL);
		if (!mu[i]) {
			retval = -ENOMEM;
			goto err_add_pdata;
		}
		retval = platform_device_add_data(the_hcd_pdev[i], &mu[i],
				sizeof(void *));
		if (retval)
			goto err_add_pdata;
		retval = platform_device_add_data(the_udc_pdev[i], &mu[i],
				sizeof(void *));
		if (retval)
			goto err_add_pdata;
	}

	retval = platform_driver_register(&mausb_hcd_driver);
	if (retval < 0)
		goto err_add_pdata;
	retval = platform_driver_register(&mausb_udc_driver);
	if (retval < 0)
		goto err_register_udc_driver;

	for (i = 0; i < mod_data.num; i++) {
		retval = platform_device_add(the_hcd_pdev[i]);
		if (retval < 0) {
			i--;
			while (i >= 0)
				platform_device_del(the_hcd_pdev[i--]);
			goto err_add_hcd;
		}
	}
	for (i = 0; i < mod_data.num; i++) {
		if (!mu[i]->hs_hcd ||
				(!mu[i]->ss_hcd && mod_data.is_super_speed)) {
			/*
			 * The hcd was added successfully but its probe
			 * function failed for some reason.
			 */
			retval = -EINVAL;
			goto err_add_udc;
		}
	}

	for (i = 0; i < mod_data.num; i++) {
		retval = platform_device_add(the_udc_pdev[i]);
		if (retval < 0) {
			i--;
			while (i >= 0)
				platform_device_del(the_udc_pdev[i]);
			goto err_add_udc;
		}
	}

	for (i = 0; i < mod_data.num; i++) {
		if (!platform_get_drvdata(the_udc_pdev[i])) {
			/*
			 * The udc was added successfully but its probe
			 * function failed for some reason.
			 */
			retval = -EINVAL;
			goto err_probe_udc;
		}
	}
	return retval;

err_probe_udc:
	for (i = 0; i < mod_data.num; i++)
		platform_device_del(the_udc_pdev[i]);
err_add_udc:
	for (i = 0; i < mod_data.num; i++)
		platform_device_del(the_hcd_pdev[i]);
err_add_hcd:
	platform_driver_unregister(&mausb_udc_driver);
err_register_udc_driver:
	platform_driver_unregister(&mausb_hcd_driver);
err_add_pdata:
	for (i = 0; i < mod_data.num; i++)
		kfree(mu[i]);
	for (i = 0; i < mod_data.num; i++)
		platform_device_put(the_udc_pdev[i]);
err_alloc_udc:
	for (i = 0; i < mod_data.num; i++)
		platform_device_put(the_hcd_pdev[i]);
	return retval;
}
module_init(init);

static void __exit cleanup(void)
{
	int i;

	for (i = 0; i < mod_data.num; i++) {
		struct mausb *mu;

		mu = *((void **)dev_get_platdata(&the_udc_pdev[i]->dev));

		platform_device_unregister(the_udc_pdev[i]);
		platform_device_unregister(the_hcd_pdev[i]);
		kfree(mu);
	}
	platform_driver_unregister(&mausb_udc_driver);
	platform_driver_unregister(&mausb_hcd_driver);
}
module_exit(cleanup);
