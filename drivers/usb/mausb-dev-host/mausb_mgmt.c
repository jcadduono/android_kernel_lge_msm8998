/*
* mausb_mgmt.c - handling MAUSB Management transfers.
*
* Copyright (C) 2015-2016 LGE Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2  of
* the License as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
*/

#include <linux/kthread.h>

#include "mausb_dev.h"
#include "stub.h"
#include "mausb_util.h"

static inline void mausb_send_response(struct mausb_device *ud,
		struct mausb_header *header)
{
	/*
	stub_send_mausb(ud,header);
	*/
}

static void mausb_dev_handle_req(struct mausb_device *ud,
	struct stub_mausb_pal *pal)
{
	/*
	struct mausb_header *resp;
	resp = kzalloc(sizeof(struct mausb_header), GFP_ATOMIC);
	memcpy( resp,header,sizeof(struct mausb_header));
	resp->type = USBDevHandleResp;
	resp->status = MAUSB_STATUS_NO_ERROR;
	mausb_send_response(ud, resp);
	*/
}

static void mausb_dev_reset_req(struct mausb_device *ud,
			struct stub_mausb_pal *pal)
{
/*
	struct mausb_header *resp;
	resp = kzalloc(sizeof(struct mausb_header), GFP_ATOMIC);
	memcpy( resp,header,sizeof(struct mausb_header));
	resp->type = MAUSBDevResetResp;
	resp->status = MAUSB_STATUS_NO_ERROR;
	mausb_send_response(ud, resp);
*/
}

/* be in spin_lock_irqsave(&sdev->priv_lock, flags) */
void mausb_enqueue_ret_unlink(struct stub_device *sdev,
			 struct stub_mausb_pal *pal,
			 __u32 seqnum, __u32 status)
{
	struct mausb_req_resp *unlink;
	struct stub_mausb_pal *unlink_pal;

	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
			"\n ---> mausb_enqueue_ret_unlink");
	unlink = kzalloc(sizeof(struct mausb_req_resp), GFP_ATOMIC);
	if (!unlink) {
		mausb_event_add(&sdev->ud, VDEV_EVENT_ERROR_MALLOC);
		return;
	}
	memcpy(unlink,pal->pdu,sizeof(struct mausb_header));

	unlink->r.cancel_resp.resrved = seqnum;
	unlink->header.u.non_iso_hdr.reqid_seqno = seqnum;
	unlink->header.base.status = -status;
	unlink->header.base.type_subtype = MAUSB_PKT_TYPE_MGMT | 0x29;
	unlink->header.base.length = sizeof(struct mausb_req_resp);
	unlink->r.cancel_resp.cancel_status = -status;

	unlink_pal = kzalloc(sizeof(struct stub_mausb_pal), GFP_ATOMIC);
	unlink_pal->pdu = unlink;
	unlink_pal->sdev = sdev;
	unlink_pal->length = sizeof(struct mausb_req_resp);
	unlink_pal->seqnum = seqnum;

	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
			"\n unlink_pal->seqnum : %u\n",unlink_pal->seqnum);
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
			"\n unlink->r.cancel_resp.resrved : %d\n",
			unlink->r.cancel_resp.resrved );
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
			"\n unlink_pal address : %p\n",unlink_pal);
	list_add_tail(&unlink_pal->list, &sdev->mausb_unlink_tx);
}

static int mausb_mgmt_handle_pkt(struct mausb_device *ud,
			struct stub_mausb_pal *pal)
{
	struct stub_device *sdev = container_of(ud, struct stub_device, ud);
	struct mausb_req_resp *req;
	req = (struct mausb_req_resp *)pal->pdu;

	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
			"\n->%s",__func__);
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
			"\nreq->header.base.type_subtype:%d",
			req->header.base.type_subtype);
	switch (req->header.base.type_subtype) {

	/* subtypes with variable length additional data */
	case EPHandleReq:
		break;
	case EPHandleResp:
		break;
	/* TODO: Dissect type-specific managment packet fields */
	case EPActivateReq:
	case EPActivateResp:
	case EPInactivateReq:
	case EPInactivateResp:
	case EPRestartReq:
	case EPRestartResp:
	case EPClearTransferReq:
	case EPClearTransferResp:
	case EPHandleDeleteReq:
		break;
	case EPHandleDeleteResp:
		break;
	case ModifyEP0Resp:
	case EPCloseStreamResp:
	case USBDevResetReq:
	case USBDevResetResp:
	case EPOpenStreamResp:
	case VendorSpecificReq:
	case VendorSpecificResp:
	/* FALLTHROUGH */

	/* subtypes with constant length additional data */
	case CapReq:
	case CapResp:
	case USBDevHandleReq:
		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,"\nUSBDevHandleReq \n");
		mausb_dev_handle_req(ud, pal);
		break;
	case USBDevHandleResp:
	case ModifyEP0Req:
	case SetDevAddrResp:
	case UpdateDevReq:
	case MAUSBSyncReq:
	case EPCloseStreamReq:
	case CancelTransferReq:
	{
		struct mausb_header *hdr;
		struct stub_mausb_pal *mausb_pal, *tmp;
		int ret =0;
		unsigned long flags;
		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT," CancelTransferReq ");
		spin_lock_irqsave(&sdev->mausb_pal_lock, flags);
		list_for_each_entry_safe(mausb_pal, tmp, &sdev->mausb_pal_submit, list) {
			hdr = (struct mausb_header *)mausb_pal->pdu;

			/* TODO: comparision should be with request id */
			if (hdr->u.non_iso_hdr.reqid_seqno !=
				req->r.cancel_req.resrved2)
					continue;

			mausb_pal->unlinking = 1;
			ret = usb_unlink_urb(mausb_pal->urb);
			spin_unlock_irqrestore(&sdev->mausb_pal_lock, flags);
			if (ret != -EINPROGRESS)
				DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
					" failed to unlink a urb %p, ret %d",
					mausb_pal->urb, ret);
			DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
				"urb %p, ret %d", mausb_pal->urb, ret);

			stub_free_mausb_pal_and_urb(pal);
			return 0;
		}

		mausb_enqueue_ret_unlink(sdev,pal,req->r.cancel_req.resrved2,0);

		spin_unlock_irqrestore(&sdev->mausb_pal_lock, flags);
		stub_free_mausb_pal_and_urb(pal);

		wake_up(&sdev->tx_waitq);
		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,"wakeup tx");
		break;
	}
	case CancelTransferResp:
		break;
	case EPOpenStreamReq:
		break;

	/* Managment packets with no additional data */
	case MAUSBDevResetReq:
		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
				"\nMAUSBDevResetReq \n");
		mausb_dev_reset_req(ud, pal);
		break;
	case MAUSBDevResetResp:
	case SetDevAddrReq:
	case UpdateDevResp:
	case DisconnectDevReq:
	case DisconnectDevResp:
	case MAUSBDevSleepReq:
	case MAUSBDevSleepResp:
	case MAUSBDevWakeReq:
	case MAUSBDevWakeResp:
	case MAUSBDevInitSleepReq:
	case MAUSBDevInitSleepResp:
	case MAUSBDevRemoteWakeReq:
	case MAUSBDevRemoteWakeResp:
	case PingReq:
	case PingResp:
	case MAUSBDevDisconnectReq:
	case MAUSBDevDisconnectResp:
	case MAUSBDevInitDisconReq:
	case MAUSBDevInitDisconResp:
	case MAUSBSyncResp:
		break;

	default:
		DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,
				"invalid management request \n");
	break;

	}
	return 0;
}

/* Code to handle packets received from MA USB host */
static inline int mausb_mgmt_dissect_pkt(struct mausb_device *ud,
		struct stub_mausb_pal *pal)
{
    return 0;
}

static struct stub_mausb_pal *dequeue_from_mausb_pal_mgmt_init(
		struct stub_device *sdev)
{
	unsigned long flags;
	struct stub_mausb_pal *pal, *tmp;
	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,"---> %s\n",__func__);

	spin_lock_irqsave(&sdev->mausb_pal_lock, flags);
	list_for_each_entry_safe(pal, tmp, &sdev->mausb_pal_mgmt_init, list) {
		spin_unlock_irqrestore(&sdev->mausb_pal_lock, flags);
		return pal;
	}
	spin_unlock_irqrestore(&sdev->mausb_pal_lock, flags);
	return NULL;
}

void mausbdev_mgmt_process_packect(struct mausb_device *ud)
{
	struct stub_device *sdev = container_of(ud, struct stub_device, ud);
	struct stub_mausb_pal *pal;
	char *buff;

	DBG_MAUSB(DBG_LEVEL_MEDIUM,DATA_TRANS_MGMT,"IN %s\n",__func__);
	while ((pal = dequeue_from_mausb_pal_mgmt_init(sdev)) != NULL) {
		buff = pal->pdu;

		mausb_mgmt_handle_pkt(ud, pal);
	}
}
