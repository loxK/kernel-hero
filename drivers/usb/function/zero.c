/* driver/usb/function/zero.c
 *
 * Zero Function Device - A Trivial Data Source
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
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
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include "usb_function.h"

struct zero_context
{
	struct usb_endpoint *in;
	struct usb_request *req0;
	struct usb_request *req1;
};

static struct zero_context _context;

static int zero_bind(struct usb_endpoint **ept, void *_ctxt)
{
	struct zero_context *ctxt = _ctxt;
	ctxt->in = ept[0];

	if ((ctxt->req0 = usb_ept_alloc_req(ctxt->in, 4096)) == NULL)
		goto req0_fail;
	if ((ctxt->req1 = usb_ept_alloc_req(ctxt->in, 4096)) == NULL)
		goto req1_fail;

	memset(ctxt->req0->buf, 0, 4096);
	memset(ctxt->req1->buf, 0, 4096);

	return 0;

req1_fail:
	usb_ept_free_req(ctxt->in, ctxt->req0);
req0_fail:
	printk("ums_bind() could not allocate requests\n");
	return -1;
}

static void zero_unbind(void *_ctxt)
{
	struct zero_context *ctxt = _ctxt;
	if (ctxt->req0) {
		usb_ept_free_req(ctxt->in, ctxt->req0);
		ctxt->req0 = 0;
	}
	if (ctxt->req1) {
		usb_ept_free_req(ctxt->in, ctxt->req1);
		ctxt->req1 = 0;
	}
	ctxt->in = 0;
}

static void zero_queue_in(struct zero_context *ctxt, struct usb_request *req);

static void zero_in_complete(struct usb_endpoint *ept, struct usb_request *req)
{
	struct zero_context *ctxt = req->context;

	if (req->status != -ENODEV)
		zero_queue_in(ctxt, req);
}

static void zero_queue_in(struct zero_context *ctxt, struct usb_request *req)
{
	req->complete = zero_in_complete;
	req->context = ctxt;
	req->length = 4096;

	usb_ept_queue_xfer(ctxt->in, req);
}

static void zero_configure(int configured, void *_ctxt)
{
	struct zero_context *ctxt = _ctxt;
	printk(KERN_INFO "zero_configure() %d\n", configured);

	if (configured) {
		zero_queue_in(ctxt, ctxt->req0);
		zero_queue_in(ctxt, ctxt->req1);
	} else {
		/* all pending requests will be canceled */
	}
}

static struct usb_function usb_func_zero = {
	.bind = zero_bind,
	.unbind = zero_unbind,
	.configure = zero_configure,

	.name = "zero",
	.context = &_context,

	.ifc_class = 0xff,
	.ifc_subclass = 0xfe,
	.ifc_protocol = 0x02,

	.ifc_name = "zero",

	.ifc_ept_count = 1,
	.ifc_ept_type = { EPT_BULK_IN },
};

void zero_init(void)
{
       usb_function_register(&usb_func_zero);
}
