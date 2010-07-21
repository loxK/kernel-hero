#ifndef USB_FUNCTION_MASS_STORAGE_H
#define USB_FUNCTION_MASS_STORAGE_H 1

//#define DEBUG
//#define VERBOSE_DEBUG
//#define DUMP_MSGS
//#define DEBUG_SCSI_CMD
/* use mass_storage command(SC_RESERVE) to enable/disable adb daemon */
#define ENABLE_SCSI_CMD_RESERVE_6_TO_SPEC_CMD

#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/switch.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/usb/ch9.h>
#include <linux/usb/mass_storage_function.h>
#include <linux/usb_usual.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#ifdef ENABLE_SCSI_CMD_RESERVE_6_TO_SPEC_CMD
#include <linux/reboot.h>
#include <linux/syscalls.h>
#endif /* ENABLE_SCSI_CMD_RESERVE_6_TO_SPEC_CMD */


/*-------------------------------------------------------------------------*/

#define DRIVER_NAME		"usb_mass_storage"
#define MAX_LUNS		8

#ifdef DEBUG
#define LDBG(lun, fmt, args...) \
	dev_dbg(&(lun)->dev , fmt , ## args)
#define MDBG(fmt,args...) \
	printk(KERN_DEBUG DRIVER_NAME ": " fmt , ## args)
#else
#define LDBG(lun, fmt, args...) \
	do { } while (0)
#define MDBG(fmt,args...) \
	do { } while (0)
#undef VERBOSE_DEBUG
#undef DUMP_MSGS
#endif /* DEBUG */

#ifdef VERBOSE_DEBUG
#define VLDBG	LDBG
#else
#define VLDBG(lun, fmt, args...) \
	do { } while (0)
#endif /* VERBOSE_DEBUG */

#define LERROR(lun, fmt, args...) \
	dev_err(&(lun)->dev , fmt , ## args)
#define LWARN(lun, fmt, args...) \
	dev_warn(&(lun)->dev , fmt , ## args)
#define LINFO(lun, fmt, args...) \
	dev_info(&(lun)->dev , fmt , ## args)

#define MINFO(fmt,args...) \
	printk(KERN_INFO DRIVER_NAME ": " fmt , ## args)

#define DBG(d, fmt, args...) \
	dev_dbg(&(d)->pdev->dev , fmt , ## args)
#define VDBG(d, fmt, args...) \
	dev_vdbg(&(d)->pdev->dev , fmt , ## args)
#define ERROR(d, fmt, args...) \
	dev_err(&(d)->pdev->dev , fmt , ## args)
#define MS_WARN(d, fmt, args...) \
	dev_warn(&(d)->pdev->dev , fmt , ## args)
#define INFO(d, fmt, args...) \
	dev_info(&(d)->pdev->dev , fmt , ## args)

#ifdef DEBUG_SCSI_CMD
#define SCSI_CMD_DBG(fmt,args...) \
	printk(KERN_DEBUG "SCSI_CMD: " fmt , ## args)
#else
#define SCSI_CMD_DBG(fmt,args...) \
	do { } while (0)
#endif

/*-------------------------------------------------------------------------*/

/* Bulk-only data structures */

/* Command Block Wrapper */
struct bulk_cb_wrap {
	__le32	Signature;		/* Contains 'USBC' */
	u32	Tag;			/* Unique per command id */
	__le32	DataTransferLength;	/* Size of the data */
	u8	Flags;			/* Direction in bit 7 */
	u8	Lun;			/* LUN (normally 0) */
	u8	Length;			/* Of the CDB, <= MAX_COMMAND_SIZE */
	u8	CDB[16];		/* Command Data Block */
};

#define USB_BULK_CB_WRAP_LEN	31
#define USB_BULK_CB_SIG		0x43425355	/* Spells out USBC */
#define USB_BULK_IN_FLAG	0x80

/* Command Status Wrapper */
struct bulk_cs_wrap {
	__le32	Signature;		/* Should = 'USBS' */
	u32	Tag;			/* Same as original command */
	__le32	Residue;		/* Amount not transferred */
	u8	Status;			/* See below */
};

#define USB_BULK_CS_WRAP_LEN	13
#define USB_BULK_CS_SIG		0x53425355	/* Spells out 'USBS' */
#define USB_STATUS_PASS		0
#define USB_STATUS_FAIL		1
#define USB_STATUS_PHASE_ERROR	2

/* Bulk-only class specific requests */
#define USB_BULK_RESET_REQUEST		0xff
#define USB_BULK_GET_MAX_LUN_REQUEST	0xfe

/* Length of a SCSI Command Data Block */
#define MAX_COMMAND_SIZE	16

/* SCSI commands that we recognize */
#define SC_FORMAT_UNIT			0x04
#define SC_INQUIRY			0x12
#define SC_MODE_SELECT_6		0x15
#define SC_MODE_SELECT_10		0x55
#define SC_MODE_SENSE_6			0x1a
#define SC_MODE_SENSE_10		0x5a
#define SC_PREVENT_ALLOW_MEDIUM_REMOVAL	0x1e
#define SC_READ_6			0x08
#define SC_READ_10			0x28
#define SC_READ_12			0xa8
#define SC_READ_CAPACITY		0x25
#define SC_READ_FORMAT_CAPACITIES	0x23
#define SC_READ_HEADER			0x44
#define SC_READ_TOC			0x43
#define SC_RELEASE			0x17
#define SC_REQUEST_SENSE		0x03
#define SC_RESERVE			0x16
#define SC_SEND_DIAGNOSTIC		0x1d
#define SC_START_STOP_UNIT		0x1b
#define SC_SYNCHRONIZE_CACHE		0x35
#define SC_TEST_UNIT_READY		0x00
#define SC_VERIFY			0x2f
#define SC_WRITE_6			0x0a
#define SC_WRITE_10			0x2a
#define SC_WRITE_12			0xaa

/* SCSI Sense Key/Additional Sense Code/ASC Qualifier values */
#define SS_NO_SENSE				0
#define SS_COMMUNICATION_FAILURE		0x040800
#define SS_INVALID_COMMAND			0x052000
#define SS_INVALID_FIELD_IN_CDB			0x052400
#define SS_LOGICAL_BLOCK_ADDRESS_OUT_OF_RANGE	0x052100
#define SS_LOGICAL_UNIT_NOT_SUPPORTED		0x052500
#define SS_MEDIUM_NOT_PRESENT			0x023a00
#define SS_MEDIUM_REMOVAL_PREVENTED		0x055302
#define SS_NOT_READY_TO_READY_TRANSITION	0x062800
#define SS_RESET_OCCURRED			0x062900
#define SS_SAVING_PARAMETERS_NOT_SUPPORTED	0x053900
#define SS_UNRECOVERED_READ_ERROR		0x031100
#define SS_WRITE_ERROR				0x030c02
#define SS_WRITE_PROTECTED			0x072700

#define SK(x)		((u8) ((x) >> 16))	/* Sense Key byte, etc. */
#define ASC(x)		((u8) ((x) >> 8))
#define ASCQ(x)		((u8) (x))


/*-------------------------------------------------------------------------*/

struct lun {
	struct file	*filp;
	loff_t		file_length;
	loff_t		num_sectors;

	unsigned int	ro : 1;
	unsigned int	prevent_medium_removal : 1;
	unsigned int	registered : 1;
	unsigned int	info_valid : 1;
	unsigned int	cdrom : 1;

	u32		sense_data;
	u32		sense_data_info;
	u32		unit_attention_data;

	struct device	dev;
};

#define backing_file_is_open(curlun)	((curlun)->filp != NULL)


static struct lun *dev_to_lun(struct device *dev)
{
	return container_of(dev, struct lun, dev);
}

/* Big enough to hold our biggest descriptor */
#define EP0_BUFSIZE	256
#define DELAYED_STATUS	(EP0_BUFSIZE + 999)	/* An impossibly large value */

/* Number of buffers we will use.  2 is enough for double-buffering */
#define NUM_BUFFERS	4

enum fsg_buffer_state {
	BUF_STATE_EMPTY = 0,
	BUF_STATE_FULL,
	BUF_STATE_BUSY
};

struct fsg_buffhd {
	void				*buf;
	enum fsg_buffer_state		state;
	struct fsg_buffhd		*next;

	/* The NetChip 2280 is faster, and handles some protocol faults
	 * better, if we don't submit any short bulk-out read requests.
	 * So we will record the intended request length here. */
	unsigned int			bulk_out_intended_length;

	struct usb_request		*inreq;
	int				inreq_busy;
	struct usb_request		*outreq;
	int				outreq_busy;
};

enum fsg_state {
	/* This one isn't used anywhere */
	FSG_STATE_COMMAND_PHASE = -10,

	FSG_STATE_DATA_PHASE,
	FSG_STATE_STATUS_PHASE,

	FSG_STATE_IDLE = 0,
	FSG_STATE_ABORT_BULK_OUT,
	FSG_STATE_RESET,
	FSG_STATE_CONFIG_CHANGE,
	FSG_STATE_EXIT,
	FSG_STATE_TERMINATED
};

enum data_direction {
	DATA_DIR_UNKNOWN = 0,
	DATA_DIR_FROM_HOST,
	DATA_DIR_TO_HOST,
	DATA_DIR_NONE
};

struct fsg_dev {
	/* lock protects: state and all the req_busy's */
	spinlock_t		lock;

	/* filesem protects: backing files in use */
	struct rw_semaphore	filesem;

	/* reference counting: wait until all LUNs are released */
	struct kref		ref;

	unsigned int		bulk_out_maxpacket;
	enum fsg_state		state;		/* For exception handling */

	u8			config, new_config;

	unsigned int		running : 1;
	unsigned int		phase_error : 1;
	unsigned int		short_packet_received : 1;
	unsigned int		bad_lun_okay : 1;

	unsigned long		atomic_bitflags;
#define REGISTERED		0
#define CLEAR_BULK_HALTS	1
#define SUSPENDED		2

	struct usb_endpoint		*bulk_in;
	struct usb_endpoint		*bulk_out;

	struct fsg_buffhd	*next_buffhd_to_fill;
	struct fsg_buffhd	*next_buffhd_to_drain;
	struct fsg_buffhd	buffhds[NUM_BUFFERS];

	int			thread_wakeup_needed;
	struct completion	thread_notifier;
	struct task_struct	*thread_task;

	int			cmnd_size;
	u8			cmnd[MAX_COMMAND_SIZE];
	enum data_direction	data_dir;
	u32			data_size;
	u32			data_size_from_cmnd;
	u32			tag;
	unsigned int		lun;
	u32			residue;
	u32			usb_amount_left;

	unsigned int		nluns;
	struct lun		*luns;
	struct lun		*curlun;

	u32				buf_size;
	const char		*vendor;
	const char		*product;
	int				release;

	struct platform_device *pdev;
	struct switch_dev sdev;
	struct wake_lock wake_lock;
	int			intf_num;
	int			cdrom_lun;
};

extern void mass_storage_stub_set_handlers(
	struct module *_p_module,
	ssize_t (*_p_show_file)(struct device *dev, struct device_attribute *attr, char *buf),
	ssize_t (*_p_store_file)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count),
	ssize_t (*_p_store_mass_storage_enable)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count),
	ssize_t (*_p_show_mass_storage_enable)(struct device *dev, struct device_attribute *attr, char *buf));

#endif
