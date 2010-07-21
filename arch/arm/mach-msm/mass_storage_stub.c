/* drivers/usb/function/mass_storage.c
 *
 * Function Driver for USB Mass Storage
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * Based heavily on the file_storage gadget driver in
 * drivers/usb/gadget/file_storage.c and licensed under the same terms:
 *
 * Copyright (C) 2003-2007 Alan Stern
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the above-listed copyright holders may not be used
 *    to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "../../../drivers/usb/function/mass_storage.h"

struct fsg_dev			*the_fsg;
EXPORT_SYMBOL(the_fsg);

static int cleanup;
static DEFINE_MUTEX(mass_storage_stub_sem);

static struct module *p_module;
static ssize_t (*p_show_file)(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t (*p_store_file)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t (*p_store_mass_storage_enable)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t (*p_show_mass_storage_enable)(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t show_file(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t rc = 0;

	mutex_lock(&mass_storage_stub_sem);
	try_module_get(p_module);
	if (p_show_file)
		rc = p_show_file(dev, attr, buf);
	module_put(p_module);
	mutex_unlock(&mass_storage_stub_sem);
	return rc;
}

static ssize_t store_file(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t rc = count;

	mutex_lock(&mass_storage_stub_sem);
	try_module_get(p_module);
	if (p_store_file)
		rc = p_store_file(dev, attr, buf, count);
	module_put(p_module);
	mutex_unlock(&mass_storage_stub_sem);
	return rc;
}

static DEVICE_ATTR(file, 0444, show_file, store_file);

/*-------------------------------------------------------------------------*/

static ssize_t store_mass_storage_enable(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	ssize_t rc = count;

	mutex_lock(&mass_storage_stub_sem);
	try_module_get(p_module);
	if (p_store_mass_storage_enable)
		rc = p_store_mass_storage_enable(dev, attr, buf, count);
	module_put(p_module);
	mutex_unlock(&mass_storage_stub_sem);
	return rc;
}

static ssize_t show_mass_storage_enable(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	ssize_t rc = 0;

	mutex_lock(&mass_storage_stub_sem);
	try_module_get(p_module);
	if (p_show_mass_storage_enable)
		rc = p_show_mass_storage_enable(dev, attr, buf);
	module_put(p_module);
	mutex_unlock(&mass_storage_stub_sem);
	return rc;
}

static DEVICE_ATTR(mass_storage_enable, 0644, show_mass_storage_enable, store_mass_storage_enable);

static int fsg_alloc(void)
{
	struct fsg_dev		*fsg;

	fsg = kzalloc(sizeof *fsg, GFP_KERNEL);
	if (!fsg)
		return -ENOMEM;
	spin_lock_init(&fsg->lock);
	init_rwsem(&fsg->filesem);
	kref_init(&fsg->ref);
	init_completion(&fsg->thread_notifier);

	the_fsg = fsg;
	return 0;
}

static ssize_t print_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_NAME);
}

static ssize_t print_switch_state(struct switch_dev *sdev, char *buf)
{
	struct fsg_dev	*fsg = container_of(sdev, struct fsg_dev, sdev);
	return sprintf(buf, "%s\n", (fsg->config ? "online" : "offline"));
}

static void fsg_release(struct kref *ref)
{
	struct fsg_dev	*fsg = container_of(ref, struct fsg_dev, ref);

	kfree(fsg->luns);
	kfree(fsg);
}

static void lun_release(struct device *dev)
{
	struct fsg_dev	*fsg = dev_get_drvdata(dev);

	kref_put(&fsg->ref, fsg_release);
}

static int fsg_remove(struct platform_device *pdev)
{
	int			i;
	struct lun		*curlun;

	/* Unregister the sysfs attribute files and the LUNs */
	for (i = 0; i < the_fsg->nluns; ++i) {
		curlun = &the_fsg->luns[i];
		if (curlun->registered) {
			device_remove_file(&curlun->dev, &dev_attr_file);
			device_unregister(&curlun->dev);
			curlun->registered = 0;
		}
	}

	device_remove_file(&the_fsg->pdev->dev, &dev_attr_mass_storage_enable);
	switch_dev_unregister(&the_fsg->sdev);
	kref_put(&the_fsg->ref, fsg_release);

	return 0;
}

static int fsg_probe(struct platform_device *pdev)
{
	struct usb_mass_storage_platform_data *pdata = pdev->dev.platform_data;
	int		rc;
	int			i;
	struct lun		*curlun;

	rc = fsg_alloc();
	if (rc != 0)
		goto fsg_alloc_fail;

	if (pdev->id == 0x01)
		the_fsg->cdrom_lun = 1;
	else
		the_fsg->cdrom_lun = -1;
	the_fsg->pdev = pdev;
	the_fsg->sdev.name = DRIVER_NAME;
	the_fsg->nluns = pdata->nluns;
	the_fsg->buf_size = pdata->buf_size;
	the_fsg->vendor = pdata->vendor;
	the_fsg->product = pdata->product;
	the_fsg->release = pdata->release;
	the_fsg->sdev.print_name = print_switch_name;
	the_fsg->sdev.print_state = print_switch_state;
	rc = switch_dev_register(&the_fsg->sdev);
	if (rc < 0)
		goto err_switch_dev_register;

	rc = device_create_file(&the_fsg->pdev->dev,
		&dev_attr_mass_storage_enable);
	if (rc != 0) {
		printk(KERN_WARNING "dev_attr_mass_storage_enable failed\n");
		goto device_create_file_fail;
	}

	wake_lock_init(&the_fsg->wake_lock, WAKE_LOCK_SUSPEND,
		       "usb_mass_storage");

	dev_attr_file.attr.mode = 0644;

	/* Find out how many LUNs there should be */
	i = the_fsg->nluns;
	if (i == 0)
		i = 1;
	if (i > MAX_LUNS) {
		ERROR(the_fsg, "invalid number of LUNs: %d\n", i);
		rc = -EINVAL;
		goto bad_lun_count_fail;
	}

	/* Create the LUNs, open their backing files, and register the
	 * LUN devices in sysfs. */
	the_fsg->luns = kzalloc(i * sizeof(struct lun), GFP_KERNEL);
	if (!the_fsg->luns) {
		rc = -ENOMEM;
		goto lun_alloc_fail;
	}
	the_fsg->nluns = i;

	for (i = 0; i < the_fsg->nluns; ++i) {
		curlun = &the_fsg->luns[i];
		curlun->ro = 0;
		if (i == the_fsg->cdrom_lun) {
			curlun->cdrom = 1;
			curlun->ro = 1;
		}
		curlun->dev.release = lun_release;
		curlun->dev.parent = &the_fsg->pdev->dev;
		dev_set_drvdata(&curlun->dev, the_fsg);
		snprintf(curlun->dev.bus_id, BUS_ID_SIZE,
				"lun%d", i);

		rc = device_register(&curlun->dev);
		if (rc != 0) {
			INFO(the_fsg, "failed to register LUN%d: %d\n", i, rc);
			goto luns_init_fail;
		}
		rc = device_create_file(&curlun->dev, &dev_attr_file);
		if (rc != 0) {
			ERROR(the_fsg, "device_create_file failed: %d\n", rc);
			device_unregister(&curlun->dev);
			goto luns_init_fail;
		}
		curlun->registered = 1;
		kref_get(&the_fsg->ref);
	}

	
	cleanup = 1;		
	return 0;

luns_init_fail:
	for (i = 0; i < the_fsg->nluns; ++i) {
		curlun = &the_fsg->luns[i];
		if (curlun->registered) {
			device_remove_file(&curlun->dev, &dev_attr_file);
			device_unregister(&curlun->dev);
			curlun->registered = 0;
		}
	}
lun_alloc_fail:
bad_lun_count_fail:
	device_remove_file(&the_fsg->pdev->dev, &dev_attr_mass_storage_enable);
device_create_file_fail:
	switch_dev_unregister(&the_fsg->sdev);
err_switch_dev_register:
	kref_put(&the_fsg->ref, fsg_release);
fsg_alloc_fail:
	return rc;
}

static struct platform_driver fsg_driver = {
	.probe = fsg_probe,
	.remove = fsg_remove,
	.driver = { .name = DRIVER_NAME, },
};

void mass_storage_stub_init(void)
{
	platform_driver_register(&fsg_driver);
}

void mass_storage_stub_exit(void)
{
	if (cleanup)
		platform_driver_unregister(&fsg_driver);
}

void mass_storage_stub_set_handlers(
	struct module *_p_module,
	ssize_t (*_p_show_file)(struct device *dev, struct device_attribute *attr, char *buf),
	ssize_t (*_p_store_file)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count),
	ssize_t (*_p_store_mass_storage_enable)(struct device *dev, struct device_attribute *attr, const char *buf, size_t count),
	ssize_t (*_p_show_mass_storage_enable)(struct device *dev, struct device_attribute *attr, char *buf)) 
{
	mutex_lock(&mass_storage_stub_sem);
	p_module = _p_module;
	p_show_file = _p_show_file;
	p_store_file = _p_store_file;
	p_store_mass_storage_enable = _p_store_mass_storage_enable;
	p_show_mass_storage_enable = _p_show_mass_storage_enable;	
	mutex_unlock(&mass_storage_stub_sem);
}
EXPORT_SYMBOL(mass_storage_stub_set_handlers);
