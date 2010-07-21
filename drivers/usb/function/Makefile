
ifeq ($(CONFIG_USB_FUNCTION_NULL),y)
	USB_FUNCTION_DRIVERS		+= null.o
endif
ifeq ($(CONFIG_USB_FUNCTION_ZERO),y)
	USB_FUNCTION_DRIVERS		+= zero.o
endif
ifeq ($(CONFIG_USB_FUNCTION_LOOPBACK),y)
	USB_FUNCTION_DRIVERS		+= loopback.o
endif
ifeq ($(CONFIG_USB_FUNCTION_ADB),y)
	USB_FUNCTION_DRIVERS		+= adb.o
endif
ifeq ($(CONFIG_USB_FUNCTION_UMS),y)
	USB_FUNCTION_DRIVERS		+= ums.o
endif
ifeq ($(CONFIG_USB_FUNCTION_MASS_STORAGE),y)
	USB_FUNCTION_DRIVERS		+= mass_storage.o
endif
ifeq ($(CONFIG_USB_FUNCTION_DIAG),y)
	USB_FUNCTION_DRIVERS		+= diag.o
endif
ifeq ($(CONFIG_USB_FUNCTION_ETHER),y)
	USB_FUNCTION_DRIVERS		+= ether.o rndis.o
endif
ifeq ($(CONFIG_USB_FUNCTION_SERIAL),y)
	USB_FUNCTION_DRIVERS		+= fserial.o
endif
ifeq ($(CONFIG_USB_FUNCTION_MODEM),y)
	USB_FUNCTION_DRIVERS		+= serial.o
endif
ifeq ($(CONFIG_USB_FUNCTION_FSYNC),y)
	USB_FUNCTION_DRIVERS		+= fsync.o
endif
ifeq ($(CONFIG_USB_FUNCTION_MTP_TUNNEL),y)
	USB_FUNCTION_DRIVERS		+= mtp_tunnel.o
endif

f_msm_hsusb-objs                       := msm_hsusb.o $(USB_FUNCTION_DRIVERS)
obj-$(CONFIG_USB_MSM_HSUSB)            += f_msm_hsusb.o
