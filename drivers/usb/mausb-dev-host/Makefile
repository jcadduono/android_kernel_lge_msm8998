obj-$(CONFIG_USB_MAUSB) += mausb-core.o
mausb-core-y := mausb_common.o mausb_event.o

obj-$(CONFIG_USB_MAUSB_DEVICE) += mausb-device.o
mausb-device-y := stub_dev.o stub_main.o stub_rx.o stub_tx.o
mausb-device-y += mausb_mgmt.o mausb_out.o mausb_in.o mausb_util.o

obj-$(CONFIG_USB_MAUSB_HCD) += mausb-hcd.o
mausb-hcd-y := mausb_hcd.o
