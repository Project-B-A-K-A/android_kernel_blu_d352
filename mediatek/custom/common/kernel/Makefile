include $(MTK_PATH_BUILD)/common.mk
$(call all-modules-src-or-makefile,$(obj),CUSTOM_KERNEL_)
obj-n := dummy.o

ifeq ($(MTK_ALPS_BOX_SUPPORT), yes)
ccflags-y += -DMTK_ALPS_BOX_SUPPORT
endif

ifneq (,$(filter ft5x06,$(CUSTOM_KERNEL_TOUCHPANEL)))
obj-y += ft_gesture_lib.o
$(obj)/ft_gesture_lib.o: $(srctree)/$(obj)/touchpanel/ft_gesture_lib.a
	cp $(srctree)/$(obj)/touchpanel/ft_gesture_lib.a $(obj)/ft_gesture_lib.o 

endif
