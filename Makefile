EXTRA_CFLAGS += -I$(src)
obj-$(CONFIG_4DPI_SPI_DMA)        += spi-bcm2708_dma.o
obj-$(CONFIG_FB_HATS)             += 4dpi.o

ifeq ($(CONFIG_LOCALVERSION),"-v7")
        4dpi-objs                       := 4d-hats.o compress-v7.o
else
        4dpi-objs                       := 4d-hats.o compress-v6.o
endif

obj-$(CONFIG_4DPI_ADS7846)        += 4dpi_touch.o
obj-$(CONFIG_4DPI_AR1020I2C)      += ar1020-i2c.o
obj-$(CONFIG_4DPI_PWMBL)          += pwmbl.o
