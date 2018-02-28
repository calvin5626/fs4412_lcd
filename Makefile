
#KERNELDIR  :=/lib/modules/$(shell  uname  -r)/build
KERNELDIR  :=/home/linux/project/linux-3.14


123:
	make  -C  $(KERNELDIR)  M=$(shell  pwd)   modules

clean:
	make  -C  $(KERNELDIR)  M=$(shell  pwd)   clean


obj-m   +=fs_lcd.o



