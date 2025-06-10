obj-m += driver.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD)
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
install:
	sudo modprobe crc8
	sudo insmod driver.ko
uninstall:
	sudo rmmod driver.ko
check:
	dmesg | tail
read:
	cat /sys/class/hwmon/hwmon2/pm_1_0_input