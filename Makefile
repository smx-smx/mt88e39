obj-m += mt88e39.o

all:
	@$(MAKE) ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C ${KDIR} M=$(PWD) modules
clean:
	@$(MAKE) ARCH=${ARCH} CROSS_COMPILE=${CROSS_COMPILE} -C ${KDIR} M=$(PWD) clean
