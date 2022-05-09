/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (c) 1998 Nicolas Souchu
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $FreeBSD$
 *
 */

#ifndef __DRMCOMPAT_LINUX_I2C_H__
#define	__DRMCOMPAT_LINUX_I2C_H__

#include <sys/param.h>
#include <sys/malloc.h>
#include <dev/iicbus/iiconf.h>
#include <linux/device.h>
/* I2C compatibility. */
#define	I2C_M_RD	IIC_M_RD
#define	I2C_M_WR	IIC_M_WR
#define	I2C_M_NOSTART	IIC_M_NOSTART
#define	I2C_M_STOP	0x0004
/* No need for us */
#define I2C_FUNC_I2C                    0
#define I2C_FUNC_SMBUS_EMUL             0
#define I2C_FUNC_SMBUS_READ_BLOCK_DATA  0
#define I2C_FUNC_SMBUS_BLOCK_PROC_CALL  0
#define I2C_FUNC_10BIT_ADDR             0

#define	I2C_CLASS_DDC	0x8

struct i2c_msg {
	uint16_t 	addr;
	uint16_t	flags;
	uint16_t	len;
	uint8_t		*buf;
};  

struct i2c_adapter
{
	device_t bsddev;	
	char name[20];
	const struct i2c_lock_operations *lock_ops;
        const struct i2c_algorithm *algo;
	void				*algo_data;
	int retries;
	struct module			*owner;
	unsigned int			class; /* I2C_CLASS_* */
	struct {
		device_t	parent;
	}	dev;
	
};
struct i2c_algorithm {
        int (*master_xfer)(struct i2c_adapter *, struct i2c_msg *, int);
        uint32_t (*functionality)(struct i2c_adapter *);
};

struct i2c_lock_operations {
        void (*lock_bus)(struct i2c_adapter *, unsigned int);
        int (*trylock_bus)(struct i2c_adapter *, unsigned int);
        void (*unlock_bus)(struct i2c_adapter *, unsigned int);
};


static inline struct i2c_adapter *
i2c_bsd_adapter(device_t dev)
{
	struct i2c_adapter *adap;
	
	adap =  malloc(sizeof(struct i2c_adapter), M_TEMP, M_WAITOK | M_ZERO);
	adap->bsddev = dev;
	strcpy(adap->name, "emulated-i2c");
	return (adap);
}

static inline int 
i2c_transfer (struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{

        int ret,retries = 0;
        retries = adap->retries == 0 ? 1 : adap->retries;

	if (adap->lock_ops)
		adap->lock_ops->lock_bus(adap, 0);
	if (adap->algo != NULL && adap->algo->master_xfer != NULL) {
		for (; retries != 0; retries--) {
			ret = adap->algo->master_xfer(adap, msgs, num);
			if (ret != -EAGAIN)
				break;
		}

	} else {
			    struct iic_msg *bsd_msgs;
				int i;

				bsd_msgs = malloc(sizeof(struct iic_msg) * num, M_TEMP, M_WAITOK);
				memcpy(bsd_msgs, msgs, sizeof(struct iic_msg) * num);
				/*inux uses 7-bit addresses but FreeBSD 8-bit */
				for (i = 0; i < num; i++) {
					bsd_msgs[i].slave = msgs[i].addr << 1;
					bsd_msgs[i].flags = msgs[i].flags;
					bsd_msgs[i].len = msgs[i].len;
					bsd_msgs[i].buf = msgs[i].buf;
				}

				ret = iicbus_transfer(adap->bsddev, bsd_msgs, num);
				free(bsd_msgs, M_TEMP);
				if (ret !=0)
					ret = (-ret);
	}
	if (adap->lock_ops)
		adap->lock_ops->unlock_bus(adap, 0);
				if (ret < 0)
					return (ret);

				return (num);


}
static inline int
i2c_add_adapter(struct i2c_adapter *adapter __unused)
{

	return 0;
}

static inline void
i2c_del_adapter(struct i2c_adapter *adapter __unused)
{
}
 	
#endif	/* __DRMCOMPAT_LINUX_I2C_H__ */
