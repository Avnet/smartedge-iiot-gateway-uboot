/*
 * Driver for Broadcom BCM2835 auxiliary SPI Controllers
 *
 * the driver does not rely on the native chipselects at all
 * but only uses the gpio type chipselects
 *
 * Based on: spi-bcm2835.c
 *
 * Copyright (C) 2015 Martin Sperl
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <common.h>
#include <clk.h>
#include <dm.h>
#include <spi.h>
#include <reset.h>
#include <fdtdec.h>
#include <malloc.h>
#include <wait_bit.h>
#include <asm/io.h>
#include <asm/gpio.h>

/*
 * spi register defines
 *
 * note there is garbage in the "official" documentation,
 * so some data is taken from the file:
 *   brcm_usrlib/dag/vmcsx/vcinclude/bcm2708_chip/aux_io.h
 * inside of:
 *   http://www.broadcom.com/docs/support/videocore/Brcm_Android_ICS_Graphics_Stack.tar.gz
 */

/* SPI register offsets */
#define BCM2835_AUX_SPI_CNTL0	0x00
#define BCM2835_AUX_SPI_CNTL1	0x04
#define BCM2835_AUX_SPI_STAT	0x08
#define BCM2835_AUX_SPI_PEEK	0x0C
#define BCM2835_AUX_SPI_IO	0x20
#define BCM2835_AUX_SPI_TXHOLD	0x30

struct spi_regs {
  u32 cntl0;
  u32 cntl1;
  u32 stat;
  u32 peek;
  u32 regx10;
  u32 regx14;
  u32 regx18;
  u32 regx1c;
  u32 io;
  u32 regx24;
  u32 regx28;
  u32 regx2c;
  u32 txhold;
};

/* Bitfields in CNTL0 */
#define BCM2835_AUX_SPI_CNTL0_SPEED	0xFFF00000
#define BCM2835_AUX_SPI_CNTL0_SPEED_MAX	0xFFF
#define BCM2835_AUX_SPI_CNTL0_SPEED_SHIFT	20
#define BCM2835_AUX_SPI_CNTL0_CS	0x000E0000
#define BCM2835_AUX_SPI_CNTL0_POSTINPUT	0x00010000
#define BCM2835_AUX_SPI_CNTL0_VAR_CS	0x00008000
#define BCM2835_AUX_SPI_CNTL0_VAR_WIDTH	0x00004000
#define BCM2835_AUX_SPI_CNTL0_DOUTHOLD	0x00003000
#define BCM2835_AUX_SPI_CNTL0_ENABLE	0x00000800
#define BCM2835_AUX_SPI_CNTL0_IN_RISING	0x00000400
#define BCM2835_AUX_SPI_CNTL0_CLEARFIFO	0x00000200
#define BCM2835_AUX_SPI_CNTL0_OUT_RISING	0x00000100
#define BCM2835_AUX_SPI_CNTL0_CPOL	0x00000080
#define BCM2835_AUX_SPI_CNTL0_MSBF_OUT	0x00000040
#define BCM2835_AUX_SPI_CNTL0_SHIFTLEN	0x0000003F

/* Bitfields in CNTL1 */
#define BCM2835_AUX_SPI_CNTL1_CSHIGH	0x00000700
#define BCM2835_AUX_SPI_CNTL1_TXEMPTY	0x00000080
#define BCM2835_AUX_SPI_CNTL1_IDLE	0x00000040
#define BCM2835_AUX_SPI_CNTL1_MSBF_IN	0x00000002
#define BCM2835_AUX_SPI_CNTL1_KEEP_IN	0x00000001

/* Bitfields in STAT */
#define BCM2835_AUX_SPI_STAT_TX_LVL	0xFF000000
#define BCM2835_AUX_SPI_STAT_RX_LVL	0x00FF0000
#define BCM2835_AUX_SPI_STAT_TX_FULL	0x00000400
#define BCM2835_AUX_SPI_STAT_TX_EMPTY	0x00000200
#define BCM2835_AUX_SPI_STAT_RX_FULL	0x00000100
#define BCM2835_AUX_SPI_STAT_RX_EMPTY	0x00000080
#define BCM2835_AUX_SPI_STAT_BUSY	0x00000040
#define BCM2835_AUX_SPI_STAT_BITCOUNT	0x0000003F

/* timeout values */
#define BCM2835_AUX_SPI_POLLING_LIMIT_US	30
#define BCM2835_AUX_SPI_POLLING_JIFFIES		2

struct bcm2835aux_spi {
	struct spi_regs *regs;
	const u8 *tx_buf;
	u8 *rx_buf;
	int tx_len;
	int rx_len;
};
struct gpio_desc cs_gpios[3];
struct bcm2835aux_spi AuxSpiBase;

static inline u32 bcm2835aux_cs_activate(void)
{
  dm_gpio_set_value(&cs_gpios[2], 0);
  mdelay(1);
}
static inline u32 bcm2835aux_cs_deactivate()
{
  mdelay(1);
  dm_gpio_set_value(&cs_gpios[2], 1);
  mdelay(1);
}

static inline u32 bcm2835aux_rd(u32 *address)
{
  u32 value;
  value = readl(address);
  //if (address > 0x3f2150d0) printf(" -%06X ", value);
	return value;
}

static inline void bcm2835aux_wr(u32 *address,
				 u32 val)
{
  //if (address > 0x3f2150d0) printf(" +%06X ", val >> 8);
  //printf("+w=%x %x ", address,val);
	writel(val, address);
	mdelay(1);
}

static inline void bcm2835aux_rd_fifo()
{
	u32 data;
	int count = min(AuxSpiBase.rx_len, 3);

	//data = bcm2835aux_rd(&AuxSpiBase.regs->io);
	data = bcm2835aux_rd(&AuxSpiBase.regs->io);
	if (AuxSpiBase.rx_buf) {
		switch (count) {
		case 4:
			*AuxSpiBase.rx_buf++ = (data >> 24) & 0xff;
			/* fallthrough */
		case 3:
			*AuxSpiBase.rx_buf++ = (data >> 16) & 0xff;
			/* fallthrough */
		case 2:
			*AuxSpiBase.rx_buf++ = (data >> 8) & 0xff;
			/* fallthrough */
		case 1:
			*AuxSpiBase.rx_buf++ = (data >> 0) & 0xff;
			/* fallthrough - no default */
		}
	}
	AuxSpiBase.rx_len -= count;
}

static inline void bcm2835aux_wr_fifo()
{
	u32 data;
	u8 byte;
	int count;
	int i;

	/* gather up to 3 bytes to write to the FIFO */
	count = min(AuxSpiBase.tx_len, 3);
	data = 0;
	for (i = 0; i < count; i++) {
		byte = AuxSpiBase.tx_buf ? *AuxSpiBase.tx_buf++ : 0;
		data |= byte << (8 * (2 - i));
	}

	/* and set the variable bit-length */
	data |= (count * 8) << 24;

	/* and decrement length */
	AuxSpiBase.tx_len -= count;

	/* write to the correct TX-register */
	if (AuxSpiBase.tx_len)
		bcm2835aux_wr(&AuxSpiBase.regs->txhold, data);
	else
		bcm2835aux_wr(&AuxSpiBase.regs->io, data);
}

static void bcm2835aux_spi_reset_hw()
{
  /* disable spi clearing fifo and interrupts */
  bcm2835aux_wr(&AuxSpiBase.regs->cntl1, 0);
	bcm2835aux_wr(&AuxSpiBase.regs->cntl0, 0x200);
	bcm2835aux_wr(&AuxSpiBase.regs->cntl1, 2);
	bcm2835aux_wr(&AuxSpiBase.regs->cntl0, 0x4c40);
	bcm2835aux_wr(&AuxSpiBase.regs->cntl1, 2);
	//bcm2835aux_wr(&AuxSpiBase.regs->cntl0, 0x1304c40);
	bcm2835aux_wr(&AuxSpiBase.regs->cntl0, 0xfff04c40);
	//printf("CNTL %x\n", bcm2835aux_rd(&AuxSpiBase.regs->cntl0));

}




static int bcm2835aux_spi_xfer(struct udevice *dev, unsigned int bitlen,
		const void *dout, void *din, unsigned long flags)
{
  //	struct bcm63xx_spi_priv *priv = dev_get_priv(dev->parent);
  //	const unsigned long *regs = priv->regs;
  u32 stat;
  //printf("\n");
	size_t data_bytes = bitlen / 8;
	if (flags & SPI_XFER_BEGIN)
	  {
	    bcm2835aux_cs_activate();
	    bcm2835aux_spi_reset_hw();
	  }
	/* configure spi */
	// Use prepare message conversion
	AuxSpiBase.rx_len = data_bytes;
	AuxSpiBase.tx_len = data_bytes;
	AuxSpiBase.rx_buf = din;
	AuxSpiBase.tx_buf = dout;
	
	/* loop until finished the transfer */
	while (AuxSpiBase.rx_len) {
		/* read status */
		stat = bcm2835aux_rd(&AuxSpiBase.regs->stat);

		/* fill in tx fifo with remaining data */
		if ((AuxSpiBase.tx_len) && (!(stat & BCM2835_AUX_SPI_STAT_TX_FULL))) {
			bcm2835aux_wr_fifo();
			continue;
		}

		/* read data from fifo for both cases */
		if (!(stat & BCM2835_AUX_SPI_STAT_RX_EMPTY)) {
			bcm2835aux_rd_fifo();
			continue;
		}
		
		if (!(stat & BCM2835_AUX_SPI_STAT_BUSY)) {
			bcm2835aux_rd_fifo();
			continue;
		}

	}
	if (flags & SPI_XFER_END)
	  {
	    bcm2835aux_cs_deactivate();
	  }

	/* and return without waiting for completion */
	return 0;
}


static int bcm2835aux_spi_probe(struct udevice *dev)
{
  int i;
  int ret;
  unsigned long *regs;
  
  //struct bcm2835aux_spi_priv *priv = dev_get_priv(dev);
  //	const unsigned long *regs =
  //		(const unsigned long *)dev_get_driver_data(dev);
	//get Chip select GPIOS
        regs =devfdt_get_addr(dev);
	ret = gpio_request_list_by_name(dev, "cs-gpios", cs_gpios,
					ARRAY_SIZE(cs_gpios), 0);
	if (ret < 0) {
		printf("Can't get %s gpios! Error: %d", dev->name, ret);
		return ret;
	}

	for(i = 0; i < ARRAY_SIZE(cs_gpios); i++) {

		dm_gpio_set_dir_flags(&cs_gpios[i],
				      GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
		dm_gpio_set_value(&cs_gpios[i], 1);
	}
	AuxSpiBase.regs = regs;
  // set base regs -0x2f = 5 (enable spi2)
     
	//writel(regs - 0x2f, 5);
	bcm2835aux_wr(regs-0x2f, 5);
	
	return 0;


}
static int bcm2835aux_spi_set_speed(struct udevice *bus, uint speed)
{
  //	struct atmel_spi_priv *priv = dev_get_priv(bus);
  //
  //	priv->freq = speed;

	return 0;
}

static int bcm2835aux_spi_set_mode(struct udevice *bus, uint mode)
{
  //struct atmel_spi_priv *priv = dev_get_priv(bus);
  //
  //	priv->mode = mode;

	return 0;
}
static int bcm2835aux_spi_claim_bus(struct udevice *bus)
{
  //struct atmel_spi_priv *priv = dev_get_priv(bus);
  //
  //	priv->mode = mode;

	return 0;
}
static int bcm2835aux_spi_release_bus(struct udevice *bus)
{
  //struct atmel_spi_priv *priv = dev_get_priv(bus);
  //
  //	priv->mode = mode;

	return 0;
}

static const struct udevice_id bcm2835aux_spi_ids[] = {
	{
		.compatible = "brcm,bcm2835-aux-spi",
	},
	{ /* sentinel */ }
};

static const struct dm_spi_ops bcm2835aux_spi_ops = {
  .claim_bus = bcm2835aux_spi_claim_bus,
  .release_bus = bcm2835aux_spi_release_bus,
	.xfer = bcm2835aux_spi_xfer,
  .set_speed = bcm2835aux_spi_set_speed,
  .set_mode = bcm2835aux_spi_set_mode,
};

U_BOOT_DRIVER(bcm2835aux_spi) = {
	.name = "bcm2835aux_spi",
	.id = UCLASS_SPI,
	.of_match = bcm2835aux_spi_ids,
	.ops = &bcm2835aux_spi_ops,
	//	.priv_auto_alloc_size = sizeof(struct bcm2835aux_spi_priv),
	.probe = bcm2835aux_spi_probe,
};


