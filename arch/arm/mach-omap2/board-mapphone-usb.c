/*
 * kernel/arch/arm/mach-omap2/board-mapphone-usb.c
 *
 * Copyright (C) 2010-2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <asm/mach-types.h>

#include <plat/board-mapphone.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/gpio_mapping.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <linux/usb/android_composite.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include "cm-regbits-34xx.h"
#include "clock.h"

#define MAPPHONE_BP_READY2_AP_GPIO      59
#define MAPPHONE_IPC_USB_SUSP_GPIO	142
#define MAX_USB_SERIAL_NUM		17
#define DIE_ID_REG_BASE			(L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)

static int mapphone_usb_port_startup(struct platform_device *dev, int port)
{
	int r;

	if (port == 2) {
		r = gpio_request(MAPPHONE_IPC_USB_SUSP_GPIO, "ipc_usb_susp");
		if (r < 0) {
			printk(KERN_WARNING "Could not request GPIO %d"
			       " for IPC_USB_SUSP\n",
			       MAPPHONE_IPC_USB_SUSP_GPIO);
			return r;
		}
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);
	} else {
		return -EINVAL;
	}
	return 0;
}

static void mapphone_usb_port_shutdown(struct platform_device *dev, int port)
{
	if (port == 2)
		gpio_free(MAPPHONE_IPC_USB_SUSP_GPIO);
}


static void mapphone_usb_port_suspend(struct platform_device *dev,
				    int port, int suspend)
{
	if (port == 2)
		gpio_set_value(MAPPHONE_IPC_USB_SUSP_GPIO, suspend);
}


static int omap_usbhost_bus_check_ctrl_standby(void)
{
	u32 val;

	val = cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_IDLEST);
	if (val & OMAP3430ES2_ST_USBHOST_STDBY_MASK) {
		return 1;
	} else {
 		return 0;
	}
}

static struct ehci_hcd_omap_platform_data usb_platform_data = {
	.port_data = {
		{ .flags = 0x0, }, /* disabled */
		{ .flags = 0x0, }, /* disabled */
		{
			.flags = EHCI_HCD_OMAP_FLAG_ENABLED |
			EHCI_HCD_OMAP_FLAG_AUTOIDLE |
			EHCI_HCD_OMAP_FLAG_NOBITSTUFF,
			.mode = EHCI_HCD_OMAP_MODE_UTMI_PHY_4PIN,
			.startup = mapphone_usb_port_startup,
			.shutdown = mapphone_usb_port_shutdown,
			.suspend = mapphone_usb_port_suspend,
		},
	},
		.usbhost_standby_status = omap_usbhost_bus_check_ctrl_standby,
};

#endif

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)

static struct resource ohci_resources[] = {
	[0] = {
		.start	= OMAP34XX_OHCI_BASE,
		.end	= OMAP34XX_OHCI_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_UHH_CONFIG_BASE,
		.end	= OMAP34XX_UHH_CONFIG_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_USBTLL_BASE,
		.end	= OMAP34XX_USBTLL_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{         /* general IRQ */
		.start	= INT_34XX_OHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ohci_dmamask = ~(u32)0;

#ifdef CONFIG_USB_SERIAL_VIATELECOM_CBP
static struct omap_usb_config dummy_usb_config_via = {
       .port_data = {
		{
			.flags = EHCI_HCD_OMAP_FLAG_ENABLED |
			EHCI_HCD_OMAP_FLAG_AUTOIDLE |
			EHCI_HCD_OMAP_FLAG_NOBITSTUFF,
			.mode = EHCI_HCD_OMAP_MODE_UTMI_TLL_2PIN,
		},
		{ .flags = 0x0, }, /* disabled */
		{ .flags = 0x0, }, /* disabled */
	},
};
#endif

static struct omap_usb_config dummy_usb_config = {
	.port_data = {
		{ .flags = 0x0, }, /* disabled */
		{ .flags = 0x0, }, /* disabled */
		{
			.flags = EHCI_HCD_OMAP_FLAG_ENABLED |
			EHCI_HCD_OMAP_FLAG_AUTOIDLE |
			EHCI_HCD_OMAP_FLAG_NOBITSTUFF,
			.mode = EHCI_HCD_OMAP_MODE_UTMI_PHY_4PIN,
			.startup = mapphone_usb_port_startup,
			.shutdown = mapphone_usb_port_shutdown,
			.suspend = mapphone_usb_port_suspend,
		},
	},
	.usbhost_standby_status = omap_usbhost_bus_check_ctrl_standby,
	.usb_remote_wake_gpio = MAPPHONE_BP_READY2_AP_GPIO,
};

static struct platform_device ohci_device = {
	.name		= "ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data	= &dummy_usb_config,
	},
	.num_resources	= ARRAY_SIZE(ohci_resources),
	.resource	= ohci_resources,
};
#endif /* OHCI specific data */

#ifdef CONFIG_USB_QSC6085_CDMA_MODEM
extern void set_cdma_modem_interface(unsigned int number);

void mapphone_init_modem_interface(void)
{
	struct device_node *node;
	const void *prop;
	int rwkup_gpio = get_gpio_by_name("bp2ap_usb_rwkup");

	if (rwkup_gpio < 0)
		dummy_usb_config.usb_remote_wake_gpio =
			MAPPHONE_BP_READY2_AP_GPIO;
	else
		dummy_usb_config.usb_remote_wake_gpio = rwkup_gpio;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_MODEM_IFACE_NUM, NULL);
	if (prop) {
		pr_err("Setting the Modem Interface num to %d\n", *(u8 *)prop);
		set_cdma_modem_interface(*(u8 *)prop);
	} else
		set_cdma_modem_interface(0);

	of_node_put(node);
	return;
}
#endif

void __init mapphone_ehci_init(void)
{
	omap_cfg_reg(AF5_34XX_GPIO142);		/*  IPC_USB_SUSP      */
	omap_cfg_reg(AD1_3430_USB3FS_PHY_MM3_RXRCV);
	omap_cfg_reg(AD2_3430_USB3FS_PHY_MM3_TXDAT);
	omap_cfg_reg(AC1_3430_USB3FS_PHY_MM3_TXEN_N);
	omap_cfg_reg(AE1_3430_USB3FS_PHY_MM3_TXSE0);

#ifdef CONFIG_USB_SERIAL_VIATELECOM_CBP
	if (mapphone_bp_get_type() == MAPPHONE_BP_VIACBP71) {
		printk(KERN_INFO "VIA BP is chosen\n");
		ohci_device.dev.platform_data  = &dummy_usb_config_via;
	}
#endif

#ifdef CONFIG_USB_QSC6085_CDMA_MODEM
	mapphone_init_modem_interface();
	platform_device_register(&ohci_device);
#endif
}

