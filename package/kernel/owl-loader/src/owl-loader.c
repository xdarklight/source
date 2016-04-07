/*
 * Initialize Owl Emulation Devices (PCIID: 168c:ff1c)
 *
 * Copyright (C) 2016 Christian Lamparter <chunkeey@googlemail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Some devices (like the Cisco Meraki Z1 Cloud Managed Teleworker Gateway)
 * need to be able to initialize the PCIe wifi device. Normally, this is done
 * during the early stages of booting linux, because the necessary init code
 * is read from the memory mapped SPI and passed to pci_enable_ath9k_fixup.
 * However,this isn't possible for devices which have the init code for the
 * Atheros chip stored on NAND. Hence, this module can be used to initialze
 * the chip when the user-space is ready to extract the init code.
 */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/completion.h>
#include <linux/firmware.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/ath9k_platform.h>

struct owl_ctx {
	struct completion eeprom_load;
};

#define AR5416_EEPROM_MAGIC 0xa55a

static void ath9k_pci_fixup(struct pci_dev *dev, const u16 *cal_data,
			    size_t cal_len)
{
	void __iomem *mem;
	const void *cal_end = (void *)cal_data + cal_len;
	const struct {
		__be16 reg;
		__be16 low_val;
		__be16 high_val;
	} __packed *data;
	u16 cmd;
	u32 bar0;
	bool swap_needed = false;

	if (*cal_data != AR5416_EEPROM_MAGIC) {
		if (*cal_data != swab16(AR5416_EEPROM_MAGIC)) {
			pr_err("pci %s: invalid calibration data\n",
			       pci_name(dev));
			return;
		}
		swap_needed = true;
	}

	pr_info("pci %s: fixup device configuration\n", pci_name(dev));

	mem = pcim_iomap(dev, 0, 0);
	if (!mem) {
		pr_err("pci %s: ioremap error\n", pci_name(dev));
		return;
	}

	pci_read_config_dword(dev, PCI_BASE_ADDRESS_0, &bar0);
	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0,
			       pci_resource_start(dev, 0));
	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	cmd |= PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY;
	pci_write_config_word(dev, PCI_COMMAND, cmd);

	/* set pointer to first reg address */
	for (data = (const void *) (cal_data + 3);
	     (const void *) data <= cal_end && data->reg != cpu_to_be16(~0);
	     data++) {
		u32 val;
		u16 reg;

		reg = data->reg;
		val = data->low_val;
		val |= data->high_val << 16;

		if (swap_needed) {
			reg = swab16(reg);
			val = swahb32(val);
		}

		__raw_writel(val, mem + reg);
		udelay(100);
	}

	pci_read_config_word(dev, PCI_COMMAND, &cmd);
	cmd &= ~(PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY);
	pci_write_config_word(dev, PCI_COMMAND, cmd);

	pci_write_config_dword(dev, PCI_BASE_ADDRESS_0, bar0);
	pcim_iounmap(dev, mem);

	pci_disable_device(dev);
}

static void owl_fw_cb(const struct firmware *fw, void *context)
{
	struct pci_dev *pdev = (struct pci_dev *) context;
	struct owl_ctx *ctx = (struct owl_ctx *) pci_get_drvdata(pdev);
	struct ath9k_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct pci_bus *bus;

	complete(&ctx->eeprom_load);

	if (!fw) {
		dev_err(&pdev->dev, "no '%s' eeprom file received.",
			pdata->eeprom_name);
		goto release;
	}

	/* also note that we are doing *u16 operations on the file */
	if (fw->size > sizeof(pdata->eeprom_data) || fw->size < 0x200 ||
	    (fw->size & 1) == 1) {
		dev_err(&pdev->dev, "eeprom file has an invalid size.");
		goto release;
	}

	memcpy(pdata->eeprom_data, fw->data, sizeof(pdata->eeprom_data));

	/* eeprom has been successfully loaded - pass the data to ath9k
	 * but remove the eeprom_name, so it doesn't try to load it too.
	 */
	pdata->eeprom_name = NULL;
	ath9k_pci_fixup(pdev, pdata->eeprom_data, fw->size);

	pci_lock_rescan_remove();
	bus = pdev->bus;
	pci_stop_and_remove_bus_device(pdev);
	/* the device should come back with the proper
	 * ProductId. But we have to initiate a rescan.
	 */
	pci_rescan_bus(bus);
	pci_unlock_rescan_remove();

release:
	release_firmware(fw);
}

static int owl_probe(struct pci_dev *pdev,
		    const struct pci_device_id *id)
{
	struct owl_ctx *ctx;
	struct ath9k_platform_data *pdata;
	int err = 0;

	if (pcim_enable_device(pdev))
		return -EIO;

	pcim_pin_device(pdev);

	/* we now have a valid dev->platform_data */
	pdata = dev_get_platdata(&pdev->dev);
	if (!pdata) {
		dev_err(&pdev->dev, "platform data missing.");
		return -ENODEV;
	}

	if (!pdata->eeprom_name) {
		dev_err(&pdev->dev, "no eeprom file defined.");
		return -ENODEV;
	}

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx) {
		dev_err(&pdev->dev, "failed to alloc device context.");
		return -ENOMEM;
	}
	init_completion(&ctx->eeprom_load);

	pci_set_drvdata(pdev, ctx);
	err = request_firmware_nowait(THIS_MODULE, true, pdata->eeprom_name,
				      &pdev->dev, GFP_KERNEL, pdev, owl_fw_cb);
	if (err) {
		dev_err(&pdev->dev, "failed to request caldata (%d).", err);
		kfree(ctx);
	}
	return err;
}

static void owl_remove(struct pci_dev *pdev)
{
	struct owl_ctx *ctx = pci_get_drvdata(pdev);

	if (ctx) {
		wait_for_completion(&ctx->eeprom_load);
		pci_set_drvdata(pdev, NULL);
		kfree(ctx);
	}
}

static const struct pci_device_id owl_pci_table[] = {
	/* PCIe Owl Emulation */
	{ PCI_VDEVICE(ATHEROS, 0xff1c) }, /* PCI-E */
	{ },
};
MODULE_DEVICE_TABLE(pci, owl_pci_table);

static struct pci_driver owl_driver = {
	.name		= "owl-loader",
	.id_table	= owl_pci_table,
	.probe		= owl_probe,
	.remove		= owl_remove,
};
module_pci_driver(owl_driver);
MODULE_AUTHOR("Christian Lamparter <chunkeey@googlemail.com>");
MODULE_DESCRIPTION("Initializes Atheros' Owl Emulation devices");
MODULE_LICENSE("GPL v2");
