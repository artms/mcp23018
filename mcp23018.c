/*
 *  Microchip MCP23018 I/O expander module
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>

#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#define MCP23018 0
#define MCP_IODIR 0x00
#define MCP_IOPOL 0x02
#define MCP_GPINTEN 0x04
#define MCP_DEFVAL 0x06
#define MCP_INTCON 0x08
#define MCP_IOCON 0x0A
#define MCP_GPPU 0x0C
#define MCP_INTF 0x0E
#define MCP_INTCAP 0x19
#define MCP_GPIO 0x12
#define MCP_OLAT 0x14

const struct regmap_range mcp23018_volatile_range[] = {
	{ .range_min = MCP_GPIO, .range_max = MCP_GPIO,}
};

const struct regmap_access_table mcp23018_volatile_cfg = {
	.yes_ranges = mcp23018_volatile_range,
	.n_yes_ranges = ARRAY_SIZE(mcp23018_volatile_range),
};

const struct reg_default mcp23028_reg_defaults[] = {
	{ .reg = MCP_IODIR,   .def = 0xffff },
	{ .reg = MCP_IOPOL,   .def = 0x0000 },
	{ .reg = MCP_GPINTEN, .def = 0x0000 },
	{ .reg = MCP_DEFVAL,  .def = 0x0000 },
	{ .reg = MCP_INTCON,  .def = 0x0000 },
	{ .reg = MCP_IOCON,   .def = 0x0000 },
	{ .reg = MCP_GPPU,    .def = 0x0000 },
	{ .reg = MCP_INTF,    .def = 0x0000 },
	{ .reg = MCP_INTCAP,  .def = 0x0000 },
	{ .reg = MCP_GPIO,    .def = 0x0000 },
	{ .reg = MCP_OLAT,    .def = 0x0000 },
};

const struct regmap_config mcp23018_regmap_config = {
	.reg_bits          = 8,
	.val_bits          = 16,
	.reg_stride        = 2,
	.disable_locking   = true,
	.can_sleep         = true,
	.max_register      = MCP_OLAT,
	.cache_type        = REGCACHE_FLAT,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,

	.volatile_table    = &mcp23018_volatile_cfg,
	.reg_defaults      = mcp23028_reg_defaults,
	.num_reg_defaults  = ARRAY_SIZE(mcp23028_reg_defaults),
};

static const struct i2c_device_id mcp23018_idtable[] = {
	{ "mcp23018", MCP23018 },
	{ /* END OF LIST */ },
};

// Device Tree matches
static const struct of_device_id mcp23018_of_match[] = {
	{ .compatible = "microchip,mcp23018" },
	{ /* END OF LIST */ },
};

static int mcp23018_i2c_probe(struct i2c_client *client)
{
	unsigned int data;
	int ret;
	struct regmap *regmap;
	struct device *dev = &client->dev;
	dev_info(&client->dev, "starting probing\n");

	regmap = devm_regmap_init_i2c(client, &mcp23018_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "unable to setup regmap\n");
		return PTR_ERR(regmap);
	}

	ret = regmap_read(regmap, MCP_IODIR, &data);
	if (ret != 0) {
		return ret;
	}

	return 0;
}

static void mcp23018_i2c_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "removing\n");
}

static struct i2c_driver mcp23018_i2c_driver = {
	.driver       = {
		.name = "mcp23018",
		.of_match_table = of_match_ptr(mcp23018_of_match),
	},
#if LINUX_VERSION_CODE < KERNEL_VERSION(6,6,0)
	.probe_new    = mcp23018_i2c_probe,
#else
	.probe        = mcp23018_i2c_probe,
#endif
	.remove       = mcp23018_i2c_remove,
	.id_table     = mcp23018_idtable,
};

static int __init mcp23018_i2c_init(void)
{
	pr_info("module mcp23018 init\n");
	return i2c_add_driver(&mcp23018_i2c_driver);
}
module_init(mcp23018_i2c_init);

static void __exit mcp23018_i2c_exit(void)
{
	pr_info("module mcp23018 exit\n");
	i2c_del_driver(&mcp23018_i2c_driver);
}
module_exit(mcp23018_i2c_exit);

MODULE_AUTHOR("Arturas Moskvinas <arturas.moskvinas@gmail.com>");
MODULE_DESCRIPTION("Microchip MCP23018 I/O expander module");
MODULE_LICENSE("GPL v2");
