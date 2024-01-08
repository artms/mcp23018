/*
 *  Microchip MCP23018 I/O expander module
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>

#include <linux/i2c.h>
#include <linux/of_device.h>

#define MCP23018 0

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
	dev_info(&client->dev, "starting probing\n");
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
