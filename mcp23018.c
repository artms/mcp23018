/*
 *  Microchip MCP23018 I/O expander module
 */

#include <linux/module.h>
#include <linux/version.h>
#include <linux/kmod.h>

#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/irq.h>

#define MCP23018 0
#define MCP_IODIR 0x00
#define MCP_IOPOL 0x02
#define MCP_GPINTEN 0x04
#define MCP_DEFVAL 0x06
#define MCP_INTCON 0x08
#define MCP_IOCON 0x0A
#define MCP_GPPU 0x0C
#define MCP_INTF 0x0E
#define MCP_INTCAP 0x10
#define MCP_GPIO 0x12
#define MCP_OLAT 0x14

// interrupt clearing condition (0 - GPIO read resets interrupt, 1 - INTCAP read resets interrupt)
#define MCP_INTCC BIT(0)
// interrupt pin mirroring (0 - not mirrored, 1 - pins are mirrored)
#define MCP_MIRROR BIT(6)

const struct regmap_range mcp23018_volatile_range[] = {
	{ .range_min = MCP_INTF, .range_max = MCP_GPIO },
};

const struct regmap_access_table mcp23018_volatile_cfg = {
	.yes_ranges = mcp23018_volatile_range,
	.n_yes_ranges = ARRAY_SIZE(mcp23018_volatile_range),
};

const struct regmap_range mcp23018_precious_range[] = {
	{ .range_min = MCP_INTCAP, .range_max = MCP_INTCAP },
};


const struct regmap_access_table mcp23018_precious_cfg = {
	.yes_ranges = mcp23018_precious_range,
	.n_yes_ranges = ARRAY_SIZE(mcp23018_precious_range),
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
	.precious_table    = &mcp23018_precious_cfg,
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

struct mcp23018 {
	struct regmap *regmap;
	struct gpio_chip gpiochip;
	struct irq_chip irqchip;
	struct device *dev;
	struct mutex lock;
	// mcp23018 only has interrupts which are either level triggered or triggered on-change (aka edge - both) so we need to track rise/false ourselves
	unsigned int irq_rise, irq_fall;
	unsigned int cached_gpio;
};

#define GPIO_NUM 16

static void mcp23s08_irq_bus_lock(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);

	mutex_lock(&mcp23018->lock);
	// making sure regmap is not accessing I2C bus during atomic stage
	regcache_cache_only(mcp23018->regmap, true);
}

static void mcp23s08_irq_bus_sync_unlock(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);

	regcache_cache_only(mcp23018->regmap, false);
	regcache_sync(mcp23018->regmap);
	mutex_unlock(&mcp23018->lock);
}

static void mcp23018_irq_mask(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);
	unsigned int hwirq = irqd_to_hwirq(data);

	regmap_update_bits(mcp23018->regmap, MCP_GPINTEN, BIT(hwirq), 0);
	gpiochip_disable_irq(chip, hwirq);
}

static void mcp23018_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);
	unsigned int hwirq = irqd_to_hwirq(data);

	gpiochip_enable_irq(chip, hwirq);
	regmap_update_bits(mcp23018->regmap, MCP_GPINTEN, BIT(hwirq), BIT(hwirq));
}

static int mcp23018_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	struct gpio_chip *chip = irq_data_get_irq_chip_data(data);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);
	unsigned int pos = irqd_to_hwirq(data);
	int ret;

	if ((flow_type & IRQ_TYPE_EDGE_BOTH) == IRQ_TYPE_EDGE_BOTH) {
		mcp23018->irq_rise |= BIT(pos);
		mcp23018->irq_fall |= BIT(pos);
		return regmap_update_bits(mcp23018->regmap, MCP_INTCON, BIT(pos), 0);
	} else if (flow_type & IRQ_TYPE_EDGE_RISING) {
		mcp23018->irq_rise |= BIT(pos);
		mcp23018->irq_fall &= ~BIT(pos);
		return regmap_update_bits(mcp23018->regmap, MCP_INTCON, BIT(pos), 0);
	} else if (flow_type & IRQ_TYPE_EDGE_FALLING) {
		mcp23018->irq_rise &= ~BIT(pos);
		mcp23018->irq_fall |= BIT(pos);
		return regmap_update_bits(mcp23018->regmap, MCP_INTCON, BIT(pos), 0);
	} else if (flow_type & IRQ_TYPE_LEVEL_HIGH) {
		mcp23018->irq_rise &= ~BIT(pos);
		mcp23018->irq_fall &= ~BIT(pos);
		ret = regmap_update_bits(mcp23018->regmap, MCP_DEFVAL, BIT(pos), 0);
		if (unlikely(ret != 0)) {
			return ret;
		}
		return regmap_update_bits(mcp23018->regmap, MCP_INTCON, BIT(pos), 1);
	} else if (flow_type & IRQ_TYPE_LEVEL_LOW) {
		mcp23018->irq_rise &= ~BIT(pos);
		mcp23018->irq_fall &= ~BIT(pos);
		ret = regmap_update_bits(mcp23018->regmap, MCP_DEFVAL, BIT(pos), 1);
		if (unlikely(ret != 0)) {
			return ret;
		}
		return regmap_update_bits(mcp23018->regmap, MCP_INTCON, BIT(pos), 1);
	}
	return -EINVAL;
}

static irqreturn_t mcp23018_irq(int irq, void *data) {
	struct mcp23018 *mcp23018 = data;
	int child_irq;
	unsigned int intf, intcon, intcap, pos, gpinten, gpio, gpio_prev, defval, gpio_state, intcap_changed, gpio_changed, defval_changed;
	unsigned long int enabled_interrupts;
	mutex_lock(&mcp23018->lock);
	if (regmap_read(mcp23018->regmap, MCP_INTF, &intf) != 0) {
		goto unlock;
	}

	// chip sees no interrupt present
	if (intf == 0) {
		goto unlock;
	}

	if (regmap_read(mcp23018->regmap, MCP_INTCON, &intcon) != 0) {
		goto unlock;
	}

	if (regmap_read(mcp23018->regmap, MCP_GPINTEN, &gpinten) != 0) {
		goto unlock;
	}
	enabled_interrupts = gpinten;

	if (regmap_read(mcp23018->regmap, MCP_DEFVAL, &defval) != 0) {
		goto unlock;
	}

	if (regmap_read(mcp23018->regmap, MCP_GPIO, &gpio) != 0) {
		goto unlock;
	}
	gpio_prev = mcp23018->cached_gpio;
	mcp23018->cached_gpio = gpio;

	// ack interrupt
	if (regmap_read(mcp23018->regmap, MCP_INTCAP, &intcap) != 0) {
		goto unlock;
	}
	mutex_unlock(&mcp23018->lock);
	for_each_set_bit(pos, &enabled_interrupts, GPIO_NUM) {
		gpio_state = gpio & BIT(pos);
		if (intf & BIT(pos)) {
			intcap_changed = (intcap & BIT(pos)) != (gpio_prev & BIT(pos));
		}
		gpio_changed = (gpio & BIT(pos)) != (gpio_prev & BIT(pos));
		defval_changed = (intcon & BIT(pos)) && (gpio_state != (defval & BIT(pos)));

		if (defval_changed || ((intcap_changed || gpio_changed) &&
		    ((mcp23018->irq_rise & BIT(pos) && gpio_state) || (mcp23018->irq_fall & BIT(pos) && !gpio_state)))) {
			child_irq = irq_find_mapping(mcp23018->gpiochip.irq.domain, pos+1);
			handle_nested_irq(child_irq);
		}
	}
	return IRQ_HANDLED;
unlock:
	mutex_unlock(&mcp23018->lock);
	return IRQ_HANDLED;
}

static const struct irq_chip mcp23018_irq_chip = {
	.name = "gpio-mcp23018",
	.irq_mask = mcp23018_irq_mask,
	.irq_unmask = mcp23018_irq_unmask,
	.irq_set_type = mcp23018_irq_set_type,
};

static int mcp23018_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
{
	int ret;
	unsigned int mask = BIT(offset);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);

	mutex_lock(&mcp23018->lock);
	ret = regmap_update_bits(mcp23018->regmap, MCP_OLAT, mask, value ? mask : 0);
	if (likely(ret == 0)) {
		ret = regmap_update_bits(mcp23018->regmap, MCP_IODIR, mask, 0);
	}
	mutex_unlock(&mcp23018->lock);
	return ret;
}

static int mcp23018_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	int ret;
	unsigned int mask = BIT(offset);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);

	mutex_lock(&mcp23018->lock);
	ret = regmap_update_bits(mcp23018->regmap, MCP_IODIR, mask, mask);
	mutex_unlock(&mcp23018->lock);
	return ret;
}

static int mcp23018_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int val;
	int ret;
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);

	mutex_lock(&mcp23018->lock);
	ret = regmap_read(mcp23018->regmap, MCP_IODIR, &val);
	mutex_unlock(&mcp23018->lock);
	if (ret < 0) {
		return ret;
	}
	return !!(val & BIT(offset));
}

static int mcp23018_set_config(struct gpio_chip *chip, unsigned int offset, unsigned long config)
{
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);
	enum pin_config_param param = pinconf_to_config_param(config);
	int ret;
	switch (param) {
		case PIN_CONFIG_BIAS_PULL_UP:
		case PIN_CONFIG_BIAS_DISABLE:
			mutex_lock(&mcp23018->lock);
			ret = regmap_update_bits(mcp23018->regmap, MCP_GPPU, BIT(offset), param == PIN_CONFIG_BIAS_PULL_UP ? BIT(offset) : 0);
			mutex_unlock(&mcp23018->lock);
			break;
		default:
			return -ENOTSUPP;
	}
	return ret;
}

static int mcp23018_get(struct gpio_chip *chip, unsigned int offset)
{
	unsigned int val;
	int ret;
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);
	mutex_lock(&mcp23018->lock);
	ret = regmap_read(mcp23018->regmap, MCP_GPIO, &val);
	if (likely(ret >= 0)) {
		mcp23018->cached_gpio = val;
		ret = !!(val & BIT(offset));
	}
	mutex_unlock(&mcp23018->lock);
	return !!(val & BIT(offset));
}

static void mcp23018_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	unsigned int mask = BIT(offset);
	struct mcp23018 *mcp23018 = gpiochip_get_data(chip);
	mutex_lock(&mcp23018->lock);
	regmap_update_bits(mcp23018->regmap, MCP_OLAT, mask, value ? mask : 0);
	mutex_unlock(&mcp23018->lock);
}

static int mcp23018_i2c_probe(struct i2c_client *client)
{
	int err;
	struct regmap *regmap;
	struct device *dev = &client->dev;
	struct mcp23018 *mcp23018;
	struct gpio_desc *reset_gpio;
	struct gpio_irq_chip *girq;

	dev_info(dev, "starting probing\n");

	// resetting chip
	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio)) {
		return PTR_ERR(reset_gpio);
	}

	mcp23018 = devm_kzalloc(dev, sizeof(*mcp23018), GFP_KERNEL);
	if (!mcp23018) {
		return -ENOMEM;
	}

	regmap = devm_regmap_init_i2c(client, &mcp23018_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "unable to setup regmap\n");
		return PTR_ERR(regmap);
	}

	mutex_init(&mcp23018->lock);
	mcp23018->regmap = regmap;
	mcp23018->dev = dev;
	mcp23018->gpiochip.label = client->name;
	mcp23018->gpiochip.base = -1;
	mcp23018->gpiochip.owner = THIS_MODULE;
	mcp23018->gpiochip.ngpio = GPIO_NUM;
	mcp23018->gpiochip.can_sleep = true;
	mcp23018->gpiochip.parent = dev;

	mcp23018->gpiochip.get = mcp23018_get;
	mcp23018->gpiochip.set = mcp23018_set;
	mcp23018->gpiochip.get_direction = mcp23018_get_direction;
	mcp23018->gpiochip.direction_input = mcp23018_direction_input;
	mcp23018->gpiochip.direction_output = mcp23018_direction_output;
	mcp23018->gpiochip.set_config = mcp23018_set_config;

	mcp23018->irqchip.name = "gpio-mcp23018";
	mcp23018->irqchip.irq_mask = mcp23018_irq_mask;
	mcp23018->irqchip.irq_unmask = mcp23018_irq_unmask;
	mcp23018->irqchip.irq_set_type = mcp23018_irq_set_type;
	mcp23018->irqchip.irq_bus_lock = mcp23s08_irq_bus_lock;
	mcp23018->irqchip.irq_bus_sync_unlock = mcp23s08_irq_bus_sync_unlock;

	girq = &mcp23018->gpiochip.irq;
	girq->chip = &mcp23018->irqchip;

	girq->parent_handler = NULL;
	girq->num_parents = 0;
	girq->parents = NULL;
	girq->default_type = IRQ_TYPE_NONE;
	girq->handler = handle_level_irq;
	girq->threaded = true;

	// IOCON register is "mirrored" hence we have to write twice, otherwise second half of 16 bits overwrites first one
	err = regmap_update_bits(regmap, MCP_IOCON, MCP_MIRROR | MCP_MIRROR << 8 | MCP_INTCC | MCP_INTCC << 8, MCP_MIRROR | MCP_MIRROR << 8 | MCP_INTCC | MCP_INTCC << 8);
	if (err != 0) {
		return err;
	}

	// check IRQF_TRIGGER_LOW???
	err = devm_request_threaded_irq(dev, client->irq, NULL, mcp23018_irq, IRQF_TRIGGER_LOW | IRQF_ONESHOT, dev_name(dev), mcp23018);
	if (err != 0) {
		return err;
	}

	return devm_gpiochip_add_data(dev, &mcp23018->gpiochip, mcp23018);
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
