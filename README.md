# STOP!!!

>[!CAUTION]
>If you're looking for Microchip MCP23018 linux kernel module - use upstream [pinctrl-mcp23s08](https://github.com/torvalds/linux/blob/master/drivers/pinctrl/pinctrl-mcp23s08.c) module instead. 
>
>This module is strictly for learning purposes!
>
Device Tree section for MCP23018 module, make sure to put it into appropriate i2c section:
```
mcp23018: gpio@20 {
	compatible = "microchip,mcp23018";
	reg = <0x20;
	gpio-controller;
	interrupt-controller;
};
```
