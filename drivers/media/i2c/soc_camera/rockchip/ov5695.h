struct ov5695_reg_list {
	char * group;
	u16 start;
	u16 end;
};

struct ov5695_reg_list ov5695_registers = {
	{
		.group = "PLL control",
		.start = 0x0100,
		.end   = 0x0100,
	},
	{
		.group = "PLL control",
		.start = 0x0103,
		.end   = 0x0103,
	},
	{
		.group = "PLL control",
		.start = 0x0300,
		.end   = 0x0313,
	},
}
