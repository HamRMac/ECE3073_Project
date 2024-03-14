module init_vga(
	CLOCK_50,
	SW,
	VGA_R,
	VGA_B,
	VGA_G,
	VGA_HS,
	VGA_VS
);

// Define IO
input CLOCK_50;
input [9:0] SW;

wire VGA_CLOCK;

output [3:0] VGA_R;
output [3:0] VGA_G;
output [3:0] VGA_B;
output VGA_HS;
output VGA_VS;
wire [18:0] VGA_ADDR;
wire [3:0] PB_input;
wire [14:0] PB_WA;
wire PB_WE;
wire [3:0] PB_output;
// Define VGA Clock Using PLL
pll_block vga_pll(
	.inclk0(CLOCK_50),
	.c0(VGA_CLOCK)
);

// Define VGA Controller
vga_controller vga_ctl(
	.VGA_DATA(SW[3:0]),
	.VGA_CLK(VGA_CLOCK),
	
	.VGA_ADDR(VGA_ADDR[18:0]),
	.VGA_R(VGA_R[3:0]),
	.VGA_G(VGA_G[3:0]),
	.VGA_B(VGA_B[3:0]),
	.VGA_HS(VGA_HS),
	.VGA_VS(VGA_VS)
);

// Define Video Ram
ram2port pixel_buffer(
	.clock(VGA_CLOCK),
	.data(PB_input[3:0]),
	.rdaddress(VGA_ADDR[14:0]),
	.wraddress(PB_WA[14:0]),
	.wren(PB_WE),
	.q(PB_output[3:0]));

endmodule
