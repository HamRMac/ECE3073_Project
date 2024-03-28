module init_proc(
	CLOCK_50,
	SW,
	KEY,
	LEDR,
	VGA_R,
	VGA_B,
	VGA_G,
	VGA_HS,
	VGA_VS,
	DRAM_ADDR,
	DRAM_BA,
	DRAM_CAS_N,
	DRAM_CKE,
	DRAM_CLK,
	DRAM_CS_N,
	DRAM_DQ,
	DRAM_LDQM,
	DRAM_RAS_N,
	DRAM_UDQM,
	DRAM_WE_N
	
);

	// Define IO
	input CLOCK_50;
	input [9:0] SW;
	input [9:0] LEDR;
	input [1:0] KEY;
	
	// Define SDRAM IO
	output [12:0] DRAM_ADDR;
	output [1:0] DRAM_BA;
	output DRAM_CAS_N;
	output DRAM_CKE;
	output DRAM_CLK;
	output DRAM_CS_N;
	inout [15:0] DRAM_DQ;
	output DRAM_LDQM;
	output DRAM_RAS_N;
	output DRAM_UDQM;
	output DRAM_WE_N;
	
	// Define VGA IO
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
	pll_VGA vga_pll(
		.inclk0(CLOCK_50),
		.c0(VGA_CLOCK)
	);

	// Define VGA Controller
	vga_controller vga_ctl(
		.VGA_DATA(PB_output[3:0]),
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
		.q(PB_output[3:0])
	);
	
	// Instantiate the SDRAM PLL
	pll_SDRAM sdramPll(
		.inclk0(CLOCK_50),
		.c0(DRAM_CLK)
	);
	
	// Instantiate NiosII proc
	niosII_processor nios2_proc(
			.clk_clk(CLOCK_50),     //   clk.clk
			.reset_reset_n(KEY[0]),
			.key_export(KEY[1:0]),       //        key.export
			.ledr_export(LEDR[9:0]),      //       ledr.export
			.pb_adr_export(PB_WA[14:0]),    //     pb_adr.export
			.pb_data_export(PB_input[3:0]),   //    pb_data.export
			.sw_export(SW[9:0]),         //         sw.export
			.pbuff_wren_export(PB_WE),
			.sdram_wire_addr(DRAM_ADDR[12:0]),  // sdram.addr
			.sdram_wire_ba(DRAM_BA[1:0]),    //      .ba
			.sdram_wire_cas_n(DRAM_CAS_N), //      .cas_n
			.sdram_wire_cke(DRAM_CKE),   //      .cke
			.sdram_wire_cs_n(DRAM_CS_N),  //      .cs_n
			.sdram_wire_dq(DRAM_DQ[15:0]),    //      .dq
			.sdram_wire_dqm({DRAM_UDQM,DRAM_LDQM}),   //      .dqm
			.sdram_wire_ras_n(DRAM_RAS_N), //      .ras_n
			.sdram_wire_we_n(DRAM_WE_N)   //      .we_n
	);

endmodule