
module niosII_processor (
	clk_clk,
	ledr_export,
	sw_export,
	key_export,
	pb_data_export,
	pb_adr_export,
	sdram_wire_addr,
	sdram_wire_ba,
	sdram_wire_cas_n,
	sdram_wire_cke,
	sdram_wire_cs_n,
	sdram_wire_dq,
	sdram_wire_dqm,
	sdram_wire_ras_n,
	sdram_wire_we_n);	

	input		clk_clk;
	output	[9:0]	ledr_export;
	input	[9:0]	sw_export;
	input	[1:0]	key_export;
	output	[3:0]	pb_data_export;
	output	[14:0]	pb_adr_export;
	output	[12:0]	sdram_wire_addr;
	output	[1:0]	sdram_wire_ba;
	output		sdram_wire_cas_n;
	output		sdram_wire_cke;
	output		sdram_wire_cs_n;
	inout	[15:0]	sdram_wire_dq;
	output	[1:0]	sdram_wire_dqm;
	output		sdram_wire_ras_n;
	output		sdram_wire_we_n;
endmodule
