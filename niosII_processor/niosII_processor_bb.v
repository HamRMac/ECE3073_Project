
module niosII_processor (
	clk_clk,
	key_export,
	ledr_export,
	pb_adr_export,
	pb_data_export,
	sdram_wire_addr,
	sdram_wire_ba,
	sdram_wire_cas_n,
	sdram_wire_cke,
	sdram_wire_cs_n,
	sdram_wire_dq,
	sdram_wire_dqm,
	sdram_wire_ras_n,
	sdram_wire_we_n,
	sw_export,
	reset_reset_n,
	pbuff_wren_export);	

	input		clk_clk;
	input	[1:0]	key_export;
	output	[9:0]	ledr_export;
	output	[14:0]	pb_adr_export;
	output	[3:0]	pb_data_export;
	output	[12:0]	sdram_wire_addr;
	output	[1:0]	sdram_wire_ba;
	output		sdram_wire_cas_n;
	output		sdram_wire_cke;
	output		sdram_wire_cs_n;
	inout	[15:0]	sdram_wire_dq;
	output	[1:0]	sdram_wire_dqm;
	output		sdram_wire_ras_n;
	output		sdram_wire_we_n;
	input	[9:0]	sw_export;
	input		reset_reset_n;
	output		pbuff_wren_export;
endmodule
