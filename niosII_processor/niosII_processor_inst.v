	niosII_processor u0 (
		.clk_clk          (<connected-to-clk_clk>),          //        clk.clk
		.ledr_export      (<connected-to-ledr_export>),      //       ledr.export
		.sw_export        (<connected-to-sw_export>),        //         sw.export
		.key_export       (<connected-to-key_export>),       //        key.export
		.pb_data_export   (<connected-to-pb_data_export>),   //    pb_data.export
		.pb_adr_export    (<connected-to-pb_adr_export>),    //     pb_adr.export
		.sdram_wire_addr  (<connected-to-sdram_wire_addr>),  // sdram_wire.addr
		.sdram_wire_ba    (<connected-to-sdram_wire_ba>),    //           .ba
		.sdram_wire_cas_n (<connected-to-sdram_wire_cas_n>), //           .cas_n
		.sdram_wire_cke   (<connected-to-sdram_wire_cke>),   //           .cke
		.sdram_wire_cs_n  (<connected-to-sdram_wire_cs_n>),  //           .cs_n
		.sdram_wire_dq    (<connected-to-sdram_wire_dq>),    //           .dq
		.sdram_wire_dqm   (<connected-to-sdram_wire_dqm>),   //           .dqm
		.sdram_wire_ras_n (<connected-to-sdram_wire_ras_n>), //           .ras_n
		.sdram_wire_we_n  (<connected-to-sdram_wire_we_n>)   //           .we_n
	);
