	niosII_ms2HW u0 (
		.clk_clk                   (<connected-to-clk_clk>),                   //                clk.clk
		.div9_tohw_external_export (<connected-to-div9_tohw_external_export>), // div9_tohw_external.export
		.div9_tosw_external_export (<connected-to-div9_tosw_external_export>), // div9_tosw_external.export
		.hexdisplays2to0_export    (<connected-to-hexdisplays2to0_export>),    //    hexdisplays2to0.export
		.hexdisplays5to3_export    (<connected-to-hexdisplays5to3_export>),    //    hexdisplays5to3.export
		.key_export                (<connected-to-key_export>),                //                key.export
		.ledr_export               (<connected-to-ledr_export>),               //               ledr.export
		.pb_adr_export             (<connected-to-pb_adr_export>),             //             pb_adr.export
		.pb_data_export            (<connected-to-pb_data_export>),            //            pb_data.export
		.pbuff_wren_export         (<connected-to-pbuff_wren_export>),         //         pbuff_wren.export
		.reset_reset_n             (<connected-to-reset_reset_n>),             //              reset.reset_n
		.sdram_wire_addr           (<connected-to-sdram_wire_addr>),           //         sdram_wire.addr
		.sdram_wire_ba             (<connected-to-sdram_wire_ba>),             //                   .ba
		.sdram_wire_cas_n          (<connected-to-sdram_wire_cas_n>),          //                   .cas_n
		.sdram_wire_cke            (<connected-to-sdram_wire_cke>),            //                   .cke
		.sdram_wire_cs_n           (<connected-to-sdram_wire_cs_n>),           //                   .cs_n
		.sdram_wire_dq             (<connected-to-sdram_wire_dq>),             //                   .dq
		.sdram_wire_dqm            (<connected-to-sdram_wire_dqm>),            //                   .dqm
		.sdram_wire_ras_n          (<connected-to-sdram_wire_ras_n>),          //                   .ras_n
		.sdram_wire_we_n           (<connected-to-sdram_wire_we_n>),           //                   .we_n
		.sw_export                 (<connected-to-sw_export>)                  //                 sw.export
	);

