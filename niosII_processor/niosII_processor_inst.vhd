	component niosII_processor is
		port (
			clk_clk                : in    std_logic                     := 'X';             -- clk
			hexdisplays2to0_export : out   std_logic_vector(23 downto 0);                    -- export
			hexdisplays5to3_export : out   std_logic_vector(23 downto 0);                    -- export
			key_export             : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- export
			ledr_export            : out   std_logic_vector(9 downto 0);                     -- export
			pb_adr_export          : out   std_logic_vector(14 downto 0);                    -- export
			pb_data_export         : out   std_logic_vector(3 downto 0);                     -- export
			pbuff_wren_export      : out   std_logic;                                        -- export
			reset_reset_n          : in    std_logic                     := 'X';             -- reset_n
			sdram_wire_addr        : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_wire_ba          : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_wire_cas_n       : out   std_logic;                                        -- cas_n
			sdram_wire_cke         : out   std_logic;                                        -- cke
			sdram_wire_cs_n        : out   std_logic;                                        -- cs_n
			sdram_wire_dq          : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_wire_dqm         : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_wire_ras_n       : out   std_logic;                                        -- ras_n
			sdram_wire_we_n        : out   std_logic;                                        -- we_n
			sw_export              : in    std_logic_vector(9 downto 0)  := (others => 'X')  -- export
		);
	end component niosII_processor;

	u0 : component niosII_processor
		port map (
			clk_clk                => CONNECTED_TO_clk_clk,                --             clk.clk
			hexdisplays2to0_export => CONNECTED_TO_hexdisplays2to0_export, -- hexdisplays2to0.export
			hexdisplays5to3_export => CONNECTED_TO_hexdisplays5to3_export, -- hexdisplays5to3.export
			key_export             => CONNECTED_TO_key_export,             --             key.export
			ledr_export            => CONNECTED_TO_ledr_export,            --            ledr.export
			pb_adr_export          => CONNECTED_TO_pb_adr_export,          --          pb_adr.export
			pb_data_export         => CONNECTED_TO_pb_data_export,         --         pb_data.export
			pbuff_wren_export      => CONNECTED_TO_pbuff_wren_export,      --      pbuff_wren.export
			reset_reset_n          => CONNECTED_TO_reset_reset_n,          --           reset.reset_n
			sdram_wire_addr        => CONNECTED_TO_sdram_wire_addr,        --      sdram_wire.addr
			sdram_wire_ba          => CONNECTED_TO_sdram_wire_ba,          --                .ba
			sdram_wire_cas_n       => CONNECTED_TO_sdram_wire_cas_n,       --                .cas_n
			sdram_wire_cke         => CONNECTED_TO_sdram_wire_cke,         --                .cke
			sdram_wire_cs_n        => CONNECTED_TO_sdram_wire_cs_n,        --                .cs_n
			sdram_wire_dq          => CONNECTED_TO_sdram_wire_dq,          --                .dq
			sdram_wire_dqm         => CONNECTED_TO_sdram_wire_dqm,         --                .dqm
			sdram_wire_ras_n       => CONNECTED_TO_sdram_wire_ras_n,       --                .ras_n
			sdram_wire_we_n        => CONNECTED_TO_sdram_wire_we_n,        --                .we_n
			sw_export              => CONNECTED_TO_sw_export               --              sw.export
		);

