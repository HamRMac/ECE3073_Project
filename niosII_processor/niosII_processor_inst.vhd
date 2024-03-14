	component niosII_processor is
		port (
			clk_clk          : in    std_logic                     := 'X';             -- clk
			ledr_export      : out   std_logic_vector(9 downto 0);                     -- export
			sw_export        : in    std_logic_vector(9 downto 0)  := (others => 'X'); -- export
			key_export       : in    std_logic_vector(1 downto 0)  := (others => 'X'); -- export
			pb_data_export   : out   std_logic_vector(3 downto 0);                     -- export
			pb_adr_export    : out   std_logic_vector(14 downto 0);                    -- export
			sdram_wire_addr  : out   std_logic_vector(12 downto 0);                    -- addr
			sdram_wire_ba    : out   std_logic_vector(1 downto 0);                     -- ba
			sdram_wire_cas_n : out   std_logic;                                        -- cas_n
			sdram_wire_cke   : out   std_logic;                                        -- cke
			sdram_wire_cs_n  : out   std_logic;                                        -- cs_n
			sdram_wire_dq    : inout std_logic_vector(15 downto 0) := (others => 'X'); -- dq
			sdram_wire_dqm   : out   std_logic_vector(1 downto 0);                     -- dqm
			sdram_wire_ras_n : out   std_logic;                                        -- ras_n
			sdram_wire_we_n  : out   std_logic                                         -- we_n
		);
	end component niosII_processor;

	u0 : component niosII_processor
		port map (
			clk_clk          => CONNECTED_TO_clk_clk,          --        clk.clk
			ledr_export      => CONNECTED_TO_ledr_export,      --       ledr.export
			sw_export        => CONNECTED_TO_sw_export,        --         sw.export
			key_export       => CONNECTED_TO_key_export,       --        key.export
			pb_data_export   => CONNECTED_TO_pb_data_export,   --    pb_data.export
			pb_adr_export    => CONNECTED_TO_pb_adr_export,    --     pb_adr.export
			sdram_wire_addr  => CONNECTED_TO_sdram_wire_addr,  -- sdram_wire.addr
			sdram_wire_ba    => CONNECTED_TO_sdram_wire_ba,    --           .ba
			sdram_wire_cas_n => CONNECTED_TO_sdram_wire_cas_n, --           .cas_n
			sdram_wire_cke   => CONNECTED_TO_sdram_wire_cke,   --           .cke
			sdram_wire_cs_n  => CONNECTED_TO_sdram_wire_cs_n,  --           .cs_n
			sdram_wire_dq    => CONNECTED_TO_sdram_wire_dq,    --           .dq
			sdram_wire_dqm   => CONNECTED_TO_sdram_wire_dqm,   --           .dqm
			sdram_wire_ras_n => CONNECTED_TO_sdram_wire_ras_n, --           .ras_n
			sdram_wire_we_n  => CONNECTED_TO_sdram_wire_we_n   --           .we_n
		);

