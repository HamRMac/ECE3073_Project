module vga_address_map(
	input [18:0] vga_controller_address,
	output [14:0] pixel_buffer_address
);

	wire [9:0] x_pos = vga_controller_address % 640;
	wire [8:0] y_pos = vga_controller_address / 640;
	
	wire [7:0] x_buffer = x_pos >> 2;
	wire [6:0] y_buffer = y_pos >> 2;
	
	assign pixel_buffer_address = (160*y_buffer) + x_buffer;

endmodule
