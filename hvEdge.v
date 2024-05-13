module hvEdge(
	pixelSum134,
	pixelSum689,
	result
)

input  [5:0] pixelSum124;
input  [5:0] pixelSum689;

output [3:0] result;

assign wire [6:0] pixelSum124Res = pixelSum124<< 1;
assign wire [6:0] pixelSum689Res = pixelSum689<< 1;

assign wire [6:0] pixel9Res = pixelSum134Res-pixelSum689Res;

assign result = pixel9Res/8; // Divide By Max Value

endmodule