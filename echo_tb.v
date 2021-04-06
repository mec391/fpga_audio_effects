`timescale 1ns/1ns
module echo_tb();
parameter word_size = 24;
parameter full_cnt = 2**word_size - 1;
reg i_clk;
reg i_rst_n;

reg [23:0] i_data;
reg i_DV;

wire signed [23:0] o_echo_out;
wire o_echo_DV;

wire [15:0] debug_ptr;

echo UUT(
.i_data (i_data),
.i_DV (i_DV),
.o_echo_out (o_echo_out),
.o_echo_DV (o_echo_DV),
.i_clk (i_clk),
.i_rst_n (i_rst_n)
	);

reg [word_size-1:0] r_counter;
reg always_on;
initial begin
r_counter = 0;
i_clk = 1;
i_rst_n = 1;
i_data = r_counter;
i_DV = 0;
always_on = 1;

while (always_on == 1)
begin
#2 i_DV = 1;
#2 i_DV = 0;

#6 i_data = r_counter;
r_counter = r_counter + 1;
end

end


always begin 
#1 i_clk = !i_clk;
end

endmodule



module echo(
input i_clk,
input i_rst_n,

input [23:0] i_data,
input i_DV,

output reg signed [23:0] o_echo_out,
output reg o_echo_DV

);
reg [15:0] r_data_ptr = 16'd0;
reg [15:0] r_echo_ptr = 16'd1;
reg r_we;
reg [23:0] r_buf_data;
wire signed [23:0] w_buf_data;
reg [15:0] r_addr;
reg [3:0] r_SM0;
reg signed [23:0] r_echo_L_hold;

parameter IDLE = 0;
parameter WRITE_READ = 1;
parameter NEXT = 3;
parameter SEND_OUT = 4;
//delay of .5 seconds: Fs = 48000, delay = .5, buffer length = 48000 * .5 = 24000
//delay gain of .5
//process: new sample comes in, write it to data_ptr, take sample from echo ptr, multiply by gain, send out and DV, update both ptrs
always@(posedge i_clk)
begin
o_echo_DV <= 0;
case(r_SM0)
IDLE: begin
if(i_DV) begin 
	r_SM0 <= WRITE_READ; 
	r_we <= 1; 
	r_addr <= r_data_ptr;
	r_buf_data <= i_data;
	end
else begin 
	r_SM0 <= r_SM0;
	r_we <= 0; 
	r_addr <= r_addr; 
	r_buf_data <= r_buf_data;
	end
end
WRITE_READ:begin
r_we <= 0; 
r_addr <= r_echo_ptr; 
r_SM0 <= NEXT;
end
NEXT:begin
r_echo_L_hold <= w_buf_data;
r_we <= 0; 
if(r_echo_ptr == 16'd23999) r_echo_ptr <= 16'd0;
else r_echo_ptr <= r_echo_ptr + 1; 
if(r_data_ptr == 16'd23999) r_data_ptr <= 16'd0;
else r_data_ptr <= r_data_ptr + 1; 
r_SM0 <= SEND_OUT;
end
SEND_OUT:begin
o_echo_out <= w_buf_data;// / 2; //gain of .5
o_echo_DV <= 1; 
r_SM0 <= IDLE;
end
default: begin
r_SM0 <= IDLE;
r_we <= 0; 
r_addr <= r_addr; 
r_buf_data <= r_buf_data;
end
endcase

end

//instantiate the single port ram:
single_port_ram spr0(
.data (r_buf_data),
.addr (r_addr),
.we (r_we),
.clk (i_clk),
.q (w_buf_data)
	);
endmodule


module single_port_ram
(
	input [23:0] data,
	input [15:0] addr,
	input we, clk,
	output [23:0] q
);

	// Declare the RAM variable
	reg [23:0] ram[0:23999];  // need 1024 samples

	// Variable to hold the registered read address
	reg [15:0] addr_reg;
	
	always @ (posedge clk)
	begin
	// Write
		if (we)
			ram[addr] <= data;
		
		addr_reg <= addr;
		
	end
		
	// Continuous assignment implies read returns NEW data.
	// This is the natural behavior of the TriMatrix memory
	// blocks in Single Port mode.  
	assign q = ram[addr_reg];
	
endmodule