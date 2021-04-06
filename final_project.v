module final_project(
input i_clk,
input i_rst_n,

output o_AUD_XCK,
inout io_AUD_BCLK,
output o_AUD_DACDAT,
inout io_ADU_DACLRCK,
input i_AUD_ADCDAT,
inout io_ADCLRCK,
output o_I2C_SCLK,
inout io_I2C_SDAT,

input i_echo_en, //route to a switch to turn on or off enable for echo
output reg [3:0] debug
);

always@(posedge i_clk)
begin
debug <= r_SM_L;
end
//48khz sampling rate w/ 50 mhz clk --> ~1k clock cycles to work with
//read from source, write to sink

//source wires (read audio in)
reg r_l_src_rdy;
wire w_l_src_DV;
wire [23:0] w_l_src_data;

reg r_r_src_rdy;
wire w_r_src_DV;
wire [23:0] w_r_src_data;

//sink wires (write audio out)
reg signed [23:0] r_l_snk_data;
reg r_l_snk_DV;
wire r_l_snk_rdy;

reg signed [23:0] r_r_snk_data;
reg r_r_snk_DV;
wire r_r_snk_rdy;

//for now route input data to output data to verify functionality
//loopback test
/*always@(posedge i_clk)
begin
r_l_src_rdy <= 1;
r_r_src_rdy <= 1;
if(w_r_src_DV) begin 
	r_r_snk_data <= w_r_src_data;
	r_r_snk_DV <= 1; 
	end
else begin 
	r_r_snk_data <= r_r_snk_data; 
	r_r_snk_DV <= 0; end
if(w_l_src_DV) begin 
	r_l_snk_data <= w_l_src_data; 
	r_l_snk_DV <= 1; 
end
else begin 
	r_l_snk_data <= r_l_snk_data;
	r_l_snk_DV <= 0; end
end
*/
parameter IDLE = 0;
parameter COMPUTE = 1;

reg [3:0] r_SM_L;
reg [3:0] r_SM_R;
reg [23:0] r_L_in;
reg [23:0] r_R_in;
reg r_R_DV;
reg r_L_DV;
reg signed [23:0] r_in_data_hold_L;
reg signed  [23:0] r_in_data_hold_R;
wire signed [23:0] w_L_echo_out;
wire signed [23:0] w_R_echo_out;
wire w_R_DV_echo;
wire w_L_DV_echo;

//SM for passing data to submodules

//left side
always@(posedge i_clk)
begin
if(~i_rst_n) begin /*reset regs*/ end
else begin
case (r_SM_L)
IDLE: begin
r_l_snk_DV <= 0;
if(w_l_src_DV) begin
r_in_data_hold_L <= w_l_src_data;
r_L_in <= w_l_src_data;
r_L_DV <= 1;
r_l_src_rdy <= 0;
r_SM_L <= COMPUTE; end
else begin
r_in_data_hold_L <= r_in_data_hold_L;
r_L_in <= r_L_in;
r_L_DV <= 0;
r_l_src_rdy <= 1;
r_SM_L <= r_SM_L; end end
COMPUTE: begin
r_L_DV <= 0;
if(w_L_DV_echo) begin // this will need modified when other componenets are added in
	if(i_echo_en) r_l_snk_data <= w_L_echo_out + r_in_data_hold_L; 
	else r_l_snk_data <= r_in_data_hold_L;
r_l_snk_DV <= 1;
r_SM_L <= IDLE; end
else begin
r_l_snk_data <= r_l_snk_data;
r_l_snk_DV <= 0;
r_SM_L <= r_SM_L; end end
default: begin /*add default */ end
endcase
end
end

//right side
always@(posedge i_clk)
begin
if(~i_rst_n) begin /*reset regs*/ end
else begin
case (r_SM_R)
IDLE: begin
r_r_snk_DV <= 0;
if(w_r_src_DV) begin
r_in_data_hold_R <= w_r_src_data;
r_R_in <= w_r_src_data;
r_R_DV <= 1;
r_r_src_rdy <= 0;
r_SM_R <= COMPUTE; end
else begin
r_in_data_hold_R <= r_in_data_hold_R;
r_R_in <= r_R_in;
r_R_DV <= 0;
r_r_src_rdy <= 1;
r_SM_R <= r_SM_R; end end
COMPUTE: begin
r_R_DV <= 0;
if(w_R_DV_echo) begin // this will need modified when other componenets are added in
	if(i_echo_en) r_r_snk_data <= w_R_echo_out + r_in_data_hold_R; 
	else r_r_snk_data <= r_in_data_hold_R;
r_r_snk_DV <= 1;
r_SM_R <= IDLE; end
else begin
r_r_snk_data <= r_r_snk_data;
r_r_snk_DV <= 0;
r_SM_R <= r_SM_R; end end
default: begin /*add default */ end
endcase
end
end
soc_system u0 (
		.clk_clk                                          (i_clk),                                          //                                         clk.clk
		.reset_reset_n                                    (i_rst_n),                                    //                                       reset.reset_n
		.audio_0_avalon_right_channel_sink_data           (r_r_snk_data),           //           audio_0_avalon_right_channel_sink.data
		.audio_0_avalon_right_channel_sink_valid          (r_r_snk_DV),          //                                            .valid
		.audio_0_avalon_right_channel_sink_ready          (r_r_snk_rdy),          //                                            .ready
		.audio_0_avalon_left_channel_sink_data            (r_l_snk_data),            //            audio_0_avalon_left_channel_sink.data
		.audio_0_avalon_left_channel_sink_valid           (r_l_snk_DV),           //                                            .valid
		.audio_0_avalon_left_channel_sink_ready           (r_l_snk_rdy),           //                                            .ready
		.audio_0_avalon_right_channel_source_ready        (r_r_src_rdy),        //         audio_0_avalon_right_channel_source.ready
		.audio_0_avalon_right_channel_source_data         (w_r_src_data),         //                                            .data
		.audio_0_avalon_right_channel_source_valid        (w_r_src_DV),        //                                            .valid
		.audio_0_avalon_left_channel_source_ready         (r_l_src_rdy),         //          audio_0_avalon_left_channel_source.ready
		.audio_0_avalon_left_channel_source_data          (w_l_src_data),          //                                            .data
		.audio_0_avalon_left_channel_source_valid         (w_l_src_DV),         //                                            .valid
		
		.audio_pll_0_audio_clk_clk                        (o_AUD_XCK),                        //                       audio_pll_0_audio_clk.clk
		.audio_and_video_config_0_external_interface_SDAT (io_I2C_SDAT), // audio_and_video_config_0_external_interface.SDAT
		.audio_and_video_config_0_external_interface_SCLK (o_I2C_SCLK), //                                            .SCLK
		.audio_0_external_interface_ADCDAT                (i_AUD_ADCDAT),                //                  audio_0_external_interface.ADCDAT
		.audio_0_external_interface_ADCLRCK               (io_ADCLRCK),               //                                            .ADCLRCK
		.audio_0_external_interface_BCLK                  (io_AUD_BCLK),                  //                                            .BCLK
		.audio_0_external_interface_DACDAT                (o_AUD_DACDAT),                //                                            .DACDAT
		.audio_0_external_interface_DACLRCK               (io_ADU_DACLRCK)                //                                            .DACLRCK
	);

//instantiate the left echo: 
echo ech0(
.i_clk (i_clk),
.i_rst_n (i_rst_n),
.i_data (r_L_in),
.i_DV (r_L_DV),
.o_echo_out (w_L_echo_out),
.o_echo_DV (w_L_DV_echo)
	);

//instantiate the right echo:
echo ech1(
.i_clk (i_clk),
.i_rst_n (i_rst_n),
.i_data (r_R_in),
.i_DV (r_R_DV),
.o_echo_out (w_R_echo_out),
.o_echo_DV (w_R_DV_echo)
	);


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
o_echo_out <= w_buf_data / 2;// / 2; //gain of .5
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