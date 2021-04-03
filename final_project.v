//IMPORTANT: NEED TO MAKE EVERYTHING SIGNED
//need to finish instantiating the echo 
//need to update the top module so that it has a state machine for adding in the echo
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
inout io_I2C_SDAT

);

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
reg [23:0] r_l_snk_data;
reg r_l_snk_DV;
wire r_l_snk_rdy;

reg [23:0] r_r_snk_data;
reg r_r_snk_DV;
wire l_r_snk_rdy;

//for now route input data to output data to verify functionality
//loopback test
always@(posedge i_clk)
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

//instantiate the echo:
echo ech0(
.i_clk (),
.i_rst_n (),
//LEFT OFF DOING THIS
	);

endmodule

module echo(
input i_clk,
input i_rst_n,

input [23:0] i_data_L,
input [23:0] i_data_R,
input i_L_DV,
input i_R_DV,

output reg signed [23:0] o_echo_L,
output [23:0] o_echo_R,
output o_L_DV,
output o_R_DV

);
reg [15:0] r_data_ptr = 16'd0;
reg [15:0] r_echo_ptr = 16'd12000;
reg r_we;
reg [23:0] r_buf_data;
wire [23:0] w_buf_data;
reg [15:0] r_addr;
reg [3:0] r_SM0;
reg signed [23:0] r_echo_L_hold;

//delay of .5 seconds: Fs = 48000, delay = .5, buffer length = 48000 * .5 = 24000
//delay gain of .5
//process: new sample comes in, write it to data_ptr, take sample from echo ptr, multiply by gain, send out and DV, update both ptrs
always@(posedge i_clk)
begin
o_L_DV <= 0;
case(r_SM0)
IDLE: begin
if(i_L_DV) begin 
	r_SM0 <= WRITE_NEW; 
	r_we <= 1; 
	r_addr <= r_data_ptr;
	r_buf_data <= i_data_L;
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
r_echo_ptr <= r_echo_ptr + 1; 
r_data_ptr <= r_data_ptr + 1; 
r_SM0 <= SEND_OUT;
end
SEND_OUT:begin
o_echo_L <= r_echo_L_hold / 2; 
o_L_DV <= 1; 
r_SM0 <= IDLE;
end
default: begin
r_SM0 <= IDLE;
r_we <= 0; 
r_addr <= r_addr; 
r_buf_data <= r_buf_data;
end
endcase

//repeat for R side of audio, can change the delay offset from L for stereo echo effect


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
	reg [23:0] ram[0:24000];  // need 1024 samples

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