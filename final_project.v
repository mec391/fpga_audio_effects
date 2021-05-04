//to do:
//pull in multiple switches:
// all off: bypass SW1: single tap delay SW2: multi tap delay SW3: ping pong delay effect (for single or multi tap delay) SW4: chorus effect
//implement multi tap echo effect to echo, stereo ping pong effect
//implement chorus effect (delay of .02 seconds to 0.03 seconds, cycling at .01Hz + delay of .02 to .05, .02Hz)
//fir low pass filter

//LEFTMOST SWITCHEST (SW8 & SW9) CONTROL AUDIO EFFECT ENABLE FOR CHROUS AND ECHO

module final_project(
input i_clk,
input i_rst_n,

output o_AUD_XCK, //audio codec control signals that need routed to specific DE1 SOC pins
inout io_AUD_BCLK,
output o_AUD_DACDAT,
inout io_ADU_DACLRCK,
input i_AUD_ADCDAT,
inout io_ADCLRCK,
output o_I2C_SCLK,
inout io_I2C_SDAT,

input i_echo_en, //route to a switch to turn on or off enable for echo
input i_chorus_en, 
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
wire signed [23:0] w_L_chorus_out;
wire signed [23:0] w_R_chorus_out;
wire w_R_DV_chorus;
wire w_L_DV_chorus;

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
if(w_L_DV_chorus) begin // this will need modified when other componenets are added in
	if(i_chorus_en && i_echo_en) r_l_snk_data <= (w_L_chorus_out >>> 1) + (w_L_echo_out >>> 1) + (r_in_data_hold_L >>> 1); 
	else if (i_echo_en) r_l_snk_data <= w_L_echo_out + r_in_data_hold_L;
	else if (i_chorus_en) r_l_snk_data <= w_L_chorus_out + r_in_data_hold_L;
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
if(w_r_src_DV) begin              //SOURCE DV is high: reg the data line for the submodule, reg/hold the data line for the output signal, DV the submodule
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
COMPUTE: begin  //submodule is running: when submodule sends back a DV, combine the hold reg with the submodule output and DV and send to SINK
r_R_DV <= 0;
if(w_R_DV_chorus) begin // this will need modified when other componenets are added in
	if(i_chorus_en && i_echo_en) r_r_snk_data <= (w_R_chorus_out >>> 1) + (w_R_echo_out >>> 1) + (r_in_data_hold_R >>> 1); 
	else if (i_echo_en) r_r_snk_data <= w_R_echo_out + r_in_data_hold_R;
	else if (i_chorus_en) r_r_snk_data <= w_R_chorus_out + r_in_data_hold_R;
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
soc_system u0 ( //this routes to platform designer for controlling the audio codec and interface
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

//instantiate the left chorus
chorus cho0(
.i_clk (i_clk),
.i_rst_n (i_rst_n),
.i_data (r_L_in),
.i_DV (r_L_DV),
.o_chorus_out (w_L_chorus_out),
.o_chorus_DV (w_L_DV_chorus)
	);
//instantiate the right chorus
chorus cho1(
.i_clk (i_clk),
.i_rst_n (i_rst_n),
.i_data (r_R_in),
.i_DV (r_R_DV),
.o_chorus_out (w_R_chorus_out),
.o_chorus_DV (w_R_DV_chorus)
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
reg [15:0] r_data_ptr = 16'd0;  //data ptr is always 24000 samples ahead of echo_ptr
reg [15:0] r_echo_ptr = 16'd1; //.5 second delay
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
if(i_DV) begin //got a DV from the top module, write to data_ptr by setting WE hi, addr to data_ptr, buffer_data to input_data
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
WRITE_READ:begin //toggle off the WE, move addr to the echo_ptr
r_we <= 0; 
r_addr <= r_echo_ptr; 
r_SM0 <= NEXT;
end
NEXT:begin //increment up pointers
r_we <= 0; 
//o_echo_out <= o_echo_out / 16; //apply feedback gain ahead of time
o_echo_out <= o_echo_out >>> 3; //apply feedback gain ahead of time
if(r_echo_ptr == 16'd23999) r_echo_ptr <= 16'd0; 
else r_echo_ptr <= r_echo_ptr + 1; 
if(r_data_ptr == 16'd23999) r_data_ptr <= 16'd0;
else r_data_ptr <= r_data_ptr + 1; 
r_SM0 <= SEND_OUT;
end
SEND_OUT:begin //take the output from the buffer, apply gain of .5 and send out with a DV
//o_echo_out <= o_echo_out + (w_buf_data / 2);// / 2; //gain of .5 on the delay, gain of ..5 on the feedback
o_echo_out <= o_echo_out + (w_buf_data >>> 2);
o_echo_DV <= 1; 
r_SM0 <= IDLE;
end
default: begin
r_SM0 <= IDLE;
r_we <= 0; 
r_addr <= r_addr; 
r_buf_data <= o_echo_out;
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


module single_port_ram //this infers BLOCK RAM
(
	input [23:0] data,
	input [15:0] addr,
	input we, clk,
	output [23:0] q
);

	// Declare the RAM variable
	reg [23:0] ram[0:23999];  // 

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

//implement chorus effect (delay of .02 seconds to 0.03 seconds, cycling at .01Hz + delay of .02 to .05, .02Hz)
//delay 1 between 960 and 1440 samples, increment/decrement @ 10 samples per second -- every 4800 DVs
//delay 2 between 950 and 2400 samples, increment/decrement @ 29 samples per second -- every 1655 Dvs
//decrease delay == move up position by 2 instead of 1
//increase delay == hold position
//maintain delay == move up position by 1

//process: new sample comes in,  write sample to data_ptr by setting WE high, take sample from ptr1, take sample from ptr 2, add and multiply by gain, send out and DV, update all ptrs and DV_counters 
module chorus(
input i_clk,
input i_rst_n,

input [23:0] i_data,
input i_DV,

output reg signed [23:0] o_chorus_out,
output reg o_chorus_DV

	);
parameter IDLE = 0;
parameter WRITE_READ = 1;
parameter READ1 = 2;
parameter READ2 = 3;
parameter READ3 = 4;
parameter READ4 = 5;
parameter READ5 = 6;
parameter UPDATE = 7;
parameter UPDATE2 = 8;

//need counters for decrementing delays and stuff
reg [15:0] updown_1 = 16'd1440;//delay 1 starts at 1440 behind
reg dec1;
reg [15:0] updown_2 = 16'd2400;//delay 2 starts at 2400 behind
reg dec2;
reg signed [23:0] r_ptr_data1;
reg signed [23:0] r_ptr_data2;
reg dv1;
reg dv2;
reg [15:0] dv_cnt1;
reg [15:0] dv_cnt2;
reg [3:0] r_SM0;
reg [15:0] r_data_ptr = 16'd2400;  //data ptr tracks input data
reg [15:0] r_delay_ptr1 = 16'd960; //delay 1 starts at 1440 behind
reg [15:0] r_delay_ptr2 = 16'd0; //delay 2 starts at 2400 behind
reg r_we;
reg [23:0] r_buf_data;
wire signed [23:0] w_buf_data;
reg [15:0] r_addr;

always@(posedge i_clk)
begin
o_chorus_DV <= 0;
case(r_SM0)
IDLE: begin
if(i_DV) begin //got a DV from the top module, write to data_ptr by setting WE hi, addr to data_ptr, buffer_data to input_data
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
WRITE_READ:begin //toggle off the WE, move addr to the ptr_1
r_we <= 0; 
r_addr <= r_delay_ptr1; 
r_SM0 <= READ1;
end
READ1:begin //apply 1 clk cycle delay
r_SM0 <= READ2;
end
READ2:begin //reg ptr1 data, move addr to the ptr_2 //apply gain of .5
r_ptr_data1 <= w_buf_data >>> 1;
r_addr <= r_delay_ptr2;
r_SM0 <= READ3;
end
READ3:begin
r_SM0 <= READ4;
end
READ4:begin //reg ptr2 data, apply gain of .5
r_ptr_data2 <= w_buf_data >>> 1;
r_SM0 <= READ5;
end
READ5:begin //assign output value, update all ptrs up and DV signals
o_chorus_out <= r_ptr_data1 + r_ptr_data2;
o_chorus_DV <= 1;

if(r_data_ptr == 16'd2399) r_data_ptr <= 16'd0;
else r_data_ptr <= r_data_ptr + 1;
if(r_delay_ptr1 == 16'd2399) r_delay_ptr1 <= 16'd0;
else r_delay_ptr1 <= r_delay_ptr1 + 1;
if(r_delay_ptr2 == 16'd2399) r_delay_ptr2 <= 16'd0;
else r_delay_ptr2 <= r_delay_ptr2 + 1;

if(dv_cnt1 == 16'd4800) begin dv1 <= 1; dv_cnt1 <= 16'd0; end
else begin dv1 <= 0; dv_cnt1 <= dv_cnt1 + 1; end
if(dv_cnt2 == 16'd1655) begin dv2 <= 1; dv_cnt2 <= 16'd0; end
else begin dv2 <= 0; dv_cnt2 <= dv_cnt2 + 1; end
r_SM0 <= UPDATE;
end
UPDATE:begin //increment up or down ptrs again based on DV signals
	//dec1 == 0 means we are decrementing down, dec1 == 1 means we are incrementing up
if (dv1) begin 
	if(updown_1 == 16'd1440) dec1 <= 0;
	else if (updown_1 == 16'd960) dec1 <= 1;
	else dec1 <= dec1;
end
if (dv2) begin 
	if(updown_2 == 16'd2400) dec2 <= 0;
	else if (updown_2 == 16'd950) dec2 <= 1;
	else dec2 <= dec2;
end

r_SM0 <= UPDATE2;
end
UPDATE2:begin
if (dv1) begin
	if (dec1 == 0) begin updown_1 <= updown_1 - 1;
		if(r_delay_ptr1 == 16'd0) r_delay_ptr1 <= 16'd2399; else r_delay_ptr1 <= r_delay_ptr1 -1; end
	else begin updown_1 <= updown_1 + 1;
		if(r_delay_ptr1 == 16'd2399) r_delay_ptr1 <= 0; else r_delay_ptr1 <= r_delay_ptr1 + 1; end
	end
if (dv2) begin
	if (dec2 == 0) begin updown_2 <= updown_2 - 1;
		if(r_delay_ptr2 == 16'd0) r_delay_ptr2 <= 16'd2399; else r_delay_ptr2 <= r_delay_ptr2 -1; end
	else begin updown_2 <= updown_2 + 1;
		if(r_delay_ptr2 == 16'd2399) r_delay_ptr2 <= 0; else r_delay_ptr2 <= r_delay_ptr2 + 1; end
	end

dv1 <= 0;
dv2 <= 0;

r_SM0 <= IDLE;
end
endcase


end
single_port_ram_chorus spr0(
.data (r_buf_data),
.addr (r_addr),
.we (r_we),
.clk (i_clk),
.q (w_buf_data)
	);
endmodule



module single_port_ram_chorus //this infers BLOCK RAM
(
	input [23:0] data,
	input [15:0] addr,
	input we, clk,
	output [23:0] q
);

	// Declare the RAM variable
	reg [23:0] ram[0:2399];  // 

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



//Pitch Shifter Module//
//functionality -- data split to 2 channels -- high to low delay with gain for ch1 and 0 delay/0 gain for ch2
// then high to low delay with crossfade up gain for ch2, ch1 stays 0 delay with crossfade down, repeat
//*ch1 is completely faded out by the time it decrements to 0 delay

//downshift -- delays increase from 0 to max instead of decrease
//pitch amount is determined by how quickly the delays change
//
/*
module pitch_shift(
input i_clk,
input i_rst_n,

input [23:0] i_data,
input i_DV,
input [3:0] i_delta_delay, //0000 for 20 samples per ms, 1111 for 80 samples per ms, 0111 for 40 samples per ms
input i_up_down, //1 for up, 0 for down

output reg signed [23:0] o_shift_out,
output reg o_shift_DV
	);
parameter IDLE = 0;
parameter ONES = 1;
parameter NEXT = 3;
parameter SEND_OUT = 4;
reg [3:0] r_SM0;
reg [11:0] r_delay_cnt_a = 12'd1; //decrement from 1439 to 0 for pitch up, 0 up to 1440 for pitch down
reg [11:0] r_delay_cnt_b = 12'd0; //decrement from 1439 to 0 for pitch up, 0 up to 1440 for pitch down
reg [7:0] r_gain_a; //this will be a number between 0 and 99 so that you can mult by 99 and then divide by 100 for gain of .99
reg [7:0] r_gain_b; //this will be a number between 0 and 99 so that you can mult by 99 and then divide by 100 for gain of .99
//30 ms delay max @ 48khz == 1.44ksamples
//process --> read data in to memory, take a and b pt values, apply gain a and b values, add and output, increment/decrement delay ptrs & increment/decrement gain cntrs
//if decrementing, when the decrementing delay reaches a threshold of about 2.5ms, start the crossfade, switch other delay from 0 to 30

//generate twice for pitch up and pitch down
reg [11:0] r_ptr = 0;
reg [23:0] r_buf_data;
reg [15:0] r_addr;
reg r_we;
//pitch up module 
always@(posedge i_clk)
begin
case(r_SM0)
IDLE: begin
if(i_DV) begin 
	if(r_delay_cnt_a > 120)r_SM0 <= DECREMENT_A; else if(r_delay_cnt_a > 0) r_SM0 <= A_B_TRANS; else ()
	r_we <= 1; 
	r_addr <= r_ptr;
	r_buf_data <= i_data;
	end
else begin 
	r_SM0 <= r_SM0;
	r_we <= 0; 
	r_addr <= r_addr; 
	r_buf_data <= r_buf_data;
	end
end


endcase
end

reg [3:0] r_SM1;
reg [11:0] r_delay_cnt_c = 12'd1439; //decrement from 1439 to 0 for pitch up, 0 up to 1440 for pitch down
reg [11:0] r_delay_cnt_d = 12'd0; //decrement from 1439 to 0 for pitch up, 0 up to 1440 for pitch down
reg [7:0] r_gain_c; //this will be a number between 0 and 99 so that you can mult by 99 and then divide by 100 for gain of .99
reg [7:0] r_gain_d; //this will be a number between 0 and 99 so that you can mult by 99 and then divide by 100 for gain of .99
//pitch down module
always@(posedge i_clk)
begin
case(r_SM1)
IDLE: begin

end


endcase
end

//assign output based on pitch up/down switch
always@(posedge i_clk)
begin

//if (*data_reg from always block*))begin

if (i_up_down) o_shift_out <= //the reg from the up block
else o_shift_out <= //the reg from the down block
o_shift_DV <= 1;

//else o_shift_DV <= 0; o_shift_out <= o_shift_out;

end


single_port_ram_pitch sprp0(
.data (),
.addr (),
.we (),
.clk (),
.q ()
	);

single_port_ram_pitch sprp1(
.data (),
.addr (),
.we (),
.clk (),
.q ()
	);

endmodule 


module single_port_ram_pitch
(
	input [23:0] data,
	input [15:0] addr,
	input we, clk,
	output [23:0] q
);

	// Declare the RAM variable
	reg [23:0] ram[0:1439];  // 

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
*/