`timescale 1ns/1ns
module final_project_tb();
parameter word_size = 8;
parameter full_cnt = 2**word_size - 1;
wire o_serial_out;
reg [word_size-1:0] i_data_bus; 

reg i_load_xmt_data; //load the data reg
reg i_byte_rdy; //ready for transmission
reg i_T_byte; //start transmission
reg i_clk;
reg i_reset_n;

final_project UUT(

	);

reg [word_size-1:0] r_counter;

initial begin
r_counter = 0;
i_clk = 1;
i_reset_n = 1;
i_data_bus = 0;
i_load_xmt_data = 0;
i_byte_rdy = 0;
i_T_byte = 0;


while(r_counter < full_cnt)
begin
#2 i_reset_n = 0; //toggle reset
#4 i_reset_n = 1;

#2 i_load_xmt_data =1;
#2 i_load_xmt_data =0;
   i_byte_rdy = 1;
#2 i_byte_rdy = 0;
   i_T_byte = 1;
#2 i_T_byte = 0;

# 24;
r_counter = r_counter + 1;
i_data_bus = i_data_bus + 1;
end
end


always begin 
#1 i_clk = !i_clk;
end

endmodule