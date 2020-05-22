module test(N, WR, clk, pulse, dir, busy);

input [7:0] N;
input WR;
input clk;

output reg pulse;
output reg dir;
output reg busy;



parameter Nmax  = 50;	// chu ky dieu khien 1ms, xung 20us -> Nmax = 499
parameter Nmax1 = 48;	// tre 1 chu ky -> so xung max xuat ra 498
parameter Nmax2 = 100;

reg [7:0] Ntemp, buffer;
reg [7:0] acc = Nmax1;



reg [10:0] clk_cnt;
reg [7:0] clk20u_cnt;
reg clk20u;
reg temp;
reg empty;
//reg dirtemp;
reg pre_WR;

initial begin
pulse = 0;
dir = 0;
busy = 0;
Ntemp = 0;
clk_cnt = 0;
clk20u_cnt = 0;
clk20u = 0;
pre_WR = 0;
temp = 0;
empty = 1;

end

always @(posedge clk )  begin
	pre_WR <= WR;
	if ({pre_WR,WR} == 2'b01) begin // neu co xung WR nap gia tri moi 				
//		if ((busy == 1) && (empty == 1)) 
//		begin
//		buffer = N[7:0];
//		empty = 0;
//		end		
//		else if (busy == 0) begin 
		Ntemp = N[6:0];
		dir = N[7];
		busy = 1;
		clk_cnt = 0;
		clk20u_cnt = 0;
		clk20u = 0;		
		acc = Nmax1; //Nmax1 = 48
		pulse = 0;
//		end
	end
	
	else 
	begin
		
		if (clk_cnt < 199)//update every 1/20M = 50ns x 200 = 10us = chieu dai xung duong / chu ki xung
			clk_cnt = clk_cnt + 1;
		else 
		begin 
			//dir = dirtemp;
			clk_cnt = 0;

			if (clk20u_cnt < (Nmax2-2))// (Nmax2-2)) //Nmax2 = 100
				begin 
				
				clk20u_cnt = clk20u_cnt + 1;
				clk20u = !clk20u;
				
				if (clk20u) 
				begin
					acc = acc + Ntemp; //Ntemp = 50, acc = Nmax1 = 49
					if (acc >= Nmax) begin //Nmax = 50
						acc = acc - Nmax;
						pulse = 1;
					end		
					else pulse = 0;
				end
				else pulse = 0;
			
				
				end
			else begin						
			
//				if (buffer != 0) begin 
//					Ntemp = buffer[6:0];
//					dir = buffer[7];
//					buffer = 0;
//					clk20u_cnt = 0;
//					empty = 1;
//				end
				busy = 0;			
				end
			end
			end
		
	end
endmodule
