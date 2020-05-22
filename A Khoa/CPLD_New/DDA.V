module dda(N, WR, clk, pulse, dir, busy);

input [7:0] N;
input WR;
input clk;

output reg pulse = 0;
output reg dir = 0;
output reg busy = 0;


parameter Nmax  = 50;	// chu ky dieu khien 1ms, xung 20us -> Nmax = 499
parameter Nmax1 = 48;	// tre 1 chu ky -> so xung max xuat ra 498
parameter Nmax2 = 100;

reg [7:0] Ntemp = 0;
reg [7:0] acc = Nmax1;

reg [7:0] clk_cnt = 0;
reg [7:0] clk5u_cnt = 0;
reg clk5u = 0;

reg dirtemp = 0;


always @(posedge clk or posedge WR)  begin
  	if (WR) begin				// neu co xung WR nap gia tri moi
		Ntemp = N[6:0];
		dirtemp = N[7];
		busy = 1;
		clk_cnt = 0;
		clk5u_cnt = 0;
		clk5u = 0;		
		acc = Nmax1; //Nmax1 = 50
		pulse = 0;
	end
	
	else begin
		if (clk_cnt < 199)//update every 1/20M = 50ns x 200 = 10us = chieu dai xung duong / chu ki xung
			clk_cnt = clk_cnt + 1;
		else begin 
			dir = dirtemp;
			clk_cnt = 0;
			if (clk5u_cnt < (Nmax2-2)) begin //Nmax2 = 100
				clk5u_cnt = clk5u_cnt + 1;
				
				clk5u = !clk5u;
				
				if (clk5u) begin
					acc = acc + Ntemp; //Ntemp = 50, acc = Nmax1 = 49
					if (acc >= Nmax) begin //Nmax = 50
						acc = acc - Nmax;
						pulse = 1;
					end		
					else 
						pulse = 0;
				end
				else begin
					pulse = 0;
				end
				//busy = 1;
			end
			else 
				busy = 0;	
		end
	end
end				
endmodule
