module dda(N1, WR, clk, pulse1, dir1, busy);

input [7:0] N1;


input WR;
input clk;

output reg pulse1 = 0;
output reg dir1   = 0;
output reg busy = 0;



parameter Nmax  = 50;	// chu ky dieu khien 1ms, xung 20us -> Nmax = 50 
parameter Nmax1 = 49;	// tre 1 chu ky -> so xung max xuat ra 49
parameter Nmax2 = 100;

reg [6:0] Ntemp1 = 0;
reg [6:0] acc1 = Nmax1;



reg [7:0] clk_cnt = 0;
reg [6:0] clk5u_cnt = 0;
reg clk5u = 0;





always @(posedge clk or posedge WR)  begin
  	if (WR) begin				// neu co xung WR nap gia tri moi
		Ntemp1   = N1[6:0];
		dir1 = N1[7];
		
		busy = 1;
		clk_cnt = 0;
		clk5u_cnt = 0;
		clk5u = 0;		
		
		acc1 = Nmax1;	
		pulse1 = 0;

	end
	
	else begin
		if (clk_cnt < 199)//update every 50ns*200=10us=1/2 chu ki xung
			clk_cnt = clk_cnt + 1;
		else begin 
		
			clk_cnt = 0;
			if (clk5u_cnt < (Nmax2-2)) 
				begin
				clk5u_cnt = clk5u_cnt + 1;
				clk5u = !clk5u;
				
				if (clk5u) 
					begin
					acc1 = acc1 + Ntemp1;

					if (acc1 >= Nmax) 
						begin
						acc1 = acc1 - Nmax;
						pulse1 = 1;
						end		
					else 
							begin 
							pulse1 = 0;
							end	
		
					end
				else 
					begin
					pulse1 = 0;

					end	
				end
			else 
				busy = 0;	
		end
	end
end				

endmodule
