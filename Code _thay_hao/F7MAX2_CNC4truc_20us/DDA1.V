module dda(N, WR, clk, pulse, dir, busy);

input [15:0] N;
input WR;
input clk;

output pulse;
output dir;
output busy;


parameter Nmax  = 125;	// chu ky dieu khien 0.5ms, xung 4us -> Nmax = 125 
parameter Nmax1 = 124;	// tre 1 chu ky -> so xung max xuat ra 124
parameter Nmax2 = 250;

reg [6:0] Ntemp = 0;
reg [7:0] acc = Nmax1;

reg [5:0] clk_cnt = 0;
reg [7:0] clk5u_cnt = 0;
reg clk5u = 0;

reg pulse = 0;
reg dir = 0;
reg busy = 0;
reg dirtemp = 0;


always @(posedge clk or posedge WR)  begin
  	if (WR) begin				// neu co xung WR nap gia tri moi
		Ntemp = N[6:0];
		dirtemp = N[15];
		busy = 1;
		clk_cnt = 0;
		clk5u_cnt = 0;
		clk5u = 0;		
		acc = Nmax1;
	end
	
	else begin
		if (clk_cnt < 39)//update every 20M/40=2us=1/2 chu ki xung
			clk_cnt = clk_cnt + 1;
		else begin 
			dir = dirtemp;
			clk_cnt = 0;
			if (clk5u_cnt < (Nmax2-2)) begin
				clk5u_cnt = clk5u_cnt + 1;
	
				clk5u = !clk5u;
				if (clk5u) begin
					acc = acc + Ntemp;
					if (acc > Nmax) begin
						acc = acc - Nmax;
						pulse = 1;
					end		
					else 
						pulse = 0;
				end
				else
					pulse = 0;
			end
			else 
				busy = 0;	
		end
	end
end				

endmodule
