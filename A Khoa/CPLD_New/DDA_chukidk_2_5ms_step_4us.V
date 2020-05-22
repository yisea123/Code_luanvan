module dda(N, WR, clk, pulse, dir, busy);

input [15:0] N;
input WR;
input clk;

output pulse;
output dir;
output busy;

//parameter Nmax = 250;	// chu ky dieu khien 2.5ms, xung 5us -> Nmax = 500 
//parameter Nmax2 = 500;	 

//parameter Nmax = 500;	// chu ky dieu khien 2.5ms, xung 5us -> Nmax = 500 
//parameter Nmax2 = 1000;	 

parameter Nmax = 625;	// chu ky dieu khien 2.5ms, xung 4us -> Nmax = 2500 
parameter Nmax2 = 1250;	 

//parameter Nmax = 488;	// chu ky dieu khien 2.5ms, xung 5us -> Nmax = 500 
//parameter Nmax2 = 488*2;

//parameter Nmax = 10;	// chu ky dieu khien 2.5ms, xung 5us -> Nmax = 10 
//parameter Nmax2 = 20;	

reg [11:0] Ntemp = 0;
reg [12:0] acc = Nmax;

reg [6:0] clk_cnt = 0;
reg [12:0] clk5u_cnt = 0;
reg clk5u = 0;

reg pulse = 0;
reg dir = 0;
reg busy = 0;
reg dirtemp = 0;


always @(posedge clk or posedge WR)  begin
  	if (WR) begin				// neu co xung WR nap gia tri moi
		Ntemp = N[11:0];
		dirtemp = N[15];
		busy = 1;
		clk_cnt = 0;
		clk5u_cnt = 0;
		clk5u = 0;		
		acc = Nmax;
	end
	
	else begin
		//if (clk_cnt < 99)//update every 20M/100=5us=1/2 chu ki xung
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
					//acc = acc + N[8:0];
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
