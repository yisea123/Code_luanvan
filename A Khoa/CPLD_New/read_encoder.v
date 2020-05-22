// Copyright (C) 1991-2010 Altera Corporation
// Your use of Altera Corporation's design tools, logic functions 
// and other software and tools, and its AMPP partner logic 
// functions, and any output files from any of the foregoing 
// (including device programming or simulation files), and any 
// associated documentation or information are expressly subject 
// to the terms and conditions of the Altera Program License 
// Subscription Agreement, Altera MegaCore Function License 
// Agreement, or other applicable license agreement, including, 
// without limitation, that your use is for the sole purpose of 
// programming logic devices manufactured by Altera and sold by 
// Altera or its authorized distributors.  Please refer to the 
// applicable agreement for further details.

module read_encoder
(
	BUSY,
	CLK_20M,
	DATA,
	DIR1,
	DIR2,
	DIR3,
	DIR4,
	DIR5,
	DIR6,
	ENC_1A,
	ENC_1B,
	ENC_2A,
	ENC_2B,
	ENC_3A,
	ENC_3B,
	ENC_4A,
	ENC_4B,
	ENC_5A,
	ENC_5B,
	ENC_6A,
	ENC_6B,
	ENC_RST,
	LIMIT_1,
	LIMIT_2,
	LIMIT_3,
	LIMIT_4,
	LIMIT_5,
	LIMIT_6,
	NADV,
	NE1,
	NOE,
	NWE,
	PULSE1,
	PULSE2,
	PULSE3,
	PULSE4,
	PULSE5,
	PULSE6,
	PULSE_WR1
);

output			BUSY;
input			CLK_20M;
inout	[15:0]	DATA;
output			DIR1;
output			DIR2;
output			DIR3;
output			DIR4;
output			DIR5;
output			DIR6;
input			ENC_1A;
input			ENC_1B;
input			ENC_2A;
input			ENC_2B;
input			ENC_3A;
input			ENC_3B;
input			ENC_4A;
input			ENC_4B;
input			ENC_5A;
input			ENC_5B;
input			ENC_6A;
input			ENC_6B;
input			ENC_RST;
input			LIMIT_1;
input			LIMIT_2;
input			LIMIT_3;
input			LIMIT_4;
input			LIMIT_5;
input			LIMIT_6;
input			NADV;
input			NE1;
input			NOE;
input			NWE;
output			PULSE1;
output			PULSE2;
output			PULSE3;
output			PULSE4;
output			PULSE5;
output			PULSE6;
input			PULSE_WR1;

endmodule
