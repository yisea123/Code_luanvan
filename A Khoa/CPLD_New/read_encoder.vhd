-- Copyright (C) 1991-2010 Altera Corporation
-- Your use of Altera Corporation's design tools, logic functions 
-- and other software and tools, and its AMPP partner logic 
-- functions, and any output files from any of the foregoing 
-- (including device programming or simulation files), and any 
-- associated documentation or information are expressly subject 
-- to the terms and conditions of the Altera Program License 
-- Subscription Agreement, Altera MegaCore Function License 
-- Agreement, or other applicable license agreement, including, 
-- without limitation, that your use is for the sole purpose of 
-- programming logic devices manufactured by Altera and sold by 
-- Altera or its authorized distributors.  Please refer to the 
-- applicable agreement for further details.

library ieee;
use ieee.std_logic_1164.all;
library altera;
use altera.altera_syn_attributes.all;

entity read_encoder is
	port
	(
		BUSY : out std_logic;
		CLK_20M : in std_logic;
		DATA : inout std_logic_vector(15 downto 0);
		DIR1 : out std_logic;
		DIR2 : out std_logic;
		DIR3 : out std_logic;
		DIR4 : out std_logic;
		DIR5 : out std_logic;
		DIR6 : out std_logic;
		ENC_1A : in std_logic;
		ENC_1B : in std_logic;
		ENC_2A : in std_logic;
		ENC_2B : in std_logic;
		ENC_3A : in std_logic;
		ENC_3B : in std_logic;
		ENC_4A : in std_logic;
		ENC_4B : in std_logic;
		ENC_5A : in std_logic;
		ENC_5B : in std_logic;
		ENC_6A : in std_logic;
		ENC_6B : in std_logic;
		ENC_RST : in std_logic;
		LIMIT_1 : in std_logic;
		LIMIT_2 : in std_logic;
		LIMIT_3 : in std_logic;
		LIMIT_4 : in std_logic;
		LIMIT_5 : in std_logic;
		LIMIT_6 : in std_logic;
		NADV : in std_logic;
		NE1 : in std_logic;
		NOE : in std_logic;
		NWE : in std_logic;
		PULSE1 : out std_logic;
		PULSE2 : out std_logic;
		PULSE3 : out std_logic;
		PULSE4 : out std_logic;
		PULSE5 : out std_logic;
		PULSE6 : out std_logic;
		PULSE_WR1 : in std_logic;
		module : in std_logic
	);

end read_encoder;

architecture ppl_type of read_encoder is

begin

end;