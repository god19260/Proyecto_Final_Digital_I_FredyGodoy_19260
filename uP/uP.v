
// Program Counter
module ProgramCounter(clock, reset, incPC, loadPC,address_RAM,PC);
input wire clock, reset, incPC, loadPC;
input wire [11:0]address_RAM;
output reg [11:0]PC;
        always @ (posedge clock or posedge reset)begin 
            if (reset == 1)
                PC <= 12'b000000000000;
            
            else if(loadPC == 1)
                PC <= address_RAM;

            else if(incPC == 1 && ~loadPC) 
            PC <= PC + 1;
        end
endmodule

// Memoria ROM
module MemoriaROM (PC, program_byte);           
input wire [11:0]PC;
output wire [7:0]program_byte;
	reg [11:0] ROM [0:4095] ;  
    
	assign program_byte = ROM[PC];

	initial begin
	  $readmemh("memory.list", ROM); 
	end

endmodule

// Fetch
module Fetch ( program_byte, clock, reset, no_phase, instr, oprnd);
input wire [7:0] program_byte; 
input wire clock, reset, no_phase; 
output reg[3:0] instr,oprnd;
always @(posedge clock or posedge reset)  
		begin
			if(reset)
				begin
					{instr,oprnd} <= 0; 
				end
			else if(no_phase==1) 
				begin
					{instr,oprnd} <= program_byte;
				end
			else
				begin
					{instr,oprnd} <= {instr,oprnd};
		end
	end

endmodule

// Phase
module Phase ( clock, reset, phase);
input wire clock, reset;
output reg phase;

	always @(posedge reset or posedge clock)
		begin
			if(reset==1)
				phase <= 0;
			else
				phase <= ~phase;
		end	
endmodule

// Flags
module Flags ( C,  Z,  clock, reset, loadFlags, c_flag,z_flag);
input wire C, Z,clock,reset, loadFlags; 
output reg c_flag, z_flag;


	always @(posedge clock or posedge reset)  
		begin
			if(reset)
				begin
					{c_flag,z_flag} <= 0; 
				end
			else if(loadFlags==1) 
				begin
					{c_flag,z_flag} <= {C,Z};
				end
			else
				begin
					{c_flag,z_flag} <= {c_flag,z_flag};
		end
	end

endmodule

// Decode
module Decode( decode_in,  decode_out);
input wire [6:0] decode_in;
output reg [12:0] decode_out;
    
    
    always @ (decode_in)
        casex(decode_in)
            // any
            7'bxxxx_xx0: decode_out <= 13'b1000_000_001000;
            // JC
            7'b0000_1x1: decode_out <= 13'b0100_000_001000;
            7'b0000_0x1: decode_out <= 13'b1000_000_001000;
            // JNC
            7'b0001_1x1: decode_out <= 13'b1000_000_001000;
            7'b0001_0x1: decode_out <= 13'b0100_000_001000;
            // CMPI
            7'b0010_xx1: decode_out <= 13'b0001_001_000010;
            // CMPM
            7'b0011_xx1: decode_out <= 13'b1001_001_100000;
            // LIT
            7'b0100_xx1: decode_out <= 13'b0011_010_000010;
            // IN
            7'b0101_xx1: decode_out <= 13'b0011_010_000100;
            // LD
            7'b0110_xx1: decode_out <= 13'b1011_010_100000;
            // ST
            7'b0111_xx1: decode_out <= 13'b1000_000_111000;
            // JZ
            7'b1000_x11: decode_out <= 13'b0100_000_001000;
            7'b1000_x01: decode_out <= 13'b1000_000_001000;
            // JNZ
            7'b1001_x11: decode_out <= 13'b1000_000_001000;
            7'b1001_x01: decode_out <= 13'b0100_000_001000;
            // ADDI
            7'b1010_xx1: decode_out <= 13'b0011_011_000010;
            // ADDM
            7'b1011_xx1: decode_out <= 13'b1011_011_100000;
            // JMP
            7'b1100_xx1: decode_out <= 13'b0100_000_001000;
            // OUT
            7'b1101_xx1: decode_out <= 13'b0000_000_001001;
            // NANDI
            7'b1110_xx1: decode_out <= 13'b0011_100_000010;
            // NANDM
            7'b1111_xx1: decode_out <= 13'b1011_100_100000;
            default: decode_out <= 13'b1111111111111;
        endcase
endmodule

// Bus de datos 1
module Buffer1 (oeOprnd, oprnd, data_bus); 
input wire oeOprnd;
input wire [3:0]oprnd;
output  [3:0] data_bus;

	assign data_bus = oeOprnd ? oprnd : 4'bz;

endmodule

// Bus de datos 2
module Buffer2 (oeALU, alu_out, data_bus); 
input wire oeALU; 
input wire [3:0] alu_out; 
output [3:0] data_bus;

	assign data_bus = oeALU ? alu_out : 4'bz;
endmodule

// Bus de datos 3
module Buffer3 (oeIN, pushbuttons, data_bus); 
input wire oeIN;
input wire [3:0] pushbuttons; 
output [3:0] data_bus;
	assign data_bus = oeIN ? pushbuttons : 4'bz;

endmodule

// Memoria RAM
module MemoriaRAM(csRAM, weRAM,address_RAM,data_bus);
input wire csRAM, weRAM; 
input wire [11:0]address_RAM; 
inout wire [3:0]data_bus;
    reg [3:0]RAM[0:4095]; 
    reg [3:0]databus;
    assign data_bus = (csRAM && ~weRAM) ? databus: 4'bzzzz; 

    always @(csRAM, weRAM, address_RAM, data_bus) begin 
        
        if (csRAM && ~weRAM)
            databus= RAM[address_RAM];
        if (csRAM && weRAM)
            RAM[address_RAM] = data_bus; 
            
        end
endmodule

// Acumulador
module Accu (alu_out,clock,reset,loadA, accu);
input wire [3:0] alu_out;
input wire clock,reset,loadA;
output reg [3:0]accu;
	
	always @(posedge clock or posedge reset)  
		begin
			if(reset)
				begin
					accu <= 4'b0000; 
				end
			else if(loadA==1) 
				begin
					accu <= alu_out;
				end
			else
				begin
					accu <= accu;
		end
	end
endmodule

// ALU
module ALU (accu, data_bus, sel, C, Z, alu_out);
input wire [3:0] accu, data_bus; 
input [2:0] sel; 
output C, Z; 
output [3:0] alu_out;
    reg [4:0] q;
    
    always @ (accu, data_bus, sel)
        case (sel)
            3'b000: q = accu; 
            3'b001: q = accu - data_bus; 
            3'b010: q = data_bus; 
            3'b011: q = accu + data_bus; 
            3'b100: q = {1'b0, ~(accu & data_bus)};
            default: q = 5'b10101;
        endcase
    
    assign alu_out = q[3:0];
    assign C = q[4];
    assign Z = ~(q[3] | q[2] | q[1] | q[0]);
    
endmodule

// Outputs
module Outputs (data_bus,clock, reset, loadOut, FF_out);
input wire [3:0] data_bus;
input wire clock,reset,loadOut;
output reg [3:0] FF_out;
	

	always @(posedge clock or posedge reset)  
		begin
			if(reset)
				begin
					FF_out<= 4'b0000; 
				end
			else if(loadOut==1) 
				begin
					FF_out <= data_bus;
				end
			else
				begin
					FF_out <= FF_out;
		end
	end
    
endmodule


//--------------------------------------------------------------------------------------------------------------------------------
//------------------ Modulo uP ---------------------------------------------------------------------------------------------------

module uP(  clock, reset, pushbuttons, phase, c_flag, z_flag,instr, oprnd, accu, data_bus, FF_out, program_byte, PC, address_RAM);
input wire clock, reset; 
input wire [3:0]pushbuttons; 
output wire phase, c_flag, z_flag; 
output wire [3:0] instr, oprnd, accu, data_bus, FF_out; 
output wire [7:0] program_byte; 
output wire [11:0] PC, address_RAM;
//Variables internas
wire [3:0]  alu_out;
wire [12:0] decode_out;
wire [6:0]  decode_in;
wire [2:0]  sel;
wire Z, C;

assign address_RAM = {oprnd, program_byte};
assign decode_in = {instr, c_flag, z_flag, phase};
assign incPC = decode_out [12];
assign loadPC = decode_out [11];
assign loadA = decode_out [10];
assign loadFlags = decode_out [9];
assign sel = decode_out [8:6];
assign csRAM = decode_out [5];
assign weRAM = decode_out [4];
assign oeALU = decode_out [3];
assign oeIN = decode_out [2];
assign oeOprnd = decode_out [1];
assign loadOut = decode_out [0];



ProgramCounter	programcounter_up	(clock, reset, incPC, loadPC, address_RAM, PC);
MemoriaROM		rom_up			    (PC, program_byte);
Fetch 			fetch_up		    (program_byte, clock, reset, ~phase, instr, oprnd);
Phase 			phase_up		    (clock, reset, phase);
Flags 			flags_up		    (C, Z, clock, reset, loadFlags, c_flag, z_flag);
Decode			decode_up		    (decode_in, decode_out);
Buffer1 		BusFetch_up		    (oeOprnd, oprnd, data_bus); 
Buffer2 		busAlu_up		    (oeALU, alu_out, data_bus); 
Buffer3 		bus_pushbuttons_up	(oeIN, pushbuttons, data_bus); 
MemoriaRAM		ram_up		    	(csRAM, weRAM, address_RAM, data_bus);
Accu 			accu_up	    	    (alu_out, clock, reset, loadA, accu);
ALU 			alu_up		      	(accu, data_bus, sel, C, Z, alu_out);
Outputs 		output_up		    (data_bus,clock,reset, loadOut,FF_out);

endmodule