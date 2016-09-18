//Adder1 Module
// Adder in the fetch stage
module adder1(a, b);
  output[15:0] b; // 16 bit output
  input[15:0] a; // 16 bit input
  reg[15:0] b;
  
  always @(*) 
  	begin
  		  b = a+2'b10; // add 2 bytes per
  	end
endmodule


//adder1 testbench 

`include "adder1.v"

module testing;
	
wire[15:0]b; 
reg[15:0]a;
adder1 g1(a,b);

initial
begin
a=16'b0000000000000000;
#5 a=b;
#5 a=b;
#5 a=b;
#5 a=b;
end

initial
$vcdpluson;
initial
$monitor($time," in address = %b , out address = %b" , a, b);
initial
#30 $finish;
endmodule

// Adder2 ; for calculating jumps and branches
module adder2(originalAddress,newAddress,outAddress);
	input [15:0] originalAddress,newAddress;
	output reg[15:0] outAddress;
	
	assign outAddress = originalAddress + newAddress; // shifted left and extended, then add to original address
	
endmodule

// Adder 2 test bench
`include "adder2.v"

module adder2_test;
	
	reg [15:0] in1,in2;
	wire[15:0] out;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," +2ByteAddr = %b, offset = %b, out = %b, decimal= ", in1,in2, out,out);
	
		adder2 myAdder2(in1,in2, out);
		
	initial
	begin
		in1=0; in2=0;
		#10 in1=0; in2=1;
		#10 in1=1; in2=1;
		#10 in1=4; in2=4;
		#10 in1=8; in2=16;
		#10 in1=32; in2=64;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			

//ALU in EX phase
module alu(in1,in2,aluop,out,outReg0,overflow);
	
	input signed[15:0] in1,in2;//  input 1 and 2
	input[3:0] aluop; // Control signal
	output reg signed[15:0] out, outReg0;
	reg signed [16:0]temp; //extra bit to test for overflow
	output reg overflow;
	reg[31:0] largeResult;
	
	always@(*)
		begin
			overflow=0;
			case(aluop)
			4'b1111: begin //load/store calc addr
				out[15:0] = in1+in2; // or signed addition
				//if(in1>0 && temp[16]==1)
				//	overflow=1'b1;
				//else 
			end
			4'b1110: begin //subtract
				out = in1-in2;
			end
			4'b1101: begin //bitwise and
				out = in1&in2;
			end
			4'b1100: begin //bitwise or
				out = in1 | in2;
			end
			4'b0001: begin // multiplication
				largeResult = in1*in2;
				out = largeResult[15:0];
				outReg0 = largeResult[31:16];
			end
			4'b0010: begin // division
				out = in1/in2;
				outReg0= in1%in2;
			
			end
			4'b1010: begin // shiftleft
				out = in1<<<in2;
				
			end
			4'b1011: begin // shiftright
				out = in1>>>in2;
					
			end
			4'b1000: begin // rotateleft
				out = (in1>>(16-in2))|(in1<<in2);
			end
			4'b1001: begin // rotateright
				out = (in1<<(16-in2))|(in1>>in2);
			end
			default: begin
				out = 16'b0000000000000000;
				outReg0 = 16'b0000000000000000;
				end
		
		endcase
	end
endmodule


			
	
	
// ALU TB
`include "alu.v"

module alu_test;
	
	reg signed[15:0] in1,in2;
	reg[3:0] aluop;
	wire[15:0] out, outReg0;
	wire overflow;

	initial
		$vcdpluson;
	
	initial
		$monitor($time,"  in1 = %b, in2 = %b, aluop = %b,out = %b, outReg0 = %b, overflow = %b ", in1,in2,aluop,out,outReg0,overflow);
	
		alu myAlu(in1,in2,aluop,out,outReg0,overflow);
		
	initial
	begin
		in1 = 2;
		in2 = 2;
		aluop=4'b1111;
		#10
		in1 = 2;
		in2 = 2;
		aluop=4'b1110;
		#10
		in1 = 2;
		in2 = 2;
		aluop=4'b0000;
		#10
		in1 = 16'b0111111111111111;
		in2 = 16'b0000001111111111;
		aluop=4'b0001;
		#10
		aluop=4'b0010;
		#10
		in1= 16'b0000100011111111;
		in2= 2;
		aluop=4'b1010;
		#10
		in1= 16'b1000000000000010;
		aluop=4'b1011;
		#10
		aluop=4'b1000;
		#10
		aluop=4'b1001;
	end
	
	initial
	begin
		#200 $finish;
	end
	endmodule
			
				
		
// ALU control unit
module aluCU(aluop,funct,out);
	
	input [1:0] aluop; // signal from CU
	input [3:0] funct; // function code; [3:0] for type-A instruction
	output [3:0] out; // data out to ALU
	reg [3:0] out;
	
	always@(*)
		begin
			if(aluop == 2'b10)
				begin
					case(funct)
						4'b1111: out <= 4'b1111; //add
						4'b1110: out <= 4'b1110; //sub
						4'b1101: out <= 4'b1101; //bitwiseand
						4'b1100: out <= 4'b1100; //bitwiseor
						4'b0001: out <= 4'b0001; //multiply
						4'b0010: out <= 4'b0010; //divide
						4'b1010: out <= 4'b1010; //shiftleft
						4'b1011: out <= 4'b1011; //shiftright
						4'b1000: out <= 4'b1000; //rotate L
						4'b1001: out <= 4'b1001; //rotate R
						default: out <=4'b0000; //nop
					endcase
				end
			else if(aluop == 2'b00)
				begin
					out <= 4'b0000; //NOP
				end
			else if(aluop == 2'b01) //load or store, calculate address
				begin 
					out <= 4'b1111;
				end
			end
		endmodule
			
// ALU CONTROL UNIT
`include "aluCU.v"

module aluCU_test;
	
	reg [1:0] aluop; // signal from CU
	reg [3:0] funct; // function code; [3:0] for type-A instruction
	wire [3:0] out; // data out to ALU
	
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," aluop = %b, funct = %b, out = %b", aluop,funct,out);
	
		aluCU myAluCU(aluop,funct,out);
		
	initial
	begin
		aluop=00;
		funct=1111;
		#5
		aluop=10;
		funct=1111;
		#5
		aluop=10;
		funct=1001;
		#5
		aluop=10;
		funct=0010;
		#5
		aluop=00;
		funct=0010;
		#5
		aluop=01;
		funct=0000;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule



// compare(or) for jumps and branches
module compare(jumpBranch,reg0,reg1,pcSrc);
	input[2:0] jumpBranch;
	input signed [15:0] reg0,reg1;
	output pcSrc;
	reg pcSrc;
	
	always@(*)
		begin
			if(jumpBranch == 3'b000)// no branch
				begin
					pcSrc = 1'b0; // use +2 byte address
				end
			else if(jumpBranch == 3'b010)
				begin
					if((reg0-reg1) > 0)// reg0>reg1
						begin
							pcSrc = 1'b1; //tell mux to use calc jump/branch address
						end
					else
						pcSrc = 1'b0; // else use +2 bytes address
				end
			else if(jumpBranch == 3'b011)
				begin
					if((reg1-reg0) > 0) //reg1>reg0
						begin
							pcSrc = 1'b1;
						end
					else
						pcSrc = 1'b0;
				end
			else if(jumpBranch == 3'b100)
				begin
					if(reg0==reg1) //reg0=reg1
						begin
							pcSrc = 1'b1;
						end
					else
						pcSrc = 1'b0;
				end
			end
		endmodule
		
				
		`include "compare.v"
//Compare test bench
module compare_test;
	
	reg [2:0] jumpBranch; // signal from CU
	reg [15:0] reg0,reg1; // input register data
	wire pcSrc;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," jumpBranch = %b, reg0 = %b, reg1 = %b, pcSrc = %b", jumpBranch,reg0,reg1,pcSrc);
	
		compare myCompare(jumpBranch,reg0,reg1,pcSrc);
		
	initial
	begin
		reg0 = 16'b0000000000000001;
		reg1 = 16'b0000000000000010;
		jumpBranch = 3'b000;
		#10
		jumpBranch = 3'b001;
		#10
		jumpBranch = 3'b010;
		#10
		jumpBranch = 3'b011;
		#10
		jumpBranch = 3'b100;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
				
// Control Unit
module controlUnit(opcode,extendLength,jumpBranch,memWrite,memToReg,aluOp,regWrite,halt);
	
	input[3:0] opcode; // 15-12 bits from Instruction
	output extendLength, jumpBranch, memWrite, memToReg, aluOp, regWrite,halt;
	
	reg [1:0] extendLength; // 2 bits required , tells sign_extend module which input bit size
	reg [2:0] jumpBranch; // 3 bits, tells the compare for conditional jumps what type of compare is required
	reg memWrite, memToReg,regWrite,halt; // control signals ; see documentation for details
	reg [1:0] aluOp ; // 2 bits required to tell ALU CU which op to perform
	wire [3:0] opcode;
	
	always@ (opcode)
		begin
			case(opcode)
				4'b0000: begin //signed addition
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //signed subtraction
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //bitwise and
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //bitwise or
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //signed multiplication
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //signed division
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //logical shift left
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //logical shift right
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //rotate left
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b0000: begin //rotate right
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b1;
				aluOp <= 2'b10;
				halt <= 1'b0;
			end
			4'b1000: begin //load
				extendLength <= 2'b00; // lowest 4 bits are significant, sign extend remaining 12
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b1; // memory to register for load
				regWrite <= 1'b1;
				aluOp <= 2'b01;
				halt <= 1'b0;
			end
			4'b1011: begin //store
				extendLength <= 2'b00; // lowest 4 bits are significant, sign extend remaining 12
				jumpBranch <= 3'b000;
				memWrite <= 1'b1;
				memToReg <= 1'b0;
				regWrite <= 1'b0;
				aluOp <= 2'b01;
				halt <= 1'b0;
			end
			4'b0100: begin //branch less than
				extendLength <= 2'b01; // lowest 8 bits significant, sign ext remaining 8
				jumpBranch <= 3'b010; // op1 < r0 ; send to PCSrc
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b0;
				aluOp <= 2'b00;
				halt <= 1'b0;
			end
			4'b0101: begin //branch greater than
				extendLength <= 2'b01; // lowest 8 bits significant, sign ext remaining 8
				jumpBranch <= 3'b011; // op1>r0
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b0;
				aluOp <= 2'b00;
				halt <= 1'b0;
			end
			4'b0110: begin //branch equal to
				extendLength <= 2'b01; // lowest 8 bits significant, sign ext remaining 8
				jumpBranch <= 3'b100 ;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b0;
				aluOp <= 2'b00;
				halt <= 1'b0;
			end
			4'b1100: begin //jump
				extendLength <= 2'b10;
				jumpBranch <= 3'b001; //unconditional jump; send to PCSrc
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b0;
				aluOp <= 2'b00;
				halt <= 1'b0;
			end
			4'b1111: begin //halt
				extendLength <= 2'b00;
				jumpBranch <= 3'b000;
				memWrite <= 1'b0;
				memToReg <= 1'b0;
				regWrite <= 1'b0;
				aluOp <= 2'b00;
				halt <= 1'b1; //send halt signal
			end
		endcase
	end
	endmodule

// control unit test bench
`include "controlUnit.v"

module  controlUnit_test;
	
	reg [15:12] opcode; //4bits
	wire  memWrite, memToReg,regWrite,halt;
	wire[2:0] jumpBranch;
	wire[1:0] aluOp, extendLength;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," opcode = %b, extendLength = %b, jumpBranch = %b, memWrite = %b, memToReg = %b, aluOp = %b, regWrite = %b, halt = %b",opcode,extendLength,jumpBranch,memWrite,memToReg,aluOp,regWrite,halt);
	
		controlUnit myControlUnit(opcode,extendLength,jumpBranch,memWrite,memToReg,aluOp,regWrite,halt);
		
	initial
	begin
		opcode = 4'b0000;
		#20 opcode = 4'b1000;
		#20 opcode = 4'b1011;
		#20 opcode = 4'b0100;
		#20 opcode = 4'b0101;
		#20 opcode = 4'b0110;
		#20 opcode = 4'b1100;
		#20 opcode = 4'b1111;
		
	end
	
	initial
	begin
		#200 $finish;
	end
	endmodule
			
					
// Data Memory
module dataMemory(memWrite,clk,reset,addr,dataIN,dataOUT);
	
	input[15:0] dataIN; // data to write
	input[15:0] addr; // 16 bit address for read or write
	input clk, reset,memWrite;
	output reg[15:0] dataOUT;
	integer j;
	
	reg [7:0]data[0:65535]; // 2 to the 16th power memort locations. 8 bits wide
	
	always@(posedge clk or negedge reset)
		begin
			if(!reset)
				begin // set all memory to 0, 1 byte at a time.
					for(j=2;j<65535;j= 1+j) // start at 2, 0 and 1 are initialized
						begin			 // to a specific value per Professor's request
							data[j] <= 8'b0;
						end
					data[0] <= 8'hAB;
					data[1] <= 8'h99; // first 2 bytes set to specific value
				end
			else if(memWrite==1'b1) // if memWrite from CU is asserted..
				begin // permission to right
					data[addr[15:8]] <= dataIN[15:8]; //little endian; byte 2
					data[addr[7:0]] <= dataIN[7:0]; // byte 1
						
				end
			end //end if
	always@(*)
		begin
			dataOUT[7:0]=data[addr[7:0]];  // send data @ addr to mux DATAOUT
			dataOUT[15:8]=data[addr[15:8]];
		end
	endmodule

	
	// Data memory test bench
`include "dataMemory.v"

module dataMemory_test;
	
	reg[15:0] dataIN; // data to write
	reg[15:0] addr; // 16 bit address for read or write
	reg clk, reset,memWrite;
	wire[15:0] dataOUT;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," clk=%b, memWrite=%b, reset=%b, addr=%b, dataIN=%b ,dataOUT=%b " , memWrite,clk,reset,addr,dataIN,dataOUT);
	
		dataMemory myDataMemory(memWrite,clk,reset,addr,dataIN,dataOUT);
		
	initial
	begin
		#10
		clk = 0; 
		reset = 1; 
		dataIN = 0; 
		addr = 0;
		memWrite = 0;
		#10 reset = 0;
		#10 dataIN = 4;
		#20 memWrite = 1;
		
	end
	
	always #10 clk = ~clk;
	
	initial #100 $finish;
	
	endmodule
			
				
//Decode , Execute Buffer

module DECEXBuff(clk,memToRegIN,memWriteIN,regWriteIN,writeRegAddrIN
	,memToRegOUT,memWriteOUT,regWriteOUT,writeRegAddrOUT,
	signExtendIN,signExtendOUT,functCodeIN,functCodeOUT,regData1OUT,regData2OUT,
	regData1IN,regData2IN,aluOpIN, aluOpOUT,toAluCUIN,toAluCUOUT
);
	
	input clk;
	input[15:0] signExtendIN; // extened value to MUX before ALU in ex phase
	output reg[15:0] signExtendOUT; 
	input memToRegIN, memWriteIN, regWriteIN; //control signals from CU
	output reg memToRegOUT, memWriteOUT, regWriteOUT; 
	input[3:0] writeRegAddrIN; // from [11:8] direct from instruction
	output reg [3:0] writeRegAddrOUT;
	input[3:0] functCodeIN;  // funct code goes to forwarding unit
	output reg[3:0] functCodeOUT;
	input[3:0] toAluCUIN; // from instruction to ALU CU
	output reg [3:0] toAluCUOUT; 
	input [15:0] regData1IN,regData2IN; // direct from registerfile
	output reg [15:0] regData1OUT,regData2OUT; // goes to mux's before ALU
	input [1:0] aluOpIN ; // control signal to ALU control unit
	output reg [1:0] aluOpOUT; 
	
		always@(posedge clk) // on posedge clk, transmit data across phases via buffer
		begin
			signExtendOUT <= signExtendIN;
			memToRegOUT <= memToRegIN;
			memWriteOUT <= memWriteIN;
			regWriteOUT <= regWriteIN;
			writeRegAddrOUT <= writeRegAddrIN;
			functCodeOUT <= functCodeIN;
			toAluCUOUT <= toAluCUIN;
			regData1OUT <= regData1IN;
			regData2OUT <= regData2IN;
			aluOpOUT <= aluOpIN;
		end
	endmodule
			
// TB for Decode/Execute buffer

`include "DECEXBuff.v"

module DECEXBuff_test;
	
	reg clk;
	reg [15:0] signExtendIN; // extened value to MUX before ALU in ex phase
	wire[15:0] signExtendOUT; 
	reg  memToRegIN, memWriteIN, regWriteIN; //control signals from CU
	wire memToRegOUT, memWriteOUT, regWriteOUT; 
	reg[3:0] writeRegAddrIN; // from [11:8] direct from instruction
	wire [3:0] writeRegAddrOUT;
	reg[3:0] functCodeIN;  // funct code goes to forwarding unit
	wire[3:0] functCodeOUT;
	reg[3:0] toAluCUIN; // from instruction to ALU CU
	wire [3:0] toAluCUOUT; 
	reg [15:0] regData1IN,regData2IN; // direct from registerfile
	wire [15:0] regData1OUT,regData2OUT; // goes to mux's before ALU
	reg [1:0] aluOpIN ; // control signal to ALU control unit
	wire [1:0] aluOpOUT; 
	
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," aluOpIN= %b,aluOpOUT= %b, regData1OUT= %b,regData2OUT= %b,toAluCUIN= %b,toAluCUOUT= %b, regData1IN= %b,regData2IN= %b, functCodeIN= %b,functCodeOUT= %b,signExtendIN= %b, signExtendOUT= %b , memToRegIN= %b,memWriteIN= %b, regWriteIN= %b, memToRegOUT= %b, memWriteOUT= %b, regWriteOUT= %b,writeRegAddrIN= %b, writeRegAddrOUT= %b",
			aluOpIN,aluOpOUT, regData1OUT,regData2OUT,toAluCUIN,toAluCUOUT, regData1IN,regData2IN, functCodeIN,functCodeOUT,signExtendIN, signExtendOUT , memToRegIN, memWriteIN, regWriteIN, memToRegOUT, memWriteOUT, regWriteOUT,writeRegAddrIN, writeRegAddrOUT
		);
	
		DECEXBuff myDECEXBuff(clk,memToRegIN,memWriteIN,regWriteIN,writeRegAddrIN
	,memToRegOUT,memWriteOUT,regWriteOUT,writeRegAddrOUT,
	signExtendIN,signExtendOUT,functCodeIN,functCodeOUT,regData1OUT,regData2OUT,
	regData1IN,regData2IN,aluOpIN, aluOpOUT,toAluCUIN,toAluCUOUT);
		
	initial
	begin
		
		clk = 0;
		signExtendIN = 16'h0FFF;
		memToRegIN = 0;
		memWriteIN = 0; 
		regWriteIN = 0;
		writeRegAddrIN = 4; // write to register 4
		functCodeIN = 4'b1101;
		toAluCUIN = 4'b1110; // to ALU control unit, [3:0] from instruct, funct code.
		regData1IN = 16;
		regData2IN = 16;
		aluOpIN =0; // 2 bit signal from control unit to ALU control unit
		#10
		signExtendIN = 16'h0FFF;
		memToRegIN = 1;
		memWriteIN = 1; 
		regWriteIN = 1;
		writeRegAddrIN = 5; // write to register 4
		functCodeIN = 4'b0000;
		toAluCUIN = 4'b0011; // to ALU control unit, [3:0] from instruct, funct code.
		regData1IN = 4;
		regData2IN = 4;
		aluOpIN =1; // 2 bit signal from control unit to ALU control unit
		#10
		signExtendIN = 16'h1124;
		memToRegIN = 0;
		memWriteIN = 0; 
		regWriteIN = 0;
		writeRegAddrIN = 2; // write to register 4
		functCodeIN = 4'b1111;
		toAluCUIN = 4'b0101; // to ALU control unit, [3:0] from instruct, funct code.
		regData1IN = 5;
		regData2IN = 5;
		aluOpIN =1; // 2 bit signal from control unit to ALU control unit
		
	end
	
	always #5 clk = ~clk;
	
	initial
	begin
		#50 $finish;
	end
	endmodule
			
				
	// EX/MEM Buffer
module EXMEMBuff(clk,memToRegIN,memWriteIN,regWriteIN,writeDataIN,aluResultIN,writeReg0IN,writeRegAddrIN
	,memToRegOUT,memWriteOUT,regWriteOUT,writeDataOUT,aluResultOUT,writeReg0OUT,writeRegAddrOUT
);
	
	input clk;
	input[15:0] writeDataIN,aluResultIN,writeReg0IN;
	output reg[15:0] writeDataOUT,aluResultOUT,writeReg0OUT;
	input memToRegIN, memWriteIN, regWriteIN;
	output reg memToRegOUT, memWriteOUT, regWriteOUT;
	input[3:0] writeRegAddrIN;
	output reg [3:0] writeRegAddrOUT;
	
	always@(posedge clk) // on posedge clk, transmit data across phases via buffer
		begin
			writeDataOUT<=writeDataIN;
			aluResultOUT<=aluResultIN;
			writeReg0OUT<=writeReg0IN;
			memToRegOUT<=memToRegIN;
			memWriteOUT<=memWriteIN;
			regWriteOUT<=regWriteIN;
			writeRegAddrOUT<=writeRegAddrIN;
		end
	endmodule
	
// TB for EXMEMBUFF

`include "EXMEMBuff.v"

module EXMEMBuff_test;
	
	reg clk;
	reg[15:0] writeDataIN,aluResultIN,writeReg0IN; // data in 15 bits
	wire[15:0] writeDataOUT,aluResultOUT,writeReg0OUT; // data out
	reg memToRegIN, memWriteIN, regWriteIN; // control signals 
	wire memToRegOUT, memWriteOUT, regWriteOUT; // control signals out
	reg[3:0] writeRegAddrIN;
	wire [3:0] writeRegAddrOUT;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," memToRegIN = %b, memWriteIN = %b, regWriteIN = %b, writeDataIN = %b, aluResultIN = %b, writeReg0IN = %b, writeRegAddrIN = %b, memToRegOUT = %b,memWriteOUT = %b,regWriteOUT = %b,writeDataOUT = %b,aluResultOUT = %b,writeReg0OUT = %b,writeRegAddrOUT = %b",clk,memToRegIN,memWriteIN,regWriteIN,writeDataIN,aluResultIN,writeReg0IN,writeRegAddrIN
	,memToRegOUT,memWriteOUT,regWriteOUT,writeDataOUT,aluResultOUT,writeReg0OUT,writeRegAddrOUT);
	
		EXMEMBuff myEXMEMBUFF(clk,memToRegIN,memWriteIN,regWriteIN,writeDataIN,aluResultIN,writeReg0IN,writeRegAddrIN
	,memToRegOUT,memWriteOUT,regWriteOUT,writeDataOUT,aluResultOUT,writeReg0OUT,writeRegAddrOUT);
		
	initial
	begin
		
		clk = 0;
		writeDataIN = 0;
		aluResultIN = 0;
		writeReg0IN = 0;
		memToRegIN = 0;
		memWriteIN = 0;
		regWriteIN = 0;
		writeRegAddrIN = 0;
		#10
		writeDataIN = 16'b0000111100001111;
		aluResultIN = 16'b0000000000001010;
		writeReg0IN = 16'b1000100010111111;
		memToRegIN = 1;
		memWriteIN = 1;
		regWriteIN = 1;
		writeRegAddrIN = 3;
		#10
		writeDataIN = 16'b1111111100001111;
		aluResultIN = 16'b0110000000001010;
		writeReg0IN = 16'b1111100010111111;
		memToRegIN = 0;
		memWriteIN = 1;
		regWriteIN = 1;
		writeRegAddrIN = 2;
		
	end
	
	always #5 clk = ~clk;
	
	initial
	begin
		#40 $finish;
	end
	endmodule
			
				
//forwarding unit
module forwardingUnit(exInstruct,memInstruct,mux1,mux3);
	input [15:12] exInstruct,memInstruct; // 12-15 bits from ex and mem buffer instruct
	output reg [1:0] mux1,mux3; // signal sent to MUX1
	
	always@(*)
		begin
			
			if(memInstruct==4'b0000) // if WB phase is a Type-A (writes to OP1)
				begin
					if(exInstruct==4'b0000) // if EX phase source reads OP1
						begin
							mux1=2; // then use the forwarded data for ALU
						end
				end
			else if(memInstruct != 4'b0000) // if WB phase is type-b/c/d
				begin
					if(exInstruct==4'b0000) // if EX phase is Type-A
						begin
							mux1=0; // use direct from reg1 data
						end
				end
			else if(memInstruct == 4'b0000) // if WB phase is typeA
				begin
					if(exInstruct==4'b1011) // if EX phase is store
						begin
							mux1=2'b01; // use sign-extended
						end
				end
			else if(exInstruct==4'b1000) // if load
				begin
					mux1=2'b01; // use sign-extend
				end
			//else mux1=0;// default use direct from reg
	end
	endmodule						
	

// forwarding unit test bench	
`include "forwardingUnit.v"

module forwardingUnit_test;
	
	reg [15:12] exInstruct,memInstruct; // 11-15 bits from ex and mem buffer instruct
	reg [3:0] exFunct; // 0-3 bits from ex buffer funct code
	wire [1:0] mux1,mux3; // signal sent to MUX1
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," exBuffInstruct = %b, memBuffInstruct = %b, mux1Output = %b, mux3Output", exInstruct,memInstruct,mux1,mux3);
	
		forwardingUnit myForwardingUnit(exInstruct,memInstruct,mux1,mux3);
		
	initial
	begin
		exInstruct = 0; // Type A
		memInstruct = 0; // A
		#10
		exInstruct = 0;  // A
		memInstruct = 4; // c
		#10
		exInstruct = 11; // store
		memInstruct = 0; //A
		#10
		exInstruct = 11; //store
		memInstruct = 4; // branch
		#10
		exInstruct = 8;  //load
		memInstruct = 0; 
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
// Hazard Control unit

//if the opcode in Decode phase uses OP1 as its source
// and if the opcode in Execute phase is a Loadword 
// which always uses OP1 as its destination,
// then a hazard is present.
//
//Then a stall must occur in the fetch/decode phase 
//while the ex phase has time to load value to mux, near ALU


module hazardUnit(opcodeDecode,opcodeExecute,PCStall);
	
	input [15:12] opcodeDecode, opcodeExecute; // used 15-12 for clarity, as opposed to [3:0]
	output reg PCStall; // control signal to PC
	
	always@(*)
		begin
			if(opcodeExecute == 1000) // if execute phase is a load
				begin
					if(opcodeDecode == (0000 || 0100 || 0101|| 0110 || 1100))
						begin    // decode phase uses OP1 as source which is
								 // type-A and type-C
								 // StoreWord will be handled via forwarding unit
						PCStall = 1;
						end
				end
			else
				PCStall = 0; // avoid latch
		end
	endmodule	`include "hazardUnit.v"

module hazardUnit_test;
	
	reg [15:12] opcodeDecode, opcodeExecute; // used 15-12 for clarity, as opposed to [3:0]
	wire PCStall; // control signal to PC
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time,"opcode of decode phase = %b, opcode of execute phase = %b", opcodeDecode, opcodeExecute, PCStall);
	
		hazardUnit myHazardUnit(opcodeDecode, opcodeExecute, PCStall);
		
	initial
	begin
		opcodeDecode = 0000; //type a
		opcodeExecute = 1011; // non-load instruct
		#10 
		opcodeDecode = 0000; //type a
		opcodeExecute = 1000 ; // load
		#10 
		opcodeDecode = 1111; //type a
		opcodeExecute = 1000 ; // load
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
//Buffer1 Module(IFID)
module IFID(nxtInstrIn,curInstrIn,nxtInstrOut,curInstrOut,clk,flush);
input[15:0]nxtInstrIn,curInstrIn;
input clk,flush;
output[15:0]nxtInstrOut,curInstrOut;
reg[15:0]nxtInstrOut,curInstrOut;
always @(posedge clk)
    begin
    if(flush)
    begin
    nxtInstrOut<=0;
    curInstrOut<=0;
    end
    else
    begin
        nxtInstrOut<=nxtInstrIn;
            curInstrOut<=curInstrIn;
    end
    end
endmodule


//IFID testbench
`include"IFIDBuff.v"
module IFID_tb;
reg[15:0]nxtinstin,curinstin;
reg clk,flush;
wire[15:0]nxtinstout,curinstout;
initial
    $vcdpluson;
    initial
        $monitor($time," Flush signal = %b, \n\t Next Inst Input = %b,Next Inst Out= %b\n\t Current Inst Input= %b,Current Inst Out= %b",flush,nxtinstin,nxtinstout,curinstin,curinstout);
    IFID a1(nxtinstin,curinstin,nxtinstout,curinstout,clk,flush);
    initial
        begin
        nxtinstin=0;curinstin=0;clk=0;flush=0;
        #12 nxtinstin = 16'b0000111100001111;
        curinstin=16'b1111000011110000;
        #10 flush = 1;
        end 
    always #5 clk= ~clk;
    initial #200 $finish;
endmodule
				
			
//Instruction Memory Module
module InstructionMem(clk,reset,InstrOut,InstrIn,cur);
input [15:0] cur;
input clk, reset;
output [15:0] InstrOut;
reg [15:0] InstrOut; 
reg [15:0] i;
reg [15:0] IMem[0:26];
always@(posedge clk or negedge reset)
begin
if (!reset)
begin  //Instruction to address and content 
IMem[0]=16'b000000000101111; 
IMem[1]=16'b0000000100101110;
IMem[2]=16'b0000001100101100; 
IMem[3]=16'b0000001100101101;
IMem[4]=16'b0000010101100001;
IMem[5]=16'b0000000101010010;
IMem[6]=16'b0000000000001110;
IMem[7]=16'b0000001000111010;
IMem[8]=16'b0000010000101011;
IMem[9]=16'b0000010100111000;
IMem[10]=16'b0000010100101001;
IMem[11]=16'b0110011101000000;
IMem[12]=16'b0000101100011111;
IMem[13]=16'b0100011101010000;
IMem[14]=16'b0000101100101111;
IMem[15]=16'b0000011100100000;
IMem[17]=16'b0000001000011111;
IMem[18]=16'b0000001000011111;
IMem[19]=16'b1000100000001001;
IMem[20]=16'b0000100010001111;
IMem[21]=16'b1011100000101001;
IMem[22]=16'b1000101000101001;
IMem[23]=16'b0000110011001111;
IMem[24]=16'b0000110111011110;
IMem[25]=16'b0000110011011111;
IMem[26]=16'b1110101111001111;
for(i=27;i<=30;i=i+1)
    begin
    IMem[i]<=16'b0000000000000000;
    end
    end
    InstrOut[15:8]=IMem[cur];
    InstrOut[7:0]=IMem[cur+1];
    end
endmodule

//mux1
module mux1(in1,in2,in3,forward,out);
	input [15:0] in1,in2,in3;
	input [1:0] forward; // signal from forwarding unit
	output reg[15:0] out;

	always@(*)
		begin
			if(forward==0)
				out=in1; // from Data1 Regfile
			else if(forward==1)
				out=in2; // from sign_extend
			else if(forward==2)
				out=in3; // from forwarding unit
			end
			
		endmodule


// mux 1 TB		
`include "mux1.v"

module mux1_test;
	
	reg [15:0] in1,in2,in3;
	reg [1:0] forward; // signal from forwarding unit
	wire[15:0] out;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," in1 = %b, in2 = %b, in3 = %b, forward = %b, out = %b", in1,in2,in3,forward,out);
	
		mux1 myMux1(in1,in2,in3,forward,out);
		
	initial
	begin
		in1 = 15;
		in2 = 4;
		in3 = 214;
		forward = 0;
		#10
		forward = 1;
		#10
		forward = 2;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
// MUX 3
// decides which data to write back to MEM
// a forwarded value, or 1 directly from OP1
module mux3(dataFromOP1,dataForwarded, signal, out);
	
	input [15:0] dataFromOP1, dataForwarded;
	input signal; // signal from Control Unit
	output reg [15:0] out; // data out to memory
	
	always@(*)
		begin
			if(signal == 1)
				out = dataForwarded;
			else
				out = dataFromOP1;
		end
	endmodule
	
	
	// MUX 3 test bench+
`include "mux3.v"

module mux3_test;
	
	reg [15:0] dataFromOP1, dataForwarded;
	reg signal; // signal from Control Unit
	wire [15:0] out; // data out to memory
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time,"  dataFromOP1= %b, dataForwarded= %b,signal= %b,out= %b", dataFromOP1, dataForwarded,signal,out);
	
		mux3 myMux3(dataFromOP1,dataForwarded, signal, out);
		
	initial
	begin
		#5
		dataFromOP1 = 16'hFFFF;
		dataForwarded = 16'h0001;
		signal = 0;
		#10
		dataFromOP1 = 16'hAAAA;
		dataForwarded = 16'hFFF0;
		signal = 0;
		#10
		dataFromOP1 = 16'h0000;
		dataForwarded = 16'h0001;
		signal = 0;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
				
	
				
// MUX 4
// mux in writeback phase
// decides which value to use to write back to registerfile
// Either Data read in from memory
// or data from ALU result

module mux4(dataFromMem, dataFromAlu, memToRegSignal,out);
	
	input [15:0] dataFromMem, dataFromAlu; 
	input memToRegSignal; // signal from Control Unit
	output reg [15:0] out; // data out to RegisterFile
	
	always@(*)
		begin
			if(memToRegSignal == 1)
				out = dataFromMem;
			else
				out = dataFromAlu;
		end
	endmodule

	
	// mux4 test bench
`include "mux4.v"

module mux4_test;
	
	reg [15:0] dataFromMem, dataFromAlu; 
	reg memToRegSignal; // signal from Control Unit
	wire [15:0] out; // data out to RegisterFile
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time,"  data_from_memory = %b, data_from_ALU = %b, memToReg = %b, data_OUT", dataFromMem, dataFromAlu,memToRegSignal,out);
	
		mux4 myMux4(dataFromMem,dataFromAlu,memToRegSignal,out);
		
	initial
	begin
		#5
		dataFromMem = 16'hFFFF;
		dataFromAlu = 16'h0001;
		memToRegSignal = 0;
		#10
		dataFromMem = 16'hAAAA;
		dataFromAlu = 16'hFFF0;
		memToRegSignal = 0;
		#10
		dataFromMem = 16'h0000;
		dataFromAlu = 16'h0001;
		memToRegSignal = 0;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
//Mux5
module mux5( sel, c_address, j_address, address);
    input 	sel;
    input   [15:0]	c_address, j_address;
    output	[15:0]	address;
    reg [15:0] address;
    always@(*) 
    begin
    if (sel==1'b1)// pcsrc signal
        address=j_address; //from adder2
    else
        address=c_address;//from adder1
    end
endmodule
				
	
//*****PC Module (pc.v)*****/
module ProgramCounter(clk,reset,stall,m_address,MuxAddress,stallInstr,hold);
    input   clk,reset,stall,hold;
    input   [15:0]	MuxAddress;
    output  [15:0] 	m_address;
    output  stallInstr;
    reg [15:0] m_address;
    reg stallInstr;
    always@(posedge clk or negedge reset)
    begin
        if (stall==1'b1);
    begin
        stallInstr=1'b1;
    end
    if(!reset)
        m_address=16'h00;
    else
        m_address=MuxAddress;
    end
endmodule


//PC_TB.V USED TO TEST THE PC MODULE
`include "pc.v"
module pc_tb;
wire[15:0]out;
reg stall,hold;
reg[9:0]Mux,Haz;
reg[7:0]Mem[0:65535];
ProgramCounter g1(stall,hold,Mem,out,Mux,Haz);
initial 
begin
	stall=1'b0;
	hold=1'b0;
	Mem=0;
	Mux=9'b000000010;
	Haz=9'b000000000;
	#5 Mem[2]=1;
	#10 hold=1'b1;
	#30 stall=1'b1;
end
initial 
begin
#100 $finish;
end
endmodule


//Stage1 Module

`include "programCounter.v"
`include "adder1.v"
`include "mux5.v"
`include "InstructionMem.v"
module stage1(hold,brnjump,reset,clk,nxt,InstrOut,AdderOut,stall);
input stall,brnjump,reset,clk,hold
input[15:0]nxt;
output[15:0]InstrOut,AdderOut;
wire[15:0]nxt,AddressToPC,AddressToInstr,AdderOut;
wire stallInstr;
wire[15:0]Top;
Mux1 g1(brnjump,nxt,Top,AddressToPC);
ProgramCounter g2(clk,reset,hold,stall,AddressToInstr,AddressToPC,stallInstr);
InstructionMem g3(clk,reset,InstructionOut,AddressToInstr,stallInstr);
adder g4(AddressToInstr,Top);
assign AdderOut=Top;
endmodule

//stage1 testbench
`include "phase1.v"
module test;
reg hold,brnjump,reset,clk,stall;
reg[15:0]nxt;
wire[15:0]InstrOut,AdderOut;
Stage1 g1(hold,brnjump,reset,clk,nxt,InstrOut,AdderOut,stall);
initial begin
clk=1'b0;
end
always #5 clk=~clk;
initial begin
reset=1'b0;
brnjump=1'b0;
stall=1'b0;
halt=1'b0;
#20 reset=1'b1; 
nxt=16'b000000000100000;
#100 brnjump=1'b0;
end
initial
$vcdpluson;
initial
$monitor($time,"      %h=InstructionOut  %b=AdderOut  %b=reset %b=clk   %b=Address to Instruction",InstrOut,AdderOut,reset,clk,Stage1.AddressToInstr);
initial
#150 $finish;
endmodule

//Phase 2

`include "registerFile.v"
`include "hazardUnit.v"
`include "adder2.v"
`include "controlUnit.v"
`include "compare.v"
`include "signExtend1.v"
`include "DECEXBuff.v"
`include "shiftLeft.v"

module phase2(instruction,originalAddr,signExtendOUT ,memToRegOUT,memWriteOUT ,regWriteOUT,writeRegAddrOUT ,
	functCodeOUT,toAluCUOUT,regData1OUT ,regData2OUT ,aluOpOUT,stall,flush,PCSrc,addressOut,opcode_ex,address_from_4,
	data_from_4,data_from_ALU,regWrite_from_4,halt
);

input [15:0] instruction,originalAddr; // 16 bit instruction for phase 1 buffer.
output reg [15:0] addressOut ; // back to phase1 MUX
output reg [15:0] writeRegAddrOUT; // go phase 3 buffer, loops back to regFile
output reg [3:0] functCodeOUT ; // to phase 3 buffer, funct
output reg [1:0] aluOpOUT; // phase 3 buffer
output reg [3:0] toAluCUOUT; // phase 3 buffer to ALU funct
output reg PCSrc,flush,stall; // back to phase1 signals
output reg[15:0] signExtendOUT, regData1OUT,regData2OUT; // out to phase 3 buffer
output reg memToRegOUT,memWriteOUT,regWriteOUT; // control signals to phase 3 buffer;
output reg halt;
wire [15:0] wireExtend, wireShift,wireData0,wireData1,wireData2;
wire [1:0] wireExtendLength, wireMemWrite, wireMemToReg,wireRegWrite,wireAluOp;
wire [2:0] wireJumpBranch;

// inputs/outputs that connect from phase 3 or 4, that return back
input [3:0] opcode_ex; // from phase 3, ex, back track
input [3:0] address_from_4; // address from phase 4, back track
input [15:0] data_from_4; // data from WB;
input [15:0] data_from_ALU; // data from ALU, reg0 ; multiplication and division
input regWrite_from_4; // regWrite from last buffer

// done connected
controlUnit t0(instruction[15:12],wireExtendLength,wireJumpBranch,wireMemWrite,wireMemToReg,wireAluOp,wireRegWrite,halt);

registerFile t1(clk,reset,instruction[11:8],instruction[7:5],address_from_4,wireData0,wireData1,wireData2,data_from_ALU,data_from_4,regWrite_from_4);
// done connected
signExtend1 t2(instruction[3:0],instruction[7:0],instruction[11:0],wireExtendLength,wireExtend); // done connected

shiftLeft t3(wireExtend, wireShift); // done connected

adder2 t4(originalAddr,wireShift,addressOut); // done connected

hazardUnit t5(instruction[15:12],opcode_ex,stall); // done connected

compare t6(wireJumpBranch,wireData0,wireData1,PCSrc); //done connected

DECEXBuff t7(clk,wireMemToReg,wireMemWrite,wireRegWrite,writeRegAddrIN
	,memToRegOUT,memWriteOUT,regWriteOUT,writeRegAddrOUT,
	wireExtend,signExtendOUT,instruction[3:0],functCodeOUT,regData1OUT,regData2OUT,
	wireData1,wireData2,wireAluOp, aluOpOUT,instruction[3:0],toAluCUOUT);
	



endmodule



//test phase 2

`include "phase2.v"

module phase2_test;
	
	reg [15:0] instruction,originalAddr; // 16 bit instruction for phase 1 buffer.
	wire [15:0] addressOut ; // back to phase1 MUX
	wire [15:0] writeRegAddrOUT; // go phase 3 buffer, loops back to regFile
	wire [3:0] functCodeOUT ; // to phase 3 buffer, funct
	wire [1:0] aluOpOUT; // phase 3 buffer
	wire [3:0] toAluCUOUT; // phase 3 buffer to ALU funct
	wire PCSrc,flush,stall; // back to phase1 signals
	wire[15:0] signExtendOUT, regData1OUT,regData2OUT; // out to phase 3 buffer
	wire memToRegOUT,memWriteOUT,regWriteOUT; // control signals to phase 3 buffer;
	wire halt;
	
	// inputs/outputs that connect from phase 3 or 4, that return back
	reg [3:0] opcode_ex; // from phase 3, ex, back track
	reg [3:0] address_from_4; // address from phase 4, back track
	reg [15:0] data_from_4; // data from WB;
	reg [15:0] data_from_ALU; // data from ALU, reg0 ; multiplication and division
	reg regWrite_from_4; // regWrite from last buffer
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," instruction= %b,",instruction);
	
		phase2 myLeftShift(instruction,originalAddr,signExtendOUT ,memToRegOUT,memWriteOUT ,regWriteOUT,writeRegAddrOUT ,
	functCodeOUT,toAluCUOUT,regData1OUT ,regData2OUT ,aluOpOUT,stall,flush,PCSrc,addressOut,opcode_ex,address_from_4,
	data_from_4,data_from_ALU,regWrite_from_4,halt);
		
	initial
	begin
		
		#5
		instruction= 16'b1111000100110000;
		originalAddr=0;
		opcode_ex=0;
		address_from_4=02;
		data_from_4= 5;
		data_from_ALU=0;
		regWrite_from_4=1;
		
		
		
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
				
//PC Module
module ProgramCounter(clk,reset,stall,hold,,m_address,MuxAddress,stallInstr);
    input   clk,reset,stall,hold;
    input   [15:0]	MuxAddress;
    output  [15:0] 	m_address;
    output  stallInstr;
    reg [15:0] m_address;
    reg stallInstr;
    always@(posedge clk or negedge reset)
    begin
        if (stall==1'b1); //NOP performed 
    begin
        stallInstr=1'b1;
    end
    else if (!reset) //reset instruction 
        m_address=16'h00;
    else
        m_address=MuxAddress; // normal operation 
    end
endmodule
	
	
//Pc testbench
`include"programCounter.v"
module pc_tb;
wire[15:0]out;
reg stall,hold
reg[9:0]Mux,Haz
reg[7:0]Mem[0:65535];
ProgramCounter g1(stall,hold,Mem,out,Mux,Haz);
initialbegin
stall=1'b0;
hold=1'b0;
Mem=0;
Mux=9'b000000010;
Haz=9'b000000000;
#5 Mem[2]=1;
#10 hold=1'b1;
#30 halt=1'b1;
#40 initial
#100 $finish;
end
endmodule

//Register File in Decode Phase
module registerFile(clk,reset,read1,read2,writeAddr,out0,out1,out2,write0,write1,regWrite);
	input clk,reset;
	input [3:0] writeAddr; //register# for writing data
	input [15:0] write0,write1; // data written in
	input [3:0] read2,read1; // read in register number
	input regWrite; // asserted for writing to register 1
	output [15:0] out0,out1,out2; // output register data 0-2
	reg[15:0] out0,out1,out2;
	reg[15:0] registerFile[0:15]; // 16 16-bit registers in the register file
	
	always@(posedge clk or negedge reset)
	begin
		if(reset) // on a reset signal
			begin // reset registers to Professor's instructions
				registerFile[1]<= 16'h0F00;
				registerFile[2]<= 16'h0050;
				registerFile[3]<= 16'hFF0F;
				registerFile[4]<= 16'hF0FF;
				registerFile[5]<= 16'h0040;
				registerFile[6]<= 16'h0024;
				registerFile[7]<= 16'h00FF;
				registerFile[8]<= 16'hAAAA;
				registerFile[9]<= 16'h0000;
				registerFile[10]<= 16'h0000;
				registerFile[11]<= 16'h0000;
				registerFile[12]<= 16'hFFFF;
				registerFile[13]<= 16'h0000;
				registerFile[14]<= 16'h0000;
				registerFile[15]<= 16'h0000;
				registerFile[16]<= 16'h0000;
				registerFile[0]<= 16'h0000;
			end
		else if(regWrite == 1'b1) // if RegWrite is asserted
			begin
			registerFile[0]<=write0;  //always accept register 0 data
			registerFile[writeAddr]<=write1; // push data from WB stage to address
			end
	end
	always@(*) //read
		begin
		out1=registerFile[read1]; //[11:8]
		out2=registerFile[read2];// [7:4]
		out0=registerFile[0];
			
		end
	endmodule
	
	
	// registerfile test bench
	`include "registerFile.v"

module registerFile_test;
	
	reg clk,reset;
	reg [3:0] writeAddr;
	reg [15:0] write0,write1;
	reg [3:0] read2,read1;
	reg regWrite; 
	wire [15:0] out1,out2,out0;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time," clk = %b,reset = %b,read1 = %b,read2 = %b,writeAddr = %b, out0 = %b,out1 = %b,out2 = %b,write0 = %b,write1 = %b,regWrite = %b",clk,reset,read1,read2,writeAddr,out0,out1,out2,write0,write1,regWrite);
	
		registerFile myRegisterFile(clk,reset,read1,read2,writeAddr,out0,out1,out2,write0,write1,regWrite);
		
	initial
	begin
		reset=1;
		#10
		reset=0;
		read1=4'b0001; // read register 1
		read2=4'b0101; // read register 5
		writeAddr=2; //write to address 2
		write1= 16'b1111000011110000; // data to be written to writeAddr
		write0= 16'b0000000000000001; // date to be written to 0
		clk=1;
		#10
		clk=0;
		#10
		clk=1;
		#10
		clk=0;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
				
		
// shiftLeft Module
module shiftLeft(signextend, out);
	
	input [15:0] signextend;
	output [15:0] out;
	
	assign out = signextend << 2;
	
	endmodule			 
	

// shift left test bench	
`include "shiftLeft.v"

module shiftLeft_test;
	
	reg [15:0] signextend;
	wire[15:0] out;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time,"input = %b, output = %b", signextend,out);
	
		shiftLeft myLeftShift(signextend, out);
		
	initial
	begin
		signextend = 0;
		#10 signextend = 16'b1111000011110000;
		#10 signextend = 16'b1111000011110011;
		#10 signextend = 16'b0111000011110010;
		#10 signextend = 16'b0011000011110001;
	end
	
	initial
	begin
		#100 $finish;
	end
	endmodule
			
				
//Sign_extender Module

module signExtend1(in1,in2,in3,extendLength,out);
	input [3:0]in1;
	input [7:0]in2;
	input [11:0]in3;
	input [1:0] extendLength;
	output [15:0] out;
	reg [15:0] out;
	
	always@(*)
		begin
			if(extendLength == 2'b00) // depending on control signal extendLength
				if(in1[3]==1'b0) // if most significant bit is 0
					out = {12'b0,in1}; //extend that 0 to the 15th bit
				else
					out = {12'b111111111111,in1};  // else extend 1's
			else if(extendLength == 2'b01) // depending on control signal extendLength
				if(in2[7]==1'b0) // if most significant bit is 0
					out = {8'b0,in2}; //extend that 0 to the 15th bit
				else
					out = {8'b11111111,in2};  // else extend 1's
			else if(extendLength == 2'b10) // depending on control signal extendLength
				if(in3[11]==1'b0) // if most significant bit is 0
					out = {4'b0,in3}; //extend that 0 to the 15th bit
				else
					out = {4'b1111,in3};  // else extend 1's
		end
	endmodule`include "signExtend1.v"

	
	
	// sign-extend test bench
module signExtend1_test;
	
	reg [3:0]in1;
	reg [7:0]in2;
	reg [11:0]in3;
	reg [1:0] extendLength;
	wire [15:0] out;
	
	initial
		$vcdpluson;
	
	initial
		$monitor($time,"   %b ", out);
	
		signExtend1 mySignExtend1(in1,in2,in3,extendLength,out);
		
	initial
	begin
		in1= 4'b1001;
		in2= 8'b10000000;
		in3= 12'b100000000000;
		extendLength = 2'b00;
		#10
		extendLength = 2'b01;
		#10
		extendLength = 2'b10;
		#10
		in1= 4'b0001;
		in2= 8'b00000000;
		in3= 12'b000000000000;
		extendLength = 2'b00;
		#10
		extendLength = 2'b01;
		#10
		extendLength = 2'b10;
	end
	
	initial
	begin
		#200 $finish;
	end
	endmodule
			
				
// adder2, test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 15:54 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 +2ByteAddr = 0000000000000000, offset = 0000000000000000, out = 0000000000000000, decimal=     0
                  10 +2ByteAddr = 0000000000000000, offset = 0000000000000001, out = 0000000000000001, decimal=     1
                  20 +2ByteAddr = 0000000000000001, offset = 0000000000000001, out = 0000000000000010, decimal=     2
                  30 +2ByteAddr = 0000000000000100, offset = 0000000000000100, out = 0000000000001000, decimal=     8
                  40 +2ByteAddr = 0000000000001000, offset = 0000000000010000, out = 0000000000011000, decimal=    24
                  50 +2ByteAddr = 0000000000100000, offset = 0000000001000000, out = 0000000001100000, decimal=    96
$finish called from file "adder2_test.v", line 28.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 15:54:37 2016
				
	
// ALU test results
	Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 15:56 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0  in1 = 0000000000000010, in2 = 0000000000000010, aluop = 1111,out = 0000000000000100, outReg0 = xxxxxxxxxxxxxxxx, overflow = 0 
                  10  in1 = 0000000000000010, in2 = 0000000000000010, aluop = 1110,out = 0000000000000000, outReg0 = xxxxxxxxxxxxxxxx, overflow = 0 
                  20  in1 = 0000000000000010, in2 = 0000000000000010, aluop = 0000,out = 0000000000000000, outReg0 = 0000000000000000, overflow = 0 
                  30  in1 = 0111111111111111, in2 = 0000001111111111, aluop = 0001,out = 0111110000000001, outReg0 = 0000000111111111, overflow = 0 
                  40  in1 = 0111111111111111, in2 = 0000001111111111, aluop = 0010,out = 0000000000100000, outReg0 = 0000000000011111, overflow = 0 
                  50  in1 = 0000100011111111, in2 = 0000000000000010, aluop = 1010,out = 0010001111111100, outReg0 = 0000000000011111, overflow = 0 
                  60  in1 = 1000000000000010, in2 = 0000000000000010, aluop = 1011,out = 1110000000000000, outReg0 = 0000000000011111, overflow = 0 
                  70  in1 = 1000000000000010, in2 = 0000000000000010, aluop = 1000,out = 0000000000001010, outReg0 = 0000000000011111, overflow = 0 
                  80  in1 = 1000000000000010, in2 = 0000000000000010, aluop = 1001,out = 1010000000000000, outReg0 = 0000000000011111, overflow = 0 
$finish called from file "alu_test.v", line 52.
$finish at simulation time                  200
           V C S   S i m u l a t i o n   R e p o r t 
Time: 200
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 15:56:02 2016


// ALU CU test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 15:57 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 aluop = 00, funct = 0111, out = 0000
                   5 aluop = 10, funct = 0111, out = 0000
                  10 aluop = 10, funct = 1001, out = 1001
                  15 aluop = 10, funct = 1010, out = 1010
                  20 aluop = 00, funct = 1010, out = 0000
                  25 aluop = 01, funct = 0000, out = 1111
$finish called from file "aluCU_test.v", line 41.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 15:57:19 2016

// compare test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 15:58 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 jumpBranch = 000, reg0 = 0000000000000001, reg1 = 0000000000000010, pcSrc = 0
                  10 jumpBranch = 001, reg0 = 0000000000000001, reg1 = 0000000000000010, pcSrc = 0
                  20 jumpBranch = 010, reg0 = 0000000000000001, reg1 = 0000000000000010, pcSrc = 0
                  30 jumpBranch = 011, reg0 = 0000000000000001, reg1 = 0000000000000010, pcSrc = 1
                  40 jumpBranch = 100, reg0 = 0000000000000001, reg1 = 0000000000000010, pcSrc = 0
$finish called from file "compare_test.v", line 34.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 15:58:07 2016

// control unit test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 15:59 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 opcode = 0000, extendLength = 00, jumpBranch = 000, memWrite = 0, memToReg = 0, aluOp = 10, regWrite = 1, halt = 0
                  20 opcode = 1000, extendLength = 00, jumpBranch = 000, memWrite = 0, memToReg = 1, aluOp = 01, regWrite = 1, halt = 0
                  40 opcode = 1011, extendLength = 00, jumpBranch = 000, memWrite = 1, memToReg = 0, aluOp = 01, regWrite = 0, halt = 0
                  60 opcode = 0100, extendLength = 01, jumpBranch = 010, memWrite = 0, memToReg = 0, aluOp = 00, regWrite = 0, halt = 0
                  80 opcode = 0101, extendLength = 01, jumpBranch = 011, memWrite = 0, memToReg = 0, aluOp = 00, regWrite = 0, halt = 0
                 100 opcode = 0110, extendLength = 01, jumpBranch = 100, memWrite = 0, memToReg = 0, aluOp = 00, regWrite = 0, halt = 0
                 120 opcode = 1100, extendLength = 10, jumpBranch = 001, memWrite = 0, memToReg = 0, aluOp = 00, regWrite = 0, halt = 0
                 140 opcode = 1111, extendLength = 00, jumpBranch = 000, memWrite = 0, memToReg = 0, aluOp = 00, regWrite = 0, halt = 1
$finish called from file "controlUnit_test.v", line 33.
$finish at simulation time                  200
           V C S   S i m u l a t i o n   R e p o r t 
Time: 200
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 15:59:15 2016

// data memory test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:00 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 clk=x, memWrite=x, reset=x, addr=xxxxxxxxxxxxxxxx, dataIN=xxxxxxxxxxxxxxxx ,dataOUT=xxxxxxxxxxxxxxxx 
                  10 clk=0, memWrite=1, reset=1, addr=0000000000000000, dataIN=0000000000000000 ,dataOUT=xxxxxxxxxxxxxxxx 
                  20 clk=0, memWrite=0, reset=0, addr=0000000000000000, dataIN=0000000000000000 ,dataOUT=0000100000001000 
                  30 clk=0, memWrite=1, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
                  40 clk=0, memWrite=0, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
                  50 clk=1, memWrite=1, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
                  60 clk=1, memWrite=0, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
                  70 clk=1, memWrite=1, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
                  80 clk=1, memWrite=0, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
                  90 clk=1, memWrite=1, reset=0, addr=0000000000000000, dataIN=0000000000000100 ,dataOUT=0000100000001000 
$finish called from file "dataMemory_test.v", line 34.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.350 seconds;       Data structure size:   3.8Mb
Sun May  1 16:00:22 2016

// DEX EX BUFFER test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 17:00 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 aluOpIN= 00,aluOpOUT= xx, regData1OUT= xxxxxxxxxxxxxxxx,regData2OUT= xxxxxxxxxxxxxxxx,toAluCUIN= 1110,toAluCUOUT= xxxx, regData1IN= 0000000000010000,regData2IN= 0000000000010000, functCodeIN= 1101,functCodeOUT= xxxx,signExtendIN= 0000111111111111, signExtendOUT= xxxxxxxxxxxxxxxx , memToRegIN= 0,memWriteIN= 0, regWriteIN= 0, memToRegOUT= x, memWriteOUT= x, regWriteOUT= x,writeRegAddrIN= 0100, writeRegAddrOUT= xxxx
                   5 aluOpIN= 00,aluOpOUT= 00, regData1OUT= 0000000000010000,regData2OUT= 0000000000010000,toAluCUIN= 1110,toAluCUOUT= 1110, regData1IN= 0000000000010000,regData2IN= 0000000000010000, functCodeIN= 1101,functCodeOUT= 1101,signExtendIN= 0000111111111111, signExtendOUT= 0000111111111111 , memToRegIN= 0,memWriteIN= 0, regWriteIN= 0, memToRegOUT= 0, memWriteOUT= 0, regWriteOUT= 0,writeRegAddrIN= 0100, writeRegAddrOUT= 0100
                  10 aluOpIN= 01,aluOpOUT= 00, regData1OUT= 0000000000010000,regData2OUT= 0000000000010000,toAluCUIN= 0011,toAluCUOUT= 1110, regData1IN= 0000000000000100,regData2IN= 0000000000000100, functCodeIN= 0000,functCodeOUT= 1101,signExtendIN= 0000111111111111, signExtendOUT= 0000111111111111 , memToRegIN= 1,memWriteIN= 1, regWriteIN= 1, memToRegOUT= 0, memWriteOUT= 0, regWriteOUT= 0,writeRegAddrIN= 0101, writeRegAddrOUT= 0100
                  15 aluOpIN= 01,aluOpOUT= 01, regData1OUT= 0000000000000100,regData2OUT= 0000000000000100,toAluCUIN= 0011,toAluCUOUT= 0011, regData1IN= 0000000000000100,regData2IN= 0000000000000100, functCodeIN= 0000,functCodeOUT= 0000,signExtendIN= 0000111111111111, signExtendOUT= 0000111111111111 , memToRegIN= 1,memWriteIN= 1, regWriteIN= 1, memToRegOUT= 1, memWriteOUT= 1, regWriteOUT= 1,writeRegAddrIN= 0101, writeRegAddrOUT= 0101
                  20 aluOpIN= 01,aluOpOUT= 01, regData1OUT= 0000000000000100,regData2OUT= 0000000000000100,toAluCUIN= 0101,toAluCUOUT= 0011, regData1IN= 0000000000000101,regData2IN= 0000000000000101, functCodeIN= 1111,functCodeOUT= 0000,signExtendIN= 0001000100100100, signExtendOUT= 0000111111111111 , memToRegIN= 0,memWriteIN= 0, regWriteIN= 0, memToRegOUT= 1, memWriteOUT= 1, regWriteOUT= 1,writeRegAddrIN= 0010, writeRegAddrOUT= 0101
                  25 aluOpIN= 01,aluOpOUT= 01, regData1OUT= 0000000000000101,regData2OUT= 0000000000000101,toAluCUIN= 0101,toAluCUOUT= 0101, regData1IN= 0000000000000101,regData2IN= 0000000000000101, functCodeIN= 1111,functCodeOUT= 1111,signExtendIN= 0001000100100100, signExtendOUT= 0001000100100100 , memToRegIN= 0,memWriteIN= 0, regWriteIN= 0, memToRegOUT= 0, memWriteOUT= 0, regWriteOUT= 0,writeRegAddrIN= 0010, writeRegAddrOUT= 0010
$finish called from file "DECEXBuff_test.v", line 80.
$finish at simulation time                   50
           V C S   S i m u l a t i o n   R e p o r t 
Time: 50
CPU Time:      0.330 seconds;       Data structure size:   0.0Mb
Sun May  1 17:00:30 2016

// EX MEM BUFF test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:15 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 memToRegIN = 0, memWriteIN = 0, regWriteIN = 0, writeDataIN = 0, aluResultIN = 0000000000000000, writeReg0IN = 0000000000000000, writeRegAddrIN = 0000000000000000, memToRegOUT = 0000,memWriteOUT = x,regWriteOUT = x,writeDataOUT = x,aluResultOUT = xxxxxxxxxxxxxxxx,writeReg0OUT = xxxxxxxxxxxxxxxx,writeRegAddrOUT = xxxxxxxxxxxxxxxx x
                   5 memToRegIN = 1, memWriteIN = 0, regWriteIN = 0, writeDataIN = 0, aluResultIN = 0000000000000000, writeReg0IN = 0000000000000000, writeRegAddrIN = 0000000000000000, memToRegOUT = 0000,memWriteOUT = 0,regWriteOUT = 0,writeDataOUT = 0,aluResultOUT = 0000000000000000,writeReg0OUT = 0000000000000000,writeRegAddrOUT = 0000000000000000 0
                  10 memToRegIN = 0, memWriteIN = 1, regWriteIN = 1, writeDataIN = 1, aluResultIN = 0000111100001111, writeReg0IN = 0000000000001010, writeRegAddrIN = 1000100010111111, memToRegOUT = 0011,memWriteOUT = 0,regWriteOUT = 0,writeDataOUT = 0,aluResultOUT = 0000000000000000,writeReg0OUT = 0000000000000000,writeRegAddrOUT = 0000000000000000 0
                  15 memToRegIN = 1, memWriteIN = 1, regWriteIN = 1, writeDataIN = 1, aluResultIN = 0000111100001111, writeReg0IN = 0000000000001010, writeRegAddrIN = 1000100010111111, memToRegOUT = 0011,memWriteOUT = 1,regWriteOUT = 1,writeDataOUT = 1,aluResultOUT = 0000111100001111,writeReg0OUT = 0000000000001010,writeRegAddrOUT = 1000100010111111 3
                  20 memToRegIN = 0, memWriteIN = 0, regWriteIN = 1, writeDataIN = 1, aluResultIN = 1111111100001111, writeReg0IN = 0110000000001010, writeRegAddrIN = 1111100010111111, memToRegOUT = 0010,memWriteOUT = 1,regWriteOUT = 1,writeDataOUT = 1,aluResultOUT = 0000111100001111,writeReg0OUT = 0000000000001010,writeRegAddrOUT = 1000100010111111 3
                  25 memToRegIN = 1, memWriteIN = 0, regWriteIN = 1, writeDataIN = 1, aluResultIN = 1111111100001111, writeReg0IN = 0110000000001010, writeRegAddrIN = 1111100010111111, memToRegOUT = 0010,memWriteOUT = 0,regWriteOUT = 1,writeDataOUT = 1,aluResultOUT = 1111111100001111,writeReg0OUT = 0110000000001010,writeRegAddrOUT = 1111100010111111 2
                  30 memToRegIN = 0, memWriteIN = 0, regWriteIN = 1, writeDataIN = 1, aluResultIN = 1111111100001111, writeReg0IN = 0110000000001010, writeRegAddrIN = 1111100010111111, memToRegOUT = 0010,memWriteOUT = 0,regWriteOUT = 1,writeDataOUT = 1,aluResultOUT = 1111111100001111,writeReg0OUT = 0110000000001010,writeRegAddrOUT = 1111100010111111 2
                  35 memToRegIN = 1, memWriteIN = 0, regWriteIN = 1, writeDataIN = 1, aluResultIN = 1111111100001111, writeReg0IN = 0110000000001010, writeRegAddrIN = 1111100010111111, memToRegOUT = 0010,memWriteOUT = 0,regWriteOUT = 1,writeDataOUT = 1,aluResultOUT = 1111111100001111,writeReg0OUT = 0110000000001010,writeRegAddrOUT = 1111100010111111 2
$finish called from file "EXMEMBuff_test.v", line 59.
$finish at simulation time                   40
           V C S   S i m u l a t i o n   R e p o r t 
Time: 40
CPU Time:      0.330 seconds;       Data structure size:   0.0Mb
Sun May  1 16:15:46 2016

// MUX1 test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:20 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:21 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 in1 = 0000000000001111, in2 = 0000000000000100, in3 = 0000000011010110, forward = 00, out = 0000000000001111
                  10 in1 = 0000000000001111, in2 = 0000000000000100, in3 = 0000000011010110, forward = 01, out = 0000000000000100
                  20 in1 = 0000000000001111, in2 = 0000000000000100, in3 = 0000000011010110, forward = 10, out = 0000000011010110
$finish called from file "mux1_test.v", line 31.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:21:38 2016


// hazard unit test results
				0opcode of decode phase = 0000, opcode of execute phase = 00110
                  10opcode of decode phase = 0000, opcode of execute phase = 10000
                  20opcode of decode phase = 0111, opcode of execute phase = 10000
$finish called from file "hazardUnit_test.v", line 30.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:20:24 2016

// mux 3 test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:22 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0  dataFromOP1= xxxxxxxxxxxxxxxx, dataForwarded= xxxxxxxxxxxxxxxx,signal= x,out= xxxxxxxxxxxxxxxx
                   5  dataFromOP1= 1111111111111111, dataForwarded= 0000000000000001,signal= 0,out= 1111111111111111
                  15  dataFromOP1= 1010101010101010, dataForwarded= 1111111111110000,signal= 0,out= 1010101010101010
                  25  dataFromOP1= 0000000000000000, dataForwarded= 0000000000000001,signal= 0,out= 0000000000000000
$finish called from file "mux3_test.v", line 35.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:22:09 2016

// mux 4 test results
AChronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:23 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0  data_from_memory = xxxxxxxxxxxxxxxx, data_from_ALU = xxxxxxxxxxxxxxxx, memToReg = x, data_OUT    x
                   5  data_from_memory = 1111111111111111, data_from_ALU = 0000000000000001, memToReg = 0, data_OUT    1
                  15  data_from_memory = 1010101010101010, data_from_ALU = 1111111111110000, memToReg = 0, data_OUT65520
                  25  data_from_memory = 0000000000000000, data_from_ALU = 0000000000000001, memToReg = 0, data_OUT    1
$finish called from file "mux4_test.v", line 35.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:23:02 2016
// register file test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:38 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0 clk = x,reset = 1,read1 = xxxx,read2 = xxxx,writeAddr = xxxx, out0 = xxxxxxxxxxxxxxxx,out1 = xxxxxxxxxxxxxxxx,out2 = xxxxxxxxxxxxxxxx,write0 = xxxxxxxxxxxxxxxx,write1 = xxxxxxxxxxxxxxxx,regWrite = x
                  10 clk = 1,reset = 0,read1 = 0001,read2 = 0101,writeAddr = 0010, out0 = xxxxxxxxxxxxxxxx,out1 = xxxxxxxxxxxxxxxx,out2 = xxxxxxxxxxxxxxxx,write0 = 0000000000000001,write1 = 1111000011110000,regWrite = x
                  20 clk = 0,reset = 0,read1 = 0001,read2 = 0101,writeAddr = 0010, out0 = xxxxxxxxxxxxxxxx,out1 = xxxxxxxxxxxxxxxx,out2 = xxxxxxxxxxxxxxxx,write0 = 0000000000000001,write1 = 1111000011110000,regWrite = x
                  30 clk = 1,reset = 0,read1 = 0001,read2 = 0101,writeAddr = 0010, out0 = xxxxxxxxxxxxxxxx,out1 = xxxxxxxxxxxxxxxx,out2 = xxxxxxxxxxxxxxxx,write0 = 0000000000000001,write1 = 1111000011110000,regWrite = x
                  40 clk = 0,reset = 0,read1 = 0001,read2 = 0101,writeAddr = 0010, out0 = xxxxxxxxxxxxxxxx,out1 = xxxxxxxxxxxxxxxx,out2 = xxxxxxxxxxxxxxxx,write0 = 0000000000000001,write1 = 1111000011110000,regWrite = x
$finish called from file "registerFile_test.v", line 41.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:38:05 2016

// shift left test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:38 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0input = 0000000000000000, output = 0000000000000000
                  10input = 1111000011110000, output = 1100001111000000
                  20input = 1111000011110011, output = 1100001111001100
                  30input = 0111000011110010, output = 1100001111001000
                  40input = 0011000011110001, output = 1100001111000100
$finish called from file "shiftLeft_test.v", line 27.
$finish at simulation time                  100
           V C S   S i m u l a t i o n   R e p o r t 
Time: 100
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:38:54 2016

// sign extend test results
Chronologic VCS simulator copyright 1991-2014
Contains Synopsys proprietary information.
Compiler version I-2014.03-2; Runtime version I-2014.03-2;  May  1 16:39 2016
VCD+ Writer I-2014.03-2 Copyright (c) 1991-2014 by Synopsys Inc.
                   0   1111111111111001 
                  10   1111111110000000 
                  20   1111100000000000 
                  30   0000000000000001 
                  40   0000000000000000 
$finish called from file "signExtend1_test.v", line 42.
$finish at simulation time                  200
           V C S   S i m u l a t i o n   R e p o r t 
Time: 200
CPU Time:      0.340 seconds;       Data structure size:   0.0Mb
Sun May  1 16:39:58 2016
