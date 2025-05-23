`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: Michal Zdanowicz
// 
// Create Date: 03/22/2025 06:26:15 PM
// Design Name: 
// Module Name: Modules
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module ProgramCounter (
    input [31:0] nextPc,
    input clk,
    output reg [31:0] pc
);

initial begin
    pc = 32'd100;
end

always @(posedge clk) begin
    pc <= nextPc;
end

endmodule

///////////////////////////////////

module InstructionMemory (
    input [31:0] pc,
    output reg [31:0] instOut    
);

reg [31:0] memory [0:63];
   
initial begin
    memory[25] = 32'b00000000001000100001100000100000;
    memory[26] = 32'b00000001001000110010000000100010;
    memory[27] = 32'b00000000011010010010100000100101;
    memory[28] = 32'b00000000011010010011000000100110;
    memory[29] = 32'b00000000011010010011100000100100;
   
end
   
always @(pc) begin
    instOut = memory[pc[7:2]];
end
   
endmodule

////////////////////

module PcAdder (
    input [31:0] pc,
    output reg [31:0] nextPc
);

    always @(*) begin
        nextPc = pc + 32'd4;
    end
endmodule

///////////////////////////////////

module IfidPiplineRegister (
    input [31:0] instOut,
    input clk,
    output reg [31:0] dinstOut
);

always @(posedge clk) begin
    dinstOut <= instOut;
end

endmodule

/////////////////////////////////////

module ControlUnit (
    input [5:0] op,
    input [5:0] func,
    input [4:0] rs,
    input [4:0] rt,
    
    input [4:0] edestReg,
    input [4:0] mdestReg,
    input mm2reg,
    input mwreg,
    input em2reg,
    input ewreg,
    
    output reg wreg,
    output reg m2reg,
    output reg wmem,
    output reg [3:0] aluc,
    output reg aluimm,
    output reg reg_rt,
    output reg [1:0] fwdb,
    output reg [1:0] fwda
);

always @(*) begin
    case (op)
        6'b000000:
                begin
                    wreg = 1'b1;
                    m2reg = 1'b0;
                    wmem = 1'b0;
                    case(func)
                        6'b100000: aluc = 4'b0010;
                        6'b100010: aluc = 4'b0100;
                        6'b100100: aluc = 4'b0001;
                        6'b100101: aluc = 4'b0000;
                        6'b100110: aluc = 4'b1111;
                    endcase
                    aluimm = 1'b0;
                    reg_rt = 1'b0;
                end
        6'b100011: //Load word instruction
        begin
            wreg = 1'b1;
            m2reg = 1'b1;
            wmem = 1'b0;
            aluc = 4'b0010;
            aluimm = 1'b1;
            reg_rt = 1'b1;
        end
     endcase
     
     fwda = 2'b00;
     fwdb = 2'b00;
     
      if (ewreg && (edestReg == rs) && !em2reg) begin
            fwda = 2'b01;  // EX stage
        end else if (mwreg && (mdestReg == rs) && !mm2reg) begin
            fwda = 2'b10;
        end else if (mwreg && (mdestReg == rs) && mm2reg) begin
            fwda = 2'b11;
        end
         if (ewreg && (edestReg == rt) && !em2reg) begin
            fwdb = 2'b01;
        end else if (mwreg && (mdestReg == rt) && !mm2reg) begin
            fwdb = 2'b10;
        end else if (mwreg && (mdestReg == rt) && mm2reg) begin
            fwdb = 2'b11;
        end
    
end
endmodule

///////////////////////////////////////

module ForwardingMuxA (
    input [1:0] fwda,
    input [31:0] qa,
    input [31:0] r,
    input [31:0] mr,
    input [31:0] mdo,
    output reg [31:0] fqa
);

always @ (*) begin
    case(fwda)
        2'b00: fqa = qa;
        2'b01: fqa = r;
        2'b10: fqa = mr;
        2'b11: fqa = mdo;
    endcase
end
endmodule

module ForwardingMuxB (
    input [1:0] fwdb,
    input [31:0] qb,
    input [31:0] r,
    input [31:0] mr,
    input [31:0] mdo,
    output reg [31:0] fqb
);

always @ (*) begin
    case(fwdb)
        2'b00: fqb = qb;
        2'b01: fqb = r;
        2'b10: fqb = mr;
        2'b11: fqb = mdo;
    endcase
end
endmodule

///////////////////////////////////////

module RegrtMultiplexer (
    input [4:0] rt,
    input [4:0] rd,
    input regrt,
    output reg [4:0] destReg
);

always @(*) begin
    if (regrt == 0)
        destReg = rd;
    else
        destReg = rt;
end  

endmodule

/////////////////////////////////////////

module RegisterFile (
    input [4:0] rs,
    input [4:0] rt,
    input clk,
    input [4:0] wdestReg,
    input [31:0] wbData,
    input wwreg,
    output reg [31:0] qa,
    output reg [31:0] qb,
    integer i
);

reg [31:0] registers [0:31];

initial begin
//    for (i = 0; i < 32; i = i + 1) begin
//        registers[i] = 32'd0;
//    end
    registers[0] = 32'h00000000;
    registers[1] = 32'hA00000AA;
    registers[2] = 32'h10000011;
    registers[3] = 32'h20000022;
    registers[4] = 32'h30000033;
    registers[5] = 32'h40000044;
    registers[6] = 32'h50000055;
    registers[7] = 32'h60000066;
    registers[8] = 32'h70000077;
    registers[9] = 32'h80000088;
    registers[10] = 32'h90000099;
end

always @(*) begin
    qa = registers[rs];
    qb = registers[rt];
end

always @ (negedge clk) begin
    if (wwreg == 1) begin
        registers[wdestReg] = wbData;
    end
end
endmodule

///////////////////////////////////////

module ImmediateExtender (
    input  [15:0] imm,
    output reg [31:0] imm32
);

always @(*) begin
    if (imm[15] == 1) begin
        imm32 = {16'b1111111111111111, imm};
    end
    else if (imm[15] == 0) begin
        imm32 = {16'b0000000000000000, imm};
    end
    else
        imm32 = {16'hXXXX, imm};
end

endmodule

///////////////////////////////////////

module IdexePipelineRegister (
    input wreg,
    input m2reg,
    input wmem,
    input [3:0] aluc,
    input aluimm,
    input [4:0] destReg,
    input [31:0] fqa,
    input [31:0] fqb,
    input [31:0] imm32,
    input clk,
    output reg ewreg,
    output reg em2reg,
    output reg ewmem,
    output reg [3:0] ealuc,
    output reg ealuimm,
    output reg [4:0] edestReg,
    output reg [31:0] eqa,
    output reg [31:0] eqb,
    output reg [31:0] eimm32
);

always @(posedge clk) begin
    ewreg  <= wreg;
    em2reg <= m2reg;
    ewmem <= wmem;
    ealuc <= aluc;
    ealuimm <= aluimm;
    edestReg <= destReg;
    eqa <= fqa;
    eqb <= fqb;
    eimm32 <= imm32;
end

endmodule

/////////////////////////////////////////////

module ALUMux(
    input [31:0] eqb,
    input [31:0] eimm32,
    input ealuimm,
    output reg [31:0] b
);
always @(*) begin
    if (ealuimm ==0) begin
        b = eqb;
    end
    else begin
        b = eimm32;
    end
end
endmodule

//////////////////////////////////////////////////

module ALU(
    input [31:0] eqa,
    input [31:0] b,
    input [3:0] ealuc,
    output reg [31:0] r
);
always @(*) begin
    case(ealuc)
        4'b0010: r = eqa + b;   // ADD
        4'b1111: r = eqa - b;   // SUB
        4'b0000: r = eqa & b;   // AND
        4'b0001: r = eqa | b;   // OR
        4'b0100: r = eqa ^ b;   // XOR
        default: r = 32'b0;        
    endcase
end
endmodule
/////////////////////////////////////////////

module EXEMEMPipelineRegister(
    input ewreg,
    input em2reg,
    input ewmem,
    input [4:0] edestReg,
    input [31:0] r,
    input [31:0] eqb,
    input clk,
    output reg mwreg,
    output reg mm2reg,
    output reg mwmem,
    output reg [4:0] mdestReg,
    output reg [31:0] mr,
    output reg [31:0] mqb
);
always @(posedge clk) begin
    mwreg  <= ewreg;
    mm2reg <= em2reg;
    mwmem <= ewmem;
    mdestReg<= edestReg;
    mr <= r;
    mqb <= eqb;
end
endmodule

////////////////////////////////////////////
module DataMemory(
    input [31:0] mr,        
    input [31:0] mqb,        
    input mwmem,          
    input clk,              
    output reg [31:0] mdo  
);

    reg [31:0] memory [0:63];

    initial begin
        memory[0] = 32'hA00000AA;
        memory[1] = 32'h10000011;
        memory[2] = 32'h20000022;
        memory[3] = 32'h30000033;
        memory[4] = 32'h40000044;
        memory[5] = 32'h50000055;
        memory[6] = 32'h60000066;
        memory[7] = 32'h70000077;
        memory[8] = 32'h80000088;
        memory[9] = 32'h90000099;
    end

    always @(*) begin
        mdo = memory[mr[7:2]];
    end

    always @(negedge clk) begin
        if (mwmem) begin
            memory[mr[7:2]] <= mqb;
        end
    end

endmodule
////////////////////////////////////////////
module MEMWBPipelineRegister(
    input mwreg,
    input mm2reg,
    input [4:0] mdestReg,
    input [31:0] mr,
    input [31:0] mdo,
    input clk,
    output reg wwreg,
    output reg wm2reg,
    output reg [4:0] wdestReg,
    output reg [31:0] wr,
    output reg [31:0] wdo
);
always @(posedge clk) begin
    wwreg <= mwreg;
    wm2reg <= mm2reg;
    wdestReg <= mdestReg;
    wr <= mr;
    wdo <= mdo;
end
endmodule

module WbMux (
    input [31:0] wr,
    input [31:0] wdo,
    input wm2reg,
    output reg [31:0] wbData
);

always @ (*) begin
    if (wm2reg == 0) begin
        wbData = wr;
    end
    if (wm2reg == 1) begin
        wbData = wdo;
    end
end
endmodule

module Datapath (
    input clk,
    output wire [31:0] pc,
    output wire [31:0] dinstOut,
    output wire ewreg,
    output wire em2reg,
    output wire ewmem,
    output wire [3:0] ealuc,
    output wire ealuimm,
    output wire [4:0] edestReg,
    output wire [31:0] eqa,
    output wire [31:0] eqb,
    output wire [31:0] eimm32,
    output wire mwreg,
    output wire mm2reg,
    output wire mwmem,
    output wire [4:0] mdestReg ,
    output wire [31:0] mr ,
    output wire [31:0] mqb ,
    output wire wwreg,
    output wire wm2reg,
    output wire [4:0] wdestReg,
    output wire [31:0] wr,
    output wire [31:0] wdo,
    output wire [31:0] wbData,
    output wire [31:0] fqa,
    output wire [31:0] fqb,
    output wire [1:0] fwda,
    output wire [1:0] fwdb
);


    wire [31:0] nextPc;
    wire [31:0] instOut;
    wire [5:0] op, func;
    wire wreg, m2reg, wmem, aluimm, reg_rt;
    wire [3:0] aluc;
    wire [4:0] rs, rt, rd, destReg;
    wire [31:0] qa, qb;
    wire [15:0] imm;
    wire [31:0] imm32;
       
     // Additional wires for ALUMux, ALU, and pipeline registers
    wire [31:0] b;       // Output from ALUMux
    wire [31:0] r;       // Output from ALU
    wire [31:0] mdo;     // Output from Data Memory
 
   
    assign op = dinstOut[31:26];
    assign func = dinstOut[5:0];
    assign rs = dinstOut[25:21];
    assign rt = dinstOut[20:16];
    assign rd = dinstOut[15:11];
    assign imm = dinstOut[15:0];


   
    ProgramCounter ProgramCounterInstance (
    .nextPc(nextPc),
    .clk(clk),
    .pc(pc)
    );
   
    PcAdder PcAdderInstance (
    .pc(pc),
    .nextPc(nextPc)
    );
   
    InstructionMemory InstructionMemoryInstance (
    .pc(pc),
    .instOut(instOut)
    );
   
    IfidPiplineRegister IfidPipelineRegisterInstance (
    .instOut(instOut),
    .clk(clk),
    .dinstOut(dinstOut)
    );
   
    ControlUnit ControlUnitInstance (
    .op(op),
    .func(func),
    .wreg(wreg),
    .m2reg(m2reg),
    .wmem(wmem),
    .aluc(aluc),
    .aluimm(aluimm),
    .reg_rt(reg_rt),
    .rs(rs),
    .rt(rt),
    .edestReg(edestReg),
    .mdestReg(mdestReg),
    .mm2reg(mm2reg),
    .mwreg(mwreg),
    .em2reg(em2reg),
    .ewreg(ewreg),
    .fwda(fwda),
    .fwdb(fwdb)
    );
    
    ForwardingMuxA ForwardingMuxAInstance (
    .qa(qa),
    .r(r),
    .mr(mr),
    .mdo(mdo),
    .fwda(fwda),
    .fqa(fqa)
    );
    
    ForwardingMuxB ForwardingMuxBInstance (
    .qb(qb),
    .r(r),
    .mr(mr),
    .mdo(mdo),
    .fwdb(fwdb),
    .fqb(fqb)
    );
   
    RegrtMultiplexer RegrtMultiplexerInstance (
    .rt(rt),
    .rd(rd),
    .regrt(reg_rt),
    .destReg(destReg)
    );
   
    RegisterFile RegisterFileInstance (
    .rs(rs),
    .rt(rt),
    .qa(qa),
    .qb(qb),
    .clk(clk),
    .wdestReg(wdestReg),
    .wbData(wbData),
    .wwreg(wwreg)
    );
   
   
    ImmediateExtender ImmediateExtenderInstance (
    .imm(imm),
    .imm32(imm32)
    );
   
    IdexePipelineRegister IdexePipelineRegisterInstance (
    .wreg(wreg),
    .m2reg(m2reg),
    .wmem(wmem),
    .aluc(aluc),
    .aluimm(aluimm),
    .destReg(destReg),
    //.qa(qa),
    //.qb(qb),
    .imm32(imm32),
    .clk(clk),
    .ewreg(ewreg),
    .em2reg(em2reg),
    .ewmem(ewmem),
    .ealuc(ealuc),
    .ealuimm(ealuimm),
    .edestReg(edestReg),
    .eqa(eqa),
    .eqb(eqb),
    .eimm32(eimm32),
    .fqa(fqa),
    .fqb(fqb)
    );
   
    ALUMux ALUMuxInstance (
    .eqb(eqb),
    .eimm32(eimm32),
    .ealuimm(ealuimm),
    .b(b)
    );
   
    // ALU instantiation
    ALU ALUInstance (
    .eqa(eqa),
    .b(b),
    .ealuc(ealuc),
    .r(r)
    );
   
    // EXE/MEM Pipeline Register instantiation
    EXEMEMPipelineRegister EXEMEMPipelineRegisterInstance (
    .ewreg(ewreg),
    .em2reg(em2reg),
    .ewmem(ewmem),
    .edestReg(edestReg),
    .r(r),
    .eqb(eqb),
    .clk(clk),
    .mwreg(mwreg),
    .mm2reg(mm2reg),
    .mwmem(mwmem),
    .mdestReg(mdestReg),
    .mr(mr),
    .mqb(mqb)
    );
   
    // Data Memory instantiation
    DataMemory DataMemoryInstance (
    .mr(mr),
    .mqb(mqb),
    .mwmem(mwmem),
    .clk(clk),
    .mdo(mdo)
    );
   
    // MEM/WB Pipeline Register instantiation
    MEMWBPipelineRegister MEMWBPipelineRegisterInstance (
    .mwreg(mwreg),
    .mm2reg(mm2reg),
    .mdestReg(mdestReg),
    .mr(mr),
    .mdo(mdo),
    .clk(clk),
    .wwreg(wwreg),
    .wm2reg(wm2reg),
    .wdestReg(wdestReg),
    .wr(wr),
    .wdo(wdo)
    );

    WbMux WbMuxInstance (
    .wr(wr),
    .wdo(wdo),
    .wm2reg(wm2reg),
    .wbData(wbData)
    );
endmodule