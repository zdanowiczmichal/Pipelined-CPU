`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:
// Engineer: Michal Zdanowicz
//
// Create Date: 03/22/2025 06:26:47 PM
// Design Name:
// Module Name: testbench
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

module testbench;
    reg clk;
   
    wire [31:0] pc;
    wire [31:0] dinstOut;
    wire ewreg;
    wire em2reg;
    wire ewmem;
    wire [3:0] ealuc;
    wire ealuimm;
    wire [4:0] edestReg;
    wire [31:0] eqa;
    wire [31:0] eqb;
    wire [31:0] eimm32;
    wire mwreg;
    wire mm2reg;
    wire mwmem;
    wire  [4:0]mdestReg;
    wire [31:0]mr ;
    wire[31:0] mqb ;
    wire wwreg;
    wire wm2reg;
    wire [4:0] wdestReg ;
    wire[31:0] wr ;
    wire [31:0] wdo;
    wire [31:0] wbData;
    wire [1:0] fwda;
    wire [1:0] fwdb;
    wire [31:0] fqa;
    wire [31:0] fqb;

   
    Datapath DatapathInstance (
        .clk(clk),
        .pc(pc),
        .dinstOut(dinstOut),
        .ewreg(ewreg),
        .em2reg(em2reg),
        .ewmem(ewmem),
        .ealuc(ealuc),
        .ealuimm(ealuimm),
        .edestReg(edestReg),
        .eqa(eqa),
        .eqb(eqb),
        .eimm32(eimm32),
        .mwreg(mwreg),
        .mm2reg(mm2reg),
        .mwmem(mwmem),
        .mdestReg(mdestReg),
        .mr(mr),
        .mqb(mqb),
        .wwreg(wwreg),
        .wm2reg(wm2reg),
        .wdestReg(wdestReg),
        .wr(wr),
        .wdo(wdo),
        .wbData(wbData),
        .fwda(fwda),
        .fwdb(fwdb),
        .fqa(fqa),
        .fqb(fqb)
    );
   
    initial begin
        clk = 0;
    end
   
    always begin
        #1 clk = ~clk;
    end

endmodule
module testbench;
    reg clk;
   
    wire [31:0] pc;
    wire [31:0] dinstOut;
    wire ewreg;
    wire em2reg;
    wire ewmem;
    wire [3:0] ealuc;
    wire ealuimm;
    wire [4:0] edestReg;
    wire [31:0] eqa;
    wire [31:0] eqb;
    wire [31:0] eimm32;
    wire mwreg;
    wire mm2reg;
    wire mwmem;
    wire  [4:0]mdestReg;
    wire [31:0]mr ;
    wire[31:0] mqb ;
    wire wwreg;
    wire wm2reg;
    wire [4:0] wdestReg ;
    wire[31:0] wr ;
    wire [31:0] wdo;
    wire [31:0] wbData;
    wire [1:0] fwda;
    wire [1:0] fwdb;
    wire [31:0] fqa;
    wire [31:0] fqb;

   
    Datapath DatapathInstance (
        .clk(clk),
        .pc(pc),
        .dinstOut(dinstOut),
        .ewreg(ewreg),
        .em2reg(em2reg),
        .ewmem(ewmem),
        .ealuc(ealuc),
        .ealuimm(ealuimm),
        .edestReg(edestReg),
        .eqa(eqa),
        .eqb(eqb),
        .eimm32(eimm32),
        .mwreg(mwreg),
        .mm2reg(mm2reg),
        .mwmem(mwmem),
        .mdestReg(mdestReg),
        .mr(mr),
        .mqb(mqb),
        .wwreg(wwreg),
        .wm2reg(wm2reg),
        .wdestReg(wdestReg),
        .wr(wr),
        .wdo(wdo),
        .wbData(wbData),
        .fwda(fwda),
        .fwdb(fwdb),
        .fqa(fqa),
        .fqb(fqb)
    );
   
    initial begin
        clk = 0;
    end
   
    always begin
        #1 clk = ~clk;
    end

endmodule

