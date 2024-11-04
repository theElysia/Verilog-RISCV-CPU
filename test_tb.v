`timescale 1ns/1ps
`include "riscv32_Instructions.v"
`include "riscv32_Consts.v"
`include "riscv32_core.v"
`include "riscv32_memory.v"
`include "riscv32_alu.v"

module riscv32i_alu_tb;

    reg [31:0] a,b;
    reg [`EXE_FUN_LEN-1:0] exe_fun;
    wire [31:0] result;

    riscv32i_alu alu1(
        .op1_data(a),
        .op2_data(b),
        .exe_fun(exe_fun),
        .result(result)
    );

    initial begin
        a<=0;
        b<=0;
        exe_fun<=0;
    end
    always #5 
    begin
        a={$random}%16;
        b={$random}%16;
    end
    always #15 exe_fun<=exe_fun+1;

endmodule


module alu16delay_tb(
    output [15:0] s,
    output [15:0] s1
);
    reg [15:0] a,b;
    reg ci;
    wire cp,cg;
    assign s1=a+b+ci;
    adder_16 ad16(.a(a), .b(b), .ci(ci), .s(s), .cp(cp), .cg(cg));
    initial ci=0;
    integer i;
    initial
    for(i=0;i<20;i=i+1)
        #20
    begin
        a={$random}%256;
        b={$random}%256;
    end
endmodule

module pr_tb;

    reg [31:0] inst;
    reg [2:0] k;

    initial begin
        inst<=32'b11000000000000000000000000000000;
        #10 inst<=32'b00000000000000000010111110000011;
        #10 inst<=32'b11000000010000000000000000000000;
        #10 inst<=32'b00000000000000000010000000000011;
    end

    always@(*)
    casex (inst)
        `LW:k=1; 
        default:k=0; 
    endcase

endmodule