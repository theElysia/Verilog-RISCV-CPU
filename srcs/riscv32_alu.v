`timescale 1ns/1ps
`include "carry_look-ahead_adder.v"
`include "riscv32_Consts.v"


module riscv32i_alu(
    input [31:0] op1_data,
    input [31:0] op2_data,
    input [`EXE_FUN_LEN-1:0] exe_fun,
    output reg [31:0] result,
    output reg br_flg
);

    reg ci;
    always @(*)
        casex(exe_fun)
            `ALU_SUB:   ci=1;
            `ALU_SLT:   ci=1;
            `ALU_SLTU:  ci=1;
            `BR_BEQ:    ci=1;
            `BR_BNE:    ci=1;
            `BR_BGE:    ci=1;
            `BR_BLTU:   ci=1;
            `BR_BGEU:   ci=1;
            default:    ci=0;
        endcase

    reg [31:0] op2_data_in;
    integer i;
    always @(*) for(i=0;i<32;i=i+1)op2_data_in[i]=op2_data[i]^ci;

    wire [31:0] adder_result;
    wire cp,cg,carry,zero;
    assign carry=(cp&ci)|cg;
    assign zero=~(|adder_result);

    adder_32 ad1(.a(op1_data), .b(op2_data_in), .ci(ci), .s(adder_result), .cp(cp), .cg(cg));

    wire [31:0] srl_out = (op1_data >> op2_data[4:0]);
    wire [31:0] sra1_out = ((32'hffffffff)<<({~op2_data[4],~op2_data[3],~op2_data[2],~op2_data[1],~op2_data[0]}))|srl_out;
    wire [31:0] sra_out = (op1_data[31])?sra1_out:srl_out;
    
    wire slt_out = (~op1_data[31]&op2_data[31])|adder_result[31];//set less than
    wire sltu_out = ~carry;

    always @(*)
        case(exe_fun)
            `ALU_ADD:       result = adder_result;
            `ALU_SUB:       result = adder_result;
            `ALU_AND:       result = op1_data&op2_data;
            `ALU_OR:        result = op1_data|op2_data;
            `ALU_XOR:       result = op1_data^op2_data;
            `ALU_SLL:       result = op1_data<<op2_data[4:0];
            `ALU_SRL:       result = srl_out;
            `ALU_SRA:       result = sra_out;
            `ALU_SLT:       result = slt_out;
            `ALU_SLTU:      result = sltu_out;
            `ALU_JALR:      result = {adder_result[31:1],1'b0};
            `ALU_COPY1:     result = op1_data;
            default:        result = 32'b0;
        endcase

    always @(*)
        case(exe_fun)
            `BR_BEQ:        br_flg = zero;
            `BR_BNE:        br_flg = ~zero;
            `BR_BLT:        br_flg = slt_out;
            `BR_BGE:        br_flg = ~slt_out;
            `BR_BLTU:       br_flg = sltu_out;
            `BR_BGEU:       br_flg = ~sltu_out;
            default:        br_flg = 0;
        endcase

endmodule


/*
    // 448 mul 4 dsp   2631 / %  4 dsp

    module riscv32i_alu_behav(
        input [31:0] op1_data,
        input [31:0] op2_data,
        input [3:0] alu_ctrl,//func3[14:12]  1:sub sra slt sltu
        output zero,
        output reg [31:0] result
    );

        wire [31:0] srl_out = op1_data >> op2_data[4:0];
        wire [31:0] sra_out = ((32'hffffffff)<<({~op2_data[4],~op2_data[3],~op2_data[2],~op2_data[1],~op2_data[0]}))|srl_out;
        wire [31:0] sr_out = (op1_data[31])?sra_out:srl_out;
        assign zero=~(|result);//16  407
        //assign zero=(result==32'h0);//20   411
    //    wire [63:0] mulresult = op1_data*op2_data;

        always @(*)
            case(alu_ctrl)
                4'b0000:result = op1_data+op2_data;
                4'b0001:result = op1_data<<op2_data[4:0];
                4'b0010:result = {~op1_data[31],op1_data[30:0]}<{~op2_data[31],op2_data[30:0]};
                4'b0011:result = op1_data<op2_data;
                4'b0100:result = op1_data^op2_data;
                4'b0101:result = srl_out;
                4'b0110:result = op1_data|op2_data;
                4'b0111:result = op1_data&op2_data;
                4'b1000:result = op1_data-op2_data;
                //4'b1111:result = mulresult[63:32];
                //4'b1110:result = mulresult[31:0];
                //4'b1101:result = op1_data/op2_data;
                //4'b1100:result = op1_data%op2_data;
                4'b1101:result = sr_out;
    //            4'b1111:
                default:result = 32'h0;
            endcase

    endmodule
*/