`timescale 1ns/1ps
`include "riscv32_Consts.v"

// LUT 410
module riscv32i_alu(
    input [31:0] op1_data,
    input [31:0] op2_data,
    input [`EXE_FUN_LEN-1:0] exe_fun,
    output reg [31:0] result,
    output reg br_flg
);

    wire slt_out = ($signed(op1_data) < $signed(op2_data));
    wire sltu_out = (op1_data < op2_data);

    always @(*)
        case(exe_fun)
            `ALU_ADD:       result = op1_data + op2_data;
            `ALU_SUB:       result = op1_data - op2_data;
            `ALU_AND:       result = op1_data & op2_data;
            `ALU_OR:        result = op1_data | op2_data;
            `ALU_XOR:       result = op1_data ^ op2_data;
            `ALU_SLL:       result = op1_data << op2_data[4:0];
            `ALU_SRL:       result = op1_data >> op2_data[4:0];
            `ALU_SRA:       result = $signed(op1_data) >>> op2_data[4:0];
            `ALU_SLT:       result = slt_out;
            `ALU_SLTU:      result = sltu_out;
            `ALU_JALR:      result = {op1_data[31:1], 1'b0};
            `ALU_COPY1:     result = op1_data;
            default:        result = 32'b0;
        endcase

    always @(*)
        case(exe_fun)
            `BR_BEQ:        br_flg = (op1_data == op2_data);
            `BR_BNE:        br_flg = (op1_data != op2_data);
            `BR_BLT:        br_flg = slt_out;
            `BR_BGE:        br_flg = ~slt_out;
            `BR_BLTU:       br_flg = sltu_out;
            `BR_BGEU:       br_flg = ~sltu_out;
            default:        br_flg = 0;
        endcase

endmodule

// LUT 616 DSP 4
module riscv32im_alu(
    input [31:0] op1_data,
    input [31:0] op2_data,
    input [`EXE_FUN_LEN-1:0] exe_fun,
    output reg [31:0] result,
    output reg br_flg
);

    wire slt_out = ($signed(op1_data) < $signed(op2_data));
    wire sltu_out = (op1_data < op2_data);
    wire [31:0] mull,mulh,mulhsu,mulhu;
    assign {mulhu,mull} = op1_data * op2_data;
    assign mulhsu = op1_data[31]?(mulhu - op2_data):mulhu;
    assign mulh = op2_data[31]?(mulhsu - op1_data):mulhsu;

    always @(*)
        case(exe_fun)
            `ALU_ADD:       result = op1_data + op2_data;
            `ALU_SUB:       result = op1_data - op2_data;
            `ALU_AND:       result = op1_data & op2_data;
            `ALU_OR:        result = op1_data | op2_data;
            `ALU_XOR:       result = op1_data ^ op2_data;
            `ALU_SLL:       result = op1_data << op2_data[4:0];
            `ALU_SRL:       result = op1_data >> op2_data[4:0];
            `ALU_SRA:       result = $signed(op1_data) >>> op2_data[4:0];
            `ALU_SLT:       result = slt_out;
            `ALU_SLTU:      result = sltu_out;
            `ALU_JALR:      result = {op1_data[31:1], 1'b0};
            `ALU_COPY1:     result = op1_data;
            `ALU_MUL:       result = mull;
            `ALU_MULH:      result = mulh;
            `ALU_MULHSU:    result = mulhsu;
            `ALU_MULHU:     result = mulhu;
            default:        result = 32'b0;
        endcase

    always @(*)
        case(exe_fun)
            `BR_BEQ:        br_flg = (op1_data == op2_data);
            `BR_BNE:        br_flg = (op1_data != op2_data);
            `BR_BLT:        br_flg = slt_out;
            `BR_BGE:        br_flg = ~slt_out;
            `BR_BLTU:       br_flg = sltu_out;
            `BR_BGEU:       br_flg = ~sltu_out;
            default:        br_flg = 0;
        endcase

endmodule
