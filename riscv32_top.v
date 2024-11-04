`timescale 1ns/1ps
`include "riscv32_Consts.v"
`include "riscv32_core.v"
`include "riscv32_memory.v"

module top_processor(
    input clock,
    input reset
);

    wire [`WORD_LEN-1:0] imem_addr, imem_inst, dmem_addr, dmem_wdata, dmem_rdata;
    wire dmem_wen, exit, gp;

    riscv32_core core1(
        .clock(clock),
        .reset(reset),
        .imem_addr(imem_addr),
        .imem_inst(imem_inst),
        .dmem_addr(dmem_addr),
        .dmem_wdata(dmem_wdata),
        .dmem_wen(dmem_wen),
        .dmem_rdata(dmem_rdata),
        .exit(exit),
        .gp(gp)
    );

    instruction_memory icache1(
        .clock(clock),
        .addr(imem_addr),
        .inst(imem_inst)
    );

    data_memory dcahce1(
        .clock(clock),
        .addr(dmem_addr),
        .wdata(dmem_wdata),
        .wen(dmem_wen),
        .rdata(dmem_rdata)
    );

endmodule