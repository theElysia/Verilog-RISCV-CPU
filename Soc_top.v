`timescale 1ns/1ps
`include "riscv32_cpu/riscv32_Consts.v"

module Soc_top (    
    input CLK_100M,
    input [15:0] SW,
    input [4:0] BTN,
    output [3:0] VGA_RED,
    output [3:0] VGA_GREEN,
    output [3:0] VGA_BLUE,
    output VGA_HS,
    output VGA_VS,
    output [7:0] SSEG_CA,
    output [3:0] SSEG_AN
);
    initial begin
        clk_cnt1=0;
        clk_cnt2=0;
    end

    reg [1:0] clk_cnt1;
    always @(posedge CLK_100M) clk_cnt1 <= clk_cnt1 + 1;

    wire clk_mem, mem_en;
    reg clk_core;
    wire [`WORD_LEN-1:0] imem_addr, imem_inst, dmem_addr, dmem_wdata, dmem_rdata;
    wire dmem_wen, exit;

    assign clk_mem = CLK_100M;
    assign mem_en = 1;
    
    wire rst_sys = SW[15];
    wire rst_vga = rst_sys || SW[14];
    wire rst_cpu = rst_sys || SW[13];

    reg [19:0] clk_cnt2;
    always @(posedge CLK_100M) clk_cnt2 <= clk_cnt2 + 1;

// step debugging
    wire [1:0] mode = SW[1:0];//00 run at 25mhz or 01 run at 50mhz or 1x step
    wire b_nxt = BTN[4];//center
    reg antijitter_btn, flg_btn;
    always @(posedge clk_cnt2[19])flg_btn <= b_nxt;
    always @(posedge clk_cnt2[19])
    if(!flg_btn && b_nxt) antijitter_btn<=1;
    else antijitter_btn<=0;

    always @(*)
    casex (mode)
        2'b1?: clk_core = antijitter_btn;
        2'b00: clk_core = clk_cnt1[1];
        2'b01: clk_core = clk_cnt1[0];
    endcase

// 7-seg led: step counter
    reg [15:0] step_cnt;
    always @(posedge clk_core)
    if(rst_cpu) step_cnt<=0;
    else step_cnt<=step_cnt+1;

    seg_led_top seg1(
        .CLK        (clk_cnt2[17]),
        .num        (step_cnt),
        .SSEG_CA    (SSEG_CA),
        .SSEG_AN    (SSEG_AN)
    );

    // for vga debugger
    wire [`WORD_LEN-1:0] core_id_reg_pc, core_id_reg_inst, core_exe_reg_op1_data, core_exe_reg_op2_data, core_exe_alu_out;
    wire [`WORD_LEN*32-1:0] core_regfile;

    riscv32_core_pipeline core1(
        .clock          (clk_core),
        .reset          (rst_cpu),
        .imem_addr      (imem_addr),
        .imem_inst      (imem_inst),
        .dmem_addr      (dmem_addr),
        .dmem_wdata     (dmem_wdata),
        .dmem_wen       (dmem_wen),
        .dmem_rdata     (dmem_rdata),
        .exit           (exit),
        // for vga debugger
        .core_id_reg_pc         (core_id_reg_pc),
        .core_id_reg_inst       (core_id_reg_inst),
        .core_exe_reg_op1_data  (core_exe_reg_op1_data),
        .core_exe_reg_op2_data  (core_exe_reg_op2_data),
        .core_exe_alu_out       (core_exe_alu_out),
        .core_regfile           (core_regfile)
    );


    instruction_memory imem1(
        .clock          (clk_mem),
        .en             (mem_en),
        .addr           (imem_addr),
        .inst           (imem_inst)
    );

    data_memory dmem1(
        .clock          (clk_mem),
        .en             (mem_en),
        .addr           (dmem_addr),
        .wdata          (dmem_wdata),
        .wen            (dmem_wen),
        .rdata          (dmem_rdata)
    );

    VGA vga1(
        .clk_25m        (clk_cnt1[1]),
        .clk_100m       (CLK_100M),
        .rst            (rst_vga),
        .hs             (VGA_HS),
        .vs             (VGA_VS),
        .vga_r          (VGA_RED),
        .vga_g          (VGA_GREEN),
        .vga_b          (VGA_BLUE),

        .pc             (core_id_reg_pc),
        .inst           (core_id_reg_inst),
        .alu_res        (core_exe_alu_out),
        .mem_wen        (dmem_wen),
        .dmem_o_data    (dmem_rdata),
        .dmem_i_data    (dmem_wdata),
        .dmem_addr      (dmem_addr),

        .regfile        (core_regfile),
        .alu_op1_val    (core_exe_reg_op1_data),
        .alu_op2_val    (core_exe_reg_op2_data)
    );

endmodule