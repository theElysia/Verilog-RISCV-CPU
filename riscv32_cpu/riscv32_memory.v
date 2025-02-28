`timescale 1ns/1ps
`include "riscv32_Consts.v"

// 32KB
module instruction_memory (
    input clock,
    input en,
    input [`WORD_LEN-1:0] addr,// 4 byte alligned
    output reg [`WORD_LEN-1:0] inst
);

    (* rom_style = "block" *) reg [31:0] rom[0:4095];
    //initial $readmemh("testbench/test3.mem", rom);
    initial $readmemh("D:/Code_Files/Xilinx/homework6/riscv32_cpu/testbench/test3.mem", rom);

    always @(posedge clock)
    if(en) inst <= rom[addr[13:2]];

/*
    blk_mem_gen_imen imem (
    .clka(clock),    // input wire clka
    .ena(en),      // input wire ena
    .addra(addr[13:2]),  // input wire [11 : 0] addra
    .douta(inst)  // output wire [31 : 0] douta
    );*/

endmodule


// 32KB
module data_memory (
    input clock,
    input en,
    input [`WORD_LEN-1:0] addr,// 4 byte alligned
    input [`WORD_LEN-1:0] wdata,
    input wen,
    output reg [`WORD_LEN-1:0] rdata
);
    
    (* ram_style = "block" *) reg [31:0] mem[0:16383];
    //initial $readmemh("testbencg/test3.mem", mem);

    always @(posedge clock)
    if(wen) mem[wdata[15:2]] <= wdata;
    else rdata <= mem[wdata[15:2]];

/*
    blk_mem_gen_dmem your_instance_name (
    .clka(clock),    // input wire clka
    .ena(en),      // input wire ena
    .wea(wen),      // input wire [0 : 0] wea
    .addra(addr[15:2]),  // input wire [13 : 0] addra
    .dina(wdata),    // input wire [31 : 0] dina
    .douta(rdata)  // output wire [31 : 0] douta
    );*/

endmodule