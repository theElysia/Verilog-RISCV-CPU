`timescale 1ns/1ps
`include "riscv32_Consts.v"


// 32KB
module instruction_memory (
    input clock,
    input [`WORD_LEN-1:0] addr,
    output [`WORD_LEN-1:0] inst
);
    
    //(*ram_style="block"*) reg [31:0] mem [0:8191];
    //always @(posedge clock)inst<=mem[addr];

    blk_mem_gen_0 inst_mem (
        .clka(clock),    // input wire clka
        .addra(addr[14:2]),  // input wire [12 : 0] addra
        .douta(inst)  // output wire [31 : 0] douta
    );

endmodule


// 128KB
module data_memory (
    input clock,
    input [`WORD_LEN-1:0] addr,
    input [`WORD_LEN-1:0] wdata,
    input wen,
    output reg [`WORD_LEN-1:0] rdata
);
    
    (*ram_style="block"*) reg [31:0] mem [0:32767];//[14:0]

    
    always @(posedge clock)
        if(wen)mem[addr[16:2]]<=wdata;
        else rdata<=mem[addr[16:2]];

/*
    always @(posedge clock)
        if(wen)begin
            mem[addr]<=wdata[7:0];
            mem[addr+1]<=wdata[15:8];
            mem[addr+2]<=wdata[23:16];
            mem[addr+3]<=wdata[31:24];
        end
        else rdata[31:0]<={mem[addr+3],mem[addr+2],mem[addr+1],mem[addr]};
*/

endmodule