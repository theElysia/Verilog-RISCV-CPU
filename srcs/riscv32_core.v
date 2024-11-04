`timescale 1ns/1ps
`include "riscv32_Consts.v"
`include "riscv32_Instructions.v"
`include "riscv32_alu.v"

module riscv32_core (
    input clock,
    input reset,
    output [`WORD_LEN-1:0] imem_addr,
    input [`WORD_LEN-1:0] imem_inst,
    output [`WORD_LEN-1:0] dmem_addr,
    output [`WORD_LEN-1:0] dmem_wdata,
    output dmem_wen,
    input [`WORD_LEN-1:0] dmem_rdata,
    output exit,
    output gp
);
    
    reg [`WORD_LEN-1:0] pc_reg, pc_next;
    wire [`WORD_LEN-1:0] pc_plus4 = pc_reg+4;
    wire br_flg, jmp_flg;
    wire [`WORD_LEN-1:0] alu_out, br_target, inst;

    reg [`WORD_LEN-1:0] regfile [0:31];
    //reg [`WORD_LEN-1:0] csr_regfile [0:4095];

//------------------------------------------------------------------------
// IF stage

    assign imem_addr = pc_reg;
    assign inst = imem_inst;

    always @(posedge clock)
    if(reset) pc_reg <= `START_ADDR;
    else pc_reg <= pc_next;

    always @(*) begin
        if(reset) pc_next = `START_ADDR;
        else if(br_flg) pc_next = br_target;
        else if(jmp_flg) pc_next = alu_out;
        //else if(inst==`ECALL)pc_next=csr_regfile[12'h0305];
        else pc_next = pc_plus4;
    end

//------------------------------------------------------------------------
// ID stage

    wire [4:0]  rs1_addr = inst[19:15],
                rs2_addr = inst[24:20],
                rd_addr = inst[11:7];
    
    wire [`WORD_LEN-1:0]
                rs1_data = (rs1_addr==0)? 32'b0: regfile[rs1_addr],
                rs2_data = (rs2_addr==0)? 32'b0: regfile[rs2_addr];
                //rs1_data = regfile[rs1_addr],
                //rs2_data = regfile[rs2_addr];

    wire [11:0] imm_i = inst[31:20];
    wire [11:0] imm_s = {inst[31:25],inst[11:7]};
    wire [11:0] imm_b = {inst[31],inst[7],inst[30:25],inst[11:8]};
    wire [19:0] imm_j = {inst[31],inst[19:12],inst[20],inst[30:21]};
    wire [19:0] imm_u = inst[31:12];
    wire [4:0]  imm_z = inst[19:15];
    // signed/unsigned extend
    wire [`WORD_LEN-1:0]
                imm_i_sext = {{20{imm_i[11]}}, imm_i},
                imm_s_sext = {{20{imm_s[11]}}, imm_s},
                imm_b_sext = {{19{imm_b[11]}}, imm_b, 1'b0},
                imm_j_sext = {{11{imm_j[19]}}, imm_j, 1'b0},
                imm_u_sext = {imm_u, 12'b0},
                imm_z_uext = {27'b0, imm_z};

    wire [`EXE_FUN_LEN-1:0] exe_fun;
    wire [`OP1_LEN-1:0] op1_sel;
    wire [`OP2_LEN-1:0] op2_sel;
    wire [`MEM_LEN-1:0] mem_wen;
    wire [`REN_LEN-1:0] rf_wen;
    wire [`WB_SEL_LEN-1:0] wb_sel;
    //wire [`MW_LEN-1:0] mw_sel;
    wire [`CSR_LEN-1:0] csr_cmd;

    localparam TRANS_INST_LEN = `EXE_FUN_LEN+`OP1_LEN+`OP2_LEN+`MEM_LEN+`REN_LEN+`WB_SEL_LEN+`CSR_LEN;
    reg [TRANS_INST_LEN-1:0] trans_inst;
    assign {exe_fun, op1_sel, op2_sel, mem_wen, rf_wen, wb_sel, csr_cmd} = trans_inst;

    reg [`WORD_LEN-1:0] op1_data, op2_data;

    always @(*)
        case (op1_sel)
            `OP1_RS1:   op1_data = rs1_data;
            `OP1_PC:    op1_data = pc_reg;
            `OP1_IMZ:   op1_data = imm_z_uext;
            default:    op1_data = 0; 
        endcase
    
    always @(*)
        case (op2_sel)
            `OP2_RS2:   op2_data = rs2_data;
            `OP2_IMI:   op2_data = imm_i_sext;
            `OP2_IMS:   op2_data = imm_s_sext;
            `OP2_IMJ:   op2_data = imm_j_sext;
            `OP2_IMU:   op2_data = imm_u_sext;
            default:    op2_data = 0; 
        endcase

    always @(*)
        casex (inst)
            `LUI:       trans_inst = {`ALU_ADD, `OP1_X,  `OP2_IMU, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `AUIPC:     trans_inst = {`ALU_ADD, `OP1_PC, `OP2_IMU, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `JAL:       trans_inst = {`ALU_ADD, `OP1_PC, `OP2_IMJ, `MEM_X, `REN_S, `WB_PC,  `CSR_X};
            `JALR:      trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_PC, `CSR_X};

            `BEQ:       trans_inst = {`BR_BEQ, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};
            `BNE:       trans_inst = {`BR_BNE, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};
            `BLT:       trans_inst = {`BR_BLT, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};
            `BGE:       trans_inst = {`BR_BGE, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};
            `BLTU:      trans_inst = {`BR_BLTU, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};
            `BGEU:      trans_inst = {`BR_BGEU, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};

            //`LB:        trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            //`LH:        trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            `LW:        trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            //`LBU:       trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            //`LHU:       trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            //`SB:        trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            //`SH:        trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            `SW:        trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_S, `REN_X, `WB_X, `CSR_X};

            `ADDI:      trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SLTI:      trans_inst = {`ALU_SLT, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SLTIU:     trans_inst = {`ALU_SLTU, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `XORI:      trans_inst = {`ALU_XOR, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `ORI:       trans_inst = {`ALU_OR,  `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `ANDI:      trans_inst = {`ALU_AND, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SLLI:      trans_inst = {`ALU_SLL, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SRLI:      trans_inst = {`ALU_SRL, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SRAI:      trans_inst = {`ALU_SRA, `OP1_RS1, `OP2_IMI, `MEM_X, `REN_S, `WB_ALU, `CSR_X};

            `ADD:       trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SUB:       trans_inst = {`ALU_SUB, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SLL:       trans_inst = {`ALU_SLL, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SLT:       trans_inst = {`ALU_SLT, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SLTU:      trans_inst = {`ALU_SLTU, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `XOR:       trans_inst = {`ALU_XOR, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SRL:       trans_inst = {`ALU_SRL, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `SRA:       trans_inst = {`ALU_SRA, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `OR:        trans_inst = {`ALU_OR,  `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `AND:       trans_inst = {`ALU_AND, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};

            //`FENCE:     trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            //`FENCE_I:   trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};
            `ECALL:     trans_inst = {`ALU_X,   `OP1_X,   `OP2_X,   `MEM_X, `REN_X, `WB_X,   `CSR_E};
            //`EBREAK:    trans_inst = {`ALU_ADD, `OP1_RS1, `OP2_IMS, `MEM_X, `REN_S, `WB_MEM, `CSR_X};

            `CSRRW:     trans_inst = {`ALU_COPY1, `OP1_RS1, `OP2_X, `MEM_X, `REN_S, `WB_CSR, `CSR_W};
            `CSRRS:     trans_inst = {`ALU_COPY1, `OP1_RS1, `OP2_X, `MEM_X, `REN_S, `WB_CSR, `CSR_S};
            `CSRRC:     trans_inst = {`ALU_COPY1, `OP1_RS1, `OP2_X, `MEM_X, `REN_S, `WB_CSR, `CSR_C};
            `CSRRWI:    trans_inst = {`ALU_COPY1, `OP1_IMZ, `OP2_X, `MEM_X, `REN_S, `WB_CSR, `CSR_W};
            `CSRRSI:    trans_inst = {`ALU_COPY1, `OP1_IMZ, `OP2_X, `MEM_X, `REN_S, `WB_CSR, `CSR_S};
            `CSRRCI:    trans_inst = {`ALU_COPY1, `OP1_IMZ, `OP2_X, `MEM_X, `REN_S, `WB_CSR, `CSR_C};
            default:    trans_inst = {`ALU_X, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_X, `WB_X, `CSR_X};
        endcase


//------------------------------------------------------------------------
// EX stage

    riscv32i_alu alu(
        .op1_data(op1_data),
        .op2_data(op2_data),
        .exe_fun(exe_fun),
        .result(alu_out),
        .br_flg(br_flg)
    );

    assign br_target = pc_reg + imm_b_sext;

//------------------------------------------------------------------------
// MEM stage

    assign dmem_addr = alu_out;
    assign dmem_wdata = rs2_data;
    assign dmem_wen = mem_wen[0];

//------------------------------------------------------------------------
// WB stage

    reg [`WORD_LEN-1:0] wb_data;

    always @(*)
        case (wb_sel)
            `WB_MEM:    wb_data = dmem_rdata; 
            `WB_PC:     wb_data = pc_plus4;
            //`WB_CSR:    wb_data = csr_rdata; 
            default:    wb_data = alu_out;
        endcase

    always @(posedge clock)
        if(rf_wen == `REN_S && rd_addr != 0) regfile[rd_addr] = wb_data;

endmodule