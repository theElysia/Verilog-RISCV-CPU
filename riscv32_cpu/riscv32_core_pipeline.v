`timescale 1ns/1ps
`include "riscv32_Consts.v"
`include "riscv32_Instructions.v" 
`include "riscv32_alu.v"

module riscv32_core_pipeline (
    input clock,
    input reset,
    output [`WORD_LEN-1:0] imem_addr,
    input [`WORD_LEN-1:0] imem_inst,
    output [`WORD_LEN-1:0] dmem_addr,
    output [`WORD_LEN-1:0] dmem_wdata,
    output dmem_wen,
    input [`WORD_LEN-1:0] dmem_rdata,
    output exit,
    // for vga debugger
    output reg [`WORD_LEN-1:0] core_id_reg_pc,
    output reg [`WORD_LEN-1:0] core_id_reg_inst,
    output reg [`WORD_LEN-1:0] core_exe_reg_op1_data,
    output reg [`WORD_LEN-1:0] core_exe_reg_op2_data,
    output reg [`WORD_LEN-1:0] core_exe_alu_out,
    output reg [`WORD_LEN*32-1:0] core_regfile
);
    
    reg [`WORD_LEN-1:0] regfile [0:31];
    //reg [`WORD_LEN-1:0] csr_regfile [0:4095];

//------------------------------------------------------------------------
// cpu rhythm

/*
    reg [2:0] clk_reg;
    wire clk_b=clk_reg[2];

    initial clk_reg=0;
    always @(posedge clock)clk_reg<=clk_reg+1;*/
    wire clk_b = clock;

    integer i;
    always @(*) begin
        core_id_reg_pc          = id_reg_pc;
        core_id_reg_inst        = id_reg_inst;
        core_exe_alu_out        = exe_alu_out;
        core_exe_reg_op1_data   = exe_reg_op1_data;
        core_exe_reg_op2_data   = exe_reg_op2_data;
        for(i=0;i<32;i=i+1) core_regfile[i*32+31 -: 32] = regfile[i];
    end

//------------------------------------------------------------------------
// pipeline state register declaration

    wire stall_flg;

    // IF/ID stage
    reg [`WORD_LEN-1:0] id_reg_pc, id_reg_inst;

    // ID/EX stage
    reg [`WORD_LEN-1:0] exe_reg_pc, exe_reg_op1_data, exe_reg_op2_data, exe_reg_rs2_data,
        exe_reg_imm_i_sext, exe_reg_imm_s_sext, exe_reg_imm_b_sext, exe_reg_imm_u_shifted,
        exe_reg_imm_z_uext;
    reg [`ADDR_LEN-1:0] exe_reg_wb_addr;
    reg [`EXE_FUN_LEN-1:0] exe_reg_exe_fun;
    reg [`MEM_LEN-1:0] exe_reg_mem_wen;
    reg [`REN_LEN-1:0] exe_reg_rf_wen;
    reg [`WB_SEL_LEN-1:0] exe_reg_wb_sel;
    reg [`CSR_ADDR_LEN-1:0] exe_reg_csr_addr;
    reg [`CSR_LEN-1:0] exe_reg_csr_cmd;

    // EX/MEM stage
    reg [`WORD_LEN-1:0] mem_reg_pc, mem_reg_op1_data, mem_reg_rs2_data,
        mem_reg_imm_z_uext, mem_reg_alu_out;
    reg [`ADDR_LEN-1:0] mem_reg_wb_addr;
    reg [`MEM_LEN-1:0] mem_reg_mem_wen;
    reg [`REN_LEN-1:0] mem_reg_rf_wen;
    reg [`WB_SEL_LEN-1:0] mem_reg_wb_sel;
    reg [`CSR_ADDR_LEN-1:0] mem_reg_csr_addr;
    reg [`CSR_LEN-1:0] mem_reg_csr_cmd;

    // MEM/WB stage
    reg [`ADDR_LEN-1:0] wb_reg_wb_addr;
    reg [`WORD_LEN-1:0] wb_reg_wb_data;
    reg [`REN_LEN-1:0] wb_reg_rf_wen;


//------------------------------------------------------------------------
// IF stage

    reg [`WORD_LEN-1:0] if_reg_pc, if_pc_next;
    wire [`WORD_LEN-1:0] if_pc_plus4 = if_reg_pc+4;
    wire [`WORD_LEN-1:0] if_inst;

    assign imem_addr = if_reg_pc;
    assign if_inst = imem_inst;

    initial if_reg_pc <= `START_ADDR;
    always @(posedge clk_b, posedge reset)
    if(reset) if_reg_pc <= `START_ADDR;
    else if_reg_pc <= if_pc_next;

    always @(*) begin
        if(reset) if_pc_next = `START_ADDR;
        else if(exe_br_flg) if_pc_next = exe_br_target;
        else if(exe_jmp_flg) if_pc_next = exe_alu_out;
        //else if(inst==`ECALL)if_pc_next=csr_regfile[12'h0305];
        else if(stall_flg) if_pc_next = if_reg_pc;
        else if(if_reg_pc==`PROGRAM_END) if_pc_next = `PROGRAM_END;
        else if_pc_next = if_pc_plus4;
    end

    assign exit = (if_reg_pc==`PROGRAM_END);

//------------------------------------------------------------------------
// IF/ID register

    always@(posedge clk_b) id_reg_pc <= stall_flg ? id_reg_pc : if_reg_pc;
    always@(posedge clk_b)
        if(exe_br_flg|exe_jmp_flg) id_reg_inst <= `BUBBLE;
        else if(stall_flg) id_reg_inst <= id_reg_inst;
        else id_reg_inst <= if_inst;

//------------------------------------------------------------------------
// ID stage
    
    //to detect stall, id from reg temporarily
    wire [4:0]  id_rs1_addr_b = id_reg_inst[19:15],
                id_rs2_addr_b = id_reg_inst[24:20];
    
    // ex data hazard -> stall
    wire id_rs1_data_hazard = (exe_reg_rf_wen == `REN_S) && (|exe_reg_wb_addr) && (id_rs1_addr_b == exe_reg_wb_addr);
    wire id_rs2_data_hazard = (exe_reg_rf_wen == `REN_S) && (|exe_reg_wb_addr) && (id_rs2_addr_b == exe_reg_wb_addr);
    assign stall_flg = id_rs1_data_hazard || id_rs2_data_hazard;
    
    // br jp stall -> id_inst=BUBBLE
    wire [`WORD_LEN-1:0] id_inst = (exe_br_flg || exe_jmp_flg || stall_flg) ? `BUBBLE : id_reg_inst;

    wire [4:0]  id_rs1_addr = id_inst[19:15],
                id_rs2_addr = id_inst[24:20],
                id_rd_addr  = id_inst[11:7];

    reg [`WORD_LEN-1:0] id_rs1_data, id_rs2_data;
    always @(*)
        if(id_rs1_addr==0) id_rs1_data = 32'b0;
        else if((id_rs1_addr == mem_reg_wb_addr) && (mem_reg_rf_wen == `REN_S)) id_rs1_data = mem_wb_data;
        else if((id_rs1_addr == wb_reg_wb_addr) && (wb_reg_rf_wen == `REN_S)) id_rs1_data = wb_reg_wb_data;
        else id_rs1_data = regfile[id_rs1_addr];
    always @(*)
        if(id_rs2_addr == 0) id_rs2_data = 32'b0;
        else if((id_rs2_addr == mem_reg_wb_addr) && (mem_reg_rf_wen == `REN_S)) id_rs2_data = mem_wb_data;
        else if((id_rs2_addr == wb_reg_wb_addr) && (wb_reg_rf_wen == `REN_S)) id_rs2_data = wb_reg_wb_data;
        else id_rs2_data = regfile[id_rs2_addr];

    wire [11:0] id_imm_i = id_inst[31:20];
    wire [11:0] id_imm_s = {id_inst[31:25],id_inst[11:7]};
    wire [11:0] id_imm_b = {id_inst[31],id_inst[7],id_inst[30:25],id_inst[11:8]};
    wire [19:0] id_imm_j = {id_inst[31],id_inst[19:12],id_inst[20],id_inst[30:21]};
    wire [19:0] id_imm_u = id_inst[31:12];
    wire [4:0]  id_imm_z = id_inst[19:15];
    // signed/unsigned extend
    wire [`WORD_LEN-1:0]
                id_imm_i_sext = {{20{id_imm_i[11]}}, id_imm_i},
                id_imm_s_sext = {{20{id_imm_s[11]}}, id_imm_s},
                id_imm_b_sext = {{19{id_imm_b[11]}}, id_imm_b, 1'b0},
                id_imm_j_sext = {{11{id_imm_j[19]}}, id_imm_j, 1'b0},
                id_imm_u_shifted = {id_imm_u, 12'b0},
                id_imm_z_uext = {27'b0, id_imm_z};

    wire [`EXE_FUN_LEN-1:0] id_exe_fun;
    wire [`OP1_LEN-1:0] id_op1_sel;
    wire [`OP2_LEN-1:0] id_op2_sel;
    wire [`MEM_LEN-1:0] id_mem_wen;
    wire [`REN_LEN-1:0] id_rf_wen;
    wire [`WB_SEL_LEN-1:0] id_wb_sel;
    //wire [`MW_LEN-1:0] mw_sel;
    wire [`CSR_LEN-1:0] id_csr_cmd;

    localparam TRANS_INST_LEN = `EXE_FUN_LEN+`OP1_LEN+`OP2_LEN+`MEM_LEN+`REN_LEN+`WB_SEL_LEN+`CSR_LEN;
    reg [TRANS_INST_LEN-1:0] trans_inst;
    assign {id_exe_fun, id_op1_sel, id_op2_sel, id_mem_wen, id_rf_wen, id_wb_sel, id_csr_cmd} = trans_inst;

    reg [`WORD_LEN-1:0] id_op1_data, id_op2_data;

    always @(*)
        case (id_op1_sel)
            `OP1_RS1:   id_op1_data = id_rs1_data;
            `OP1_PC:    id_op1_data = id_reg_pc;
            `OP1_IMZ:   id_op1_data = id_imm_z_uext;
            default:    id_op1_data = 0; 
        endcase
    
    always @(*)
        case (id_op2_sel)
            `OP2_RS2:   id_op2_data = id_rs2_data;
            `OP2_IMI:   id_op2_data = id_imm_i_sext;
            `OP2_IMS:   id_op2_data = id_imm_s_sext;
            `OP2_IMJ:   id_op2_data = id_imm_j_sext;
            `OP2_IMU:   id_op2_data = id_imm_u_shifted;
            default:    id_op2_data = 0; 
        endcase

    wire [11:0] id_csr_addr = (id_csr_cmd == `CSR_E) ? 12'h342 : id_inst[31:20];   

    always @(*)
        casex (id_inst)
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

            `ALU_MUL:   trans_inst = {`ALU_MUL,    `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `ALU_MULH:  trans_inst = {`ALU_MULH,   `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `ALU_MULHSU:trans_inst = {`ALU_MULHSU, `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};
            `ALU_MULHU: trans_inst = {`ALU_MULHU,  `OP1_RS1, `OP2_RS2, `MEM_X, `REN_S, `WB_ALU, `CSR_X};

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
// ID/EX register

    always @(posedge clk_b) begin
        exe_reg_pc              <= id_reg_pc;
        exe_reg_op1_data        <= id_op1_data;
        exe_reg_op2_data        <= id_op2_data;
        exe_reg_rs2_data        <= id_rs2_data;
        exe_reg_wb_addr         <= id_rd_addr;
        exe_reg_rf_wen          <= id_rf_wen;
        exe_reg_exe_fun         <= id_exe_fun;
        exe_reg_wb_sel          <= id_wb_sel;
        exe_reg_imm_i_sext      <= id_imm_i_sext;
        exe_reg_imm_s_sext      <= id_imm_s_sext;
        exe_reg_imm_b_sext      <= id_imm_b_sext;
        exe_reg_imm_u_shifted   <= id_imm_u_shifted;
        exe_reg_imm_z_uext      <= id_imm_z_uext;
        exe_reg_csr_addr        <= id_csr_addr;
        exe_reg_csr_cmd         <= id_csr_cmd;
        exe_reg_mem_wen         <= id_mem_wen;
    end

//------------------------------------------------------------------------
// EX stage

    wire [`WORD_LEN-1:0] exe_alu_out, exe_br_target;
    wire exe_br_flg, exe_jmp_flg;

    riscv32im_alu alu(
        .op1_data(exe_reg_op1_data),
        .op2_data(exe_reg_op2_data),
        .exe_fun(exe_reg_exe_fun),
        .result(exe_alu_out),
        .br_flg(exe_br_flg)
    );

    assign exe_br_target = (exe_reg_pc + exe_reg_imm_b_sext);

    assign exe_jmp_flg = (exe_reg_wb_sel == `WB_PC);

//------------------------------------------------------------------------
// EX/MEM register

    always @(posedge clk_b) begin
        mem_reg_pc              <= exe_reg_pc;
        mem_reg_op1_data        <= exe_reg_op1_data;
        mem_reg_rs2_data        <= exe_reg_rs2_data;
        mem_reg_wb_addr         <= exe_reg_wb_addr;
        mem_reg_alu_out         <= exe_alu_out;
        mem_reg_rf_wen          <= exe_reg_rf_wen;
        mem_reg_wb_sel          <= exe_reg_wb_sel;
        mem_reg_imm_z_uext      <= exe_reg_imm_z_uext;
        mem_reg_csr_addr        <= exe_reg_csr_addr;
        mem_reg_csr_cmd         <= exe_reg_csr_cmd;
        mem_reg_mem_wen         <= exe_reg_mem_wen;
    end

//------------------------------------------------------------------------
// MEM stage

    assign dmem_addr = mem_reg_alu_out;
    assign dmem_wdata = mem_reg_rs2_data;
    assign dmem_wen = mem_reg_mem_wen[0];

    reg [`WORD_LEN-1:0] mem_wb_data;
    always @(*)
        case (mem_reg_wb_sel)
            `WB_MEM: mem_wb_data = dmem_rdata;
            `WB_PC:  mem_wb_data = mem_reg_pc + 4;
        //    `WB_CSR: mem_wb_data = csr_rdata;
            default: mem_wb_data = mem_reg_alu_out;
        endcase

//------------------------------------------------------------------------
// MEM/WB register

    always @(posedge clk_b) begin
        wb_reg_wb_addr          <= mem_reg_wb_addr;
        wb_reg_rf_wen           <= mem_reg_rf_wen;
        wb_reg_wb_data          <= mem_wb_data;
    end

//------------------------------------------------------------------------
// WB stage

    always @(posedge clk_b)
        if(wb_reg_rf_wen == `REN_S && (|wb_reg_wb_addr)) regfile[wb_reg_wb_addr] = wb_reg_wb_data;

endmodule