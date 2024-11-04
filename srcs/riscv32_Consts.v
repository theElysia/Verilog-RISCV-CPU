`ifndef RISCV32_CONSTS
`define RISCV32_CONSTS

`define WORD_LEN        32
`define START_ADDR      `WORD_LEN'b0
`define BUBBLE          `WORD_LEN'h00000013     // [ADDI x0,x0,0] = BUBBLE
`define UNIMP           `WORD_LEN'hc0001073     // [CSRRW x0, cycle, x0]
`define ADDR_LEN        5
`define CSR_ADDR_LEN    12
`define VLEN            128
`define LMUL_LEN        2
`define SEW_LEN         11
`define VL_ADDR         12'hc20
`define VTYPE_ADDR      12'hc21


`define EXE_FUN_LEN     5
`define ALU_X           `EXE_FUN_LEN'd0
`define ALU_ADD         `EXE_FUN_LEN'd1
`define ALU_SUB         `EXE_FUN_LEN'd2
`define ALU_AND         `EXE_FUN_LEN'd3
`define ALU_OR          `EXE_FUN_LEN'd4
`define ALU_XOR         `EXE_FUN_LEN'd5
`define ALU_SLL         `EXE_FUN_LEN'd6
`define ALU_SRL         `EXE_FUN_LEN'd7
`define ALU_SRA         `EXE_FUN_LEN'd8
`define ALU_SLT         `EXE_FUN_LEN'd9
`define ALU_SLTU        `EXE_FUN_LEN'd10
`define BR_BEQ          `EXE_FUN_LEN'd11
`define BR_BNE          `EXE_FUN_LEN'd12
`define BR_BLT          `EXE_FUN_LEN'd13
`define BR_BGE          `EXE_FUN_LEN'd14
`define BR_BLTU         `EXE_FUN_LEN'd15
`define BR_BGEU         `EXE_FUN_LEN'd16
`define ALU_JALR        `EXE_FUN_LEN'd17
`define ALU_COPY1       `EXE_FUN_LEN'd18
`define ALU_VADDVV      `EXE_FUN_LEN'd19
`define VSET            `EXE_FUN_LEN'd20
`define ALU_PCBT        `EXE_FUN_LEN'd21


`define OP1_LEN         2
`define OP1_X           `OP1_LEN'd0
`define OP1_RS1         `OP1_LEN'd1
`define OP1_PC          `OP1_LEN'd2
`define OP1_IMZ         `OP1_LEN'd3

`define OP2_LEN         3
`define OP2_X           `OP2_LEN'd0
`define OP2_RS2         `OP2_LEN'd1
`define OP2_IMI         `OP2_LEN'd2
`define OP2_IMS         `OP2_LEN'd3
`define OP2_IMJ         `OP2_LEN'd4
`define OP2_IMU         `OP2_LEN'd5

`define MEM_LEN         2
`define MEM_X           `MEM_LEN'd0
`define MEM_S           `MEM_LEN'd1
`define MEM_V           `MEM_LEN'd2

`define REN_LEN         2
`define REN_X           `REN_LEN'd0
`define REN_S           `REN_LEN'd1
`define REN_V           `REN_LEN'd2

`define WB_SEL_LEN      3
`define WB_X            `WB_SEL_LEN'd0
`define WB_ALU          `WB_SEL_LEN'd0
`define WB_MEM          `WB_SEL_LEN'd1
`define WB_PC           `WB_SEL_LEN'd2
`define WB_CSR          `WB_SEL_LEN'd3
`define WB_MEM_V        `WB_SEL_LEN'd4
`define WB_ALU_V        `WB_SEL_LEN'd5
`define WB_VL           `WB_SEL_LEN'd6

`define MW_LEN          3
`define MW_X            `MW_LEN'd0
`define MW_W            `MW_LEN'd1
`define MW_H            `MW_LEN'd2
`define MW_B            `MW_LEN'd3
`define MW_HU           `MW_LEN'd4
`define MW_BU           `MW_LEN'd5

`define CSR_LEN         3
`define CSR_X           `CSR_LEN'd0
`define CSR_W           `CSR_LEN'd1
`define CSR_S           `CSR_LEN'd2
`define CSR_C           `CSR_LEN'd3
`define CSR_E           `CSR_LEN'd4
`define CSR_V           `CSR_LEN'd5

`endif  //RISCV32_CONSTS