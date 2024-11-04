`timescale 1ns/1ps

// delay(logic gate):   cp4 1;  cg4 2; co ci+2
// but in fpga, it is realized by LUT
// so it maybe more wasteful
module adder_cpg (
    input [3:0] cp,
    input [3:0] cg,
    input ci,
    output cp4,
    output cg4,
    output [2:0] co
);
    
    assign cp4 = &cp[3:0];

    assign cg4 = cg[3]|(cp[3]&cg[2])|(cp[3]&cp[2]&cg[1])|(cp[3]&cp[2]&cp[1]&cg[0]);

    assign co[2] = cg[2]|(cp[2]&cg[1])|(cp[2]&cp[1]&cg[0])|(cp[2]&cp[1]&cp[0]&ci);

    assign co[1] = cg[1]|(cp[1]&cg[0])|(cp[1]&cp[0]&ci);

    assign co[0] = cg[0]|(cp[0]&ci);

endmodule


// delay: cp 2; cg co 3; s 4
module adder_4 (
    input [3:0] a,
    input [3:0] b,
    input ci,
    output reg [3:0] s,
    output cp,
    output cg
);

    wire [3:0] icp = a|b;
    wire [3:0] icg = a&b;
    wire [3:0] ico;
    assign ico[0] = ci;

    adder_cpg cpg1(.cp(icp), .cg(icg), .ci(ci), .cp4(cp), .cg4(cg), .co(ico[3:1]));

    integer i;
    always@(*)for(i=0;i<4;i=i+1) s[i] = ico[i]^(icp[i]&~icg[i]);
    
endmodule


// delay: cp 4; cg 5; s 8
module adder_16 (
    input [15:0] a,
    input [15:0] b,
    input ci,
    output [15:0] s,
    output cp,
    output cg
);

    wire [3:0] ico, icp, icg;
    assign ico[0] = ci;

    genvar gen_i;
    generate
        for(gen_i=0;gen_i<4;gen_i=gen_i+1)begin:gen_adder_4
            adder_4 ad1(.a(a[4*gen_i +: 4]), .b(b[4*gen_i +: 4]), .ci(ico[gen_i]),
            .s(s[4*gen_i +: 4]), .cp(icp[gen_i]), .cg(icg[gen_i]));
        end
    endgenerate

    adder_cpg cpg2(.cp(icp), .cg(icg), .ci(ci), .cp4(cp), .cg4(cg), .co(ico[3:1]));
    
endmodule


// delay: cp 6; cg 7; s 12
module adder_32 (
    input [31:0] a,
    input [31:0] b,
    input ci,
    output [31:0] s,
    output cp,
    output cg
);

    wire [1:0] ico, icp, icg;
    assign ico[0] = ci;
    
    adder_16 ad1(.a(a[15:0]), .b(b[15:0]), .ci(ico[0]),
    .s(s[15:0]), .cp(icp[0]), .cg(icg[0]));
    adder_16 ad2(.a(a[31:16]), .b(b[31:16]), .ci(ico[1]),
    .s(s[31:16]), .cp(icp[1]), .cg(icg[1]));

    assign  ico[1] = icg[0]|(icp[0]&ico[0]),
            cp = &icp,
            cg = icg[1]|(icp[1]&icg[0]);

endmodule


// delay: cp 6; cg 7; s 12
module adder_64 (
    input [63:0] a,
    input [63:0] b,
    input ci,
    output [63:0] s,
    output cp,
    output cg
);

    wire [3:0] ico, icp, icg;
    assign ico[0] = ci;

    genvar gen_i;
    generate
        for(gen_i=0;gen_i<4;gen_i=gen_i+1)begin:gen_adder_16
            adder_16 ad1(.a(a[16*gen_i +: 16]), .b(b[16*gen_i +: 16]), .ci(ico[gen_i]),
            .s(s[16*gen_i +: 16]), .cp(icp[gen_i]), .cg(icg[gen_i]));
        end
    endgenerate

    adder_cpg cpg3(.cp(icp), .cg(icg), .ci(ci), .cp4(cp), .cg4(cg), .co(ico[3:1]));
    
endmodule