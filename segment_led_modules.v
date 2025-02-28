`timescale 1ns/1ps

/* driven low active
0:7'b1000000
1:7'b1111001
2:7'b0100100
3:7'b0110000
4:7'b0011001
5:7'b0010010
6:7'b0000010
7:7'b1111000
8:7'b0000000
9:7'b0010000
A:7'b0001000
b:7'b0000011
c:7'b0100111
d:7'b0100001
E:7'b0000110
F:7'b0001110
*/


module display_7segment(
    input CP,//suggest >=4*60Hz
    input RST,
    input [31:0] input_patten,//combine with digit_to_7seg_patten
    output reg [7:0] SSEG_CA,
    output reg [3:0] SSEG_AN
);

    reg [1:0] reg_count;

    always@(posedge CP)
    if(RST)reg_count<=0;
    else reg_count<=reg_count+1;

    always@(posedge CP)
    case(reg_count)
        2'b00:begin SSEG_AN<=4'b0111;SSEG_CA<=input_patten[31:24];end
        2'b01:begin SSEG_AN<=4'b1011;SSEG_CA<=input_patten[23:16];end
        2'b10:begin SSEG_AN<=4'b1101;SSEG_CA<=input_patten[15:8];end
        2'b11:begin SSEG_AN<=4'b1110;SSEG_CA<=input_patten[7:0];end
    endcase

endmodule



module digit_to_7seg_patten(
    input [3:0] input_digit,//allowed hexadecimal number
    output reg [6:0] patten//don't contain decimal point
);

    always@(input_digit)
    case(input_digit)
        4'h0:patten<=7'b1000000;
        4'h1:patten<=7'b1111001;
        4'h2:patten<=7'b0100100;
        4'h3:patten<=7'b0110000;
        4'h4:patten<=7'b0011001;
        4'h5:patten<=7'b0010010;
        4'h6:patten<=7'b0000010;
        4'h7:patten<=7'b1111000;
        4'h8:patten<=7'b0000000;
        4'h9:patten<=7'b0010000;
        4'ha:patten<=7'b0001000;
        4'hb:patten<=7'b0000011;
        4'hc:patten<=7'b0100111;
        4'hd:patten<=7'b0100001;
        4'he:patten<=7'b0000110;
        4'hf:patten<=7'b0001110;
        default:patten<=7'b1111111;
    endcase

endmodule

module seg_led_top(
    input CLK,
    input [15:0] num,
    output [7:0] SSEG_CA,
    output [3:0] SSEG_AN
);

    wire [31:0] led_patten;
    assign led_patten[31]=1'b1,led_patten[23]=1'b1,led_patten[15]=1'b1,led_patten[7]=1'b1;

    digit_to_7seg_patten dg1(.input_digit(num[3:0]), .patten(led_patten[6:0]));
    digit_to_7seg_patten dg2(.input_digit(num[7:4]), .patten(led_patten[14:8]));
    digit_to_7seg_patten dg3(.input_digit(num[11:8]), .patten(led_patten[22:16]));
    digit_to_7seg_patten dg4(.input_digit(num[15:12]), .patten(led_patten[30:24]));

    display_7segment display_led_7segment(.CP(CLK), .input_patten(led_patten),
    .SSEG_CA(SSEG_CA), .SSEG_AN(SSEG_AN));


endmodule