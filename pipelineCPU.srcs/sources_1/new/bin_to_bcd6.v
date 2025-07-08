`timescale 1ns / 1ps
module bin_to_bcd6(
    input  wire [31:0] bin,
    input wire [31:0]addr,
    output reg  [3:0] d7,
    output reg  [3:0] d6,
    output reg  [3:0] d5,
    output reg  [3:0] d3,
    output reg  [3:0] d2,
    output reg  [3:0] d1,
    output reg  [3:0] d0
);
    integer i;
    reg [27:0] value;
    reg [3:0] digit [0:7];
    always @(*) begin
        value[12:0] <= bin[12:0];
        value[27:16]<=addr[11:0];
        value[15:13]<=3'b0;
        for(i=0;i<4;i=i+1) begin
            digit[i] = value[12:0] % 10;
            value[12:0] = value[12:0] / 10;
        end
        for(i=5;i<8;i=i+1)begin
            digit[i] = value[27:16] % 10;
            value[27:16] = value[27:16] / 10;
        end
        d0 = digit[0];
        d1 = digit[1];
        d2 = digit[2];
        d3 = digit[3];
        d5 = digit[5];
        d6 = digit[6];
        d7 = digit[7];
    end
endmodule
