`timescale 1ns / 1ps
module bin_to_bcd6(
    input  wire [31:0] bin,
    output reg  [3:0] d5,
    output reg  [3:0] d4,
    output reg  [3:0] d3,
    output reg  [3:0] d2,
    output reg  [3:0] d1,
    output reg  [3:0] d0
);
    integer i;
    reg [31:0] value;
    reg [3:0] digit [0:5];
    always @(*) begin
        value = bin;
        for(i=0;i<6;i=i+1) begin
            digit[i] = value % 10;
            value = value / 10;
        end
        d0 = digit[0];
        d1 = digit[1];
        d2 = digit[2];
        d3 = digit[3];
        d4 = digit[4];
        d5 = digit[5];
    end
endmodule
