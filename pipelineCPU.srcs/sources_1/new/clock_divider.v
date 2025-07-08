`timescale 1ns / 1ps
module clock_divider #(parameter WIDTH = 27)(
    input wire clk_in,
    input wire reset,
    output wire clk_out
);
    reg [WIDTH-1:0] counter;
    always @(posedge clk_in or posedge reset) begin
        if (reset)
            counter <= 0;
        else
            counter <= counter + 1;
    end
    assign clk_out = counter[WIDTH-1];
endmodule
