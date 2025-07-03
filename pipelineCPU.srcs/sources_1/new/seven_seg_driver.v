module seven_seg_driver(
    input wire clk,
    input wire reset,
    input wire [3:0] d0,
    input wire [3:0] d1,
    input wire [3:0] d2,
    input wire [3:0] d3,
    input wire [3:0] d4,
    input wire [3:0] d5,
    input wire [3:0] d6,
    input wire [3:0] d7,
    output reg [7:0] seg,
    output reg [7:0] an
);
    reg [2:0] scan;
    reg [3:0] cur;
    always @(posedge clk or posedge reset) begin
        if (reset)
            scan <= 0;
        else
            scan <= scan + 1;
    end
    always @(*) begin
        case(scan)
            3'd0: begin an = 8'b11111110; cur = d0; end
            3'd1: begin an = 8'b11111101; cur = d1; end
            3'd2: begin an = 8'b11111011; cur = d2; end
            3'd3: begin an = 8'b11110111; cur = d3; end
            3'd4: begin an = 8'b11101111; cur = d4; end
            3'd5: begin an = 8'b11011111; cur = d5; end
            3'd6: begin an = 8'b10111111; cur = d6; end
            default: begin an = 8'b01111111; cur = d7; end
        endcase
    end
    always @(*) begin
        case(cur)
            4'h0: seg = 8'b11000000;
            4'h1: seg = 8'b11111001;
            4'h2: seg = 8'b10100100;
            4'h3: seg = 8'b10110000;
            4'h4: seg = 8'b10011001;
            4'h5: seg = 8'b10010010;
            4'h6: seg = 8'b10000010;
            4'h7: seg = 8'b11111000;
            4'h8: seg = 8'b10000000;
            4'h9: seg = 8'b10010000;
            4'ha: seg = 8'b10001000;
            4'hb: seg = 8'b10000011;
            4'hc: seg = 8'b11000110;
            4'hd: seg = 8'b10100001;
            4'he: seg = 8'b10000110;
            4'hf: seg = 8'b10001110;
            default: seg = 8'b11111111;
        endcase
    end
endmodule
