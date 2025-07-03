`timescale 1ns / 1ps
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
    output  [7:0] seg,
    output  [7:0] an
);

    reg [14:0] cnt;
    
   always @ (posedge clk, posedge reset)
      if (reset)
        cnt <= 0;
      else
        cnt <= cnt + 1'b1;
 
    wire seg7_clk = cnt[14]; 
    
    reg [2:0] seg7_addr;
  
  always @ (posedge seg7_clk, posedge reset)
    if(reset)
      seg7_addr <= 0;
    else
      seg7_addr <= seg7_addr + 1'b1;
      
    reg [7:0] o_sel_r;
  
  always @ (*)
    case(seg7_addr)
      7 : o_sel_r = 8'b01111111;
      6 : o_sel_r = 8'b10111111;
      5 : o_sel_r = 8'b11011111;
      4 : o_sel_r = 8'b11101111;
      3 : o_sel_r = 8'b11110111;
      2 : o_sel_r = 8'b11111011;
      1 : o_sel_r = 8'b11111101;
      0 : o_sel_r = 8'b11111110;
    endcase
  
  reg [31:0] i_data_store;
  always @ (posedge clk, posedge reset)
    if(reset)
      i_data_store <= 0;
    else begin
      i_data_store[3:0] <= d0; 
      i_data_store[7:4] <= d1;
      i_data_store[11:8] <= d2;
      i_data_store[15:12] <= d3;
      i_data_store[19:16] <= d4;
      i_data_store[23:20] <= d5;
      i_data_store[27:24] <= d6;
      i_data_store[31:28] <= d7;
    end
  reg [7:0] seg_data_r;
  always @ (*)
    case(seg7_addr)
      0 : seg_data_r = i_data_store[3:0];
      1 : seg_data_r = i_data_store[7:4];
      2 : seg_data_r = i_data_store[11:8];
      3 : seg_data_r = i_data_store[15:12];
      4 : seg_data_r = i_data_store[19:16];
      5 : seg_data_r = i_data_store[23:20];
      6 : seg_data_r = i_data_store[27:24];
      7 : seg_data_r = i_data_store[31:28];
    endcase
  
  reg [7:0] o_seg_r;
  always @ (posedge clk, posedge reset)
    if(reset)
      o_seg_r <= 8'hff;
    else
      case(seg_data_r)
          4'h0 : o_seg_r <= 8'hC0;
          4'h1 : o_seg_r <= 8'hF9;
          4'h2 : o_seg_r <= 8'hA4;
          4'h3 : o_seg_r <= 8'hB0;
          4'h4 : o_seg_r <= 8'h99;
          4'h5 : o_seg_r <= 8'h92;
          4'h6 : o_seg_r <= 8'h82;
          4'h7 : o_seg_r <= 8'hF8;
          4'h8 : o_seg_r <= 8'h80;
          4'h9 : o_seg_r <= 8'h90;
          4'hA : o_seg_r <= 8'h88;
          4'hB : o_seg_r <= 8'h83;
          4'hC : o_seg_r <= 8'hC6;
          4'hD : o_seg_r <= 8'hA1;
          4'hE : o_seg_r <= 8'h86;
          4'hF : o_seg_r <= 8'h8E;
      endcase
      
  assign an = o_sel_r;
  assign seg = o_seg_r;

endmodule
   /* reg [2:0] scan;
    reg [3:0] cur;
    always @(posedge seg7_clk or posedge reset) begin
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
endmodule*/
