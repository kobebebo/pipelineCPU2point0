module forward_unit(
    input  wire [4:0]  ID_EX_rs1,
    input  wire [4:0]  ID_EX_rs2,
    input  wire [4:0]  EX_MEM_rd,
    input  wire        EX_MEM_reg_write,
    input  wire [4:0]  MEM_WB_rd,
    input  wire        MEM_WB_reg_write,
    output reg  [1:0]  forwardA,
    output reg  [1:0]  forwardB
);
    always @(*) begin
        forwardA = 2'b00;
        forwardB = 2'b00;
        if (EX_MEM_reg_write && EX_MEM_rd != 5'd0 && EX_MEM_rd == ID_EX_rs1) begin
            forwardA = 2'b10; // forward from EX_MEM½×¶Î
        end else if (MEM_WB_reg_write && MEM_WB_rd != 5'd0 && MEM_WB_rd == ID_EX_rs1) begin
            forwardA = 2'b01; // forward from MEM_WB½×¶Î
        end
        if (EX_MEM_reg_write && EX_MEM_rd != 5'd0 && EX_MEM_rd == ID_EX_rs2) begin
            forwardB = 2'b10;
        end else if (MEM_WB_reg_write && MEM_WB_rd != 5'd0 && MEM_WB_rd == ID_EX_rs2) begin
            forwardB = 2'b01;
        end
    end
endmodule
