module mem_stage(
    input  wire [31:0] alu_result, // 来自EX阶段的计算地址
    input  wire [31:0] store_val,  // Store指令要写入的数据
    input  wire        mem_read,   // 访存读取使能
    input  wire        mem_write,  // 访存写入使能
    input  wire [2:0]  mem_op,     // 存储操作类型(funct3)，区分字节/半字/字及符号扩展
    input  wire        clk,
    input  wire        reset,
    output reg  [31:0] data_out    // 读出的数据（扩展为32位）
);
    // 简单数据存储器：大小设定为256字节 (64字)
    localparam DMEM_BYTES = 256;
    reg [7:0] dmem [0:DMEM_BYTES-1];
    
    // 初始化数据存储器（可选）：可用 $readmemh 载入初始数据
    initial begin
        $readmemh("datamem_init.hex", dmem);
    end
    
    wire [7:0] addr = alu_result[7:0];
    // 地址低2位
    wire [1:0] addr_offset = addr[1:0]; 
    // 计算字对齐地址的基址索引（字节数组索引）
    wire [31:0] base_index = addr & 32'hFFFFFFFC;
    integer offset;
    reg [7:0] byte_val;reg [15:0] half_val;reg [31:0] word_val;//reg [7:0] byte_val;
    always @(*) begin
        data_out = 32'b0;
        if (mem_read) begin
            case (mem_op)
                3'b000: begin // LB: 读1字节，有符号扩展
                    offset = addr_offset;
                    byte_val = dmem[addr];
                    data_out = {{24{byte_val[7]}}, byte_val};
                end
                3'b001: begin // LH: 读2字节，有符号扩展
                    // 组合两个字节，小端序
                    half_val = { dmem[addr + 1], dmem[addr] };
                    data_out = {{16{half_val[15]}}, half_val};
                end
                3'b010: begin // LW: 读4字节
                    word_val = { dmem[addr+3], dmem[addr+2], dmem[addr+1], dmem[addr] };
                    data_out = word_val;
                end
                3'b100: begin // LBU: 读1字节，零扩展
                    byte_val = dmem[addr];
                    data_out = {24'b0, byte_val};
                end
                3'b101: begin // LHU: 读2字节，零扩展
                    //reg [15:0] half_val;
                    half_val = { dmem[addr + 1], dmem[addr] };
                    data_out = {16'b0, half_val};
                end
                default: begin
                    data_out = 32'b0;
                end
            endcase
        end
    end
    
    // 写存储器：同步写在时钟上升沿执行
    always @(posedge clk) begin
        if (mem_write) begin
            case (mem_op)
                3'b000: begin // SB: 写1字节
                    dmem[addr] <= store_val[7:0];
                end
                3'b001: begin // SH: 写2字节
                    dmem[addr]   <= store_val[7:0];
                    dmem[addr+1] <= store_val[15:8];
                end
                3'b010: begin // SW: 写4字节
                    dmem[addr]   <= store_val[7:0];
                    dmem[addr+1] <= store_val[15:8];
                    dmem[addr+2] <= store_val[23:16];
                    dmem[addr+3] <= store_val[31:24];
                end
                default: ;
            endcase
        end
    end

endmodule
