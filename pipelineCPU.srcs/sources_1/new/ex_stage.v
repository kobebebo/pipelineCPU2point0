`timescale 1ns / 1ps
module ex_stage(
    input  wire [31:0] pc,         // 来自ID/EX的当前指令PC
    input  wire [31:0] rs1_val,    // 寄存器rs1值（ID/EX）
    input  wire [31:0] rs2_val,    // 寄存器rs2值（ID/EX）
    input  wire [31:0] imm,        // 立即数（ID/EX）
    input  wire [4:0]  rs1_idx,    // 源寄存器1号（用于前递判断）
    input  wire [4:0]  rs2_idx,    // 源寄存器2号
    input  wire [4:0]  rd_idx,     // 目的寄存器号（ID/EX）
    input  wire        reg_write_in, // 该指令是否写回寄存器
    input  wire        mem_read_in,  // 是否为load指令
    input  wire        mem_write_in, // 是否为store指令
    input  wire [2:0]  mem_op_in,    // 内存操作类型（funct3）
    input  wire        branch_in,    // 是否为条件分支指令
    input  wire [1:0]   jump_in,      // 是否为跳转指令 (JAL/JALR)
    input  wire [3:0]  alu_op,       // ALU操作码（由控制单元给出）
    input  wire        alu_src1_pc,  // ALU操作数1来源：1=PC, 0=寄存器rs1
    input  wire        alu_src2_imm, // ALU操作数2来源：1=立即数,0=寄存器rs2
    // 前递相关输入：来自EX/MEM和MEM/WB阶段结果用于可能的前递
    input  wire [4:0]  ex_mem_rd,
    input  wire        ex_mem_reg_write,
    input  wire [31:0] ex_mem_alu_result,
    input  wire [4:0]  mem_wb_rd,
    input  wire        mem_wb_reg_write,
    input  wire [31:0] mem_wb_value,
    // 输出
    output reg  [31:0] alu_result,    // ALU运算结果输出（EX/MEM）
    output reg  [31:0] forwarded_rs2, // 转发后的rs2值（给store使用的值）
    output reg         branch_taken,  // 分支是否成立（跳转信号）
    output reg  [31:0] branch_target, // 跳转目标地址
    output wire [4:0]  rd_idx_out,    // 传递目的寄存器号
    output wire        reg_write_out, // 传递是否写回信号
    output wire        mem_read_out,  // 传递访存读控制
    output wire        mem_write_out, // 传递访存写控制
    output wire [2:0]  mem_op_out     // 传递内存操作类型
);
    // 将ID阶段的控制信号直接传递输出（大部分控制信号无需修改直接进入下一流水段）
    assign rd_idx_out    = rd_idx;
    assign reg_write_out = reg_write_in;
    assign mem_read_out  = mem_read_in;
    assign mem_write_out = mem_write_in;
    assign mem_op_out    = mem_op_in;
    
    // 前递：选择EX阶段操作数的最终值
    // 默认使用来自ID/EX寄存器的值，如有后续结果相关则替换
    reg [31:0] op_a, op_b;
    reg [31:0] rs2_val_forwarded; // 用于store的rs2最终值
    always @(*) begin
        // 前递给操作数A (rs1)
        if (ex_mem_reg_write && ex_mem_rd != 5'd0 && ex_mem_rd == rs1_idx) begin
            op_a = ex_mem_alu_result;  // 来自上条指令EX/MEM阶段的结果
        end else if (mem_wb_reg_write && mem_wb_rd != 5'd0 && mem_wb_rd == rs1_idx) begin
            op_a = mem_wb_value;       // 来自两条前指令MEM/WB阶段的结果
        end else begin
            op_a = rs1_val;            // 无前递，直接用寄存器值
        end
        
        // 前递给操作数B (rs2) - 注意: ALU的第二操作数可能用imm，不一定用rs2
        if (ex_mem_reg_write && ex_mem_rd != 5'd0 && ex_mem_rd == rs2_idx) begin
            rs2_val_forwarded = ex_mem_alu_result;
        end else if (mem_wb_reg_write && mem_wb_rd != 5'd0 && mem_wb_rd == rs2_idx) begin
            rs2_val_forwarded = mem_wb_value;
        end else begin
            rs2_val_forwarded = rs2_val;
        end
        
        // 根据控制，决定ALU真正输入：
        // 操作数1:
        if (alu_src1_pc) 
            op_a = pc;       // 使用当前PC作为A操作数
        // 否则op_a已是正确的rs1值(含前递)
        // 操作数2:
        if (alu_src2_imm) 
            op_b = imm;      // 使用立即数作为B操作数
        else 
            op_b = rs2_val_forwarded;  // 使用寄存器值作为B操作数
    end
    
    // ALU计算
    reg zero_flag;
    always @(*) begin
        case (alu_op)
            4'b0000: alu_result = op_a + op_b;            // ADD
            4'b0001: alu_result = op_a - op_b;            // SUB
            4'b0010: alu_result = op_a << (op_b[4:0]);    // SLL (逻辑左移)
            4'b0011: alu_result = ($signed(op_a) < $signed(op_b)) ? 32'd1 : 32'd0;  // SLT (有符号比较)
            4'b0100: alu_result = (op_a < op_b) ? 32'd1 : 32'd0;  // SLTU (无符号比较)
            4'b0101: alu_result = op_a ^ op_b;            // XOR
            4'b0110: alu_result = op_a >> (op_b[4:0]);    // SRL (逻辑右移)
            4'b0111: alu_result = ($signed(op_a)) >>> (op_b[4:0]); // SRA (算术右移)
            4'b1000: alu_result = op_a | op_b;            // OR
            4'b1001: alu_result = op_a & op_b;            // AND
            4'b1010: alu_result = op_b;                   // PASSB (直接输出第二操作数，用于LUI)
            default: alu_result = op_a + op_b;
        endcase
        // 计算零标志用于分支判断
        
        zero_flag = (alu_result == 32'b0);
    //end
    
    
    
    // 分支与跳转判定
    //always @(*) begin
        branch_taken = 1'b0;
        branch_target = 32'b0;
        if (branch_in) begin
            // 条件分支，根据funct3决定判断条件
            case (mem_op_in[2:0])  // reuse mem_op_in存储了原instr的funct3位，可表示分支类型
                3'b000: branch_taken = (op_a == op_b);   // BEQ
                3'b001: branch_taken = (op_a != op_b);   // BNE
                3'b100: branch_taken = ($signed(op_a) < $signed(op_b));  // BLT
                3'b101: branch_taken = ($signed(op_a) >= $signed(op_b)); // BGE
                3'b110: branch_taken = (op_a < op_b);    // BLTU
                3'b111: branch_taken = (op_a >= op_b);   // BGEU (无符号>=)
                default: branch_taken = 1'b0;
            endcase
            if (branch_taken) begin
                // 目标地址 = 当前PC + 立即数偏移
                branch_target = pc + imm;
            end
        end
        if (jump_in[1]||jump_in[0]) begin
            // JAL or JALR: 无条件跳转
            branch_taken = 1'b1;
            if (jump_in[1]) begin
                // JAL: 目标 = pc + imm (imm在ID阶段已计算为21位偏移)
                branch_target = pc + imm;
            end else begin
                // JALR: opcode 1100111
                // 目标 = (rs1 + imm) & ~1
                branch_target = (op_a + imm) & 32'hFFFFFFFE;
            end
            // JAL/JALR需要将返回地址PC+4写入rd:
            // 这里直接将 alu_result 改为 pc+4，以便写回阶段写入
            alu_result = pc + 32'd4;
        end
    end
    // forwarded_rs2 用于访存阶段Store的数据输入
    always @(*) begin
        forwarded_rs2 = rs2_val_forwarded;
    end
endmodule