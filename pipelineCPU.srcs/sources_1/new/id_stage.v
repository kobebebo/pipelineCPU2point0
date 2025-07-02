module id_stage(
    input  wire [31:0] instr,        // 来自IF阶段的指令代码
    input  wire [31:0] pc,           // 来自IF阶段的当前指令地址（PC）
    output wire [4:0]  rs1, rs2, rd, // 源寄存器编号(rs1, rs2)和目标寄存器编号(rd)
    output reg  [31:0] imm,          // 立即数扩展值
    output reg         reg_write,    // 是否写回寄存器
    output reg         mem_read,     // 是否访存读取
    output reg         mem_write,    // 是否访存写入
    output reg  [2:0]  mem_op,       // 存储器操作类型：用于区分LB/LH/LW等（funct3直接作为标识）
    output reg         branch,       // 是否为分支指令（条件跳转）
    output reg  [1:0]  jump,         // 是否为无条件跳转指令（JAL/JALR）[1]=jal,[0]=jalr
    output reg  [3:0]  alu_op,       // ALU 操作选择码（由指令功能确定，自定义编码4位）
    output reg         alu_src1_pc,  // ALU操作数1来源：1=使用PC值, 0=使用寄存器rs1值
    output reg         alu_src2_imm, // ALU操作数2来源：1=使用立即数, 0=使用寄存器rs2值
    output wire [31:0] rs1_val, rs2_val, // 从寄存器堆读取的源操作数值
    input  wire [4:0]  wb_rd_idx,    // 写回阶段要写入的寄存器号
    input  wire [31:0] wb_data_in,   // 写回的数据
    input  wire        wb_reg_write, // 写回使能信号
    input  wire        clk,
    input  wire        reset,
    input  wire        hazard_stall  // 冒险暂停信号，高电平表示本周期ID阶段指令停顿，不更新输出
);
    // 将指令字段拆分
    wire [6:0] opcode = instr[6:0];
    wire [4:0] rs1_idx = instr[19:15];
    wire [4:0] rs2_idx = instr[24:20];
    wire [4:0] rd_idx  = instr[11:7];
    wire [2:0] funct3  = instr[14:12];
    wire [6:0] funct7  = instr[31:25];
    
    assign rs1 = rs1_idx;
    assign rs2 = rs2_idx;
    assign rd  = rd_idx;
    
    // 寄存器堆定义 (32 x 32位 寄存器)
    reg [31:0] regs [0:31];
    integer i;
    // 异步读，同步写实现：
    assign rs1_val = regs[rs1_idx];
    assign rs2_val = regs[rs2_idx];
    
    // 初始化寄存器（复位时）
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i=0; i<32; i=i+1) begin
                regs[i] <= 32'b0;
            end
            regs[2] <= 32'd128;  // 初始化x2 (sp) = 128:contentReference[oaicite:26]{index=26}
        end else begin
            // 写回阶段 - 在时钟上升沿写寄存器
            if (wb_reg_write && wb_rd_idx != 5'd0) begin
                regs[wb_rd_idx] <= wb_data_in;
            end
        end
    end
    
    // 控制信号默认值（每个周期组合逻辑计算）
    always @(*) begin
        // 默认值（大多数信号默认为0，在匹配指令时设置为1）
        reg_write   = 1'b0;
        mem_read    = 1'b0;
        mem_write   = 1'b0;
        mem_op      = 3'b000;
        branch      = 1'b0;
        jump        = 2'b00;
        alu_src1_pc = 1'b0;
        alu_src2_imm= 1'b0;
        alu_op      = 4'b0000;
        imm         = 32'b0;
        
        // 提取指令不同类型的立即数位段并符号扩展/对齐
        case (opcode)
            7'b0110011: begin
                // R-type: 没有立即数
                imm = 32'b0;
            end
            7'b0010011, // I-type (算术/逻辑立即数 和 JALR, LB/LH/LW等Load类)
            7'b0000011, // I-type (Load类)
            7'b1100111: begin // JALR
                // I型：imm[11:0] = instr[31:20]，符号扩展至32位
                imm = {{20{instr[31]}}, instr[31:20]};
            end
            7'b0100011: begin
                // S-type (Store): imm[11:0] = instr[31:25] concat instr[11:7]
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            end
            7'b1100011: begin
                // B-type (Branch): imm[12|10:5|4:1|11] = instr[31|30:25|11:8|7] (12位立即数,最低位0)
                imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            end
            7'b0110111, // U-type (LUI)
            7'b0010111: begin // U-type (AUIPC)
                // U型：imm[31:12] = instr[31:12], 低12位填0
                imm = {instr[31:12], 12'b0};
            end
            7'b1101111: begin
                // J-type (JAL): imm[20|10:1|11|19:12] = instr[31|30:21|20|19:12] (21位含符号,最低位0)
                imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            end
            default: begin
                imm = 32'b0;
            end
        endcase
        
        // 根据指令操作码和功能码生成控制信号
        case (opcode)
            7'b0110011: begin  // R-type运算指令
                reg_write = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b0;
                // 根据 funct3/funct7 设置 alu_op
                case (funct3)
                    3'b000: alu_op = (funct7[5] ? 4'b0001  // SUB
                                             : 4'b0000); // ADD
                    3'b001: alu_op = 4'b0010; // SLL (逻辑左移)
                    3'b010: alu_op = 4'b0011; // SLT (有符号比较)
                    3'b011: alu_op = 4'b0100; // SLTU (无符号比较)
                    3'b100: alu_op = 4'b0101; // XOR
                    3'b101: alu_op = (funct7[5] ? 4'b0111  // SRA
                                             : 4'b0110); // SRL
                    3'b110: alu_op = 4'b1000; // OR
                    3'b111: alu_op = 4'b1001; // AND
                    default: alu_op = 4'b0000;
                endcase
            end
            7'b0010011: begin  // I-type 算术/逻辑立即数指令
                reg_write = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1;  // 第二操作数来自立即数
                case (funct3)
                    3'b000: alu_op = 4'b0000; // ADDI -> ADD
                    3'b010: alu_op = 4'b0011; // SLTI -> SLT
                    3'b011: alu_op = 4'b0100; // SLTIU -> SLTU
                    3'b100: alu_op = 4'b0101; // XORI -> XOR
                    3'b110: alu_op = 4'b1000; // ORI -> OR
                    3'b111: alu_op = 4'b1001; // ANDI -> AND
                    3'b001: begin 
                        // SLLI: imm[5:0]的高位应为0（确保逻辑移位）
                        alu_op = 4'b0010; // SLL
                    end
                    3'b101: begin 
                        // SRLI/SRAI: 根据立即数高位区分
                        alu_op = (instr[31:25] == 7'b0100000) ? 4'b0111 // SRAI
                                                              : 4'b0110; // SRLI
                    end
                    default: alu_op = 4'b0000;
                endcase
            end
            7'b0000011: begin  // I-type Load指令 (LB/LH/LW/LBU/LHU)
                reg_write = 1'b1;
                mem_read  = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1; // 地址 = rs1 + imm
                alu_op    = 4'b0000; // 地址计算使用ADD
                mem_op    = funct3;  // 保存load类型，后续用于数据扩展
            end
            7'b0100011: begin  // S-type Store指令 (SB/SH/SW)
                mem_write = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1; // 地址 = rs1 + imm
                alu_op    = 4'b0000; // 地址计算使用ADD
                mem_op    = funct3;  // 保存store类型，后续用于写存储器字节数
            end
            7'b1100011: begin  // B-type 分支指令 (BEQ/BNE/BLT/BGE/BLTU/BGEU)
                branch   = 1'b1;
                // 分支不写寄存器，不访存，但需要计算分支条件
                reg_write = 1'b0;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b0;
                alu_op    = 4'b0000; // ALU默认ADD或SUB均可，这里不实际用ALU输出
                // 分支条件判断将在EX阶段根据rs1_val/rs2_val和funct3进行
            end
            7'b1101111: begin  // J-type 无条件跳转 (JAL)
                jump[1]     = 1'b1;
                reg_write= 1'b1;  // JAL需要写回链接地址到rd
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b0;
                alu_op   = 4'b0000;
                // JAL: rd = PC+4 (链接地址)，PC将跳转至 PC + imm (目标地址)
                // 链接地址的计算可在EX阶段完成
            end
            7'b1100111: begin  // I-type 寄存器跳转 (JALR)
                jump[0]     = 1'b1;
                reg_write= 1'b1;  // JALR写回链接地址
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1; // 可以先用ALU计算目标地址 = rs1 + imm
                alu_op   = 4'b0000;
                // JALR: rd = PC+4， 新PC = (rs1 + imm) & ~1
                // PC+4 和 新PC 计算在EX阶段处理
            end
            7'b0110111: begin  // U-type LUI
                reg_write= 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1;
                // LUI: 将imm（高20位）加载到rd。可视作ALU运算: 0 + imm
                alu_op   = 4'b1010; // 自定义 PASSB 操作码，让ALU输出第二操作数（imm）
            end
            7'b0010111: begin  // U-type AUIPC
                reg_write= 1'b1;
                alu_src1_pc = 1'b1;  // 第一个操作数用PC
                alu_src2_imm= 1'b1;  // 第二操作数用imm
                alu_op   = 4'b0000;  // ALU做加法计算 PC + imm
            end
            default: begin
                // 未定义指令（包括系统指令ECALL/EBREAK等）：不执行操作
                reg_write= 1'b0;
                mem_read = 1'b0;
                mem_write= 1'b0;
                branch   = 1'b0;
                jump     = 2'b00;
                alu_op   = 4'b0000;
            end
        endcase
        
        // 如需要，可以在此添加hazard_stall对控制信号的影响（例如遇到暂停则保持上周期信号）。
        // 然而在本设计中，hazard_stall主要通过冻结流水寄存器实现，无需在此额外处理组合逻辑。
    end

endmodule
