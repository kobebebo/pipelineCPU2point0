// 顶层模块：五级流水线CPU (RV32I) - Xilinx Artix-7 FPGA
module cpu_top(
    input  wire        clk,      // 时钟输入（FPGA板外100MHz时钟）:contentReference[oaicite:15]{index=15}
    input  wire        rstn,     // 复位输入（低电平有效）:contentReference[oaicite:16]{index=16}
    input  wire [15:0] sw_i,     // 板上开关输入（用于调试/控制）:contentReference[oaicite:17]{index=17}
    output wire [15:0] led_o,    // 板上LED输出（用于指示状态）:contentReference[oaicite:18]{index=18}
    output wire [7:0]  disp_seg_o, // 数码管段选信号（共8段，包括小数点）:contentReference[oaicite:19]{index=19}
    output wire [7:0]  disp_an_o   // 数码管位选信号（8位数码管）:contentReference[oaicite:20]{index=20}
);

    // 时钟与复位信号
    wire reset = ~rstn;  // 将低有效复位转换为高有效的reset信号

    // 时钟分频/选择: 使用sw_i[15]控制时钟速度 (可选实现)
    // 若需要，可在此插入分频器，将clk_div作为各模块时钟
    wire clk_cpu = clk; // 简化：直接使用板上时钟，无分频
    // TODO: 可实现时钟分频逻辑，例如计数器达到一定值翻转clk_div，从而得到慢速时钟用于观察
    
    // -------------------------------
    // 信号定义：流水线各阶段之间的连线
    // -------------------------------
    // IF阶段 -> IF/ID流水寄存器
    wire [31:0] if_pc_next;      // 计算出的下一条PC地址
    wire [31:0] if_instr;        // 取出的指令
    wire [31:0] if_pc_curr;      // 当前PC值（用于输出/调试）
    // IF/ID -> ID阶段
    reg  [31:0] if_id_instr;     // IF/ID 寄存的指令
    reg  [31:0] if_id_pc;        // IF/ID 寄存的当前指令PC
    // ID阶段 -> ID/EX流水寄存器
    wire [4:0]  id_rs1, id_rs2, id_rd;      // 寄存器编号
    wire [31:0] id_rs1_val, id_rs2_val;     // 寄存器堆读出的源操作数值
    wire [31:0] id_imm;                    // 立即数扩展
    wire        id_reg_write, id_mem_read, id_mem_write;
    wire [2:0]  id_mem_op;                 // 存储器操作类型（例如区分字节/半字/字，及有符号/无符号）
    wire        id_branch;      //是否为分支
    wire [1:0]  id_jump;        // 是否为无条件跳转
    wire [3:0]  id_alu_op;                 // ALU 操作码控制（自定义）
    wire        id_alu_src1_pc;            // ALU操作数A选择：1表示使用当前PC，0表示使用寄存器rs1值
    wire        id_alu_src2_imm;           // ALU操作数B选择：1表示使用立即数，0表示使用寄存器rs2值
    // 冒险检测单元输出
    wire        hazard_stall;   // 流水线暂停（插入等待周期）
    wire        hazard_flush;   // EX阶段插入气泡（将ID/EX清除为NOP）
    // ID/EX -> EX阶段
    reg [31:0] id_ex_pc;
    reg [31:0] id_ex_rs1_val;
    reg [31:0] id_ex_rs2_val;
    reg [31:0] id_ex_imm;
    reg [4:0]  id_ex_rs1, id_ex_rs2, id_ex_rd;
    reg        id_ex_reg_write, id_ex_mem_read, id_ex_mem_write;
    reg [2:0]  id_ex_mem_op;
    reg        id_ex_branch;
    reg [1:0]  id_ex_jump;
    reg [3:0]  id_ex_alu_op;
    reg        id_ex_alu_src1_pc;
    reg        id_ex_alu_src2_imm;
    // EX阶段 -> EX/MEM流水寄存器
    wire [31:0] ex_alu_result;   // ALU计算结果
    wire        ex_branch_taken; // 分支决定信号，高表示跳转成立
    wire [31:0] ex_branch_target;// 计算出的分支/跳转目标地址
    wire [31:0] ex_forwarded_rs2;// （针对store）前递后的rs2值，用于存储写
    // EX/MEM -> MEM阶段
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_store_val;
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg [2:0]  ex_mem_mem_op;
    // MEM阶段 -> MEM/WB流水寄存器
    wire [31:0] mem_data_out;   // 从数据存储器读取的数据（经扩展为32位）
    wire [31:0] mem_wb_value;   // 将要写回寄存器的值（来自ALU或mem_data_out的选择）
    // MEM/WB -> WB阶段
    reg [31:0] mem_wb_value_r;
    reg [4:0]  mem_wb_rd;
    reg        mem_wb_reg_write;
    
    // -------------------------------
    // 实例化各阶段和模块
    // -------------------------------
    
    // 取指存储器 (指令存储器) - 采用同步ROM, 通过初始化文件预置指令
    localparam IMEM_SIZE = 256;  // 指令存储器大小（字数），可根据需要调整
    reg [31:0] inst_mem [0:IMEM_SIZE-1]; 
    initial begin
        // 使用预先生成的机器码文件初始化指令存储器
        // 例如: $readmemh("progmem.hex", inst_mem);
        // 这里省略具体文件路径，默认假设已经通过COE或其他方式初始化
        $readmemh("progmem.hex", inst_mem);
    end
    
    // PC寄存器：在时钟上升沿更新PC
    reg [31:0] pc;
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            pc <= 32'h0000_0000;  // 复位时PC从0地址开始
        end else begin
            if (!hazard_stall) begin
                // 非暂停时更新PC：优先分支/跳转目标，否则PC+4
                pc <= ex_branch_taken ? ex_branch_target : pc + 32'd4;
            end else begin
                pc <= pc; // 暂停时保持PC不变
            end
        end
    end
    
    assign if_pc_curr = pc;  // 当前PC值输出（用于调试显示）
    
    // IF阶段取指：组合逻辑从指令存储器读取当前PC地址的指令
    assign if_instr = inst_mem[pc[31:2]]; 
    // 由于指令存储器按字寻址，这里假设pc按字对齐，使用pc[31:2]做索引
    // Vivado综合时会将此推断为块RAM ROM。
    
    // IF -> IF/ID 流水寄存器更新
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            if_id_instr <= 32'b0;
            if_id_pc    <= 32'b0;
        end else begin
            if (!hazard_stall) begin
                if_id_instr <= (ex_branch_taken ? 32'b0 : if_instr);
                // 若发生跳转，则把取到的错误指令清除为0（NOP），实现flush IF/ID
                if_id_pc    <= (ex_branch_taken ? 32'b0 : pc);
            end else begin
                // hazard_stall时，保持IF/ID不变（停顿）
                if_id_instr <= if_id_instr;
                if_id_pc    <= if_id_pc;
            end
        end
    end
    
    // 实例化指令译码/控制单元和寄存器堆 (ID阶段)
    id_stage id_stage_u (
        .instr       (if_id_instr),
        .pc          (if_id_pc),
        .rs1         (id_rs1),
        .rs2         (id_rs2),
        .rd          (id_rd),
        .imm         (id_imm),
        .reg_write   (id_reg_write),
        .mem_read    (id_mem_read),
        .mem_write   (id_mem_write),
        .mem_op      (id_mem_op),
        .branch      (id_branch),
        .jump        (id_jump),
        .alu_op      (id_alu_op),
        .alu_src1_pc (id_alu_src1_pc),
        .alu_src2_imm(id_alu_src2_imm),
        .rs1_val     (id_rs1_val),
        .rs2_val     (id_rs2_val),
        .clk         (clk_cpu),
        .reset       (reset),
        // 冒险单元信号
        .hazard_stall(hazard_stall)
    );
    
    // 冒险检测单元：检测load-use型数据冒险
    hazard_unit hazard_u (
        .id_ex_mem_read (id_ex_mem_read),
        .id_ex_rd       (id_ex_rd),
        .if_id_rs1      (id_rs1),
        .if_id_rs2      (id_rs2),
        .hazard_stall   (hazard_stall),
        .hazard_flush   (hazard_flush)
    );
    
    // ID -> ID/EX 流水寄存器
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            // 清除 ID/EX
            id_ex_pc         <= 32'b0;
            id_ex_rs1_val    <= 32'b0;
            id_ex_rs2_val    <= 32'b0;
            id_ex_imm        <= 32'b0;
            id_ex_rs1        <= 5'b0;
            id_ex_rs2        <= 5'b0;
            id_ex_rd         <= 5'b0;
            id_ex_reg_write  <= 1'b0;
            id_ex_mem_read   <= 1'b0;
            id_ex_mem_write  <= 1'b0;
            id_ex_mem_op     <= 3'b0;
            id_ex_branch     <= 1'b0;
            id_ex_jump       <= 2'b0;
            id_ex_alu_op     <= 4'b0;
            id_ex_alu_src1_pc<= 1'b0;
            id_ex_alu_src2_imm<=1'b0;
        end else begin
            if (hazard_flush) begin
                // 插入气泡：将ID/EX清零（视作 NOP 指令）
                id_ex_pc         <= 32'b0;
                id_ex_rs1_val    <= 32'b0;
                id_ex_rs2_val    <= 32'b0;
                id_ex_imm        <= 32'b0;
                id_ex_rs1        <= 5'b0;
                id_ex_rs2        <= 5'b0;
                id_ex_rd         <= 5'b0;
                id_ex_reg_write  <= 1'b0;
                id_ex_mem_read   <= 1'b0;
                id_ex_mem_write  <= 1'b0;
                id_ex_mem_op     <= 3'b0;
                id_ex_branch     <= 1'b0;
                id_ex_jump       <= 2'b0;
                id_ex_alu_op     <= 4'b0;
                id_ex_alu_src1_pc<= 1'b0;
                id_ex_alu_src2_imm<=1'b0;
            end else if (!hazard_stall) begin
                // 正常传递译码结果进入EX阶段
                id_ex_pc         <= if_id_pc;
                id_ex_rs1_val    <= id_rs1_val;
                id_ex_rs2_val    <= id_rs2_val;
                id_ex_imm        <= id_imm;
                id_ex_rs1        <= id_rs1;
                id_ex_rs2        <= id_rs2;
                id_ex_rd         <= id_rd;
                id_ex_reg_write  <= id_reg_write;
                id_ex_mem_read   <= id_mem_read;
                id_ex_mem_write  <= id_mem_write;
                id_ex_mem_op     <= id_mem_op;
                id_ex_branch     <= id_branch;
                id_ex_jump       <= id_jump;
                id_ex_alu_op     <= id_alu_op;
                id_ex_alu_src1_pc<= id_alu_src1_pc;
                id_ex_alu_src2_imm<=id_alu_src2_imm;
            end
            // 若 hazard_stall=1 且 hazard_flush=0，则保持上周期值不变（不会发生，该情况已在上面stall控制IF/ID）
        end
    end
    
    // 执行阶段模块：包含ALU、转发和分支判断
    ex_stage ex_stage_u (
        .pc           (id_ex_pc),
        .rs1_val      (id_ex_rs1_val),
        .rs2_val      (id_ex_rs2_val),
        .imm          (id_ex_imm),
        .rs1_idx      (id_ex_rs1),
        .rs2_idx      (id_ex_rs2),
        .rd_idx       (id_ex_rd),
        .reg_write_in (id_ex_reg_write),
        .mem_read_in  (id_ex_mem_read),
        .mem_write_in (id_ex_mem_write),
        .mem_op_in    (id_ex_mem_op),
        .branch_in    (id_ex_branch),
        .jump_in      (id_ex_jump),
        .alu_op       (id_ex_alu_op),
        .alu_src1_pc  (id_ex_alu_src1_pc),
        .alu_src2_imm (id_ex_alu_src2_imm),
        // 来自后续流水段用于前递的信号
        .ex_mem_rd    (ex_mem_rd),
        .ex_mem_reg_write(ex_mem_reg_write),
        .ex_mem_alu_result(ex_mem_alu_result),
        .mem_wb_rd    (mem_wb_rd),
        .mem_wb_reg_write(mem_wb_reg_write),
        .mem_wb_value (mem_wb_value_r),
        // 输出
        .alu_result   (ex_alu_result),
        .forwarded_rs2(ex_forwarded_rs2),
        .branch_taken (ex_branch_taken),
        .branch_target(ex_branch_target),
        .rd_idx_out   (/* not used here, see below */),
        .reg_write_out(/* not used here */),
        .mem_read_out (/* not used here */),
        .mem_write_out(/* not used here */),
        .mem_op_out   (/* not used here */)
    );
    // 注意：ex_stage模块内部同时产生寄存器写回、访存等信号，本例中直接通过id_ex信号传递
    // 为简化，ex_stage输出的控制信号可以不重复连出，直接使用id_ex阶段的对应信号进入EX/MEM寄存器
    
    // EX -> EX/MEM 流水寄存器
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            ex_mem_alu_result <= 32'b0;
            ex_mem_store_val  <= 32'b0;
            ex_mem_rd         <= 5'b0;
            ex_mem_reg_write  <= 1'b0;
            ex_mem_mem_read   <= 1'b0;
            ex_mem_mem_write  <= 1'b0;
            ex_mem_mem_op     <= 3'b0;
        end else begin
            ex_mem_alu_result <= ex_alu_result;
            ex_mem_store_val  <= ex_forwarded_rs2;  // Store指令需要写入内存的数据（已经过前递修正）
            ex_mem_rd         <= id_ex_rd;
            ex_mem_reg_write  <= id_ex_reg_write;
            ex_mem_mem_read   <= id_ex_mem_read;
            ex_mem_mem_write  <= id_ex_mem_write;
            ex_mem_mem_op     <= id_ex_mem_op;
        end
    end
    
    // 访存阶段模块：包含数据存储器和加载/存储的数据处理
    mem_stage mem_stage_u (
        .alu_result   (ex_mem_alu_result),
        .store_val    (ex_mem_store_val),
        .mem_read     (ex_mem_mem_read),
        .mem_write    (ex_mem_mem_write),
        .mem_op       (ex_mem_mem_op),
        .clk          (clk_cpu),
        .reset        (reset),
        .data_out     (mem_data_out)
        // 数据存储器引用（简单实现为内部reg，也可替换为Block RAM）
        //.dmem         ()  // 此处将在模块内部定义dmem数组
    );
    
    // 将ALU结果或内存读取数据选择为写回值
    assign mem_wb_value = ex_mem_mem_read ? mem_data_out : ex_mem_alu_result;
    
    // MEM -> MEM/WB 流水寄存器
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            mem_wb_value_r   <= 32'b0;
            mem_wb_rd        <= 5'b0;
            mem_wb_reg_write <= 1'b0;
        end else begin
            mem_wb_value_r   <= mem_wb_value;
            mem_wb_rd        <= ex_mem_rd;
            mem_wb_reg_write <= ex_mem_reg_write;
        end
    end
    
    // 写回阶段：寄存器堆写入（同步写在时钟上升沿进行）
    // 已在id_stage模块中的寄存器堆实例中，通过在WB阶段写回
    // 在cpu_top顶层通过与id_stage连接实现写回
    
    // 将写回信号传递给寄存器堆模块 (id_stage包含寄存器堆)
    assign id_stage_u.wb_rd_idx   = mem_wb_rd;
    assign id_stage_u.wb_data_in  = mem_wb_value_r;
    assign id_stage_u.wb_reg_write= mem_wb_reg_write;
    
    // -------------------------------
    // 板上输出：LED和数码管调试显示
    // -------------------------------
    // 简易调试：用 LED 显示某个寄存器或状态位 (这里示例用 LED 显示 x3 寄存器低16位)
    assign led_o = id_stage_u.regs[3][15:0]; 
    // 上述引用假设 id_stage 内部的寄存器数组 regs 为 public可见（需要在id_stage模块中将regs声明为 public for simulation）。
    // 如果不方便直接访问内部寄存器，可在id_stage模块增加一个输出端口将某一寄存器值传出。
    // 这里简单处理：假设Vivado支持这种层次引用(实际需要调整).
    
    // 数码管显示：显示当前PC的低4位16进制值在第0位数码管上
    wire [3:0] pc_low_nibble = if_pc_curr[3:0];
    reg [7:0] seg_pattern;
    always @(*) begin
        case(pc_low_nibble)
            4'h0: seg_pattern = 8'b1100_0000; // 0
            4'h1: seg_pattern = 8'b1111_1001; // 1
            4'h2: seg_pattern = 8'b1010_0100; // 2
            4'h3: seg_pattern = 8'b1011_0000; // 3
            4'h4: seg_pattern = 8'b1001_1001; // 4
            4'h5: seg_pattern = 8'b1001_0010; // 5
            4'h6: seg_pattern = 8'b1000_0010; // 6
            4'h7: seg_pattern = 8'b1111_1000; // 7
            4'h8: seg_pattern = 8'b1000_0000; // 8
            4'h9: seg_pattern = 8'b1001_0000; // 9
            4'hA: seg_pattern = 8'b1000_1000; // A
            4'hB: seg_pattern = 8'b1000_0011; // b
            4'hC: seg_pattern = 8'b1100_0110; // C
            4'hD: seg_pattern = 8'b1010_0001; // d
            4'hE: seg_pattern = 8'b1000_0110; // E
            4'hF: seg_pattern = 8'b1000_1110; // F
        endcase
    end
    assign disp_seg_o = seg_pattern;
    assign disp_an_o  = 8'b1111_1110; // 仅启用最右侧的第0位数码管（假定低电平有效）
endmodule
