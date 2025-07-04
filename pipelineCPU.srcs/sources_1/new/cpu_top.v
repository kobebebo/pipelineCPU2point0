`timescale 1ns / 1ps
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

    // 调试/控制信号
    wire debug_pause   = sw_i[13];      // =1暂停CPU运行
    wire display_regs  = sw_i[14];     // =1显示寄存器
    wire fast_clk_sel  = sw_i[15];     // =1使用原始时钟

    // 时钟分频：当sw_i[15]=0时使用慢速时钟，便于观察
    wire slow_clk;
    clock_divider #(.WIDTH(26)) div_u(
        .clk_in (clk),
        .reset  (reset),
        .clk_out(slow_clk)
    );
    wire clk_cpu = fast_clk_sel ? clk : slow_clk;
    
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
    // 调试显示相关
    reg  [4:0]  dbg_reg_idx;
    wire [31:0] dbg_reg_val;
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
            if (!hazard_stall && !debug_pause) begin
                // 非暂停时更新PC：优先分支/跳转目标，否则PC+4
                pc <= ex_branch_taken ? ex_branch_target : pc + 32'd4;
            end else begin
                pc <= pc; // 暂停时保持PC不变
            end
        end
    end
    
    // IF阶段取指：组合逻辑从指令存储器读取当前PC地址的指令
    assign if_instr = inst_mem[pc[31:2]]; 
    assign if_pc_curr = if_instr;  // 当前PC值输出（用于调试显示）
    
    // 由于指令存储器按字寻址，这里假设pc按字对齐，使用pc[31:2]做索引
    // Vivado综合时会将此推断为块RAM ROM。
    
    // IF -> IF/ID 流水寄存器更新
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            if_id_instr <= 32'b0;
            if_id_pc    <= 32'b0;
        end else begin
            if (!hazard_stall && !debug_pause) begin
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
        .hazard_stall(hazard_stall),
        .dbg_reg_idx (dbg_reg_idx),
        .dbg_reg_val (dbg_reg_val)
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
    // Branch taken in EX stage requires flushing ID/EX stage
    wire branch_flush = id_ex_branch|id_ex_jump[1]|id_ex_jump[0]|id_branch|id_jump[0]|id_jump[1];
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
            if (hazard_flush || branch_flush) begin
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
            end else if (!hazard_stall && !debug_pause) begin
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
        end else if (!debug_pause) begin
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
        end else if (!debug_pause) begin
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
    // LED显示示例：直接显示x3寄存器低16位
    assign led_o = id_stage_u.regs[3][15:0];

    // 调试寄存器索引循环
    reg [31:0] disp_cnt;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            disp_cnt <= 0;
            dbg_reg_idx <= 0;
        end else if (display_regs) begin
            disp_cnt <= disp_cnt + 1;
            if (disp_cnt[26] == 1'b1) begin
                disp_cnt <=0;
                if (dbg_reg_idx == 5'd31)
                    dbg_reg_idx <= 0;
                else
                    dbg_reg_idx <= dbg_reg_idx + 1;
            end
        end else begin
            disp_cnt <= 0;
            dbg_reg_idx <= 0;
        end
    end

    // PC和寄存器值数字准备
    wire [3:0] pc_d0 = if_pc_curr[3:0];
    wire [3:0] pc_d1 = if_pc_curr[7:4];
    wire [3:0] pc_d2 = if_pc_curr[11:8];
    wire [3:0] pc_d3 = if_pc_curr[15:12];
    wire [3:0] pc_d4 = if_pc_curr[19:16];
    wire [3:0] pc_d5 = if_pc_curr[23:20];
    wire [3:0] pc_d6 = if_pc_curr[27:24];
    wire [3:0] pc_d7 = if_pc_curr[31:28];

    wire [3:0] bcd0,bcd1,bcd2,bcd3,bcd4,bcd5;
    bin_to_bcd6 bcd_u(
        .bin(dbg_reg_val),
        .d5(bcd5),.d4(bcd4),.d3(bcd3),.d2(bcd2),.d1(bcd1),.d0(bcd0)
    );

    reg [3:0] dig0,dig1,dig2,dig3,dig4,dig5,dig6,dig7;
    always @(*) begin
        if (display_regs) begin
            dig0 = bcd0; dig1 = bcd1; dig2 = bcd2; dig3 = bcd3; dig4 = bcd4; dig5 = bcd5;
            dig6 = dbg_reg_idx[3:0];
            dig7 = {3'b0, dbg_reg_idx[4]};
        end else begin
            dig0 = pc_d0; dig1 = pc_d1; dig2 = pc_d2; dig3 = pc_d3;
            dig4 = pc_d4; dig5 = pc_d5; dig6 = pc_d6; dig7 = pc_d7;
        end
    end

    // 数码管扫描驱动
    seven_seg_driver disp_u(
        .clk   (clk),
        .reset (reset),
        .d0(dig0),.d1(dig1),.d2(dig2),.d3(dig3),
        .d4(dig4),.d5(dig5),.d6(dig6),.d7(dig7),
        .seg(disp_seg_o),
        .an (disp_an_o)
    );
endmodule