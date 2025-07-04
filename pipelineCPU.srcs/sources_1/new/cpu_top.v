`timescale 1ns / 1ps
// ����ģ�飺�弶��ˮ��CPU (RV32I) - Xilinx Artix-7 FPGA
module cpu_top(
    input  wire        clk,      // ʱ�����루FPGA����100MHzʱ�ӣ�:contentReference[oaicite:15]{index=15}
    input  wire        rstn,     // ��λ���루�͵�ƽ��Ч��:contentReference[oaicite:16]{index=16}
    input  wire [15:0] sw_i,     // ���Ͽ������루���ڵ���/���ƣ�:contentReference[oaicite:17]{index=17}
    output wire [15:0] led_o,    // ����LED���������ָʾ״̬��:contentReference[oaicite:18]{index=18}
    output wire [7:0]  disp_seg_o, // ����ܶ�ѡ�źţ���8�Σ�����С���㣩:contentReference[oaicite:19]{index=19}
    output wire [7:0]  disp_an_o   // �����λѡ�źţ�8λ����ܣ�:contentReference[oaicite:20]{index=20}
);

    // ʱ���븴λ�ź�
    wire reset = ~rstn;  // ������Ч��λת��Ϊ����Ч��reset�ź�

    // ����/�����ź�
    wire debug_pause   = sw_i[13];      // =1��ͣCPU����
    wire display_regs  = sw_i[14];     // =1��ʾ�Ĵ���
    wire fast_clk_sel  = sw_i[15];     // =1ʹ��ԭʼʱ��

    // ʱ�ӷ�Ƶ����sw_i[15]=0ʱʹ������ʱ�ӣ����ڹ۲�
    wire slow_clk;
    clock_divider #(.WIDTH(26)) div_u(
        .clk_in (clk),
        .reset  (reset),
        .clk_out(slow_clk)
    );
    wire clk_cpu = fast_clk_sel ? clk : slow_clk;
    
    // -------------------------------
    // �źŶ��壺��ˮ�߸��׶�֮�������
    // -------------------------------
    // IF�׶� -> IF/ID��ˮ�Ĵ���
    wire [31:0] if_pc_next;      // ���������һ��PC��ַ
    wire [31:0] if_instr;        // ȡ����ָ��
    wire [31:0] if_pc_curr;      // ��ǰPCֵ���������/���ԣ�
    // IF/ID -> ID�׶�
    reg  [31:0] if_id_instr;     // IF/ID �Ĵ��ָ��
    reg  [31:0] if_id_pc;        // IF/ID �Ĵ�ĵ�ǰָ��PC
    // ID�׶� -> ID/EX��ˮ�Ĵ���
    wire [4:0]  id_rs1, id_rs2, id_rd;      // �Ĵ������
    wire [31:0] id_rs1_val, id_rs2_val;     // �Ĵ����Ѷ�����Դ������ֵ
    wire [31:0] id_imm;                    // ��������չ
    wire        id_reg_write, id_mem_read, id_mem_write;
    wire [2:0]  id_mem_op;                 // �洢���������ͣ����������ֽ�/����/�֣����з���/�޷��ţ�
    wire        id_branch;      //�Ƿ�Ϊ��֧
    wire [1:0]  id_jump;        // �Ƿ�Ϊ��������ת
    wire [3:0]  id_alu_op;                 // ALU ��������ƣ��Զ��壩
    wire        id_alu_src1_pc;            // ALU������Aѡ��1��ʾʹ�õ�ǰPC��0��ʾʹ�üĴ���rs1ֵ
    wire        id_alu_src2_imm;           // ALU������Bѡ��1��ʾʹ����������0��ʾʹ�üĴ���rs2ֵ
    // ð�ռ�ⵥԪ���
    wire        hazard_stall;   // ��ˮ����ͣ������ȴ����ڣ�
    wire        hazard_flush;   // EX�׶β������ݣ���ID/EX���ΪNOP��
    // ������ʾ���
    reg  [4:0]  dbg_reg_idx;
    wire [31:0] dbg_reg_val;
    // ID/EX -> EX�׶�
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
    // EX�׶� -> EX/MEM��ˮ�Ĵ���
    wire [31:0] ex_alu_result;   // ALU������
    wire        ex_branch_taken; // ��֧�����źţ��߱�ʾ��ת����
    wire [31:0] ex_branch_target;// ������ķ�֧/��תĿ���ַ
    wire [31:0] ex_forwarded_rs2;// �����store��ǰ�ݺ��rs2ֵ�����ڴ洢д
    // EX/MEM -> MEM�׶�
    reg [31:0] ex_mem_alu_result;
    reg [31:0] ex_mem_store_val;
    reg [4:0]  ex_mem_rd;
    reg        ex_mem_reg_write;
    reg        ex_mem_mem_read;
    reg        ex_mem_mem_write;
    reg [2:0]  ex_mem_mem_op;
    // MEM�׶� -> MEM/WB��ˮ�Ĵ���
    wire [31:0] mem_data_out;   // �����ݴ洢����ȡ�����ݣ�����չΪ32λ��
    wire [31:0] mem_wb_value;   // ��Ҫд�ؼĴ�����ֵ������ALU��mem_data_out��ѡ��
    // MEM/WB -> WB�׶�
    reg [31:0] mem_wb_value_r;
    reg [4:0]  mem_wb_rd;
    reg        mem_wb_reg_write;
    
    // -------------------------------
    // ʵ�������׶κ�ģ��
    // -------------------------------
    
    // ȡָ�洢�� (ָ��洢��) - ����ͬ��ROM, ͨ����ʼ���ļ�Ԥ��ָ��
    localparam IMEM_SIZE = 256;  // ָ��洢����С�����������ɸ�����Ҫ����
    reg [31:0] inst_mem [0:IMEM_SIZE-1]; 
    initial begin
        // ʹ��Ԥ�����ɵĻ������ļ���ʼ��ָ��洢��
        // ����: $readmemh("progmem.hex", inst_mem);
        // ����ʡ�Ծ����ļ�·����Ĭ�ϼ����Ѿ�ͨ��COE��������ʽ��ʼ��
        $readmemh("progmem.hex", inst_mem);
    end
    
    // PC�Ĵ�������ʱ�������ظ���PC
    reg [31:0] pc;
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            pc <= 32'h0000_0000;  // ��λʱPC��0��ַ��ʼ
        end else begin
            if (!hazard_stall && !debug_pause) begin
                // ����ͣʱ����PC�����ȷ�֧/��תĿ�꣬����PC+4
                pc <= ex_branch_taken ? ex_branch_target : pc + 32'd4;
            end else begin
                pc <= pc; // ��ͣʱ����PC����
            end
        end
    end
    
    // IF�׶�ȡָ������߼���ָ��洢����ȡ��ǰPC��ַ��ָ��
    assign if_instr = inst_mem[pc[31:2]]; 
    assign if_pc_curr = if_instr;  // ��ǰPCֵ��������ڵ�����ʾ��
    
    // ����ָ��洢������Ѱַ���������pc���ֶ��룬ʹ��pc[31:2]������
    // Vivado�ۺ�ʱ�Ὣ���ƶ�Ϊ��RAM ROM��
    
    // IF -> IF/ID ��ˮ�Ĵ�������
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            if_id_instr <= 32'b0;
            if_id_pc    <= 32'b0;
        end else begin
            if (!hazard_stall && !debug_pause) begin
                if_id_instr <= (ex_branch_taken ? 32'b0 : if_instr);
                // ��������ת�����ȡ���Ĵ���ָ�����Ϊ0��NOP����ʵ��flush IF/ID
                if_id_pc    <= (ex_branch_taken ? 32'b0 : pc);
            end else begin
                // hazard_stallʱ������IF/ID���䣨ͣ�٣�
                if_id_instr <= if_id_instr;
                if_id_pc    <= if_id_pc;
            end
        end
    end
    
    // ʵ����ָ������/���Ƶ�Ԫ�ͼĴ����� (ID�׶�)
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
        // ð�յ�Ԫ�ź�
        .hazard_stall(hazard_stall),
        .dbg_reg_idx (dbg_reg_idx),
        .dbg_reg_val (dbg_reg_val)
    );
    
    // ð�ռ�ⵥԪ�����load-use������ð��
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
    // ID -> ID/EX ��ˮ�Ĵ���
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            // ��� ID/EX
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
                // �������ݣ���ID/EX���㣨���� NOP ָ�
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
                // ������������������EX�׶�
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
            // �� hazard_stall=1 �� hazard_flush=0���򱣳�������ֵ���䣨���ᷢ�����������������stall����IF/ID��
        end
    end
    
    // ִ�н׶�ģ�飺����ALU��ת���ͷ�֧�ж�
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
        // ���Ժ�����ˮ������ǰ�ݵ��ź�
        .ex_mem_rd    (ex_mem_rd),
        .ex_mem_reg_write(ex_mem_reg_write),
        .ex_mem_alu_result(ex_mem_alu_result),
        .mem_wb_rd    (mem_wb_rd),
        .mem_wb_reg_write(mem_wb_reg_write),
        .mem_wb_value (mem_wb_value_r),
        // ���
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
    // ע�⣺ex_stageģ���ڲ�ͬʱ�����Ĵ���д�ء��ô���źţ�������ֱ��ͨ��id_ex�źŴ���
    // Ϊ�򻯣�ex_stage����Ŀ����źſ��Բ��ظ�������ֱ��ʹ��id_ex�׶εĶ�Ӧ�źŽ���EX/MEM�Ĵ���
    
    // EX -> EX/MEM ��ˮ�Ĵ���
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
            ex_mem_store_val  <= ex_forwarded_rs2;  // Storeָ����Ҫд���ڴ�����ݣ��Ѿ���ǰ��������
            ex_mem_rd         <= id_ex_rd;
            ex_mem_reg_write  <= id_ex_reg_write;
            ex_mem_mem_read   <= id_ex_mem_read;
            ex_mem_mem_write  <= id_ex_mem_write;
            ex_mem_mem_op     <= id_ex_mem_op;
        end
    end
    
    // �ô�׶�ģ�飺�������ݴ洢���ͼ���/�洢�����ݴ���
    mem_stage mem_stage_u (
        .alu_result   (ex_mem_alu_result),
        .store_val    (ex_mem_store_val),
        .mem_read     (ex_mem_mem_read),
        .mem_write    (ex_mem_mem_write),
        .mem_op       (ex_mem_mem_op),
        .clk          (clk_cpu),
        .reset        (reset),
        .data_out     (mem_data_out)
        // ���ݴ洢�����ã���ʵ��Ϊ�ڲ�reg��Ҳ���滻ΪBlock RAM��
        //.dmem         ()  // �˴�����ģ���ڲ�����dmem����
    );
    
    // ��ALU������ڴ��ȡ����ѡ��Ϊд��ֵ
    assign mem_wb_value = ex_mem_mem_read ? mem_data_out : ex_mem_alu_result;
    
    // MEM -> MEM/WB ��ˮ�Ĵ���
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
    
    // д�ؽ׶Σ��Ĵ�����д�루ͬ��д��ʱ�������ؽ��У�
    // ����id_stageģ���еļĴ�����ʵ���У�ͨ����WB�׶�д��
    // ��cpu_top����ͨ����id_stage����ʵ��д��
    
    // ��д���źŴ��ݸ��Ĵ�����ģ�� (id_stage�����Ĵ�����)
    assign id_stage_u.wb_rd_idx   = mem_wb_rd;
    assign id_stage_u.wb_data_in  = mem_wb_value_r;
    assign id_stage_u.wb_reg_write= mem_wb_reg_write;
    
    // -------------------------------
    // ���������LED������ܵ�����ʾ
    // -------------------------------
    // LED��ʾʾ����ֱ����ʾx3�Ĵ�����16λ
    assign led_o = id_stage_u.regs[3][15:0];

    // ���ԼĴ�������ѭ��
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

    // PC�ͼĴ���ֵ����׼��
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

    // �����ɨ������
    seven_seg_driver disp_u(
        .clk   (clk),
        .reset (reset),
        .d0(dig0),.d1(dig1),.d2(dig2),.d3(dig3),
        .d4(dig4),.d5(dig5),.d6(dig6),.d7(dig7),
        .seg(disp_seg_o),
        .an (disp_an_o)
    );
endmodule