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

    // ʱ�ӷ�Ƶ/ѡ��: ʹ��sw_i[15]����ʱ���ٶ� (��ѡʵ��)
    // ����Ҫ�����ڴ˲����Ƶ������clk_div��Ϊ��ģ��ʱ��
    wire clk_cpu = clk; // �򻯣�ֱ��ʹ�ð���ʱ�ӣ��޷�Ƶ
    // TODO: ��ʵ��ʱ�ӷ�Ƶ�߼�������������ﵽһ��ֵ��תclk_div���Ӷ��õ�����ʱ�����ڹ۲�
    
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
            if (!hazard_stall) begin
                // ����ͣʱ����PC�����ȷ�֧/��תĿ�꣬����PC+4
                pc <= ex_branch_taken ? ex_branch_target : pc + 32'd4;
            end else begin
                pc <= pc; // ��ͣʱ����PC����
            end
        end
    end
    
    assign if_pc_curr = pc;  // ��ǰPCֵ��������ڵ�����ʾ��
    
    // IF�׶�ȡָ������߼���ָ��洢����ȡ��ǰPC��ַ��ָ��
    assign if_instr = inst_mem[pc[31:2]]; 
    // ����ָ��洢������Ѱַ���������pc���ֶ��룬ʹ��pc[31:2]������
    // Vivado�ۺ�ʱ�Ὣ���ƶ�Ϊ��RAM ROM��
    
    // IF -> IF/ID ��ˮ�Ĵ�������
    always @(posedge clk_cpu or posedge reset) begin
        if (reset) begin
            if_id_instr <= 32'b0;
            if_id_pc    <= 32'b0;
        end else begin
            if (!hazard_stall) begin
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
        .hazard_stall(hazard_stall)
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
            if (hazard_flush) begin
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
            end else if (!hazard_stall) begin
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
        end else begin
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
        end else begin
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
    // ���׵��ԣ��� LED ��ʾĳ���Ĵ�����״̬λ (����ʾ���� LED ��ʾ x3 �Ĵ�����16λ)
    assign led_o = id_stage_u.regs[3][15:0]; 
    // �������ü��� id_stage �ڲ��ļĴ������� regs Ϊ public�ɼ�����Ҫ��id_stageģ���н�regs����Ϊ public for simulation����
    // ���������ֱ�ӷ����ڲ��Ĵ���������id_stageģ������һ������˿ڽ�ĳһ�Ĵ���ֵ������
    // ����򵥴�������Vivado֧�����ֲ������(ʵ����Ҫ����).
    
    // �������ʾ����ʾ��ǰPC�ĵ�4λ16����ֵ�ڵ�0λ�������
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
    assign disp_an_o  = 8'b1111_1110; // ���������Ҳ�ĵ�0λ����ܣ��ٶ��͵�ƽ��Ч��
endmodule
