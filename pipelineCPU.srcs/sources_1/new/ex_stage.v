`timescale 1ns / 1ps
module ex_stage(
    input  wire [31:0] pc,         // ����ID/EX�ĵ�ǰָ��PC
    input  wire [31:0] rs1_val,    // �Ĵ���rs1ֵ��ID/EX��
    input  wire [31:0] rs2_val,    // �Ĵ���rs2ֵ��ID/EX��
    input  wire [31:0] imm,        // ��������ID/EX��
    input  wire [4:0]  rs1_idx,    // Դ�Ĵ���1�ţ�����ǰ���жϣ�
    input  wire [4:0]  rs2_idx,    // Դ�Ĵ���2��
    input  wire [4:0]  rd_idx,     // Ŀ�ļĴ����ţ�ID/EX��
    input  wire        reg_write_in, // ��ָ���Ƿ�д�ؼĴ���
    input  wire        mem_read_in,  // �Ƿ�Ϊloadָ��
    input  wire        mem_write_in, // �Ƿ�Ϊstoreָ��
    input  wire [2:0]  mem_op_in,    // �ڴ�������ͣ�funct3��
    input  wire        branch_in,    // �Ƿ�Ϊ������ָ֧��
    input  wire [1:0]   jump_in,      // �Ƿ�Ϊ��תָ�� (JAL/JALR)
    input  wire [3:0]  alu_op,       // ALU�����루�ɿ��Ƶ�Ԫ������
    input  wire        alu_src1_pc,  // ALU������1��Դ��1=PC, 0=�Ĵ���rs1
    input  wire        alu_src2_imm, // ALU������2��Դ��1=������,0=�Ĵ���rs2
    // ǰ��������룺����EX/MEM��MEM/WB�׶ν�����ڿ��ܵ�ǰ��
    input  wire [4:0]  ex_mem_rd,
    input  wire        ex_mem_reg_write,
    input  wire [31:0] ex_mem_alu_result,
    input  wire [4:0]  mem_wb_rd,
    input  wire        mem_wb_reg_write,
    input  wire [31:0] mem_wb_value,
    // ���
    output reg  [31:0] alu_result,    // ALU�����������EX/MEM��
    output reg  [31:0] forwarded_rs2, // ת�����rs2ֵ����storeʹ�õ�ֵ��
    output reg         branch_taken,  // ��֧�Ƿ��������ת�źţ�
    output reg  [31:0] branch_target, // ��תĿ���ַ
    output wire [4:0]  rd_idx_out,    // ����Ŀ�ļĴ�����
    output wire        reg_write_out, // �����Ƿ�д���ź�
    output wire        mem_read_out,  // ���ݷô������
    output wire        mem_write_out, // ���ݷô�д����
    output wire [2:0]  mem_op_out     // �����ڴ��������
);
    // ��ID�׶εĿ����ź�ֱ�Ӵ���������󲿷ֿ����ź������޸�ֱ�ӽ�����һ��ˮ�Σ�
    assign rd_idx_out    = rd_idx;
    assign reg_write_out = reg_write_in;
    assign mem_read_out  = mem_read_in;
    assign mem_write_out = mem_write_in;
    assign mem_op_out    = mem_op_in;
    
    // ǰ�ݣ�ѡ��EX�׶β�����������ֵ
    // Ĭ��ʹ������ID/EX�Ĵ�����ֵ�����к������������滻
    reg [31:0] op_a, op_b;
    reg [31:0] rs2_val_forwarded; // ����store��rs2����ֵ
    always @(*) begin
        // ǰ�ݸ�������A (rs1)
        if (ex_mem_reg_write && ex_mem_rd != 5'd0 && ex_mem_rd == rs1_idx) begin
            op_a = ex_mem_alu_result;  // ��������ָ��EX/MEM�׶εĽ��
        end else if (mem_wb_reg_write && mem_wb_rd != 5'd0 && mem_wb_rd == rs1_idx) begin
            op_a = mem_wb_value;       // ��������ǰָ��MEM/WB�׶εĽ��
        end else begin
            op_a = rs1_val;            // ��ǰ�ݣ�ֱ���üĴ���ֵ
        end
        
        // ǰ�ݸ�������B (rs2) - ע��: ALU�ĵڶ�������������imm����һ����rs2
        if (ex_mem_reg_write && ex_mem_rd != 5'd0 && ex_mem_rd == rs2_idx) begin
            rs2_val_forwarded = ex_mem_alu_result;
        end else if (mem_wb_reg_write && mem_wb_rd != 5'd0 && mem_wb_rd == rs2_idx) begin
            rs2_val_forwarded = mem_wb_value;
        end else begin
            rs2_val_forwarded = rs2_val;
        end
        
        // ���ݿ��ƣ�����ALU�������룺
        // ������1:
        if (alu_src1_pc) 
            op_a = pc;       // ʹ�õ�ǰPC��ΪA������
        // ����op_a������ȷ��rs1ֵ(��ǰ��)
        // ������2:
        if (alu_src2_imm) 
            op_b = imm;      // ʹ����������ΪB������
        else 
            op_b = rs2_val_forwarded;  // ʹ�üĴ���ֵ��ΪB������
    end
    
    // ALU����
    reg zero_flag;
    always @(*) begin
        case (alu_op)
            4'b0000: alu_result = op_a + op_b;            // ADD
            4'b0001: alu_result = op_a - op_b;            // SUB
            4'b0010: alu_result = op_a << (op_b[4:0]);    // SLL (�߼�����)
            4'b0011: alu_result = ($signed(op_a) < $signed(op_b)) ? 32'd1 : 32'd0;  // SLT (�з��űȽ�)
            4'b0100: alu_result = (op_a < op_b) ? 32'd1 : 32'd0;  // SLTU (�޷��űȽ�)
            4'b0101: alu_result = op_a ^ op_b;            // XOR
            4'b0110: alu_result = op_a >> (op_b[4:0]);    // SRL (�߼�����)
            4'b0111: alu_result = ($signed(op_a)) >>> (op_b[4:0]); // SRA (��������)
            4'b1000: alu_result = op_a | op_b;            // OR
            4'b1001: alu_result = op_a & op_b;            // AND
            4'b1010: alu_result = op_b;                   // PASSB (ֱ������ڶ�������������LUI)
            default: alu_result = op_a + op_b;
        endcase
        // �������־���ڷ�֧�ж�
        
        zero_flag = (alu_result == 32'b0);
    //end
    
    
    
    // ��֧����ת�ж�
    //always @(*) begin
        branch_taken = 1'b0;
        branch_target = 32'b0;
        if (branch_in) begin
            // ������֧������funct3�����ж�����
            case (mem_op_in[2:0])  // reuse mem_op_in�洢��ԭinstr��funct3λ���ɱ�ʾ��֧����
                3'b000: branch_taken = (op_a == op_b);   // BEQ
                3'b001: branch_taken = (op_a != op_b);   // BNE
                3'b100: branch_taken = ($signed(op_a) < $signed(op_b));  // BLT
                3'b101: branch_taken = ($signed(op_a) >= $signed(op_b)); // BGE
                3'b110: branch_taken = (op_a < op_b);    // BLTU
                3'b111: branch_taken = (op_a >= op_b);   // BGEU (�޷���>=)
                default: branch_taken = 1'b0;
            endcase
            if (branch_taken) begin
                // Ŀ���ַ = ��ǰPC + ������ƫ��
                branch_target = pc + imm;
            end
        end
        if (jump_in[1]||jump_in[0]) begin
            // JAL or JALR: ��������ת
            branch_taken = 1'b1;
            if (jump_in[1]) begin
                // JAL: Ŀ�� = pc + imm (imm��ID�׶��Ѽ���Ϊ21λƫ��)
                branch_target = pc + imm;
            end else begin
                // JALR: opcode 1100111
                // Ŀ�� = (rs1 + imm) & ~1
                branch_target = (op_a + imm) & 32'hFFFFFFFE;
            end
            // JAL/JALR��Ҫ�����ص�ַPC+4д��rd:
            // ����ֱ�ӽ� alu_result ��Ϊ pc+4���Ա�д�ؽ׶�д��
            alu_result = pc + 32'd4;
        end
    end
    // forwarded_rs2 ���ڷô�׶�Store����������
    always @(*) begin
        forwarded_rs2 = rs2_val_forwarded;
    end
endmodule