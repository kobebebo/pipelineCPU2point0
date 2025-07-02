module id_stage(
    input  wire [31:0] instr,        // ����IF�׶ε�ָ�����
    input  wire [31:0] pc,           // ����IF�׶εĵ�ǰָ���ַ��PC��
    output wire [4:0]  rs1, rs2, rd, // Դ�Ĵ������(rs1, rs2)��Ŀ��Ĵ������(rd)
    output reg  [31:0] imm,          // ��������չֵ
    output reg         reg_write,    // �Ƿ�д�ؼĴ���
    output reg         mem_read,     // �Ƿ�ô��ȡ
    output reg         mem_write,    // �Ƿ�ô�д��
    output reg  [2:0]  mem_op,       // �洢���������ͣ���������LB/LH/LW�ȣ�funct3ֱ����Ϊ��ʶ��
    output reg         branch,       // �Ƿ�Ϊ��ָ֧�������ת��
    output reg  [1:0]  jump,         // �Ƿ�Ϊ��������תָ�JAL/JALR��[1]=jal,[0]=jalr
    output reg  [3:0]  alu_op,       // ALU ����ѡ���루��ָ���ȷ�����Զ������4λ��
    output reg         alu_src1_pc,  // ALU������1��Դ��1=ʹ��PCֵ, 0=ʹ�üĴ���rs1ֵ
    output reg         alu_src2_imm, // ALU������2��Դ��1=ʹ��������, 0=ʹ�üĴ���rs2ֵ
    output wire [31:0] rs1_val, rs2_val, // �ӼĴ����Ѷ�ȡ��Դ������ֵ
    input  wire [4:0]  wb_rd_idx,    // д�ؽ׶�Ҫд��ļĴ�����
    input  wire [31:0] wb_data_in,   // д�ص�����
    input  wire        wb_reg_write, // д��ʹ���ź�
    input  wire        clk,
    input  wire        reset,
    input  wire        hazard_stall  // ð����ͣ�źţ��ߵ�ƽ��ʾ������ID�׶�ָ��ͣ�٣����������
);
    // ��ָ���ֶβ��
    wire [6:0] opcode = instr[6:0];
    wire [4:0] rs1_idx = instr[19:15];
    wire [4:0] rs2_idx = instr[24:20];
    wire [4:0] rd_idx  = instr[11:7];
    wire [2:0] funct3  = instr[14:12];
    wire [6:0] funct7  = instr[31:25];
    
    assign rs1 = rs1_idx;
    assign rs2 = rs2_idx;
    assign rd  = rd_idx;
    
    // �Ĵ����Ѷ��� (32 x 32λ �Ĵ���)
    reg [31:0] regs [0:31];
    integer i;
    // �첽����ͬ��дʵ�֣�
    assign rs1_val = regs[rs1_idx];
    assign rs2_val = regs[rs2_idx];
    
    // ��ʼ���Ĵ�������λʱ��
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i=0; i<32; i=i+1) begin
                regs[i] <= 32'b0;
            end
            regs[2] <= 32'd128;  // ��ʼ��x2 (sp) = 128:contentReference[oaicite:26]{index=26}
        end else begin
            // д�ؽ׶� - ��ʱ��������д�Ĵ���
            if (wb_reg_write && wb_rd_idx != 5'd0) begin
                regs[wb_rd_idx] <= wb_data_in;
            end
        end
    end
    
    // �����ź�Ĭ��ֵ��ÿ����������߼����㣩
    always @(*) begin
        // Ĭ��ֵ��������ź�Ĭ��Ϊ0����ƥ��ָ��ʱ����Ϊ1��
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
        
        // ��ȡָ�ͬ���͵�������λ�β�������չ/����
        case (opcode)
            7'b0110011: begin
                // R-type: û��������
                imm = 32'b0;
            end
            7'b0010011, // I-type (����/�߼������� �� JALR, LB/LH/LW��Load��)
            7'b0000011, // I-type (Load��)
            7'b1100111: begin // JALR
                // I�ͣ�imm[11:0] = instr[31:20]��������չ��32λ
                imm = {{20{instr[31]}}, instr[31:20]};
            end
            7'b0100011: begin
                // S-type (Store): imm[11:0] = instr[31:25] concat instr[11:7]
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};
            end
            7'b1100011: begin
                // B-type (Branch): imm[12|10:5|4:1|11] = instr[31|30:25|11:8|7] (12λ������,���λ0)
                imm = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
            end
            7'b0110111, // U-type (LUI)
            7'b0010111: begin // U-type (AUIPC)
                // U�ͣ�imm[31:12] = instr[31:12], ��12λ��0
                imm = {instr[31:12], 12'b0};
            end
            7'b1101111: begin
                // J-type (JAL): imm[20|10:1|11|19:12] = instr[31|30:21|20|19:12] (21λ������,���λ0)
                imm = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};
            end
            default: begin
                imm = 32'b0;
            end
        endcase
        
        // ����ָ�������͹��������ɿ����ź�
        case (opcode)
            7'b0110011: begin  // R-type����ָ��
                reg_write = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b0;
                // ���� funct3/funct7 ���� alu_op
                case (funct3)
                    3'b000: alu_op = (funct7[5] ? 4'b0001  // SUB
                                             : 4'b0000); // ADD
                    3'b001: alu_op = 4'b0010; // SLL (�߼�����)
                    3'b010: alu_op = 4'b0011; // SLT (�з��űȽ�)
                    3'b011: alu_op = 4'b0100; // SLTU (�޷��űȽ�)
                    3'b100: alu_op = 4'b0101; // XOR
                    3'b101: alu_op = (funct7[5] ? 4'b0111  // SRA
                                             : 4'b0110); // SRL
                    3'b110: alu_op = 4'b1000; // OR
                    3'b111: alu_op = 4'b1001; // AND
                    default: alu_op = 4'b0000;
                endcase
            end
            7'b0010011: begin  // I-type ����/�߼�������ָ��
                reg_write = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1;  // �ڶ�����������������
                case (funct3)
                    3'b000: alu_op = 4'b0000; // ADDI -> ADD
                    3'b010: alu_op = 4'b0011; // SLTI -> SLT
                    3'b011: alu_op = 4'b0100; // SLTIU -> SLTU
                    3'b100: alu_op = 4'b0101; // XORI -> XOR
                    3'b110: alu_op = 4'b1000; // ORI -> OR
                    3'b111: alu_op = 4'b1001; // ANDI -> AND
                    3'b001: begin 
                        // SLLI: imm[5:0]�ĸ�λӦΪ0��ȷ���߼���λ��
                        alu_op = 4'b0010; // SLL
                    end
                    3'b101: begin 
                        // SRLI/SRAI: ������������λ����
                        alu_op = (instr[31:25] == 7'b0100000) ? 4'b0111 // SRAI
                                                              : 4'b0110; // SRLI
                    end
                    default: alu_op = 4'b0000;
                endcase
            end
            7'b0000011: begin  // I-type Loadָ�� (LB/LH/LW/LBU/LHU)
                reg_write = 1'b1;
                mem_read  = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1; // ��ַ = rs1 + imm
                alu_op    = 4'b0000; // ��ַ����ʹ��ADD
                mem_op    = funct3;  // ����load���ͣ���������������չ
            end
            7'b0100011: begin  // S-type Storeָ�� (SB/SH/SW)
                mem_write = 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1; // ��ַ = rs1 + imm
                alu_op    = 4'b0000; // ��ַ����ʹ��ADD
                mem_op    = funct3;  // ����store���ͣ���������д�洢���ֽ���
            end
            7'b1100011: begin  // B-type ��ָ֧�� (BEQ/BNE/BLT/BGE/BLTU/BGEU)
                branch   = 1'b1;
                // ��֧��д�Ĵ��������ô棬����Ҫ�����֧����
                reg_write = 1'b0;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b0;
                alu_op    = 4'b0000; // ALUĬ��ADD��SUB���ɣ����ﲻʵ����ALU���
                // ��֧�����жϽ���EX�׶θ���rs1_val/rs2_val��funct3����
            end
            7'b1101111: begin  // J-type ��������ת (JAL)
                jump[1]     = 1'b1;
                reg_write= 1'b1;  // JAL��Ҫд�����ӵ�ַ��rd
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b0;
                alu_op   = 4'b0000;
                // JAL: rd = PC+4 (���ӵ�ַ)��PC����ת�� PC + imm (Ŀ���ַ)
                // ���ӵ�ַ�ļ������EX�׶����
            end
            7'b1100111: begin  // I-type �Ĵ�����ת (JALR)
                jump[0]     = 1'b1;
                reg_write= 1'b1;  // JALRд�����ӵ�ַ
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1; // ��������ALU����Ŀ���ַ = rs1 + imm
                alu_op   = 4'b0000;
                // JALR: rd = PC+4�� ��PC = (rs1 + imm) & ~1
                // PC+4 �� ��PC ������EX�׶δ���
            end
            7'b0110111: begin  // U-type LUI
                reg_write= 1'b1;
                alu_src1_pc = 1'b0;
                alu_src2_imm= 1'b1;
                // LUI: ��imm����20λ�����ص�rd��������ALU����: 0 + imm
                alu_op   = 4'b1010; // �Զ��� PASSB �����룬��ALU����ڶ���������imm��
            end
            7'b0010111: begin  // U-type AUIPC
                reg_write= 1'b1;
                alu_src1_pc = 1'b1;  // ��һ����������PC
                alu_src2_imm= 1'b1;  // �ڶ���������imm
                alu_op   = 4'b0000;  // ALU���ӷ����� PC + imm
            end
            default: begin
                // δ����ָ�����ϵͳָ��ECALL/EBREAK�ȣ�����ִ�в���
                reg_write= 1'b0;
                mem_read = 1'b0;
                mem_write= 1'b0;
                branch   = 1'b0;
                jump     = 2'b00;
                alu_op   = 4'b0000;
            end
        endcase
        
        // ����Ҫ�������ڴ����hazard_stall�Կ����źŵ�Ӱ�죨����������ͣ�򱣳��������źţ���
        // Ȼ���ڱ�����У�hazard_stall��Ҫͨ��������ˮ�Ĵ���ʵ�֣������ڴ˶��⴦������߼���
    end

endmodule
