module mem_stage(
    input  wire [31:0] alu_result, // ����EX�׶εļ����ַ
    input  wire [31:0] store_val,  // Storeָ��Ҫд�������
    input  wire        mem_read,   // �ô��ȡʹ��
    input  wire        mem_write,  // �ô�д��ʹ��
    input  wire [2:0]  mem_op,     // �洢��������(funct3)�������ֽ�/����/�ּ�������չ
    input  wire        clk,
    input  wire        reset,
    output reg  [31:0] data_out    // ���������ݣ���չΪ32λ��
);
    // �����ݴ洢������С�趨Ϊ256�ֽ� (64��)
    localparam DMEM_BYTES = 256;
    reg [7:0] dmem [0:DMEM_BYTES-1];
    
    // ��ʼ�����ݴ洢������ѡ�������� $readmemh �����ʼ����
    initial begin
        $readmemh("datamem_init.hex", dmem);
    end
    
    wire [7:0] addr = alu_result[7:0];
    // ��ַ��2λ
    wire [1:0] addr_offset = addr[1:0]; 
    // �����ֶ����ַ�Ļ�ַ�������ֽ�����������
    wire [31:0] base_index = addr & 32'hFFFFFFFC;
    integer offset;
    reg [7:0] byte_val;reg [15:0] half_val;reg [31:0] word_val;//reg [7:0] byte_val;
    always @(*) begin
        data_out = 32'b0;
        if (mem_read) begin
            case (mem_op)
                3'b000: begin // LB: ��1�ֽڣ��з�����չ
                    offset = addr_offset;
                    byte_val = dmem[addr];
                    data_out = {{24{byte_val[7]}}, byte_val};
                end
                3'b001: begin // LH: ��2�ֽڣ��з�����չ
                    // ��������ֽڣ�С����
                    half_val = { dmem[addr + 1], dmem[addr] };
                    data_out = {{16{half_val[15]}}, half_val};
                end
                3'b010: begin // LW: ��4�ֽ�
                    word_val = { dmem[addr+3], dmem[addr+2], dmem[addr+1], dmem[addr] };
                    data_out = word_val;
                end
                3'b100: begin // LBU: ��1�ֽڣ�����չ
                    byte_val = dmem[addr];
                    data_out = {24'b0, byte_val};
                end
                3'b101: begin // LHU: ��2�ֽڣ�����չ
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
    
    // д�洢����ͬ��д��ʱ��������ִ��
    always @(posedge clk) begin
        if (mem_write) begin
            case (mem_op)
                3'b000: begin // SB: д1�ֽ�
                    dmem[addr] <= store_val[7:0];
                end
                3'b001: begin // SH: д2�ֽ�
                    dmem[addr]   <= store_val[7:0];
                    dmem[addr+1] <= store_val[15:8];
                end
                3'b010: begin // SW: д4�ֽ�
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
