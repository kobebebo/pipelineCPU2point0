`timescale 1ns / 1ps
module hazard_unit(
    input wire        id_ex_mem_read,  // ID/EX�׶�ָ���Ƿ�ΪMemRead (Load)
    input wire [4:0]  id_ex_rd,        // ID/EX�׶�ָ���Ŀ��Ĵ�����
    input wire [4:0]  if_id_rs1,       // IF/ID�׶�ָ���Դ�Ĵ���1��
    input wire [4:0]  if_id_rs2,       // IF/ID�׶�ָ���Դ�Ĵ���2��
    output reg        hazard_stall,    // �������ˮ����ͣ�ź�
    output reg        hazard_flush     // ��������������źţ����ID/EX��
);
    always @(*) begin
        // ��ʼĬ����ð��
        hazard_stall = 1'b0;
        hazard_flush = 1'b0;
        // ���load-useð������
        if (id_ex_mem_read && 
           ((id_ex_rd != 5'd0) &&  // ǰ��ָ��д�ؼĴ�����Ӧ��x0:contentReference[oaicite:44]{index=44}
            ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2)))) begin
            hazard_stall = 1'b1;
            hazard_flush = 1'b1;
        end
    end
endmodule