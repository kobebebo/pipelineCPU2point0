`timescale 1ns / 1ps
module hazard_unit(
    input wire        id_ex_mem_read,  // ID/EX阶段指令是否为MemRead (Load)
    input wire [4:0]  id_ex_rd,        // ID/EX阶段指令的目标寄存器号
    input wire [4:0]  if_id_rs1,       // IF/ID阶段指令的源寄存器1号
    input wire [4:0]  if_id_rs2,       // IF/ID阶段指令的源寄存器2号
    output reg        hazard_stall,    // 输出：流水线暂停信号
    output reg        hazard_flush     // 输出：插入气泡信号（清除ID/EX）
);
    always @(*) begin
        // 初始默认无冒险
        hazard_stall = 1'b0;
        hazard_flush = 1'b0;
        // 检查load-use冒险条件
        if (id_ex_mem_read && 
           ((id_ex_rd != 5'd0) &&  // 前条指令写回寄存器不应是x0
            ((id_ex_rd == if_id_rs1) || (id_ex_rd == if_id_rs2)))) begin
            hazard_stall = 1'b1;
            hazard_flush = 1'b1;
        end
    end
endmodule