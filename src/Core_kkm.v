module Core_kkm(output wire [63:0] o_pm_addr,
                output wire o_pm_cs,
                input wire [31:0] i_pm_data,
                output wire [63:0] o_dm_addr,
                output wire o_dm_cs,
                output wire o_dm_rw,
                output wire [63:0] o_dm_data,
                input wire [63:0] i_dm_data,
                input wire i_clk,
                input wire i_rst);
    
    wire i_read0, i_read1, i_write;
    wire [4:0] r_addr0, r_addr1, w_addr;
    wire [63:0] data1, data2;
    wire [63:0] w_data;
    
    assign i_read0 = 1'b1;
    assign i_read1 = 1'b1;
    
    RegFile_32x64bit U0(
    .i_read0(i_read0),
    .i_read_addr0(r_addr0),
    .o_read_data0(data1),
    .i_read1(i_read1),
    .i_read_addr1(r_addr1),
    .o_read_data1(data2),
    .i_write(i_write),
    .i_write_addr(w_addr),
    .i_write_data(w_data),
    .i_clk(i_clk),
    .i_rst(i_rst)
    );
    
    reg [63:0] ProgramCounter;
    
    reg [63:0] PC_IFIDp;
    reg [31:0] inst_IFIDp;
    reg [63:0] PC_IDEXp;
    reg [31:0] inst_IDEXp;
    wire [63:0] data1_IDEXp, data2_IDEXp;
    reg [4:0] registerRS1_IDEXp, registerRS2_IDEXp, registerRD_IDEXp;
    reg signed [63:0] imm_IDEXp;
    reg [1:0] ALUOp_IDEXp;
    reg ALUSrc_IDEXp, Branch_IDEXp, MemRead_IDEXp, MemWrite_IDEXp, RegWrite_IDEXp, MemtoReg_IDEXp;
    reg [4:0] inst_EXMEMp;
    reg Branch_EXMEMp, MemRead_EXMEMp, MemWrite_EXMEMp, RegWrite_EXMEMp, MemtoReg_EXMEMp;
    reg [63:0] ALUresult_EXMEMp, data1_ALU, data2_ALU, data2_EXMEMp;
    reg signed [63:0] beqimm_IDEXp, PCforbeq_IDEXp;
    reg [4:0] registerRD_EXMEMp;
    reg [3:0] operation;
    reg RegWrite_MEMWBp, MemtoReg_MEMWBp;
    reg [63:0] ALUresult_MEMWBp;
    
    wire stall;
    wire [1:0] ForwardA, ForwardB, ForwardC, ForwardD;
    wire [63:0] data_MEMWBp;
    reg [4:0] registerRD_MEMWBp;
    reg [63:0] data1_comp, data2_comp;
    
    wire zero;
    assign zero = (data2_comp == data1_comp) ? 1'b1: 1'b0;
    
    assign data1_IDEXp = data1;
    assign data2_IDEXp = data2;
    assign data_MEMWBp = i_dm_data;
    assign o_pm_cs     = 1'b1;
    assign o_pm_addr   = ProgramCounter;
    assign o_dm_cs     = (MemRead_EXMEMp | MemWrite_EXMEMp) ? 1'b1: 1'b0;
    assign o_dm_rw     = (MemRead_EXMEMp) ? 1'b0: 1'b1;
    assign o_dm_data   = data2_EXMEMp;
    assign o_dm_addr   = (ALUresult_EXMEMp << 3);

    assign stall       = (
                            ( 
                                MemRead_IDEXp & 
                                (registerRD_IDEXp != 5'd0) & 
                                (
                                    (registerRD_IDEXp == inst_IFIDp[19:15]) | (registerRD_IDEXp == inst_IFIDp[24:20])
                                )
                            ) | (
                                    inst_IFIDp[6] & 
                                    (
                                        (registerRD_IDEXp == inst_IFIDp[19:15]) | (registerRD_IDEXp == inst_IFIDp[24:20])
                                    )
                                )
                        ) ? 1'b1: 1'b0;
    
    assign ForwardA = (
                            RegWrite_EXMEMp & 
                            (registerRD_EXMEMp != 5'd0) & 
                            (registerRD_EXMEMp == registerRS1_IDEXp) 
                        ) ? 2'b10:
                        (
                            RegWrite_MEMWBp & 
                            (registerRD_MEMWBp != 5'd0) & 
                            (registerRD_MEMWBp == registerRS1_IDEXp) &
                            ~ (
                                RegWrite_EXMEMp & (registerRD_EXMEMp != 5'd0) & (registerRD_EXMEMp == registerRS1_IDEXp)
                            )
                        ) ? 2'b01: 2'b00;
    
    assign ForwardB = (
                            RegWrite_EXMEMp & 
                            (registerRD_EXMEMp != 5'd0) &
                            (registerRD_EXMEMp == registerRS2_IDEXp)
                        ) ? 2'b10:
                        (
                            RegWrite_MEMWBp &
                            (registerRD_MEMWBp != 5'd0) &
                            (registerRD_MEMWBp == registerRS2_IDEXp) &
                            ~ (
                                RegWrite_EXMEMp & (registerRD_EXMEMp != 5'd0) & (registerRD_EXMEMp == registerRS2_IDEXp)
                            )
                        ) ? 2'b01: 2'b00;
    
    assign ForwardC = (registerRD_EXMEMp == inst_IFIDp[19:15]) ? 2'b10:
                    ((registerRD_MEMWBp == inst_IFIDp[19:15]) & (registerRD_EXMEMp != inst_IFIDp[19:15])) ? 2'b01: 2'b00;
    
    assign ForwardD = (registerRD_EXMEMp == inst_IFIDp[24:20]) ? 2'b10:
                    ((registerRD_MEMWBp == inst_IFIDp[24:20]) & (registerRD_EXMEMp != inst_IFIDp[24:20])) ? 2'b01: 2'b00;
    
    initial
    begin
        ProgramCounter <= 64'd0;
    end
    
    // IF stage
    always@(posedge i_clk)
    begin
        if (stall)
        begin
            ProgramCounter <= ProgramCounter;
            PC_IFIDp       <= PC_IFIDp;
            inst_IFIDp     <= inst_IFIDp;
        end
        else
        begin
            PC_IFIDp <= ProgramCounter;
            if (Branch_IDEXp & zero)
            begin
                ProgramCounter <= PCforbeq_IDEXp;
                inst_IFIDp     <= i_pm_data;
            end
            else
            begin
                ProgramCounter <= ProgramCounter+4;
                inst_IFIDp     <= i_pm_data;
            end
        end
    end
    
    // ID stage
    always@(posedge i_clk)
    begin
        if (stall)
        begin
            ALUOp_IDEXp  <= 2'b00;
            ALUSrc_IDEXp <= 1'b0;
            Branch_IDEXp = 1'b0;
            MemRead_IDEXp  <= 1'b0;
            MemWrite_IDEXp <= 1'b0;
            RegWrite_IDEXp <= 1'b0;
            MemtoReg_IDEXp <= 1'b0;
        end
        else
        begin
            if (inst_IFIDp[6])       // SB
            begin
                ALUOp_IDEXp  <= 2'b01;
                ALUSrc_IDEXp <= 1'b0;
                Branch_IDEXp = 1'b1;
                MemRead_IDEXp  <= 1'b0;
                MemWrite_IDEXp <= 1'b0;
                RegWrite_IDEXp <= 1'b0;
                MemtoReg_IDEXp <= 1'b0;
            end
            else if (inst_IFIDp[5] & inst_IFIDp[4])    // R
            begin
                ALUOp_IDEXp  <= 2'b10;
                ALUSrc_IDEXp <= 1'b0;
                Branch_IDEXp = 1'b0;
                MemRead_IDEXp  <= 1'b0;
                MemWrite_IDEXp <= 1'b0;
                RegWrite_IDEXp <= 1'b1;
                MemtoReg_IDEXp <= 1'b0;
            end
            else if (~inst_IFIDp[5] & ~inst_IFIDp[4])  // ld
            begin
                ALUOp_IDEXp  <= 2'b00;
                ALUSrc_IDEXp <= 1'b1;
                Branch_IDEXp = 1'b0;
                MemRead_IDEXp  <= 1'b1;
                MemWrite_IDEXp <= 1'b0;
                RegWrite_IDEXp <= 1'b1;
                MemtoReg_IDEXp <= 1'b1;
            end
            else if (inst_IFIDp[4])  // addi
            begin
                ALUOp_IDEXp  <= 2'b00;
                ALUSrc_IDEXp <= 1'b1;
                Branch_IDEXp = 1'b0;
                MemRead_IDEXp  <= 1'b0;
                MemWrite_IDEXp <= 1'b0;
                RegWrite_IDEXp <= 1'b1;
                MemtoReg_IDEXp <= 1'b0;
            end
            else    // S
            begin
                ALUOp_IDEXp  <= 2'b00;
                ALUSrc_IDEXp <= 1'b1;
                Branch_IDEXp = 1'b0;
                MemRead_IDEXp  <= 1'b0;
                MemWrite_IDEXp <= 1'b1;
                RegWrite_IDEXp <= 1'b0;
                MemtoReg_IDEXp <= 1'b0;
            end
        end
        
        if (inst_IFIDp[6])  // SB
            imm_IDEXp <= {{43{inst_IFIDp[31]}}, inst_IFIDp[7], inst_IFIDp[30:25], inst_IFIDp[11:8]};
        else if (inst_IFIDp[5] & ~inst_IFIDp[4])  // S
            imm_IDEXp <= {{43{inst_IFIDp[31]}}, inst_IFIDp[30:25], inst_IFIDp[11:7]};
        else    // I or R
            imm_IDEXp <= {{43{inst_IFIDp[31]}}, inst_IFIDp[30:20]};
        
        if (ForwardC == 2'b00)
            data1_comp = data1;
        else if (ForwardC == 2'b10)
            data1_comp = ALUresult_EXMEMp;
        else
            data1_comp = w_data;
            if (ForwardD == 2'b00)
                data2_comp = data2;
            else if (ForwardD == 2'b10)
                data2_comp = ALUresult_EXMEMp;
            else
                data2_comp = w_data;
        
        inst_IDEXp        <= inst_IFIDp;
        registerRS1_IDEXp <= inst_IFIDp[19:15];
        registerRS2_IDEXp <= inst_IFIDp[24:20];
        PC_IDEXp          <= PC_IFIDp;
        registerRD_IDEXp  <= inst_IFIDp[11:7];
        beqimm_IDEXp = (imm_IDEXp << 1);
        PCforbeq_IDEXp <= PC_IFIDp + beqimm_IDEXp;
    end
    
    assign r_addr0 = inst_IFIDp[19:15];
    assign r_addr1 = inst_IFIDp[24:20];
    
    // EX stage
    always@(posedge i_clk)
    begin
        if (~ALUOp_IDEXp[1] & ~ALUOp_IDEXp[0])     // I S
            operation = 4'b0010;
        else if (ALUOp_IDEXp[0])                 // SB
            operation = 4'b0110;
        else if (inst_IDEXp[30])
            operation = 4'b0110;
        else if (inst_IDEXp[13] & inst_IDEXp[12])
            operation = 4'b0000;
        else if (inst_IDEXp[13] | inst_IDEXp[12])
            operation = 4'b0001;
        else
            operation = 4'b0010;
        
        if (ForwardA == 2'b10)
            data1_ALU = ALUresult_EXMEMp;
        else if (ForwardA == 2'b01)
            data1_ALU = w_data;
        else
            data1_ALU = data1_IDEXp;
            if (ForwardB == 2'b10)
                data2_ALU = ALUresult_EXMEMp;
            else if (ForwardB == 2'b01)
                data2_ALU = w_data;
            else
                data2_ALU = data2_IDEXp;
        
        if (ALUSrc_IDEXp)
        begin
            if (operation == 4'b0000)
                ALUresult_EXMEMp <= data1_ALU & imm_IDEXp;
            else if (operation == 4'b0001)
                ALUresult_EXMEMp <= data1_ALU | imm_IDEXp;
            else if (operation == 4'b0010)
                ALUresult_EXMEMp <= data1_ALU + imm_IDEXp;
            else if (operation == 4'b0110)
                ALUresult_EXMEMp <= data1_ALU - imm_IDEXp;
        end
        else
        begin
            if (operation == 4'b0000)
                ALUresult_EXMEMp <= data1_ALU & data2_ALU;
            else if (operation == 4'b0001)
                ALUresult_EXMEMp <= data1_ALU | data2_ALU;
            else if (operation == 4'b0010)
                ALUresult_EXMEMp <= data1_ALU + data2_ALU;
            else if (operation == 4'b0110)
                ALUresult_EXMEMp <= data1_ALU - data2_ALU;
        end
                
        inst_EXMEMp       <= inst_IDEXp[11:7];
        registerRD_EXMEMp <= registerRD_IDEXp;
        Branch_EXMEMp     <= Branch_IDEXp;
        MemRead_EXMEMp    <= MemRead_IDEXp;
        MemWrite_EXMEMp   <= MemWrite_IDEXp;
        RegWrite_EXMEMp   <= RegWrite_IDEXp;
        MemtoReg_EXMEMp   <= MemtoReg_IDEXp;
        data2_EXMEMp      <= data2;
    end
            
    // MEM stage
    always@(posedge i_clk)
    begin
        ALUresult_MEMWBp  <= ALUresult_EXMEMp;
        RegWrite_MEMWBp   <= RegWrite_EXMEMp;
        MemtoReg_MEMWBp   <= MemtoReg_EXMEMp;
        registerRD_MEMWBp <= registerRD_EXMEMp;
    end
            
    assign i_write = (RegWrite_MEMWBp) ? 1'b1: 1'b0;
    assign w_addr  = registerRD_MEMWBp[4:0];
    assign w_data  = (MemtoReg_MEMWBp) ? data_MEMWBp: ALUresult_MEMWBp;
            
endmodule

