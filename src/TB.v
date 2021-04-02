// Testbench Code for Design

module TB();
    localparam INITIAL_FILE    = "test3.hex";
    localparam CLK_PERIOD      = 30;
    localparam RESET_DELAY     = CLK_PERIOD*5.2;
    localparam SIMULATION_TIME = CLK_PERIOD*100.2;
    localparam PM_ADDR_WIDTH = 10;
    localparam DM_ADDR_WIDTH = 10;
    
    //program memory bus signals
    wire [63:0] pm_addr;
    wire [31:0] pm_read_data;
    wire pm_cs;
    
    //data memory bus signals
    wire [63:0] dm_addr;
    wire [63:0] dm_read_data;
    wire [63:0] dm_write_data;
    wire dm_rw;
    wire dm_cs;
    
    //clock&reset
    reg rst;
    reg clk;
    
    //processor core
    Core_kkm Core(
    .o_pm_addr (pm_addr),
    .o_pm_cs (pm_cs),
    .i_pm_data (pm_read_data),
    .o_dm_addr (dm_addr),
    .o_dm_cs (dm_cs),
    .o_dm_rw (dm_rw),
    .o_dm_data (dm_write_data),
    .i_dm_data (dm_read_data),
    .i_clk (clk),
    .i_rst (rst)
    );
    
    //data memory
    SRAM#(
    .WIDTH(64),
    .ADDR_WIDTH(DM_ADDR_WIDTH)
    )dm(
    .i_data (dm_write_data),
    .i_addr (dm_addr[DM_ADDR_WIDTH+2:3]),
    .o_data (dm_read_data),
    .i_cs (dm_cs),
    .i_we ({64{dm_rw}}),
    .i_clk (clk)
    );
    
    //program memory
    SRAM#(
    .INITIAL_FILE(INITIAL_FILE),
    .WIDTH(32),
    .ADDR_WIDTH(PM_ADDR_WIDTH)
    )pm(
    .i_data (32'bx),
    .i_addr (pm_addr[PM_ADDR_WIDTH+1:2]),
    .o_data (pm_read_data),
    .i_cs (pm_cs),
    .i_we (32'b0),
    .i_clk (clk)
    );
    
    //clock&reset generator
    /*
     initial 
     begin
        rst = 1'b0;
        #(RESET_DELAY)
        rst = 1'b1;
        #(CLK_PERIOD)
        rst = 1'b0;
     end
     */

    initial 
    begin
        clk = 0;
        forever # (CLK_PERIOD/2)
            clk = ~clk;
    end
    
    //signal dump
    initial 
    begin
        $dumpfile("TB.vcd");
        $dumpvars(0,TB);
        #(SIMULATION_TIME)
        $dumpflush;
        $finish;
    end
    
endmodule

