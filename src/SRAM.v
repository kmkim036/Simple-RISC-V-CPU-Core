// SRAM code
// Single-port Synchronous RAM
// Size: width X 2^(ADDR_WIDTH) bits

module SRAM#(parameter INITIAL_FILE = "",
             parameter WIDTH = 32,
             parameter ADDR_WIDTH = 10)
            (input wire [WIDTH-1:0] i_data,
             input wire [ADDR_WIDTH-1:0] i_addr,
             output reg [WIDTH-1:0] o_data,
             input wire i_cs,
             input wire [WIDTH-1:0] i_we,
             input wire i_clk);

    localparam DEPTH = 2**ADDR_WIDTH;
    reg [WIDTH-1:0] mem[0:DEPTH-1];
    
    always@(posedge i_clk)
        if (i_cs & (~|i_we)) 
            o_data <= #1 mem[i_addr];
        
    always@(posedge i_clk)
        if (i_cs & (i_we))
            mem[i_addr] <= #1 (~i_we & mem[i_addr]) | (i_we & i_data);
    
    initial 
        begin
        if (INITIAL_FILE! = "")
            begin
                $readmemh(INITIAL_FILE, mem);
                $display("%s was loaded successfully.", INITIAL_FILE);
            end
        end

endmodule
