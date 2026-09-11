`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_fifo
// Project: Bosio 3DoF Display Output Core
// Description:
//   Synchronous FIFO buffering 24-bit RGB pixels between AGU/DMA Cache
//   and AXI4-Stream Video Output Master.
//   Depth: 1024 words. Decouples raster generation from downstream backpressure.
// ============================================================================

module bosio_out_fifo #(
    parameter DATA_WIDTH = 24,
    parameter ADDR_WIDTH = 10 // Depth = 1024
)(
    input  wire                   clk,
    input  wire                   rst_n,
    input  wire                   flush,

    // Write Interface (from AGU Sampler)
    input  wire                   wr_en,
    input  wire [DATA_WIDTH-1:0]  din,
    output wire                   full,
    output wire [ADDR_WIDTH:0]    count,

    // Read Interface (to AXI-Stream Video Out)
    input  wire                   rd_en,
    output wire [DATA_WIDTH-1:0]  dout,
    output wire                   empty
);

    localparam DEPTH = 1 << ADDR_WIDTH;

    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [ADDR_WIDTH-1:0] wr_ptr;
    reg [ADDR_WIDTH-1:0] rd_ptr;
    reg [ADDR_WIDTH:0]   fifo_cnt;

    assign full  = (fifo_cnt >= (DEPTH - 16)); // Early almost-full flag to prevent overflow
    assign empty = (fifo_cnt == 0);
    assign count = fifo_cnt;

    // Asynchronous read data for low latency stream fire
    assign dout  = mem[rd_ptr];

    // Write Logic
    always @(posedge clk) begin
        if (wr_en && (fifo_cnt < DEPTH)) begin
            mem[wr_ptr] <= din;
        end
    end

    // Pointer & Count Management
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wr_ptr   <= {ADDR_WIDTH{1'b0}};
            rd_ptr   <= {ADDR_WIDTH{1'b0}};
            fifo_cnt <= {(ADDR_WIDTH+1){1'b0}};
        end else if (flush) begin
            wr_ptr   <= {ADDR_WIDTH{1'b0}};
            rd_ptr   <= {ADDR_WIDTH{1'b0}};
            fifo_cnt <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            case ({wr_en && (fifo_cnt < DEPTH), rd_en && !empty})
                2'b10: begin
                    wr_ptr   <= wr_ptr + 1'b1;
                    fifo_cnt <= fifo_cnt + 1'b1;
                end
                2'b01: begin
                    rd_ptr   <= rd_ptr + 1'b1;
                    fifo_cnt <= fifo_cnt - 1'b1;
                end
                2'b11: begin
                    wr_ptr   <= wr_ptr + 1'b1;
                    rd_ptr   <= rd_ptr + 1'b1;
                end
                default: ;
            endcase
        end
    end

endmodule
