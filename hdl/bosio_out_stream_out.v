`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_stream_out
// Project: Bosio 3DoF Display Output Core (Clean-Slate Architecture)
// Description:
//   AXI4-Stream Video Output Master reading from Pixel FIFO.
//   Generates strictly compliant Video timing:
//   - tuser (SOF) asserted ONLY on first pixel (x=0, y=0).
//   - tlast (EOL) asserted ONLY on line end (x = screen_w - 1).
//   - Pixel counters advance strictly on stream_fire (tvalid & tready).
//   - Zero clock latency mismatch, 100% video timing lock with v_axi4s_vid_out.
// ============================================================================

module bosio_out_stream_out #(
    parameter VIDEO_TDATA_WIDTH = 24
)(
    input  wire                             clk,
    input  wire                             rst_n,

    // Control & Config
    input  wire                             enable,
    input  wire [15:0]                      screen_w,           // 1280
    input  wire [15:0]                      screen_h,           // 720

    // FIFO Interface
    input  wire [VIDEO_TDATA_WIDTH-1:0]     fifo_dout,
    input  wire                             fifo_empty,
    output wire                             fifo_rd_en,

    // AXI4-Stream Video Master Interface
    output wire [VIDEO_TDATA_WIDTH-1:0]     m_axis_video_tdata,
    output wire                             m_axis_video_tvalid,
    input  wire                             m_axis_video_tready,
    output wire                             m_axis_video_tuser,   // SOF
    output wire                             m_axis_video_tlast,   // EOL

    output reg  [15:0]                      frame_counter
);

    reg [15:0] pix_x;
    reg [15:0] pix_y;

    wire stream_fire = m_axis_video_tvalid && m_axis_video_tready;

    assign m_axis_video_tvalid = enable && !fifo_empty;
    assign m_axis_video_tdata  = fifo_dout;
    assign m_axis_video_tuser  = (pix_x == 16'd0) && (pix_y == 16'd0);
    assign m_axis_video_tlast  = (pix_x + 1'b1 == screen_w);
    assign fifo_rd_en          = stream_fire;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pix_x         <= 16'd0;
            pix_y         <= 16'd0;
            frame_counter <= 16'd0;
        end else if (!enable) begin
            pix_x         <= 16'd0;
            pix_y         <= 16'd0;
        end else if (stream_fire) begin
            if (pix_x + 1'b1 == screen_w) begin
                pix_x <= 16'd0;
                if (pix_y + 1'b1 == screen_h) begin
                    pix_y         <= 16'd0;
                    frame_counter <= frame_counter + 1'b1;
                end else begin
                    pix_y <= pix_y + 1'b1;
                end
            end else begin
                pix_x <= pix_x + 1'b1;
            end
        end
    end

endmodule
