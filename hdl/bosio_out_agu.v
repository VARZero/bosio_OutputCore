`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_agu
// Project: Bosio 3DoF Display Output Core (Clean-Slate Architecture)
// Description:
//   High-Speed 74.25MHz Address Generation Unit (AGU):
//   - Zero-multiplier DDA Phase Accumulator for Screen -> Texture coordinates.
//   - Programmable FOV scaling matching 48-degree high-legibility perspective.
//   - Fully pipelined 2D Roll Rotation meeting 74.25MHz/100MHz timing closure.
//   - Emits SOF (Start of Frame) and EOL (End of Line) for AXI4-Stream Video.
// ============================================================================

module bosio_out_agu (
    input  wire                             clk,
    input  wire                             rst_n,

    // Timing & Enable
    input  wire                             enable,
    input  wire                             pixel_ready,        // Ready from downstream video stream
    input  wire [15:0]                      screen_w,           // 1280
    input  wire [15:0]                      screen_h,           // 720
    input  wire [7:0]                       fov_h_deg,          // e.g. 48

    // 3DoF Pose Inputs
    input  wire [4:0]                       center_face_id,     // 1 ~ 20
    input  wire signed [15:0]               roll_sin,           // Q1.15
    input  wire signed [15:0]               roll_cos,           // Q1.15

    // Raster Output Stream
    output reg                              out_pixel_valid,
    output reg                              out_sof,            // Start of Frame
    output reg                              out_eol,            // End of Line
    output reg  [4:0]                       out_face_id,
    output reg  [7:0]                       out_tex_u,          // 0 ~ 255
    output reg  [7:0]                       out_tex_v,          // 0 ~ 255
    output reg                              out_in_bounds,      // 1 if inside face texture, 0 if background
    output reg  [15:0]                      out_screen_x,
    output reg  [15:0]                      out_screen_y,
    output reg                              vblank_active,
    output reg                              frame_busy
);

    // Raster Counters
    reg [15:0] cur_x;
    reg [15:0] cur_y;
    reg        generating;

    // ------------------------------------------------------------------------
    // 1. DDA Phase Step Calculation in Q8.16
    // ------------------------------------------------------------------------
    // Step size = fov_h_deg * 468 in Q8.16. For FOV=48 deg, step = 22464 (~0.3428 texels/pixel)
    wire [15:0] step_q16 = (fov_h_deg != 8'd0) ? (fov_h_deg * 9'd468) : 16'd22464;

    // Start positions centered around (W/2, H/2)
    wire signed [31:0] start_accum_u = -$signed({16'd0, (screen_w >> 1)}) * $signed({16'd0, step_q16});
    wire signed [31:0] start_accum_v = -$signed({16'd0, (screen_h >> 1)}) * $signed({16'd0, step_q16});

    reg signed [31:0] accum_u;
    reg signed [31:0] accum_v;

    // ------------------------------------------------------------------------
    // 2. 3-Stage Pipeline for Screen Roll Rotation & Texture Mapping
    // ------------------------------------------------------------------------
    // Stage 1: Centered Coordinates
    reg signed [15:0] pipe1_u_center;
    reg signed [15:0] pipe1_v_center;
    reg [15:0]        pipe1_x, pipe1_y;
    reg               pipe1_sof, pipe1_eol;
    reg               pipe1_valid;
    reg [4:0]         pipe1_fid;

    // Stage 2: 2D Matrix Rotation Multipliers
    reg signed [31:0] pipe2_rot_u;
    reg signed [31:0] pipe2_rot_v;
    reg [15:0]        pipe2_x, pipe2_y;
    reg               pipe2_sof, pipe2_eol;
    reg               pipe2_valid;
    reg [4:0]         pipe2_fid;
    reg signed [15:0] ru, rv;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe1_valid    <= 1'b0;
            pipe1_sof      <= 1'b0;
            pipe1_eol      <= 1'b0;
            pipe1_u_center <= 16'sd0;
            pipe1_v_center <= 16'sd0;
            pipe1_x        <= 16'd0;
            pipe1_y        <= 16'd0;
            pipe1_fid      <= 5'd1;

            pipe2_valid    <= 1'b0;
            pipe2_sof      <= 1'b0;
            pipe2_eol      <= 1'b0;
            pipe2_rot_u    <= 32'sd0;
            pipe2_rot_v    <= 32'sd0;
            pipe2_x        <= 16'd0;
            pipe2_y        <= 16'd0;
            pipe2_fid      <= 5'd1;

            out_pixel_valid<= 1'b0;
            out_sof        <= 1'b0;
            out_eol        <= 1'b0;
            out_face_id    <= 5'd1;
            out_tex_u      <= 8'd0;
            out_tex_v      <= 8'd0;
            out_in_bounds  <= 1'b0;
            out_screen_x   <= 16'd0;
            out_screen_y   <= 16'd0;
        end else begin
            // Stage 1 Register
            pipe1_valid    <= generating && pixel_ready;
            pipe1_sof      <= (cur_x == 16'd0 && cur_y == 16'd0);
            pipe1_eol      <= (cur_x == screen_w - 1'b1);
            pipe1_x        <= cur_x;
            pipe1_y        <= cur_y;
            pipe1_fid      <= center_face_id;
            pipe1_u_center <= accum_u[31:16];
            pipe1_v_center <= accum_v[31:16];

            // Stage 2 Register (2D Roll Rotation)
            pipe2_valid    <= pipe1_valid;
            pipe2_sof      <= pipe1_sof;
            pipe2_eol      <= pipe1_eol;
            pipe2_x        <= pipe1_x;
            pipe2_y        <= pipe1_y;
            pipe2_fid      <= pipe1_fid;
            pipe2_rot_u    <= ($signed(roll_cos) * pipe1_u_center) - ($signed(roll_sin) * pipe1_v_center);
            pipe2_rot_v    <= ($signed(roll_sin) * pipe1_u_center) + ($signed(roll_cos) * pipe1_v_center);

            // Stage 3 Output Register (Offset to center & boundary check)
            out_pixel_valid<= pipe2_valid;
            out_sof        <= pipe2_sof;
            out_eol        <= pipe2_eol;
            out_face_id    <= pipe2_fid;
            out_screen_x   <= pipe2_x;
            out_screen_y   <= pipe2_y;

            ru = (pipe2_rot_u >>> 15) + 16'sd128; // Centered at 128
            rv = (pipe2_rot_v >>> 15) + 16'sd128;

            if (ru >= 16'sd0 && ru <= 16'sd255 && rv >= 16'sd0 && rv <= 16'sd255) begin
                out_in_bounds <= 1'b1;
                out_tex_u     <= ru[7:0];
                out_tex_v     <= rv[7:0];
            end else begin
                out_in_bounds <= 1'b0; // Outside 256x256 window -> background
                out_tex_u     <= 8'd0;
                out_tex_v     <= 8'd0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 3. Raster Scan State Machine (720p: 1280 x 720)
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            generating    <= 1'b0;
            frame_busy    <= 1'b0;
            vblank_active <= 1'b1;
            cur_x         <= 16'd0;
            cur_y         <= 16'd0;
            accum_u       <= 32'sd0;
            accum_v       <= 32'sd0;
        end else begin
            if (enable && !generating) begin
                generating    <= 1'b1;
                frame_busy    <= 1'b1;
                vblank_active <= 1'b0;
                cur_x         <= 16'd0;
                cur_y         <= 16'd0;
                accum_u       <= start_accum_u;
                accum_v       <= start_accum_v;
            end

            if (generating && pixel_ready) begin
                if (cur_x == screen_w - 1'b1) begin
                    cur_x   <= 16'd0;
                    accum_u <= start_accum_u;
                    if (cur_y == screen_h - 1'b1) begin
                        cur_y         <= 16'd0;
                        accum_v       <= start_accum_v;
                        generating    <= 1'b0;
                        frame_busy    <= 1'b0;
                        vblank_active <= 1'b1; // Enter VBLANK
                    end else begin
                        cur_y   <= cur_y + 1'b1;
                        accum_v <= accum_v + $signed({16'd0, step_q16});
                    end
                end else begin
                    cur_x   <= cur_x + 1'b1;
                    accum_u <= accum_u + $signed({16'd0, step_q16});
                end
            end
        end
    end

endmodule
