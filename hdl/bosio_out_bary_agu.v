`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_bary_agu
// Project: Bosio 3DoF Display Output Core (Hierarchical 7-7-7/8 Architecture)
// Description:
//   Pipelined Continuous Multi-Face Barycentric AGU:
//   - Raster scan coordinates centered on screen matching FOV scaling.
//   - 2D Roll rotation matrix.
//   - Bit-exact barycentric coordinate calculation for canonical equilateral face.
//   - Multi-Face Classification & Edge-Crossing Reflection:
//     * Seamlessly transitions across Edge 0, Edge 1, and Edge 2 into Neighbor faces.
//     * Reflects barycentric coordinates into canonical space with zero DSPs.
//   - 7-7-7/8 Sub-triangle classification (Outer 0/1/2 vs Center inverted).
//   - Micro-tile quantization (0..210) and intra-tile texel offset generation.
//   - 4-Slot Multi-Face BRAM address generation: {face_slot[1:0], tile_id[7:0], tv[2:0], tu[2:0]}.
// ============================================================================

module bosio_out_bary_agu (
    input  wire                                 clk,
    input  wire                                 rst_n,

    // Control & Configuration
    input  wire                                 enable,
    input  wire                                 pixel_ready,        // Ready from downstream FIFO/stream
    input  wire [15:0]                          screen_w,           // 1280
    input  wire [15:0]                          screen_h,           // 720
    input  wire [7:0]                           fov_h_deg,          // e.g. 48

    input  wire signed [15:0]                   roll_sin,           // Q1.15
    input  wire signed [15:0]                   roll_cos,           // Q1.15
    input  wire signed [15:0]                   cam_offset_u,       // DDA accum units
    input  wire signed [15:0]                   cam_offset_v,       // DDA accum units
    input  wire [4:0]                           center_face_id,     // 1 ~ 20
    input  wire [4:0]                           n0_face_id,         // Edge 0 Neighbor
    input  wire [4:0]                           n1_face_id,         // Edge 1 Neighbor
    input  wire [4:0]                           n2_face_id,         // Edge 2 Neighbor
    input  wire                                 tile_mode_16,       // 1: 16x16, 0: 8x8 texels/tile

    // 211-Bit Sparse Activity Bitmask (7 words of 32 bits = 224 bits)
    input  wire [31:0]                          active_mask_0,
    input  wire [31:0]                          active_mask_1,
    input  wire [31:0]                          active_mask_2,
    input  wire [31:0]                          active_mask_3,
    input  wire [31:0]                          active_mask_4,
    input  wire [31:0]                          active_mask_5,
    input  wire [31:0]                          active_mask_6,

    // Generated Pixel Stream
    output reg                                  out_pixel_valid,
    output reg                                  out_sof,
    output reg  [4:0]                           out_face_id,
    output reg  [7:0]                           out_tile_id,        // 0 ~ 210
    output reg  [15:0]                          out_bram_addr,
    output reg                                  out_tile_active,    // 1: Read BRAM, 0: Output Deep Space
    output reg  [15:0]                          out_screen_x,
    output reg  [15:0]                          out_screen_y,

    // Status Signals
    output reg                                  frame_busy,
    output reg                                  vblank_active
);

    // ------------------------------------------------------------------------
    // 1. DDA Phase Step Calculation in Q8.16
    // ------------------------------------------------------------------------
    reg [15:0]         step_q16_reg;
    reg signed [31:0]  start_accum_u_reg;
    reg signed [31:0]  start_accum_v_reg;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            step_q16_reg       <= 16'd22464;
            start_accum_u_reg  <= -32'sd14376960;
            start_accum_v_reg  <=  32'sd8087040;
        end else begin
            step_q16_reg       <= (fov_h_deg != 8'd0) ? (fov_h_deg * 9'd468) : 16'd22464;
            start_accum_u_reg  <= -$signed({16'd0, (screen_w >> 1)}) * $signed({16'd0, step_q16_reg});
            start_accum_v_reg  <=  $signed({16'd0, (screen_h >> 1)}) * $signed({16'd0, step_q16_reg});
        end
    end

    reg [15:0] cur_x;
    reg [15:0] cur_y;
    reg signed [31:0] accum_u;
    reg signed [31:0] accum_v;
    reg generating;

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
                accum_u       <= start_accum_u_reg;
                accum_v       <= start_accum_v_reg;
            end

            if (generating && pixel_ready) begin
                if (cur_x == screen_w - 1'b1) begin
                    cur_x   <= 16'd0;
                    accum_u <= start_accum_u_reg;
                    if (cur_y == screen_h - 1'b1) begin
                        cur_y         <= 16'd0;
                        accum_v       <= start_accum_v_reg;
                        generating    <= 1'b0;
                        frame_busy    <= 1'b0;
                        vblank_active <= 1'b1;
                    end else begin
                        cur_y   <= cur_y + 1'b1;
                        accum_v <= accum_v - $signed({16'd0, step_q16_reg});
                    end
                end else begin
                    cur_x   <= cur_x + 1'b1;
                    accum_u <= accum_u + $signed({16'd0, step_q16_reg});
                end
            end
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 1: Centered Screen Coordinates
    // ------------------------------------------------------------------------
    reg               p1_valid, p1_sof;
    reg [15:0]        p1_x, p1_y;
    reg signed [15:0] p1_u, p1_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p1_valid <= 1'b0;
            p1_sof   <= 1'b0;
            p1_x     <= 16'd0;
            p1_y     <= 16'd0;
            p1_u     <= 16'sd0;
            p1_v     <= 16'sd0;
        end else begin
            p1_valid <= generating && pixel_ready;
            p1_sof   <= generating && pixel_ready && (cur_x == 16'd0 && cur_y == 16'd0);
            p1_x     <= cur_x;
            p1_y     <= cur_y;
            p1_u     <= accum_u[31:16] + cam_offset_u;
            p1_v     <= accum_v[31:16] + cam_offset_v;
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 2: 2D Roll Rotation (DSP Multipliers)
    // ------------------------------------------------------------------------
    reg               p2_valid, p2_sof;
    reg [15:0]        p2_x, p2_y;
    reg signed [31:0] p2_rot_u, p2_rot_v;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p2_valid <= 1'b0;
            p2_sof   <= 1'b0;
            p2_x     <= 16'd0;
            p2_y     <= 16'd0;
            p2_rot_u <= 32'sd0;
            p2_rot_v <= 32'sd0;
        end else begin
            p2_valid <= p1_valid;
            p2_sof   <= p1_sof;
            p2_x     <= p1_x;
            p2_y     <= p1_y;
            p2_rot_u <= ($signed(roll_cos) * p1_u) - ($signed(roll_sin) * p1_v);
            p2_rot_v <= ($signed(roll_sin) * p1_u) + ($signed(roll_cos) * p1_v);
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 3: Canonical Equilateral Coordinates (px, py in Q1.15)
    // Centroid at (0.5, sqrt(3)/6) = (16384, 9459)
    // ------------------------------------------------------------------------
    reg               p3_valid, p3_sof;
    reg [15:0]        p3_x, p3_y;
    reg signed [17:0] p3_px, p3_py;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p3_valid <= 1'b0;
            p3_sof   <= 1'b0;
            p3_x     <= 16'd0;
            p3_y     <= 16'd0;
            p3_px    <= 18'sd0;
            p3_py    <= 18'sd0;
        end else begin
            p3_valid <= p2_valid;
            p3_sof   <= p2_sof;
            p3_x     <= p2_x;
            p3_y     <= p2_y;
            p3_px    <= ($signed(p2_rot_u[30:15]) <<< 7) + 18'sd16384;
            p3_py    <= ($signed(p2_rot_v[30:15]) <<< 7) + 18'sd9459;
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 4: Center Barycentric Coordinates in Q1.15
    // l0 = (py * 37837) >>> 15   (where 37837 ~= 2/sqrt(3) in Q1.15)
    // l2 = px - (l0 >>> 1)
    // l1 = 32768 - l0 - l2
    // ------------------------------------------------------------------------
    reg               p4_valid, p4_sof;
    reg [15:0]        p4_x, p4_y;
    reg signed [17:0] p4_l0, p4_l1, p4_l2;

    wire signed [35:0] mult_l0 = $signed(p3_py) * $signed(36'sd37837);
    reg signed [17:0] l0_temp, l2_temp, l1_temp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p4_valid  <= 1'b0;
            p4_sof    <= 1'b0;
            p4_x      <= 16'd0;
            p4_y      <= 16'd0;
            p4_l0     <= 18'sd0;
            p4_l1     <= 18'sd0;
            p4_l2     <= 18'sd0;
        end else begin
            l0_temp = mult_l0[32:15];
            l2_temp = p3_px - (l0_temp >>> 1);
            l1_temp = 18'sd32768 - l0_temp - l2_temp;

            p4_valid  <= p3_valid;
            p4_sof    <= p3_sof;
            p4_x      <= p3_x;
            p4_y      <= p3_y;
            p4_l0     <= l0_temp;
            p4_l1     <= l1_temp;
            p4_l2     <= l2_temp;
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 5: Multi-Face Classification & Edge-Crossing Reflection
    // Selects slot (0: Center, 1: N0, 2: N1, 3: N2) and reflects coordinates
    // ------------------------------------------------------------------------
    reg               p5_valid, p5_sof;
    reg [15:0]        p5_x, p5_y;
    reg [1:0]         p5_slot;
    reg [4:0]         p5_fid;
    reg signed [17:0] p5_l0, p5_l1, p5_l2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p5_valid <= 1'b0;
            p5_sof   <= 1'b0;
            p5_x     <= 16'd0;
            p5_y     <= 16'd0;
            p5_slot  <= 2'd0;
            p5_fid   <= 5'd1;
            p5_l0    <= 18'sd0;
            p5_l1    <= 18'sd0;
            p5_l2    <= 18'sd0;
        end else begin
            p5_valid <= p4_valid;
            p5_sof   <= p4_sof;
            p5_x     <= p4_x;
            p5_y     <= p4_y;

            if (p4_l0 >= 18'sd0 && p4_l1 >= 18'sd0 && p4_l2 >= 18'sd0) begin
                // Region 0: Center Triangle
                p5_slot <= 2'd0;
                p5_fid  <= center_face_id;
                p5_l0   <= p4_l0;
                p5_l1   <= p4_l1;
                p5_l2   <= p4_l2;
            end else if (p4_l0 <= p4_l1 && p4_l0 <= p4_l2) begin
                // Region 1: Cross Edge 0 (Bottom) -> Neighbor 0
                p5_slot <= 2'd1;
                p5_fid  <= n0_face_id;
                p5_l0   <= -p4_l0;
                p5_l1   <= p4_l2 + p4_l0;
                p5_l2   <= p4_l1 + p4_l0;
            end else if (p4_l1 <= p4_l0 && p4_l1 <= p4_l2) begin
                // Region 2: Cross Edge 1 (Right) -> Neighbor 1
                p5_slot <= 2'd2;
                p5_fid  <= n1_face_id;
                p5_l0   <= -p4_l1;
                p5_l1   <= p4_l0 + p4_l1;
                p5_l2   <= p4_l2 + p4_l1;
            end else begin
                // Region 3: Cross Edge 2 (Left) -> Neighbor 2
                p5_slot <= 2'd3;
                p5_fid  <= n2_face_id;
                p5_l0   <= -p4_l2;
                p5_l1   <= p4_l1 + p4_l2;
                p5_l2   <= p4_l0 + p4_l2;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 6: Sub-Triangle Classification & Local Barycentric (mu0, mu1, mu2)
    // ------------------------------------------------------------------------
    wire [15:0] c_l0 = (p5_l0 < 18'sd0) ? 16'd0 : ((p5_l0 > 18'sd32768) ? 16'd32768 : p5_l0[15:0]);
    wire [15:0] c_l1 = (p5_l1 < 18'sd0) ? 16'd0 : ((p5_l1 > 18'sd32768) ? 16'd32768 : p5_l1[15:0]);
    wire [15:0] c_l2 = (p5_l2 < 18'sd0) ? 16'd0 : ((p5_l2 > 18'sd32768) ? 16'd32768 : p5_l2[15:0]);

    reg               p6_valid, p6_sof;
    reg [15:0]        p6_x, p6_y;
    reg [1:0]         p6_slot;
    reg [4:0]         p6_fid;
    reg [1:0]         p6_region;     // 0: Outer 0, 1: Outer 1, 2: Outer 2, 3: Center
    reg [15:0]        p6_mu0, p6_mu1, p6_mu2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p6_valid  <= 1'b0;
            p6_sof    <= 1'b0;
            p6_x      <= 16'd0;
            p6_y      <= 16'd0;
            p6_slot   <= 2'd0;
            p6_fid    <= 5'd1;
            p6_region <= 2'd0;
            p6_mu0    <= 16'd0;
            p6_mu1    <= 16'd0;
            p6_mu2    <= 16'd0;
        end else begin
            p6_valid <= p5_valid;
            p6_sof   <= p5_sof;
            p6_x     <= p5_x;
            p6_y     <= p5_y;
            p6_slot  <= p5_slot;
            p6_fid   <= p5_fid;

            if (c_l0 >= 16'd16384) begin
                p6_region <= 2'd0; // Outer 0 (Top)
                p6_mu0    <= (c_l0 << 1) - 16'd32768;
                p6_mu1    <= (c_l1 << 1);
                p6_mu2    <= (c_l2 << 1);
            end else if (c_l1 >= 16'd16384) begin
                p6_region <= 2'd1; // Outer 1 (Bottom-Left)
                p6_mu0    <= (c_l0 << 1);
                p6_mu1    <= (c_l1 << 1) - 16'd32768;
                p6_mu2    <= (c_l2 << 1);
            end else if (c_l2 >= 16'd16384) begin
                p6_region <= 2'd2; // Outer 2 (Bottom-Right)
                p6_mu0    <= (c_l0 << 1);
                p6_mu1    <= (c_l1 << 1);
                p6_mu2    <= (c_l2 << 1) - 16'd32768;
            end else begin
                p6_region <= 2'd3; // Center Inverted
                p6_mu0    <= 16'd32768 - (c_l0 << 1);
                p6_mu1    <= 16'd32768 - (c_l1 << 1);
                p6_mu2    <= 16'd32768 - (c_l2 << 1);
            end
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 7: Grid Scaling by N=7 (Outer) or N=8 (Center)
    // ------------------------------------------------------------------------
    reg               p7_valid, p7_sof;
    reg [15:0]        p7_x, p7_y;
    reg [1:0]         p7_slot;
    reg [4:0]         p7_fid;
    reg [1:0]         p7_region;
    reg [18:0]        p7_s0, p7_s1, p7_s2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p7_valid  <= 1'b0;
            p7_sof    <= 1'b0;
            p7_x      <= 16'd0;
            p7_y      <= 16'd0;
            p7_slot   <= 2'd0;
            p7_fid    <= 5'd1;
            p7_region <= 2'd0;
            p7_s0     <= 19'd0;
            p7_s1     <= 19'd0;
            p7_s2     <= 19'd0;
        end else begin
            p7_valid  <= p6_valid;
            p7_sof    <= p6_sof;
            p7_x      <= p6_x;
            p7_y      <= p6_y;
            p7_slot   <= p6_slot;
            p7_fid    <= p6_fid;
            p7_region <= p6_region;

            if (p6_region == 2'd3) begin
                // Center (N=8): mu << 3
                p7_s0 <= {p6_mu0, 3'd0};
                p7_s1 <= {p6_mu1, 3'd0};
                p7_s2 <= {p6_mu2, 3'd0};
            end else begin
                // Outer (N=7): (mu << 3) - mu
                p7_s0 <= {p6_mu0, 3'd0} - {3'd0, p6_mu0};
                p7_s1 <= {p6_mu1, 3'd0} - {3'd0, p6_mu1};
                p7_s2 <= {p6_mu2, 3'd0} - {3'd0, p6_mu2};
            end
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 8: Tile ID & Intra-Tile Texel Quantization
    // ------------------------------------------------------------------------
    reg               p8_valid, p8_sof;
    reg [15:0]        p8_x, p8_y;
    reg [1:0]         p8_slot;
    reg [4:0]         p8_fid;
    reg [7:0]         p8_tile_id;
    reg [15:0]        p8_bram_addr;

    reg [3:0]  f0, f1, f2, u, col;
    reg        is_inv;
    reg [5:0]  sub_idx;
    reg [7:0]  base_off;
    reg [7:0]  full_tid;
    reg [14:0] frac_u, frac_v;
    reg [3:0]  tu, tv;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p8_valid     <= 1'b0;
            p8_sof       <= 1'b0;
            p8_x         <= 16'd0;
            p8_y         <= 16'd0;
            p8_slot      <= 2'd0;
            p8_fid       <= 5'd1;
            p8_tile_id   <= 8'd0;
            p8_bram_addr <= 16'd0;
        end else begin
            f0 = p7_s0[18:15];
            f1 = p7_s1[18:15];
            f2 = p7_s2[18:15];

            if (p7_region == 2'd3) begin
                // Center (N=8)
                u = (f0 >= 4'd7) ? 4'd0 : (4'd7 - f0);
                col = (f2 > u) ? u : f2;
                is_inv = (f0 + f1 + f2 == 4'd6);
                sub_idx = (u * u) + (col << 1) + (is_inv ? 6'd1 : 6'd0);
                base_off = 8'd147;
            end else begin
                // Outer (N=7)
                u = (f0 >= 4'd6) ? 4'd0 : (4'd6 - f0);
                col = (f2 > u) ? u : f2;
                is_inv = (f0 + f1 + f2 == 4'd5);
                sub_idx = (u * u) + (col << 1) + (is_inv ? 6'd1 : 6'd0);
                base_off = p7_region * 8'd49;
            end

            full_tid = base_off + sub_idx;
            if (full_tid > 8'd210) full_tid = 8'd210;

            // Intra-tile fractional coordinates
            frac_u = p7_s2[14:0];
            frac_v = p7_s0[14:0];
            if (is_inv) begin
                frac_u = 15'd32767 - frac_u;
                frac_v = 15'd32767 - frac_v;
            end

            if (tile_mode_16) begin
                tu = frac_u[14:11]; // 4 bits [0..15]
                tv = frac_v[14:11]; // 4 bits [0..15]
                p8_bram_addr <= {full_tid, tv, tu};
            end else begin
                tu = {1'b0, frac_u[14:12]}; // 3 bits [0..7]
                tv = {1'b0, frac_v[14:12]}; // 3 bits [0..7]
                p8_bram_addr <= {p7_slot, full_tid, tv[2:0], tu[2:0]};
            end

            p8_valid   <= p7_valid;
            p8_sof     <= p7_sof;
            p8_x       <= p7_x;
            p8_y       <= p7_y;
            p8_slot    <= p7_slot;
            p8_fid     <= p7_fid;
            p8_tile_id <= full_tid;
        end
    end

    // ------------------------------------------------------------------------
    // Pipeline Stage 9: Output Registration & 211-Bit Sparse Occupancy Evaluation
    // ------------------------------------------------------------------------
    reg [31:0] cur_mask_word;
    always @(*) begin
        case (p8_tile_id[7:5])
            3'd0: cur_mask_word = active_mask_0;
            3'd1: cur_mask_word = active_mask_1;
            3'd2: cur_mask_word = active_mask_2;
            3'd3: cur_mask_word = active_mask_3;
            3'd4: cur_mask_word = active_mask_4;
            3'd5: cur_mask_word = active_mask_5;
            3'd6: cur_mask_word = active_mask_6;
            default: cur_mask_word = 32'd0;
        endcase
    end
    wire cur_tile_is_active = cur_mask_word[p8_tile_id[4:0]];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_pixel_valid <= 1'b0;
            out_sof         <= 1'b0;
            out_face_id     <= 5'd1;
            out_tile_id     <= 8'd0;
            out_bram_addr   <= 16'd0;
            out_tile_active <= 1'b0;
            out_screen_x    <= 16'd0;
            out_screen_y    <= 16'd0;
        end else begin
            out_pixel_valid <= p8_valid;
            out_sof         <= p8_sof;
            out_face_id     <= p8_fid;
            out_tile_id     <= p8_tile_id;
            out_bram_addr   <= p8_bram_addr;
            out_tile_active <= cur_tile_is_active; // Sparse Culling: 1 if active, 0 if empty space
            out_screen_x    <= p8_x;
            out_screen_y    <= p8_y;
        end
    end

endmodule
