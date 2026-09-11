`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_reg_ctrl
// Project: Bosio 3DoF Display Output Core (Hierarchical 7-7-7/8 Architecture)
// Description:
//   AXI4-Lite Slave register control interface for Zynq Processing System (PS).
//   Controls display enable, pose source mode, fallback SW pose, base address,
//   resolution, 211-bit sparse activity bitmasks, and tile mode configuration.
// ============================================================================

module bosio_out_reg_ctrl #(
    parameter C_S_AXI_DATA_WIDTH = 32,
    parameter C_S_AXI_ADDR_WIDTH = 7
)(
    // Clocks and Resets
    input  wire                                 clk,
    input  wire                                 rst_n,

    // AXI4-Lite Write Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_awaddr,
    input  wire [2:0]                           s_axi_awprot,
    input  wire                                 s_axi_awvalid,
    output reg                                  s_axi_awready,

    // AXI4-Lite Write Data Channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]       s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0]   s_axi_wstrb,
    input  wire                                 s_axi_wvalid,
    output reg                                  s_axi_wready,

    // AXI4-Lite Write Response Channel
    output reg  [1:0]                           s_axi_bresp,
    output reg                                  s_axi_bvalid,
    input  wire                                 s_axi_bready,

    // AXI4-Lite Read Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]       s_axi_araddr,
    input  wire [2:0]                           s_axi_arprot,
    input  wire                                 s_axi_arvalid,
    output reg                                  s_axi_arready,

    // AXI4-Lite Read Data Channel
    output reg  [C_S_AXI_DATA_WIDTH-1:0]       s_axi_rdata,
    output reg  [1:0]                           s_axi_rresp,
    output reg                                  s_axi_rvalid,
    input  wire                                 s_axi_rready,

    // ------------------------------------------------------------------------
    // Control Registers Outputs
    // ------------------------------------------------------------------------
    output wire                                 ip_enable,           // CR[0]
    output wire                                 soft_reset,          // CR[1] (pulsed)
    output wire                                 continuous_mode,     // CR[2]
    output wire                                 pose_src_sel,        // CR[3]: 0=Reg, 1=Sensor Stream
    output wire                                 test_pattern_en,     // CR[4]
    output wire                                 force_fetch,         // CR[5] (pulsed)
    output reg  [31:0]                          fb_base_addr,        // 0x08
    output reg  [15:0]                          screen_w,            // 0x0C[15:0]
    output reg  [15:0]                          screen_h,            // 0x0C[31:16]
    output reg  signed [15:0]                   reg_yaw_deg_q8,      // 0x10[31:16] (0~360 deg Q8.8)
    output reg  signed [15:0]                   reg_pitch_deg_q8,    // 0x10[15:0]  (-90~+90 deg Q8.8)
    output reg  signed [15:0]                   reg_roll_deg_q8,     // 0x14[15:0]  (-180~+180 deg Q8.8)
    output reg  [7:0]                           fov_h_deg,           // 0x18[7:0]
    output reg  [7:0]                           fov_v_deg,           // 0x18[23:16]
    output reg  [31:0]                          face_stride,         // 0x1C (default 0x40000 = 256KB)

    // 211-Bit Sparse Activity Bitmask (7 words of 32 bits = 224 bits)
    output reg  [31:0]                          active_mask_0,       // 0x40 (Tiles   0 ~  31)
    output reg  [31:0]                          active_mask_1,       // 0x44 (Tiles  32 ~  63)
    output reg  [31:0]                          active_mask_2,       // 0x48 (Tiles  64 ~  95)
    output reg  [31:0]                          active_mask_3,       // 0x4C (Tiles  96 ~ 127)
    output reg  [31:0]                          active_mask_4,       // 0x50 (Tiles 128 ~ 159)
    output reg  [31:0]                          active_mask_5,       // 0x54 (Tiles 160 ~ 191)
    output reg  [31:0]                          active_mask_6,       // 0x58 (Tiles 192 ~ 210)
    output reg                                  tile_mode_16,        // 0x5C[0]: 1=16x16, 0=8x8

    // ------------------------------------------------------------------------
    // Status & Debug Inputs from Datapath
    // ------------------------------------------------------------------------
    input  wire                                 frame_busy,
    input  wire                                 dma_busy,
    input  wire                                 vblank_active,
    input  wire [4:0]                           active_face_id,      // 1 ~ 20
    input  wire                                 sensor_stream_active,
    input  wire signed [31:0]                   sensor_yaw_mrad,
    input  wire signed [31:0]                   sensor_pitch_mrad,
    input  wire signed [31:0]                   sensor_roll_mrad,
    input  wire [31:0]                          sensor_pkt_count,
    input  wire [15:0]                          frame_counter,
    input  wire [31:0]                          debug_last_rdata,
    input  wire [31:0]                          debug_dma_state,
    input  wire [31:0]                          debug_rvalid_cnt
);

    // Register Offsets
    localparam ADDR_CR          = 7'h00;
    localparam ADDR_SR          = 7'h04;
    localparam ADDR_FB_BASE     = 7'h08;
    localparam ADDR_SCREEN_RES  = 7'h0C;
    localparam ADDR_POSE_AZ_EL  = 7'h10;
    localparam ADDR_POSE_ROLL   = 7'h14;
    localparam ADDR_FOV_CONFIG  = 7'h18;
    localparam ADDR_FACE_STRIDE = 7'h1C;
    localparam ADDR_ACTIVE_FACE = 7'h20;
    localparam ADDR_SENSOR_YAW  = 7'h24;
    localparam ADDR_SENSOR_PITCH= 7'h28;
    localparam ADDR_SENSOR_ROLL = 7'h2C;
    localparam ADDR_SENSOR_PKTS = 7'h30;

    localparam ADDR_MASK_0      = 7'h40;
    localparam ADDR_MASK_1      = 7'h44;
    localparam ADDR_MASK_2      = 7'h48;
    localparam ADDR_MASK_3      = 7'h4C;
    localparam ADDR_MASK_4      = 7'h50;
    localparam ADDR_MASK_5      = 7'h54;
    localparam ADDR_MASK_6      = 7'h58;
    localparam ADDR_TILE_CFG    = 7'h5C;

    reg [31:0] cr_r;
    reg [4:0]  soft_rst_cnt;
    reg        force_fetch_r;

    assign ip_enable        = cr_r[0];
    assign soft_reset       = (soft_rst_cnt != 5'd0);
    assign continuous_mode  = cr_r[2];
    assign pose_src_sel     = cr_r[3];
    assign test_pattern_en  = cr_r[4];
    assign force_fetch      = force_fetch_r;

    // Status Register
    wire [31:0] sr_wire = {
        frame_counter,                  // [31:16]
        3'd0, active_face_id,           // [15:8]
        3'd0,
        vblank_active,                  // [4]
        dma_busy,                       // [3]
        sensor_stream_active,           // [2]
        frame_busy,                     // [1]
        !frame_busy                     // [0] Idle
    };

    // ------------------------------------------------------------------------
    // Write Process
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_awready   <= 1'b0;
            s_axi_wready    <= 1'b0;
            s_axi_bvalid    <= 1'b0;
            s_axi_bresp     <= 2'b00;
            cr_r            <= 32'h00000000; // Disabled by default, SW enables after configuring
            soft_rst_cnt    <= 5'd0;
            force_fetch_r   <= 1'b0;
            fb_base_addr    <= 32'h15C00000;
            screen_w        <= 16'd1280;
            screen_h        <= 16'd720;
            reg_yaw_deg_q8  <= 16'sd0;
            reg_pitch_deg_q8<= 16'sd0;
            reg_roll_deg_q8 <= 16'sd0;
            fov_h_deg       <= 8'd48;
            fov_v_deg       <= 8'd36;
            face_stride     <= 32'h00040000; // 256KB per face

            // Default all tiles active
            active_mask_0   <= 32'hFFFFFFFF;
            active_mask_1   <= 32'hFFFFFFFF;
            active_mask_2   <= 32'hFFFFFFFF;
            active_mask_3   <= 32'hFFFFFFFF;
            active_mask_4   <= 32'hFFFFFFFF;
            active_mask_5   <= 32'hFFFFFFFF;
            active_mask_6   <= 32'hFFFFFFFF;
            tile_mode_16    <= 1'b1; // Default 16x16
        end else begin
            // Self-clearing pulse counters
            if (soft_rst_cnt != 5'd0) soft_rst_cnt <= soft_rst_cnt - 1'b1;
            if (force_fetch_r)        force_fetch_r <= 1'b0;

            // Handshake logic
            if (~s_axi_awready && s_axi_awvalid && s_axi_wvalid) begin
                s_axi_awready <= 1'b1;
                s_axi_wready  <= 1'b1;
            end else begin
                s_axi_awready <= 1'b0;
                s_axi_wready  <= 1'b0;
            end

            if (s_axi_awready && s_axi_awvalid && s_axi_wready && s_axi_wvalid) begin
                s_axi_bvalid <= 1'b1;
                s_axi_bresp  <= 2'b00; // OKAY

                case (s_axi_awaddr[6:0])
                    ADDR_CR: begin
                        cr_r <= s_axi_wdata & ~32'h00000022; // Clear pulse bits 1 and 5
                        if (s_axi_wdata[1]) soft_rst_cnt  <= 5'd16;
                        if (s_axi_wdata[5]) force_fetch_r <= 1'b1;
                    end
                    ADDR_FB_BASE:     fb_base_addr     <= s_axi_wdata;
                    ADDR_SCREEN_RES:  begin
                        screen_w <= s_axi_wdata[15:0];
                        screen_h <= s_axi_wdata[31:16];
                    end
                    ADDR_POSE_AZ_EL:  begin
                        reg_pitch_deg_q8 <= $signed(s_axi_wdata[15:0]);
                        reg_yaw_deg_q8   <= $signed(s_axi_wdata[31:16]);
                    end
                    ADDR_POSE_ROLL:   reg_roll_deg_q8  <= $signed(s_axi_wdata[15:0]);
                    ADDR_FOV_CONFIG:  begin
                        fov_h_deg <= s_axi_wdata[7:0];
                        fov_v_deg <= s_axi_wdata[23:16];
                    end
                    ADDR_FACE_STRIDE: face_stride      <= s_axi_wdata;
                    ADDR_MASK_0:      active_mask_0    <= s_axi_wdata;
                    ADDR_MASK_1:      active_mask_1    <= s_axi_wdata;
                    ADDR_MASK_2:      active_mask_2    <= s_axi_wdata;
                    ADDR_MASK_3:      active_mask_3    <= s_axi_wdata;
                    ADDR_MASK_4:      active_mask_4    <= s_axi_wdata;
                    ADDR_MASK_5:      active_mask_5    <= s_axi_wdata;
                    ADDR_MASK_6:      active_mask_6    <= s_axi_wdata;
                    ADDR_TILE_CFG:    tile_mode_16     <= s_axi_wdata[0];
                    default: ;
                endcase
            end else if (s_axi_bready && s_axi_bvalid) begin
                s_axi_bvalid <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------------
    // Read Process
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            s_axi_arready <= 1'b0;
            s_axi_rvalid  <= 1'b0;
            s_axi_rresp   <= 2'b00;
            s_axi_rdata   <= 32'd0;
        end else begin
            if (~s_axi_arready && s_axi_arvalid) begin
                s_axi_arready <= 1'b1;
            end else begin
                s_axi_arready <= 1'b0;
            end

            if (s_axi_arready && s_axi_arvalid) begin
                s_axi_rvalid <= 1'b1;
                s_axi_rresp  <= 2'b00; // OKAY

                case (s_axi_araddr[6:0])
                    ADDR_CR:          s_axi_rdata <= cr_r | {26'd0, force_fetch_r, 3'd0, soft_reset, 1'd0};
                    ADDR_SR:          s_axi_rdata <= sr_wire;
                    ADDR_FB_BASE:     s_axi_rdata <= fb_base_addr;
                    ADDR_SCREEN_RES:  s_axi_rdata <= {screen_h, screen_w};
                    ADDR_POSE_AZ_EL:  s_axi_rdata <= {reg_yaw_deg_q8, reg_pitch_deg_q8};
                    ADDR_POSE_ROLL:   s_axi_rdata <= {{16{reg_roll_deg_q8[15]}}, reg_roll_deg_q8};
                    ADDR_FOV_CONFIG:  s_axi_rdata <= {8'd0, fov_v_deg, 8'd0, fov_h_deg};
                    ADDR_FACE_STRIDE: s_axi_rdata <= face_stride;
                    ADDR_ACTIVE_FACE: s_axi_rdata <= {27'd0, active_face_id};
                    ADDR_SENSOR_YAW:  s_axi_rdata <= sensor_yaw_mrad;
                    ADDR_SENSOR_PITCH:s_axi_rdata <= sensor_pitch_mrad;
                    ADDR_SENSOR_ROLL: s_axi_rdata <= sensor_roll_mrad;
                    ADDR_SENSOR_PKTS: s_axi_rdata <= sensor_pkt_count;
                    7'h34:            s_axi_rdata <= debug_last_rdata;
                    7'h38:            s_axi_rdata <= debug_dma_state;
                    7'h3C:            s_axi_rdata <= debug_rvalid_cnt;
                    ADDR_MASK_0:      s_axi_rdata <= active_mask_0;
                    ADDR_MASK_1:      s_axi_rdata <= active_mask_1;
                    ADDR_MASK_2:      s_axi_rdata <= active_mask_2;
                    ADDR_MASK_3:      s_axi_rdata <= active_mask_3;
                    ADDR_MASK_4:      s_axi_rdata <= active_mask_4;
                    ADDR_MASK_5:      s_axi_rdata <= active_mask_5;
                    ADDR_MASK_6:      s_axi_rdata <= active_mask_6;
                    ADDR_TILE_CFG:    s_axi_rdata <= {31'd0, tile_mode_16};
                    default:          s_axi_rdata <= 32'd0;
                endcase
            end else if (s_axi_rready && s_axi_rvalid) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end

endmodule
