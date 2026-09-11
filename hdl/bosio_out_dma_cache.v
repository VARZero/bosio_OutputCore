`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_dma_cache
// Project: Bosio 3DoF Display Output Core (Hierarchical 7-7-7/8 Architecture)
// Description:
//   AXI4 Master Burst Read DMA & Ping-Pong Dual-Bank BRAM Cache:
//   - Dual 64K x 24-bit Block RAM buffers for glitch-free ping-pong display.
//   - 4-Slot Multi-Face Fetch: Center Face (Slot 0), N0 (Slot 1), N1 (Slot 2), N2 (Slot 3).
//   - Fetches 16K words per face (total 64K words) via 16-beat bursts.
//   - Bank swap synchronized to Start-of-Frame (SOF) guarantees zero screen tearing.
//   - 2-cycle pipelined texture sampler provides read responses to AGU.
// ============================================================================

module bosio_out_dma_cache #(
    parameter C_M_AXI_ADDR_WIDTH = 32,
    parameter C_M_AXI_DATA_WIDTH = 32,
    parameter C_M_AXI_BURST_LEN  = 16,
    parameter TOTAL_PIXELS       = 65536 // 64K words
)(
    input  wire                             clk,
    input  wire                             rst_n,

    // Control & Status
    input  wire                             enable,
    input  wire                             test_pattern_en,
    input  wire                             force_fetch,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0]    fb_base_addr,
    input  wire [31:0]                      face_stride,
    input  wire [4:0]                       center_face_id,       // Current view face (1 ~ 20)
    input  wire [4:0]                       n0_face_id,           // Edge 0 Neighbor
    input  wire [4:0]                       n1_face_id,           // Edge 1 Neighbor
    input  wire [4:0]                       n2_face_id,           // Edge 2 Neighbor
    input  wire                             sof_pulse,            // Start-of-Frame pulse for bank swap

    // 211-Bit Sparse Activity Bitmask (7 words = 224 bits)
    input  wire [31:0]                      active_mask_0,
    input  wire [31:0]                      active_mask_1,
    input  wire [31:0]                      active_mask_2,
    input  wire [31:0]                      active_mask_3,
    input  wire [31:0]                      active_mask_4,
    input  wire [31:0]                      active_mask_5,
    input  wire [31:0]                      active_mask_6,

    output reg                              dma_busy,

    // AXI4 Full Master Read Interface
    output wire [C_M_AXI_ADDR_WIDTH-1:0]    m_axi_araddr,
    output wire [7:0]                       m_axi_arlen,
    output wire [2:0]                       m_axi_arsize,
    output wire [1:0]                       m_axi_arburst,
    output wire [1:0]                       m_axi_arlock,
    output wire [3:0]                       m_axi_arcache,
    output wire [2:0]                       m_axi_arprot,
    output wire [3:0]                       m_axi_arqos,
    output reg                              m_axi_arvalid,
    input  wire                             m_axi_arready,

    input  wire [C_M_AXI_DATA_WIDTH-1:0]    m_axi_rdata,
    input  wire [1:0]                       m_axi_rresp,
    input  wire                             m_axi_rlast,
    input  wire                             m_axi_rvalid,
    output wire                             m_axi_rready,

    // AGU Sampler Read Port
    input  wire                             sampler_valid,
    input  wire                             sampler_tile_active,  // 1: Active Tile, 0: Background
    input  wire [4:0]                       sampler_face_id,
    input  wire [15:0]                      sampler_bram_addr,    // 16-bit address {slot[1:0], tile_id[7:0], tv[2:0], tu[2:0]}
    output reg  [23:0]                      pixel_rgb,
    output reg                              pixel_valid,

    // Hardware Debug Diagnostics
    output reg  [31:0]                      debug_last_rdata,
    output wire [31:0]                      debug_dma_state,
    output reg  [31:0]                      debug_rvalid_cnt
);

    // Constant AXI4 Master parameters
    assign m_axi_arlen   = C_M_AXI_BURST_LEN - 1; // 15 (16 beats)
    assign m_axi_arsize  = 3'b010; // 4 bytes (32-bit)
    assign m_axi_arburst = 2'b01;  // INCR burst
    assign m_axi_arlock  = 2'b00;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot  = 3'b000;
    assign m_axi_arqos   = 4'b0000;
    assign m_axi_rready  = 1'b1;   // Always ready to accept read data into BRAM

    localparam SLOT_WORDS   = 16384; // 16K words per face (64 KB)
    localparam SLOT_BURSTS  = SLOT_WORDS / C_M_AXI_BURST_LEN; // 1024 bursts
    localparam TOTAL_BURSTS = TOTAL_PIXELS / C_M_AXI_BURST_LEN; // 4096 bursts

    // ------------------------------------------------------------------------
    // Ping-Pong Dual-Bank Block RAM Cache (64K x 24-bit each)
    // ------------------------------------------------------------------------
    (* ram_style = "block" *) reg [23:0] bram_bank0 [0:TOTAL_PIXELS-1];
    (* ram_style = "block" *) reg [23:0] bram_bank1 [0:TOTAL_PIXELS-1];

    reg        display_bank;        // 0: Bank 0 displays, Bank 1 fetches. 1: Bank 1 displays, Bank 0 fetches.
    reg [4:0]  displayed_face_id;   // Face ID currently in display bank
    reg [4:0]  target_face_id;      // Face ID currently being fetched

    wire [15:0] rd_addr = sampler_bram_addr;
    reg  [23:0] bank0_dout, bank1_dout;
    reg         pipe_valid;
    reg         pipe_tile_active;
    reg         pipe_test_en;
    reg [15:0]  pipe_addr;

    // Synchronous Read Ports for BRAM & 1-stage control pipeline
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pipe_valid       <= 1'b0;
            pipe_tile_active <= 1'b0;
            pipe_test_en     <= 1'b0;
            pipe_addr        <= 16'd0;
        end else begin
            pipe_valid       <= sampler_valid;
            pipe_tile_active <= sampler_tile_active;
            pipe_test_en     <= test_pattern_en;
            pipe_addr        <= sampler_bram_addr;
        end
    end

    always @(posedge clk) begin
        bank0_dout <= bram_bank0[rd_addr];
        bank1_dout <= bram_bank1[rd_addr];
    end

    // ------------------------------------------------------------------------
    // DMA Control Registers & Multi-Face Address Calculation
    // ------------------------------------------------------------------------
    localparam S_IDLE       = 3'd0;
    localparam S_SCAN       = 3'd1;
    localparam S_AR_REQ     = 3'd2;
    localparam S_BURST_DATA = 3'd3;
    localparam S_WAIT_SWAP  = 3'd4;

    reg [2:0]   state;
    reg [1:0]   dma_slot_idx;       // 0: Center, 1: N0, 2: N1, 3: N2
    reg [7:0]   scan_tile_id;       // 0 ~ 210 (211 Macro-tiles)
    reg [1:0]   tile_burst_cnt;     // 0 ~ 3 (4 bursts of 16 beats = 64 words per tile)
    reg [C_M_AXI_ADDR_WIDTH-1:0] current_dma_addr;
    reg [15:0]  dma_wr_addr;

    assign m_axi_araddr = current_dma_addr;

    function [31:0] get_face_base_addr(input [4:0] fid);
        begin
            if (fid > 5'd0) begin
                get_face_base_addr = fb_base_addr + ((fid - 1'b1) * face_stride);
            end else begin
                get_face_base_addr = fb_base_addr;
            end
        end
    endfunction

    reg [4:0] cur_face_id;
    always @(*) begin
        case (dma_slot_idx)
            2'd0: cur_face_id = target_face_id;
            2'd1: cur_face_id = n0_face_id;
            2'd2: cur_face_id = n1_face_id;
            2'd3: cur_face_id = n2_face_id;
            default: cur_face_id = target_face_id;
        endcase
    end

    // 211-Bit Sparse Occupancy Mask Lookup for current scan_tile_id
    reg [31:0] cur_mask_word;
    always @(*) begin
        case (scan_tile_id[7:5])
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
    wire scan_tile_active = cur_mask_word[scan_tile_id[4:0]];

    // Diagnostic Probes
    assign debug_dma_state = {7'd0, dma_slot_idx, scan_tile_id[6:0], state, (state == S_WAIT_SWAP), display_bank, dma_wr_addr};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            debug_last_rdata <= 32'd0;
            debug_rvalid_cnt <= 32'd0;
        end else if (m_axi_rvalid && m_axi_rready) begin
            debug_last_rdata <= m_axi_rdata;
            debug_rvalid_cnt <= debug_rvalid_cnt + 1'b1;
        end
    end

    // ------------------------------------------------------------------------
    // DMA Write Process into Inactive (Fetch) Bank
    // ------------------------------------------------------------------------
    wire fetch_bank = ~display_bank;

    always @(posedge clk) begin
        if (m_axi_rvalid && m_axi_rready) begin
            if (fetch_bank == 1'b1) begin
                bram_bank1[dma_wr_addr] <= m_axi_rdata[23:0];
            end else begin
                bram_bank0[dma_wr_addr] <= m_axi_rdata[23:0];
            end
        end
    end

    // ------------------------------------------------------------------------
    // Sparse AXI Master DMA Fetch FSM: Scans 211-bit mask and fetches ONLY active tiles
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state             <= S_IDLE;
            m_axi_arvalid     <= 1'b0;
            dma_busy          <= 1'b0;
            display_bank      <= 1'b0;
            displayed_face_id <= 5'd0;
            target_face_id    <= 5'd0;
            dma_slot_idx      <= 2'd0;
            scan_tile_id      <= 8'd0;
            tile_burst_cnt    <= 2'd0;
            current_dma_addr  <= 32'd0;
            dma_wr_addr       <= 16'd0;
        end else if (!enable || force_fetch) begin
            state             <= S_IDLE;
            m_axi_arvalid     <= 1'b0;
            dma_busy          <= 1'b0;
            displayed_face_id <= 5'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    m_axi_arvalid <= 1'b0;
                    if (enable && (center_face_id != 5'd0) && ((displayed_face_id != center_face_id) || force_fetch)) begin
                        target_face_id <= center_face_id;
                        dma_slot_idx   <= 2'd0;
                        scan_tile_id   <= 8'd0;
                        dma_busy       <= 1'b1;
                        state          <= S_SCAN;
                    end else begin
                        dma_busy <= 1'b0;
                    end
                end

                S_SCAN: begin
                    if (scan_tile_id <= 8'd210) begin
                        if (!scan_tile_active) begin
                            // Fast 1-cycle skip for inactive tile!
                            scan_tile_id <= scan_tile_id + 1'b1;
                        end else begin
                            // Active tile: prepare 4x 16-beat bursts (64 words = 256 bytes)
                            // In DDR, tile data offset = scan_tile_id * 64 words * 4 bytes/word = scan_tile_id * 256
                            current_dma_addr <= get_face_base_addr(cur_face_id) + ({16'd0, scan_tile_id} * 18'd256);
                            dma_wr_addr      <= {dma_slot_idx, scan_tile_id, 6'd0};
                            tile_burst_cnt   <= 2'd0;
                            state            <= S_AR_REQ;
                        end
                    end else begin
                        // Completed 211 tiles for current slot
                        if (dma_slot_idx == 2'd3) begin
                            // All 4 slots (Center + N0 + N1 + N2) fetched!
                            state <= S_WAIT_SWAP;
                        end else begin
                            dma_slot_idx <= dma_slot_idx + 1'b1;
                            scan_tile_id <= 8'd0;
                            state        <= S_SCAN;
                        end
                    end
                end

                S_AR_REQ: begin
                    m_axi_arvalid <= 1'b1;
                    if (m_axi_arready && m_axi_arvalid) begin
                        m_axi_arvalid <= 1'b0;
                        state         <= S_BURST_DATA;
                    end
                end

                S_BURST_DATA: begin
                    if (m_axi_rvalid && m_axi_rready) begin
                        dma_wr_addr <= dma_wr_addr + 1'b1;

                        if (m_axi_rlast) begin
                            if (tile_burst_cnt == 2'd3) begin
                                // Finished all 4 bursts for this tile (64 words)
                                scan_tile_id <= scan_tile_id + 1'b1;
                                state        <= S_SCAN;
                            end else begin
                                tile_burst_cnt   <= tile_burst_cnt + 1'b1;
                                current_dma_addr <= current_dma_addr + (C_M_AXI_BURST_LEN * 4);
                                state            <= S_AR_REQ;
                            end
                        end
                    end
                end

                S_WAIT_SWAP: begin
                    dma_busy <= 1'b1;
                    if (sof_pulse) begin
                        display_bank      <= ~display_bank;
                        displayed_face_id <= target_face_id;
                        dma_busy          <= 1'b0;
                        state             <= S_IDLE;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Output Multiplexer (BRAM Sample vs Fallback Procedural HUD vs Deep Space)
    // ------------------------------------------------------------------------
    wire [23:0] active_bram_pixel = (display_bank == 1'b0) ? bank0_dout : bank1_dout;

    // Procedural HUD Test Pattern
    wire is_tile_edge = (pipe_addr[3:0] == 4'd0) || (pipe_addr[7:4] == 4'd0);
    wire [23:0] hud_test_rgb = is_tile_edge ? 24'h00FFCC : 24'h0A281E;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pixel_rgb   <= 24'h000000;
            pixel_valid <= 1'b0;
        end else begin
            pixel_valid <= pipe_valid;
            if (pipe_tile_active) begin
                if (pipe_test_en) begin
                    pixel_rgb <= hud_test_rgb;
                end else begin
                    pixel_rgb <= active_bram_pixel;
                end
            end else begin
                // Inactive Background Tile: Pure Black Space (Zero DDR access required)
                pixel_rgb <= 24'h000000;
            end
        end
    end

endmodule
