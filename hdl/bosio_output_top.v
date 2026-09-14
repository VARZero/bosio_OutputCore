`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_output_top
// Project: Bosio 3DoF Display Output Core (Hierarchical 7-7-7/8 Architecture)
// Target: Xilinx Zynq-7020 (PYNQ-Z2) / Vivado 2020.2
// Description:
//   Top-level IP core for 3DoF Icosahedral/Spherical Display Output:
//   - AXI4-Lite Slave for register control, status, and 211-bit sparse masks.
//   - AXI4-Stream Slave for direct 96-bit mrad Sensor Hub orientation.
//   - AXI4 Full Master for DDR 20-Face Framebuffer Texture prefetching.
//   - AXI4-Stream Video Master (RGB888) for HDMI / Display output.
//   - Pipelined 7-7-7/8 Barycentric AGU with sub-triangle micro-tile quantization.
//   - Dual-bank BRAM cache with SOF-synchronized ping-pong switching.
// ============================================================================

module bosio_output_top #(
    parameter C_S_AXI_LITE_DATA_WIDTH = 32,
    parameter C_S_AXI_LITE_ADDR_WIDTH = 7,
    parameter C_M_AXI_ADDR_WIDTH      = 32,
    parameter C_M_AXI_DATA_WIDTH      = 32,
    parameter SENSOR_TDATA_WIDTH      = 96,
    parameter VIDEO_TDATA_WIDTH       = 24
)(
    // Clocks and Resets
    input  wire                                     aclk,
    input  wire                                     aresetn,

    // ------------------------------------------------------------------------
    // 1. AXI4-Lite Slave Interface (Control & Status)
    // ------------------------------------------------------------------------
    input  wire [C_S_AXI_LITE_ADDR_WIDTH-1:0]       s_axi_lite_awaddr,
    input  wire [2:0]                               s_axi_lite_awprot,
    input  wire                                     s_axi_lite_awvalid,
    output wire                                     s_axi_lite_awready,

    input  wire [C_S_AXI_LITE_DATA_WIDTH-1:0]       s_axi_lite_wdata,
    input  wire [(C_S_AXI_LITE_DATA_WIDTH/8)-1:0]   s_axi_lite_wstrb,
    input  wire                                     s_axi_lite_wvalid,
    output wire                                     s_axi_lite_wready,

    output wire [1:0]                               s_axi_lite_bresp,
    output wire                                     s_axi_lite_bvalid,
    input  wire                                     s_axi_lite_bready,

    input  wire [C_S_AXI_LITE_ADDR_WIDTH-1:0]       s_axi_lite_araddr,
    input  wire [2:0]                               s_axi_lite_arprot,
    input  wire                                     s_axi_lite_arvalid,
    output wire                                     s_axi_lite_arready,

    output wire [C_S_AXI_LITE_DATA_WIDTH-1:0]       s_axi_lite_rdata,
    output wire [1:0]                               s_axi_lite_rresp,
    output wire                                     s_axi_lite_rvalid,
    input  wire                                     s_axi_lite_rready,

    // ------------------------------------------------------------------------
    // 2. AXI4-Stream Slave Interface (Sensor Hub IMU in Milliradians)
    // ------------------------------------------------------------------------
    input  wire [SENSOR_TDATA_WIDTH-1:0]            s_axis_sensor_tdata,
    input  wire                                     s_axis_sensor_tvalid,
    output wire                                     s_axis_sensor_tready,

    // ------------------------------------------------------------------------
    // 3. AXI4 Full Master Interface (DDR Framebuffer Texture Fetch)
    // ------------------------------------------------------------------------
    output wire [C_M_AXI_ADDR_WIDTH-1:0]            m_axi_araddr,
    output wire [7:0]                               m_axi_arlen,
    output wire [2:0]                               m_axi_arsize,
    output wire [1:0]                               m_axi_arburst,
    output wire [1:0]                               m_axi_arlock,
    output wire [3:0]                               m_axi_arcache,
    output wire [2:0]                               m_axi_arprot,
    output wire [3:0]                               m_axi_arqos,
    output wire                                     m_axi_arvalid,
    input  wire                                     m_axi_arready,

    input  wire [C_M_AXI_DATA_WIDTH-1:0]            m_axi_rdata,
    input  wire [1:0]                               m_axi_rresp,
    input  wire                                     m_axi_rlast,
    input  wire                                     m_axi_rvalid,
    output wire                                     m_axi_rready,

    // ------------------------------------------------------------------------
    // 4. AXI4-Stream Video Master Interface (HDMI Output)
    // ------------------------------------------------------------------------
    output wire [VIDEO_TDATA_WIDTH-1:0]             m_axis_video_tdata,
    output wire                                     m_axis_video_tvalid,
    input  wire                                     m_axis_video_tready,
    output wire                                     m_axis_video_tuser,   // SOF
    output wire                                     m_axis_video_tlast    // EOL
);

 // Version 2 register ABI. All scene/camera mutations are staged.
 reg enabled,pose_pending,scene_request,patch_request,sensor_mode,aa_enable;reg [1:0] resolution;
 reg [7:0] aa_threshold,aa_strength;
 reg [2:0] sensor_invert;
 reg [31:0] scene_base,scene_words;reg [7:0] cfg_index;
 reg awgot,wgot,bvalid,rvalid;reg [6:0] awaddr;reg [31:0] wdata,rdata;reg [3:0] wstrb;
 wire write_fire=awgot&&wgot&&!bvalid;
 wire sw_cfg_we=write_fire&&(awaddr==7'h64)&&(&wstrb);
 wire cfg_ack,frame_start,scene_valid,scene_pending,cache_busy,cache_error;
 wire [31:0] received;
 wire [15:0] frame_counter;
 wire [31:0] sensor_yaw,sensor_pitch,sensor_roll,sensor_packets;wire sensor_active;
 wire signed [31:0] sensor_yaw_adjusted = sensor_invert[0] ? -$signed(sensor_yaw) : $signed(sensor_yaw);
 wire signed [31:0] sensor_pitch_adjusted = sensor_invert[1] ? -$signed(sensor_pitch) : $signed(sensor_pitch);
 wire signed [31:0] sensor_roll_adjusted = sensor_invert[2] ? -$signed(sensor_roll) : $signed(sensor_roll);
 wire sensor_pose_we,sensor_pose_commit,sensor_pose_busy;
 wire [7:0] sensor_pose_idx;wire signed [31:0] sensor_pose_data;
 wire [31:0] sensor_applied_count;
 wire selected_cfg_we=sensor_mode?sensor_pose_we:sw_cfg_we;
 wire [7:0] selected_cfg_idx=sensor_mode?sensor_pose_idx:cfg_index;
 wire [31:0] selected_cfg_data=sensor_mode?sensor_pose_data:wdata;
 wire selected_commit=sensor_mode?sensor_pose_commit:pose_pending;
 assign s_axi_lite_awready=!awgot&&!bvalid;
 assign s_axi_lite_wready=!wgot&&!bvalid;
 assign s_axi_lite_bvalid=bvalid;assign s_axi_lite_bresp=0;
 assign s_axi_lite_arready=!rvalid;
 assign s_axi_lite_rvalid=rvalid;assign s_axi_lite_rdata=rdata;assign s_axi_lite_rresp=0;
 always @(posedge aclk)begin
  if(!aresetn)begin
   enabled<=0;resolution<=1;scene_base<=0;scene_words<=0;cfg_index<=0;sensor_mode<=0;sensor_invert<=0;
   aa_enable<=1;aa_threshold<=8'd24;aa_strength<=8'd64;
   awgot<=0;wgot<=0;bvalid<=0;rvalid<=0;rdata<=0;awaddr<=0;wdata<=0;wstrb<=0;
   pose_pending<=0;scene_request<=0;patch_request<=0;
  end else begin
   scene_request<=0;patch_request<=0;
   if(cfg_ack&&!sensor_mode)pose_pending<=0;
   if(s_axi_lite_awready&&s_axi_lite_awvalid)begin awgot<=1;awaddr<=s_axi_lite_awaddr;end
   if(s_axi_lite_wready&&s_axi_lite_wvalid)begin wgot<=1;wdata<=s_axi_lite_wdata;wstrb<=s_axi_lite_wstrb;end
   if(bvalid&&s_axi_lite_bready)bvalid<=0;
   if(write_fire)begin
    awgot<=0;wgot<=0;bvalid<=1;
    // Driver uses full 32-bit writes. Other strobes do not mutate the ABI.
    if(&wstrb)case(awaddr)
     7'h00:enabled<=wdata[0];
     7'h08:scene_base<=wdata;
     7'h0c:scene_words<=wdata;
     7'h20:sensor_invert<=wdata[2:0];
     7'h1c:begin aa_enable<=wdata[0];aa_threshold<=wdata[15:8];aa_strength<=wdata[23:16];end
     7'h5c:resolution<=wdata[1:0];
     7'h60:cfg_index<=wdata[7:0];
     7'h64:if(cfg_index<179)cfg_index<=cfg_index+1'b1;
     7'h68:if(wdata[0]&&!sensor_mode)pose_pending<=1;
     7'h6c:begin if(wdata[0])scene_request<=1;if(wdata[1])patch_request<=1;end
     7'h78:begin sensor_mode<=wdata[0];if(wdata[0])pose_pending<=0;end
     default:;
    endcase
   end
   if(rvalid&&s_axi_lite_rready)rvalid<=0;
   if(s_axi_lite_arvalid&&s_axi_lite_arready)begin
    rvalid<=1;
    case(s_axi_lite_araddr)
     7'h00:rdata<={31'b0,enabled};
     7'h04:rdata<={frame_counter,10'b0,cache_error,scene_pending,(pose_pending||(sensor_mode&&sensor_pose_busy)),cache_busy,scene_valid,enabled};
     7'h08:rdata<=scene_base;
     7'h0c:rdata<=scene_words;
     7'h20:rdata<={29'b0,sensor_invert};
     7'h1c:rdata<={8'b0,aa_strength,aa_threshold,7'b0,aa_enable};
     7'h24:rdata<=sensor_yaw;7'h28:rdata<=sensor_pitch;7'h2c:rdata<=sensor_roll;7'h30:rdata<=sensor_packets;
     7'h5c:rdata<={30'b0,resolution};
     7'h60:rdata<={24'b0,cfg_index};
     7'h68:rdata<={31'b0,pose_pending};
     7'h70:rdata<=received;
     7'h74:rdata<=196608;
     7'h78:rdata<={sensor_applied_count[15:0],13'b0,sensor_pose_busy,sensor_active,sensor_mode};
     7'h7c:rdata<=32'h42533234;
     default:rdata<=0;
    endcase
   end
  end
 end
 bosio_out_sensor_rx u_sensor(
  .clk(aclk),.rst_n(aresetn),.s_axis_sensor_tdata(s_axis_sensor_tdata),.s_axis_sensor_tvalid(s_axis_sensor_tvalid),.s_axis_sensor_tready(s_axis_sensor_tready),
  .out_yaw_mrad(sensor_yaw),.out_pitch_mrad(sensor_pitch),.out_roll_mrad(sensor_roll),.sensor_stream_active(sensor_active),.sensor_pkt_count(sensor_packets));
 bosio_v2_sensor_pose u_sensor_pose(
  .clk(aclk),.rst_n(aresetn),.enable(sensor_mode),.sensor_active(sensor_active),.packet_count(sensor_packets),
  .yaw_mrad(sensor_yaw_adjusted),.pitch_mrad(sensor_pitch_adjusted),.roll_mrad(sensor_roll_adjusted),.commit_ack(cfg_ack),
  .cfg_we(sensor_pose_we),.cfg_idx(sensor_pose_idx),.cfg_data(sensor_pose_data),.commit(sensor_pose_commit),
  .busy(sensor_pose_busy),.applied_count(sensor_applied_count));
 wire [10:0] fifo_count;wire full,empty,rd;wire [23:0] dout;
 wire v0,v1,v2,pv,aa_valid;wire [4:0] f0,f1;wire [31:0] n0,n2,den;wire [15:0] l0,l2;wire [1:0] res0,res1;wire [12:0] directory_addr;wire [9:0] cell_addr;wire [23:0] pixel,aa_pixel;
 bosio_v2_projector u_projector(.clk(aclk),.rst_n(aresetn),.enable(enabled),.ready(fifo_count<900),
  .cfg_we(selected_cfg_we),.cfg_idx(selected_cfg_idx),.cfg_data(selected_cfg_data),.commit(selected_commit),.commit_ack(cfg_ack),
  .scene_valid(scene_valid),.scene_pending(scene_pending),.frame_start(frame_start),.resolution(resolution),
  .valid(v0),.face(f0),.n0(n0),.n2(n2),.den(den),.cell_resolution(res0));
 bosio_v2_normalize u_normalize(.clk(aclk),.rst_n(aresetn),.iv(v0),.iface(f0),.ires(res0),.a(n0),.c(n2),.d(den),.ov(v1),.oface(f1),.ores(res1),.l0(l0),.l2(l2));
 bosio_v2_tile_address u_address(.clk(aclk),.rst_n(aresetn),.iv(v1),.iface(f1),.ires(res1),.l0(l0),.l2(l2),.ov(v2),.directory_addr(directory_addr),.cell_addr(cell_addr));
 bosio_v2_cache u_cache(.clk(aclk),.rst_n(aresetn),.request(scene_request),.patch_request(patch_request),.base(scene_base),.word_count(scene_words),
  .frame_start(frame_start),.scene_valid(scene_valid),.pending(scene_pending),.busy(cache_busy),.error(cache_error),.received(received),
  .araddr(m_axi_araddr),.arlen(m_axi_arlen),.arvalid(m_axi_arvalid),.arready(m_axi_arready),.rdata(m_axi_rdata),.rresp(m_axi_rresp),.rlast(m_axi_rlast),.rvalid(m_axi_rvalid),.rready(m_axi_rready),
  .sample_valid(v2),.directory_addr(directory_addr),.cell_addr(cell_addr),.pixel_valid(pv),.pixel(pixel));
 assign m_axi_arsize=2;assign m_axi_arburst=1;assign m_axi_arlock=0;assign m_axi_arcache=3;assign m_axi_arprot=0;assign m_axi_arqos=0;
 bosio_v2_edge_aa u_edge_aa(.clk(aclk),.rst_n(aresetn),.enable(aa_enable),.threshold(aa_threshold),.strength(aa_strength),
  .iv(pv),.pixel_in(pixel),.ov(aa_valid),.pixel_out(aa_pixel));
 bosio_out_fifo u_fifo(.clk(aclk),.rst_n(aresetn),.flush(!enabled),.wr_en(aa_valid),.din(aa_pixel),.full(full),.count(fifo_count),.rd_en(rd),.dout(dout),.empty(empty));
 bosio_out_stream_out u_video(.clk(aclk),.rst_n(aresetn),.enable(enabled),.screen_w(16'd1280),.screen_h(16'd720),.fifo_dout(dout),.fifo_empty(empty),.fifo_rd_en(rd),
  .m_axis_video_tdata(m_axis_video_tdata),.m_axis_video_tvalid(m_axis_video_tvalid),.m_axis_video_tready(m_axis_video_tready),.m_axis_video_tuser(m_axis_video_tuser),.m_axis_video_tlast(m_axis_video_tlast),.frame_counter(frame_counter));
endmodule
