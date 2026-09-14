`timescale 1ns/1ps
// AXI4-Stream video timing generated from the ordered pixel FIFO.
module bosio_out_stream_out(
 input wire clk,rst_n,enable,
 input wire [15:0] screen_w,screen_h,
 input wire [23:0] fifo_dout,
 input wire fifo_empty,
 output wire fifo_rd_en,
 output wire [23:0] m_axis_video_tdata,
 output wire m_axis_video_tvalid,
 input wire m_axis_video_tready,
 output wire m_axis_video_tuser,
 output wire m_axis_video_tlast,
 output reg [15:0] frame_counter
);
 reg [15:0] pix_x,pix_y;
 assign m_axis_video_tdata=fifo_dout;
 assign m_axis_video_tvalid=enable&&!fifo_empty;
 assign m_axis_video_tuser=m_axis_video_tvalid&&(pix_x==0)&&(pix_y==0);
 assign m_axis_video_tlast=m_axis_video_tvalid&&(pix_x+1'b1==screen_w);
 wire stream_fire=m_axis_video_tvalid&&m_axis_video_tready;
 assign fifo_rd_en=stream_fire;
 always @(posedge clk)begin
  if(!rst_n)begin pix_x<=0;pix_y<=0;frame_counter<=0;end
  else if(!enable)begin pix_x<=0;pix_y<=0;end
  else if(stream_fire)begin
   if(pix_x+1'b1==screen_w)begin
    pix_x<=0;
    if(pix_y+1'b1==screen_h)begin pix_y<=0;frame_counter<=frame_counter+1'b1;end
    else pix_y<=pix_y+1'b1;
   end else pix_x<=pix_x+1'b1;
  end
 end
endmodule
