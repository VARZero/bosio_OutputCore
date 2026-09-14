`timescale 1ns/1ps
// One-pixel-per-clock, edge-adaptive antialiasing for the projected RGB stream.
// Four arithmetic stages keep the 100 MHz AXI clock timing-safe.
module bosio_v2_edge_aa #(
 parameter SCREEN_WIDTH=1280
)(
 input wire clk,rst_n,
 input wire enable,
 input wire [7:0] threshold,
 input wire [7:0] strength,
 input wire iv,
 input wire [23:0] pixel_in,
 output reg ov,
 output reg [23:0] pixel_out
);
 // The Zynq-7020 design has little block RAM left. A 1280 x 24-bit line
 // therefore intentionally uses distributed RAM (120 RAM256X1S primitives).
 (* ram_style="distributed" *) reg [23:0] linebuf[0:SCREEN_WIDTH-1];
 reg [10:0] x;
 reg [9:0] y;
 reg [23:0] left_pixel;

 // Stage 0: capture the current, left and previous-row samples.
 reg v0,first_col0,first_row0;
 reg [23:0] cur0,left0,up0;
 wire [7:0] yc0={2'b0,cur0[23:18]}+{1'b0,cur0[15:9]}+{2'b0,cur0[7:2]};
 wire [7:0] yl0={2'b0,left0[23:18]}+{1'b0,left0[15:9]}+{2'b0,left0[7:2]};
 wire [7:0] yu0={2'b0,up0[23:18]}+{1'b0,up0[15:9]}+{2'b0,up0[7:2]};

 // Stage 1: register luma so edge comparison is a separate timing stage.
 reg v1,first_col1,first_row1,enable1;
 reg [7:0] yc1,yl1,yu1,threshold1,strength1;
 reg [23:0] center1,left1,up1;
 wire [7:0] dl1=yc1>=yl1?yc1-yl1:yl1-yc1;
 wire [7:0] du1=yc1>=yu1?yc1-yu1:yu1-yc1;
 wire edge_l1=!first_col1&&(dl1>threshold1);
 wire edge_u1=!first_row1&&(du1>threshold1);
 wire [23:0] neighbor1=(edge_l1&&(!edge_u1||dl1>=du1))?left1:up1;

 // Stage 2: register edge selection and configuration.
 reg v2,filter2;
 reg [23:0] center2,neighbor2;
 reg [7:0] strength2;

 // Stage 3: three signed multiplies.
 reg v3,filter3;
 reg [23:0] center3;
 reg signed [17:0] scaled_r3,scaled_g3,scaled_b3;

 // Stage 4 only adds, rounds and shifts the registered products.
 function [7:0] finish_blend;
  input [7:0] center;
  input signed [17:0] scaled;
  reg signed [18:0] sum;
  begin
   sum=$signed({1'b0,center,8'b0})+scaled+19'sd128;
   finish_blend=sum[15:8];
  end
 endfunction

 always @(posedge clk)begin
  if(!rst_n)begin
   x<=0;y<=0;left_pixel<=0;
   v0<=0;v1<=0;v2<=0;v3<=0;ov<=0;
   first_col0<=0;first_row0<=0;cur0<=0;left0<=0;up0<=0;
   first_col1<=0;first_row1<=0;enable1<=0;
   yc1<=0;yl1<=0;yu1<=0;threshold1<=0;strength1<=0;
   center1<=0;left1<=0;up1<=0;
   filter2<=0;center2<=0;neighbor2<=0;strength2<=0;
   filter3<=0;center3<=0;scaled_r3<=0;scaled_g3<=0;scaled_b3<=0;
   pixel_out<=0;
  end else begin
   v0<=iv;
   v1<=v0;
   v2<=v1;
   v3<=v2;
   ov<=v3;

   if(iv)begin
    cur0<=pixel_in;
    left0<=x==0?pixel_in:left_pixel;
    up0<=linebuf[x];
    first_col0<=x==0;
    first_row0<=y==0;
    linebuf[x]<=pixel_in;
    left_pixel<=pixel_in;
    if(x==SCREEN_WIDTH-1)begin
     x<=0;
     if(y==719)y<=0;else y<=y+1'b1;
    end else x<=x+1'b1;
   end

   if(v0)begin
    center1<=cur0;left1<=left0;up1<=up0;
    yc1<=yc0;yl1<=yl0;yu1<=yu0;
    first_col1<=first_col0;first_row1<=first_row0;
    enable1<=enable;threshold1<=threshold;strength1<=strength;
   end

   if(v1)begin
    center2<=center1;
    neighbor2<=neighbor1;
    filter2<=enable1&&(edge_l1||edge_u1)&&(strength1!=0);
    strength2<=strength1;
   end

   if(v2)begin
    center3<=center2;
    filter3<=filter2;
    scaled_r3<=($signed({1'b0,neighbor2[23:16]})-$signed({1'b0,center2[23:16]}))*$signed({1'b0,strength2});
    scaled_g3<=($signed({1'b0,neighbor2[15:8]})-$signed({1'b0,center2[15:8]}))*$signed({1'b0,strength2});
    scaled_b3<=($signed({1'b0,neighbor2[7:0]})-$signed({1'b0,center2[7:0]}))*$signed({1'b0,strength2});
   end

   if(v3)begin
    if(filter3)
     pixel_out<={finish_blend(center3[23:16],scaled_r3),
                  finish_blend(center3[15:8],scaled_g3),
                  finish_blend(center3[7:0],scaled_b3)};
    else pixel_out<=center3;
   end
  end
 end
endmodule
