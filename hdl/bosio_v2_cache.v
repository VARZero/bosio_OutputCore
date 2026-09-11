`timescale 1ns/1ps
// Immutable scene snapshots: palette, global 20*211 directory, packed cells.
// Bank swap occurs only between raster frames after the pipeline drains.
// 196608 bytes per bank; arbitrary active faces, no center+neighbor limitation.
module bosio_v2_cache(
 input wire clk,rst_n,request,input wire [31:0] base,word_count,
 input wire frame_start,output reg scene_valid,output reg pending,
 output wire busy,output reg error,output reg [31:0] received,
 output reg [31:0] araddr,output wire [7:0] arlen,output reg arvalid,input wire arready,
 input wire [31:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
 input wire sample_valid,input wire [12:0] directory_addr,input wire [9:0] cell_addr,
 output reg pixel_valid,output reg [23:0] pixel
);
 localparam DIR_START=256,DATA_START=4476,DATA_WORDS=49152,MAX_WORDS=53632;
 (* ram_style="block" *)reg [31:0] data0[0:DATA_WORDS-1],data1[0:DATA_WORDS-1];
 (* ram_style="block" *)reg [31:0] dir0[0:4219],dir1[0:4219];
 (* ram_style="block" *)reg [23:0] pal0[0:255],pal1[0:255];
 reg bank,fetch_bank,queued;reg [2:0] state;reg [31:0] count,total;reg [3:0] beat;
 localparam IDLE=0,AR=1,DATA=2,WAIT=3;
 assign busy=(state!=IDLE)||queued;assign arlen=15;assign rready=(state==DATA);
 wire take=rready&&rvalid;
 wire [31:0] di=count-DATA_START;
 always @(posedge clk)if(take)begin
  if(count<256)begin if(fetch_bank)pal1[count[7:0]]<=rdata[23:0];else pal0[count[7:0]]<=rdata[23:0];end
  else if(count<DATA_START)begin if(fetch_bank)dir1[count-DIR_START]<=rdata;else dir0[count-DIR_START]<=rdata;end
  else if(di<DATA_WORDS)begin if(fetch_bank)data1[di]<=rdata;else data0[di]<=rdata;end
 end
 always @(posedge clk)begin
  if(!rst_n)begin bank<=0;fetch_bank<=1;queued<=0;state<=IDLE;count<=0;total<=0;beat<=0;arvalid<=0;araddr<=0;pending<=0;scene_valid<=0;error<=0;received<=0;end
  else begin
   if(request)queued<=1;
   if(frame_start&&pending)begin bank<=fetch_bank;pending<=0;scene_valid<=1;end
   case(state)
    IDLE:if(queued)begin
     queued<=0;
     if(base[5:0]!=0||word_count<4480||word_count>MAX_WORDS||word_count[3:0]!=0)error<=1;
     else begin count<=0;total<=word_count;araddr<=base;fetch_bank<=~bank;error<=0;arvalid<=1;beat<=0;state<=AR;end
    end
    AR:if(arvalid&&arready)begin arvalid<=0;state<=DATA;end
    DATA:if(take)begin
     received<=received+1'b1;count<=count+1'b1;beat<=beat+1'b1;
     if(rresp!=0||(rlast!=(beat==15)))error<=1;
     if(rlast)begin
      if(count+1>=total)begin pending<=1;state<=WAIT;end
      else begin araddr<=araddr+64;arvalid<=1;beat<=0;state<=AR;end
     end
    end
    WAIT:if(frame_start&&pending)state<=IDLE;
    default:state<=IDLE;
   endcase
  end
 end
 reg [31:0] d0,d1;reg v0,b0,ok0;reg [9:0] c0;
 always @(posedge clk)begin
  if(directory_addr<4220)begin d0<=dir0[directory_addr];d1<=dir1[directory_addr];end
  v0<=sample_valid&&rst_n;b0<=bank;ok0<=directory_addr<4220;c0<=cell_addr;
 end
 wire [31:0] entry=b0?d1:d0;
 reg [31:0] addr;reg v1,b1,ok1;
 always @(posedge clk)begin
  addr<=entry+{22'b0,c0};v1<=v0&&rst_n;b1<=b0;ok1<=ok0&&(entry!=32'hffffffff);
 end
 reg [31:0] w0,w1;reg v2,b2,ok2;reg [1:0] byte2;
 always @(posedge clk)begin
  if(addr<196608)begin w0<=data0[addr[17:2]];w1<=data1[addr[17:2]];end
  v2<=v1&&rst_n;b2<=b1;byte2<=addr[1:0];ok2<=ok1&&(addr<196608);
 end
 // Register BRAM output/mux and byte selection separately from palette lookup.
 reg [31:0] w0a,w1a;reg v2a,b2a,ok2a;reg [1:0] byte2a;
 always @(posedge clk)begin
  w0a<=w0;w1a<=w1;v2a<=v2&&rst_n;b2a<=b2;ok2a<=ok2;byte2a<=byte2;
 end
 wire [31:0] wordsel=b2a?w1a:w0a;
 reg [7:0] index;reg vi,bi,oki;
 always @(posedge clk)begin
  index<=wordsel>>(byte2a*8);vi<=v2a&&rst_n;bi<=b2a;oki<=ok2a;
 end
 reg [23:0] p0,p1;reg v3,b3,ok3;
 always @(posedge clk)begin
  p0<=pal0[index];p1<=pal1[index];v3<=vi&&rst_n;b3<=bi;ok3<=oki;
  pixel_valid<=v3&&rst_n;pixel<=ok3?(b3?p1:p0):24'b0;
 end
endmodule
