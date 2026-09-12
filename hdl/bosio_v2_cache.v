`timescale 1ns/1ps
// Immutable full snapshots plus frame-atomic BPT1 partial tile updates.
// Both BRAM banks are mirrored after every transaction so consecutive patches
// always start from the same scene. The display bank changes only at frame_start.
module bosio_v2_cache(
 input wire clk,rst_n,request,patch_request,input wire [31:0] base,word_count,
 input wire frame_start,output reg scene_valid,output reg pending,
 output wire busy,output reg error,output reg [31:0] received,
 output reg [31:0] araddr,output wire [7:0] arlen,output reg arvalid,input wire arready,
 input wire [31:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
 input wire sample_valid,input wire [12:0] directory_addr,input wire [9:0] cell_addr,
 output reg pixel_valid,output reg [23:0] pixel
);
 localparam DIR_START=256,DATA_START=4476,DATA_WORDS=49152,MAX_WORDS=53632;
 localparam IDLE=0,AR=1,DATA=2,WAIT_FRAME=3;
 (* ram_style="block" *)reg [31:0] data0[0:DATA_WORDS-1],data1[0:DATA_WORDS-1];
 (* ram_style="block" *)reg [31:0] dir0[0:4219],dir1[0:4219];
 (* ram_style="block" *)reg [23:0] pal0[0:255],pal1[0:255];
 reg bank,fetch_bank,queued_full,queued_patch,mode_patch,mirror_pass;
 reg [2:0] state;reg [31:0] count,total,transaction_base;reg [3:0] beat;
 reg patch_valid,patch_header;reg [9:0] patch_tile_words,patch_phase;
 reg [12:0] patch_records,patch_record;reg [15:0] patch_offset;
 assign busy=(state!=IDLE)||queued_full||queued_patch;
 assign arlen=15;assign rready=(state==DATA);wire take=rready&&rvalid;
 wire [31:0] full_data_index=count-DATA_START;
 wire full_data_take=take&&!mode_patch&&count>=DATA_START&&full_data_index<DATA_WORDS;
 wire patch_data_take=take&&mode_patch&&count>=16&&!patch_header&&patch_valid&&
                      ({16'b0,patch_offset}+patch_phase)<DATA_WORDS;
 wire data_take=full_data_take||patch_data_take;
 wire [31:0] data_write_index=mode_patch?({16'b0,patch_offset}+patch_phase):full_data_index;

 // Write port: full snapshots populate palette/directory/data. BPT1 packets
 // contain a 16-word file header and records of 16 header words + tile payload.
 always @(posedge clk)if(take)begin
  if(!mode_patch)begin
   if(count<256)begin if(fetch_bank)pal1[count[7:0]]<=rdata[23:0];else pal0[count[7:0]]<=rdata[23:0];end
   else if(count<DATA_START)begin if(fetch_bank)dir1[count-DIR_START]<=rdata;else dir0[count-DIR_START]<=rdata;end
  end
  if(data_take)begin if(fetch_bank)data1[data_write_index]<=rdata;else data0[data_write_index]<=rdata;end
 end

 task reset_patch_parser;
  begin patch_valid<=1;patch_header<=1;patch_tile_words<=0;patch_phase<=0;patch_records<=0;patch_record<=0;patch_offset<=0;end
 endtask

 always @(posedge clk)begin
  if(!rst_n)begin
   bank<=0;fetch_bank<=1;queued_full<=0;queued_patch<=0;mode_patch<=0;mirror_pass<=0;
   state<=IDLE;count<=0;total<=0;transaction_base<=0;beat<=0;arvalid<=0;araddr<=0;
   pending<=0;scene_valid<=0;error<=0;received<=0;reset_patch_parser;
  end else begin
   if(request)queued_full<=1;
   if(patch_request)queued_patch<=1;
   case(state)
    IDLE:begin
     if(queued_full||queued_patch)begin
      mode_patch<=!queued_full;queued_full<=0;if(!queued_full)queued_patch<=0;
      transaction_base<=base;total<=word_count;count<=0;beat<=0;mirror_pass<=0;fetch_bank<=~bank;
      araddr<=base;arvalid<=1;error<=0;reset_patch_parser;
      if(base[5:0]!=0||word_count[3:0]!=0||
         (queued_full&&(word_count<4480||word_count>MAX_WORDS))||
         (!queued_full&&(word_count<48||word_count>MAX_WORDS)))begin error<=1;arvalid<=0;state<=IDLE;end
      else state<=AR;
     end
    end
    AR:if(arvalid&&arready)begin arvalid<=0;state<=DATA;end
    DATA:if(take)begin
     received<=received+1;count<=count+1;beat<=beat+1;
     if(rresp!=0||(rlast!=(beat==15)))begin error<=1;patch_valid<=0;end
     if(mode_patch)begin
      if(count==0 && rdata!=32'h42505431)begin patch_valid<=0;error<=1;end
      if(count==1)begin patch_tile_words<=rdata[9:0];if(rdata!=16&&rdata!=64&&rdata!=256)begin patch_valid<=0;error<=1;end end
      if(count==2)patch_records<=rdata[12:0];
      if(count>=16)begin
       if(patch_header)begin
        if(patch_phase==0)patch_offset<=rdata[15:0];
        if(patch_phase==15)begin patch_header<=0;patch_phase<=0;end else patch_phase<=patch_phase+1;
       end else begin
        if(patch_offset+patch_phase>=DATA_WORDS)begin patch_valid<=0;error<=1;end
        if(patch_phase+1>=patch_tile_words)begin patch_record<=patch_record+1;patch_header<=1;patch_phase<=0;end else patch_phase<=patch_phase+1;
       end
      end
     end
     if(rlast)begin
      if(count+1>=total)begin
       if(mode_patch && !patch_valid)state<=IDLE;
       else if(mirror_pass)state<=IDLE;
       else begin pending<=1;state<=WAIT_FRAME;end
      end else begin araddr<=araddr+64;arvalid<=1;beat<=0;state<=AR;end
     end
    end
    WAIT_FRAME:if(frame_start&&pending)begin
     bank<=fetch_bank;pending<=0;scene_valid<=1;
     // Replay the same DDR transaction into the former display bank.
     fetch_bank<=bank;mirror_pass<=1;count<=0;beat<=0;araddr<=transaction_base;arvalid<=1;reset_patch_parser;state<=AR;
    end
    default:state<=IDLE;
   endcase
  end
 end

 reg [31:0] d0,d1;reg v0,b0,ok0;reg [9:0] c0;
 always @(posedge clk)begin
  if(directory_addr<4220)begin d0<=dir0[directory_addr];d1<=dir1[directory_addr];end
  v0<=sample_valid&&rst_n;b0<=bank;ok0<=directory_addr<4220;c0<=cell_addr;
 end
 wire [31:0] entry=b0?d1:d0;reg [31:0] addr;reg v1,b1,ok1;
 always @(posedge clk)begin addr<=entry+{22'b0,c0};v1<=v0&&rst_n;b1<=b0;ok1<=ok0&&(entry!=32'hffffffff);end
 reg [31:0] w0,w1;reg v2,b2,ok2;reg [1:0] byte2;
 always @(posedge clk)begin
  if(addr<196608)begin w0<=data0[addr[17:2]];w1<=data1[addr[17:2]];end
  v2<=v1&&rst_n;b2<=b1;byte2<=addr[1:0];ok2<=ok1&&(addr<196608);
 end
 reg [31:0] w0a,w1a;reg v2a,b2a,ok2a;reg [1:0] byte2a;
 always @(posedge clk)begin w0a<=w0;w1a<=w1;v2a<=v2&&rst_n;b2a<=b2;ok2a<=ok2;byte2a<=byte2;end
 wire [31:0] wordsel=b2a?w1a:w0a;reg [7:0] index;reg vi,bi,oki;
 always @(posedge clk)begin index<=wordsel>>(byte2a*8);vi<=v2a&&rst_n;bi<=b2a;oki<=ok2a;end
 reg [23:0] p0,p1;reg v3,b3,ok3;
 always @(posedge clk)begin
  p0<=pal0[index];p1<=pal1[index];v3<=vi&&rst_n;b3<=bi;ok3<=oki;
  pixel_valid<=v3&&rst_n;pixel<=ok3?(b3?p1:p0):24'b0;
 end
endmodule
