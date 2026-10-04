`timescale 1ns/1ps
// BS25: DDR owns the scene. Only palette/directory and read-cache lines use BRAM.
// Requests are queued before the variable-latency cache; pixels remain in order.
module bosio_ddr_scene_cache #(
 parameter LINE_BYTES=64,CACHE_BYTES=16384,WAYS=2,QUEUE_BITS=10
)(
 input wire clk,rst_n,request,patch_request,input wire [31:0] base,word_count,
 input wire frame_start,output reg scene_valid,output wire pending,busy,
 output reg error,output reg [31:0] received,
 output wire [31:0] araddr,output wire [7:0] arlen,output wire arvalid,input wire arready,
 input wire [31:0] rdata,input wire [1:0] rresp,input wire rlast,rvalid,output wire rready,
 input wire sample_valid,input wire [12:0] directory_addr,input wire [9:0] cell_addr,
 output wire sample_ready,frame_ready,input wire output_ready,
 output reg pixel_valid,output reg [23:0] pixel,
 output wire [31:0] hits,misses,stall_cycles,active_scene_base
);
 localparam META_WORDS=4480,DATA_START=4476,DEPTH=1<<QUEUE_BITS;
 localparam LOAD_IDLE=0,LOAD_AR=1,LOAD_DATA=2,LOAD_WAIT=3;
 (* ram_style="block" *)reg [31:0] dir0[0:4219];
 (* ram_style="block" *)reg [31:0] dir1[0:4219];
 (* ram_style="block" *)reg [23:0] pal0[0:255];
 (* ram_style="block" *)reg [23:0] pal1[0:255];
 reg bank,load_bank;reg [1:0] load_state;reg [31:0] active_base,active_words;
 reg [31:0] staged_base,staged_words,load_count;reg [3:0] load_beat;
 reg [31:0] active_data_base,active_data_bytes;
 assign active_scene_base=active_base;
 reg queued,queued_patch;
 assign pending=load_state==LOAD_WAIT;
 assign busy=load_state!=LOAD_IDLE||queued;
 // Projection has ~50 cycles in flight after ready drops; leave 256 entries.
 reg [QUEUE_BITS:0] count;reg [QUEUE_BITS-1:0] wr,rd;
 (* ram_style="distributed" *)reg [22:0] queue[0:DEPTH-1];
 wire [22:0] head=queue[rd];
 reg s1_valid,s2_valid,s3_valid;reg [32:0] s2_offset;reg s2_ok;
 reg [31:0] s3_addr;reg [2:0] s3_user;reg [9:0] s1_cell;
 reg [31:0] dir0_read,dir1_read;
 wire [31:0] s1_entry=bank?dir1_read:dir0_read;
 reg [23:0] pal0_read,pal1_read;reg pal_valid,pal_ok,pal_bank;
 reg s1_ok;
 wire cache_req_ready,cache_idle,cache_rsp_valid,cache_rsp_error;
 wire [31:0] cache_rsp_data;wire [2:0] cache_rsp_user;
 wire s3_ready=!s3_valid||cache_req_ready;
 wire s2_ready=!s2_valid||s3_ready;
 wire s1_ready=!s1_valid||s2_ready;
 wire pop=count!=0&&s1_ready;
 assign sample_ready=count<DEPTH-256;
 assign frame_ready=count==0&&!s1_valid&&!s2_valid&&!s3_valid&&cache_idle&&!pal_valid&&!pixel_valid;
 wire commit_scene=frame_start&&pending&&frame_ready;
 wire valid_cell=s2_ok&&!s2_offset[32]&&s2_offset[31:0]<active_data_bytes;
 always @(posedge clk)begin
  if(!rst_n)begin count<=0;wr<=0;rd<=0;s1_valid<=0;s2_valid<=0;s3_valid<=0;pixel_valid<=0;pixel<=0;pal_valid<=0;pal_ok<=0;pal_bank<=0;end
  else begin
   if(sample_valid&&count<DEPTH)begin queue[wr]<={directory_addr,cell_addr};wr<=wr+1'b1;end
   if(pop)rd<=rd+1'b1;
   case({sample_valid&&count<DEPTH,pop})
    2'b10:count<=count+1'b1;2'b01:count<=count-1'b1;default:;
   endcase
   if(s1_ready)begin
    s1_valid<=pop;s1_ok<=head[22:10]<4220;s1_cell<=head[9:0];
   end
   if(s2_ready)begin
    s2_valid<=s1_valid;s2_offset<={1'b0,s1_entry}+s1_cell;
    s2_ok<=s1_ok&&s1_entry!=32'hffffffff;
   end
   if(s3_ready)begin
    s3_valid<=s2_valid;s3_user<={valid_cell,s2_offset[1:0]};
    // Bounds check and physical addition are parallel, not one long carry chain.
    s3_addr<=active_data_base+{s2_offset[31:2],2'b0};
   end
   pal_valid<=cache_rsp_valid&&output_ready;
   pal_ok<=cache_rsp_user[2]&&!cache_rsp_error;pal_bank<=bank;
   pixel_valid<=pal_valid;pixel<=pal_ok?(pal_bank?pal1_read:pal0_read):24'b0;
  end
 end
 // Give each RAM its own synchronous output register; mux after the RAMs.
 always @(posedge clk)begin
  if(pop&&head[22:10]<4220)begin dir0_read<=dir0[head[22:10]];dir1_read<=dir1[head[22:10]];end
  if(cache_rsp_valid&&output_ready)begin
   pal0_read<=pal0[(cache_rsp_data>>(cache_rsp_user[1:0]*8))&255];
   pal1_read<=pal1[(cache_rsp_data>>(cache_rsp_user[1:0]*8))&255];
  end
 end
 wire [31:0] c_araddr;wire [7:0] c_arlen;wire c_arvalid,c_arready,c_rready;
 // Keep ownership through RLAST; never interleave metadata and cache bursts.
 reg bus_active,bus_cache;
 wire select_cache=bus_active?bus_cache:c_arvalid;
 wire loader_ar=load_state==LOAD_AR;
 assign araddr=select_cache?c_araddr:staged_base+load_count*4;
 assign arlen=select_cache?c_arlen:8'd15;
 assign arvalid=!bus_active&&(c_arvalid||loader_ar);
 assign c_arready=arready&&!bus_active&&select_cache;
 assign rready=bus_active&&(bus_cache?c_rready:load_state==LOAD_DATA);
 wire loader_take=bus_active&&!bus_cache&&rvalid&&rready;
 varzero_cache_line #(.LINE_BYTES(LINE_BYTES),.CACHE_BYTES(CACHE_BYTES),.WAYS(WAYS)) cache(
  .clk(clk),.rst_n(rst_n),.invalidate(commit_scene),
  .req_valid(s3_valid),.req_ready(cache_req_ready),
  .req_addr(s3_user[2]?s3_addr:active_base),.req_user(s3_user),
  .rsp_valid(cache_rsp_valid),.rsp_ready(output_ready),.rsp_data(cache_rsp_data),
  .rsp_user(cache_rsp_user),.rsp_error(cache_rsp_error),
  .prefetch_valid(1'b0),.prefetch_ready(),.prefetch_addr(32'b0),
  .idle(cache_idle),.hits(hits),.misses(misses),.stall_cycles(stall_cycles),
  .araddr(c_araddr),.arlen(c_arlen),.arsize(),.arvalid(c_arvalid),.arready(c_arready),
  .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid&&bus_active&&bus_cache),.rready(c_rready));
 always @(posedge clk)begin
  if(!rst_n)begin bus_active<=0;bus_cache<=0;end
  else begin
   if(arvalid&&arready)begin bus_active<=1;bus_cache<=select_cache;end
   if(rvalid&&rready&&rlast)bus_active<=0;
  end
 end
 // Metadata RAM has independent read/write ports and is never reset wholesale.
 always @(posedge clk)if(loader_take)begin
  if(load_count<256)begin
   if(load_bank)pal1[load_count[7:0]]<=rdata[23:0];else pal0[load_count[7:0]]<=rdata[23:0];
  end else if(load_count<DATA_START)begin
   if(load_bank)dir1[load_count-256]<=rdata;else dir0[load_count-256]<=rdata;
  end
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   bank<=0;load_bank<=1;load_state<=LOAD_IDLE;scene_valid<=0;error<=0;received<=0;
   active_base<=0;active_words<=4480;staged_base<=0;staged_words<=0;
   active_data_base<=0;active_data_bytes<=0;
   load_count<=0;load_beat<=0;queued<=0;queued_patch<=0;
  end else begin
   if(sample_valid&&count==DEPTH)error<=1;
   if(cache_rsp_valid&&cache_rsp_error)error<=1;
   if(request||patch_request)begin queued<=1;queued_patch<=patch_request&&!request;end
   case(load_state)
    LOAD_IDLE:if(queued)begin
     queued<=0;error<=0;staged_base<=base;staged_words<=word_count;load_count<=0;load_beat<=0;
     load_bank<=queued_patch?bank:~bank;
     if(base[5:0]!=0||word_count<4480||word_count>1084816||word_count[3:0]!=0||
        (queued_patch&&(!scene_valid||word_count!=active_words)))error<=1;
     else load_state<=queued_patch?LOAD_WAIT:LOAD_AR;
    end
    LOAD_AR:if(arvalid&&arready&&!select_cache)load_state<=LOAD_DATA;
    LOAD_DATA:if(loader_take)begin
     load_count<=load_count+1;load_beat<=load_beat+1'b1;received<=received+1;
     if(rresp!=0||rlast!=(load_beat==15))error<=1;
     if(rlast)begin
      if(load_count==META_WORDS-1)begin
       if(error||rresp!=0||load_beat!=15)load_state<=LOAD_IDLE;
       else load_state<=LOAD_WAIT;
      end else load_state<=LOAD_AR;
     end
    end
    LOAD_WAIT:if(commit_scene)begin
     active_base<=staged_base;active_words<=staged_words;bank<=load_bank;
     active_data_base<=staged_base+DATA_START*4;
     active_data_bytes<=(staged_words-DATA_START)*4;
     scene_valid<=1;load_state<=LOAD_IDLE;
    end
   endcase
  end
 end
endmodule
