`timescale 1ns/1ps
// Read-only, in-order AXI cache. Hit throughput: one 32-bit request/clock.
// One outstanding fill; responses retain caller metadata across a miss.
module varzero_cache_line #(
 parameter LINE_BYTES=64, CACHE_BYTES=16384, WAYS=2,
 parameter AXI_DATA_WIDTH=32, USER_WIDTH=3
)(
 input wire clk,rst_n,invalidate,
 input wire req_valid,output wire req_ready,input wire [31:0] req_addr,
 input wire [USER_WIDTH-1:0] req_user,
 output reg rsp_valid,input wire rsp_ready,output wire [31:0] rsp_data,
 output reg [USER_WIDTH-1:0] rsp_user,output reg rsp_error,
 input wire prefetch_valid,output wire prefetch_ready,input wire [31:0] prefetch_addr,
 output wire idle,output reg [31:0] hits,misses,stall_cycles,
 output reg [31:0] araddr,output wire [7:0] arlen,output wire [2:0] arsize,
 output wire arvalid,input wire arready,
 input wire [AXI_DATA_WIDTH-1:0] rdata,input wire [1:0] rresp,
 input wire rlast,rvalid,output wire rready
);
 localparam BEAT_BYTES=AXI_DATA_WIDTH/8, BEATS=LINE_BYTES/BEAT_BYTES;
 localparam SETS=CACHE_BYTES/(LINE_BYTES*WAYS);
 localparam OFF=$clog2(LINE_BYTES), SB=$clog2(SETS), BB=$clog2(BEAT_BYTES);
 localparam WB=(WAYS>1)?$clog2(WAYS):1;
 localparam IDLE=0,AR=1,FILL=2,REPLAY=3,FAIL=4;
 reg [2:0] state;
 reg [31:0] tags[0:WAYS*SETS-1];reg [WAYS*SETS-1:0] valid_bits;
 reg [WB-1:0] replace_way[0:SETS-1];
 reg [31:0] held_addr;reg [USER_WIDTH-1:0] held_user;
 reg held_prefetch,fill_error;reg [WB-1:0] fill_way,rsp_way;
 reg [SB-1:0] fill_set;integer fill_beat;
 reg [BB-1:0] rsp_lane;
 wire space=!rsp_valid||rsp_ready;
 assign req_ready=state==IDLE&&space&&!invalidate;
 assign prefetch_ready=req_ready&&!req_valid;
 assign idle=state==IDLE&&!rsp_valid;
 wire demand=req_valid&&req_ready;
 wire speculative=prefetch_valid&&prefetch_ready;
 wire accept=demand||speculative;
 wire [31:0] address=speculative?prefetch_addr:req_addr;
 wire [SB-1:0] set_index=(address>>OFF);
 wire [31:0] tag=address>>(OFF+SB);
 reg hit;reg [WB-1:0] hit_way,victim;integer w;
 always @* begin
  hit=0;hit_way=0;victim=replace_way[set_index];
  for(w=WAYS-1;w>=0;w=w-1)begin
   if(!valid_bits[w*SETS+set_index])victim=w;
   if(valid_bits[w*SETS+set_index]&&tags[w*SETS+set_index]==tag)begin hit=1;hit_way=w;end
  end
 end
 wire replay=state==REPLAY&&space;
 wire [31:0] read_address=replay?held_addr:address;
 wire [SB-1:0] read_set=read_address>>OFF;
 localparam BEAT_INDEX_BITS=(BEATS>1)?$clog2(BEATS):1;
 wire [BEAT_INDEX_BITS-1:0] read_beat=(read_address>>BB)&(BEATS-1);
 wire [31:0] read_index=read_set*BEATS+read_beat;
 wire read_en=(demand&&hit)||replay;
 wire [AXI_DATA_WIDTH-1:0] read_words[0:WAYS-1];
 genvar g;
 generate for(g=0;g<WAYS;g=g+1)begin:data_way
  (* ram_style="block" *)reg [AXI_DATA_WIDTH-1:0] memory[0:SETS*BEATS-1];
  reg [AXI_DATA_WIDTH-1:0] read_word;
  always @(posedge clk)begin
   if(read_en)read_word<=memory[read_index];
   if(state==FILL&&rvalid&&rready&&fill_way==g)
    memory[fill_set*BEATS+fill_beat]<=rdata;
  end
  assign read_words[g]=read_word;
 end endgenerate
 assign rsp_data=rsp_error?32'b0:(read_words[rsp_way]>>(rsp_lane*8));
 assign arvalid=state==AR;
 assign arlen=BEATS-1;assign arsize=BB;
 assign rready=state==FILL;
 integer i;
 initial begin
  if(LINE_BYTES<16||LINE_BYTES>1024||(LINE_BYTES&(LINE_BYTES-1))!=0)
   $error("LINE_BYTES must be a power of two between 16 and 1024");
  if(SETS<2||(SETS&(SETS-1))!=0||CACHE_BYTES%(LINE_BYTES*WAYS)!=0)
   $error("Cache must contain a power-of-two number of sets >= 2");
  if(WAYS<1||WAYS>4||(WAYS&(WAYS-1))!=0)
   $error("WAYS must be 1, 2, or 4");
  if(AXI_DATA_WIDTH!=32&&AXI_DATA_WIDTH!=64&&AXI_DATA_WIDTH!=128)
   $error("AXI_DATA_WIDTH must be 32, 64, or 128");
 end
 always @(posedge clk)begin
  if(!rst_n)begin
   state<=IDLE;valid_bits<=0;rsp_valid<=0;rsp_user<=0;rsp_error<=0;
   rsp_way<=0;rsp_lane<=0;held_addr<=0;held_user<=0;held_prefetch<=0;
   fill_way<=0;fill_set<=0;fill_beat<=0;fill_error<=0;araddr<=0;
   hits<=0;misses<=0;stall_cycles<=0;
   for(i=0;i<SETS;i=i+1)replace_way[i]<=0;
  end else begin
   if(rsp_valid&&rsp_ready)rsp_valid<=0;
   if(req_valid&&!req_ready)stall_cycles<=stall_cycles+1;
   if(invalidate&&idle)valid_bits<=0;
   case(state)
    IDLE:if(accept)begin
     if(hit)begin
      if(demand)begin
       hits<=hits+1;rsp_valid<=1;rsp_way<=hit_way;rsp_lane<=address[BB-1:0];
       rsp_user<=req_user;rsp_error<=0;
      end
     end else begin
      if(demand)misses<=misses+1;
      held_addr<=address;held_user<=req_user;held_prefetch<=speculative;
      fill_way<=victim;fill_set<=set_index;fill_beat<=0;fill_error<=0;
      valid_bits[victim*SETS+set_index]<=0;
      araddr<=(address>>OFF)<<OFF;state<=AR;
     end
    end
    AR:if(arready)state<=FILL;
    FILL:if(rvalid)begin
     if(rresp!=0||(rlast!=(fill_beat==BEATS-1)))fill_error<=1;
     if(rlast)begin
      if(fill_error||rresp!=0||fill_beat!=BEATS-1)state<=held_prefetch?IDLE:FAIL;
      else begin
       tags[fill_way*SETS+fill_set]<=held_addr>>(OFF+SB);
       valid_bits[fill_way*SETS+fill_set]<=1;
       replace_way[fill_set]<=fill_way==WAYS-1?0:fill_way+1'b1;
       state<=held_prefetch?IDLE:REPLAY;
      end
     end else if(fill_beat<BEATS-1)fill_beat<=fill_beat+1;
    end
    REPLAY:if(space)begin
     rsp_valid<=1;rsp_error<=0;rsp_way<=fill_way;rsp_lane<=held_addr[BB-1:0];
     rsp_user<=held_user;state<=IDLE;
    end
    FAIL:if(space)begin rsp_valid<=1;rsp_error<=1;rsp_user<=held_user;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule
