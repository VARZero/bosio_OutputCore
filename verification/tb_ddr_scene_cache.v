`timescale 1ns/1ps
module tb_ddr_scene_cache;
 reg clk=0,rst_n=0,request=0,patch_request=0,frame_start=0;
 reg [31:0] base=32'h10000000,word_count=4544;
 wire scene_valid,pending,busy,error;wire [31:0] received,araddr,hits,misses,stalls;
 wire [7:0] arlen;wire arvalid,rready;reg arready=0,rvalid=0,rlast=0;
 reg [31:0] rdata=0;reg [1:0] rresp=0;
 reg sample_valid=0,output_ready=1;reg [12:0] directory_addr=0;reg [9:0] cell_addr=0;
 wire sample_ready,frame_ready,pixel_valid;wire [23:0] pixel;
 reg [31:0] mem[0:16383];reg [31:0] burst_addr;
 integer b,beats,i,got=0,sent=0;reg [23:0] expected[0:4095];
 always #5 clk=~clk;
 bosio_ddr_scene_cache #(.CACHE_BYTES(1024)) dut(
 .clk(clk),.rst_n(rst_n),.request(request),.patch_request(patch_request),.base(base),.word_count(word_count),
 .frame_start(frame_start),.scene_valid(scene_valid),.pending(pending),.busy(busy),.error(error),.received(received),
 .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),
 .sample_valid(sample_valid),.directory_addr(directory_addr),.cell_addr(cell_addr),.sample_ready(sample_ready),.frame_ready(frame_ready),
 .output_ready(output_ready),.pixel_valid(pixel_valid),.pixel(pixel),.hits(hits),.misses(misses),.stall_cycles(stalls));
 initial forever begin
  wait(arvalid);@(negedge clk);burst_addr=araddr;beats=arlen+1;arready=1;
  @(negedge clk);arready=0;
  for(b=0;b<beats;b=b+1)begin
   repeat(b%2)@(negedge clk);
   rdata=mem[((burst_addr-32'h10000000)>>2)+b];rvalid=1;rlast=b==beats-1;
   @(posedge clk);while(!rready)@(posedge clk);
   @(negedge clk);rvalid=0;rlast=0;
  end
 end
 always @(posedge clk)if(pixel_valid)begin
  if(pixel!==expected[got])$fatal(1,"pixel %d got %h expected %h",got,pixel,expected[got]);
  got=got+1;
 end
 task commit;begin
  wait(pending&&frame_ready);@(negedge clk);frame_start=1;
  @(negedge clk);frame_start=0;wait(!busy);
 end endtask
 task sample;input [12:0] tile;input [9:0] sample_cell;input [23:0] color;begin
  @(negedge clk);while(!sample_ready)@(negedge clk);
  sample_valid=1;directory_addr=tile;cell_addr=sample_cell;expected[sent]=color;sent=sent+1;
 end endtask
 initial begin
  for(i=0;i<16384;i=i+1)mem[i]=0;
  for(i=256;i<4476;i=i+1)mem[i]=32'hffffffff;
  mem[1]=24'h123456;mem[2]=24'habcdef;mem[256]=0;
  for(i=4476;i<4544;i=i+1)mem[i]=32'h01010101;
  // Second immutable DDR snapshot shares metadata, but has new tile cells.
  for(i=0;i<4544;i=i+1)mem[8192+i]=mem[i];
  for(i=4476;i<4544;i=i+1)mem[8192+i]=32'h02020202;
  repeat(5)@(negedge clk);rst_n=1;request=1;@(negedge clk);request=0;commit;
  for(i=0;i<100;i=i+1)sample(0,i%256,24'h123456);
  sample(1,0,24'h000000);sample(8191,0,24'h000000);
  @(negedge clk);sample_valid=0;
  wait(frame_ready);if(error||got!=sent)$fatal(1,"first scene");
  // Backpressure and a pending scene swap must drain all old pixel requests.
  output_ready=0;
  for(i=0;i<12;i=i+1)sample(0,i,24'h123456);
  @(negedge clk);sample_valid=0;base=32'h10008000;patch_request=1;
  @(negedge clk);patch_request=0;repeat(20)@(negedge clk);
  if(frame_ready)$fatal(1,"frame boundary accepted undrained pixels");
  output_ready=1;commit;
  for(i=0;i<100;i=i+1)sample(0,i%256,24'habcdef);
  @(negedge clk);sample_valid=0;wait(frame_ready);
  if(error||got!=sent||received!=4480)$fatal(1,"scene commit/cache invalidation");
  $display("DDR_SCENE_CACHE_PASS pixels=%d hits=%d misses=%d",got,hits,misses);$finish;
 end
 initial begin #1000000;$fatal(1,"timeout");end
endmodule
