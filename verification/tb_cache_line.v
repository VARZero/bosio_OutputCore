`timescale 1ns/1ps
module tb_cache_line;
 parameter LINE_BYTES=64,AXI_DATA_WIDTH=32,WAYS=2;
 reg clk=0,rst_n=0,invalidate=0,req_valid=0,rsp_ready=1,prefetch_valid=0;
 reg [31:0] req_addr=0,prefetch_addr=0;reg [7:0] req_user=0;
 wire req_ready,rsp_valid,rsp_error,prefetch_ready,idle;
 wire [31:0] rsp_data,hits,misses,stalls,araddr;wire [7:0] rsp_user,arlen;wire [2:0] arsize;
 wire arvalid,rready;reg arready=0,rvalid=0,rlast=0;reg [1:0] rresp=0;
 reg [AXI_DATA_WIDTH-1:0] rdata=0;
 integer beat,lane,got=0,sent=0,bursts=0,n;reg [31:0] burst_base;reg [31:0] expected[0:255];
 reg expected_error[0:255];reg inject_error=0;
 always #5 clk=~clk;
 varzero_cache_line #(.LINE_BYTES(LINE_BYTES),.CACHE_BYTES(1024),.WAYS(WAYS),
  .AXI_DATA_WIDTH(AXI_DATA_WIDTH),.USER_WIDTH(8)) dut(
  .clk(clk),.rst_n(rst_n),.invalidate(invalidate),.req_valid(req_valid),.req_ready(req_ready),
  .req_addr(req_addr),.req_user(req_user),.rsp_valid(rsp_valid),.rsp_ready(rsp_ready),
  .rsp_data(rsp_data),.rsp_user(rsp_user),.rsp_error(rsp_error),
  .prefetch_valid(prefetch_valid),.prefetch_ready(prefetch_ready),.prefetch_addr(prefetch_addr),
  .idle(idle),.hits(hits),.misses(misses),.stall_cycles(stalls),
  .araddr(araddr),.arlen(arlen),.arsize(arsize),.arvalid(arvalid),.arready(arready),
  .rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready));
 function [31:0] value;input [31:0] address;begin value=address^32'hac531927;end endfunction
 initial forever begin
  wait(arvalid);repeat(3)@(negedge clk);burst_base=araddr;
  if(araddr%LINE_BYTES!=0||arlen!=(LINE_BYTES/(AXI_DATA_WIDTH/8))-1)$fatal(1,"burst alignment");
  arready=1;@(negedge clk);arready=0;bursts=bursts+1;
  for(beat=0;beat<=arlen;beat=beat+1)begin
   repeat(beat%3)@(negedge clk);
   for(lane=0;lane<AXI_DATA_WIDTH/32;lane=lane+1)
    rdata[lane*32+:32]=value(burst_base+beat*(AXI_DATA_WIDTH/8)+lane*4);
   rvalid=1;rlast=beat==arlen;rresp=inject_error&&beat==0?2'b10:2'b00;
   @(posedge clk);while(!rready)@(posedge clk);
   @(negedge clk);rvalid=0;rlast=0;rresp=0;
  end
 end
 always @(posedge clk)if(rst_n&&rsp_valid&&rsp_ready)begin
  if(rsp_error!==expected_error[rsp_user]||rsp_data!==expected[rsp_user])$fatal(1,"response user=%d data=%h expected=%h error=%b",rsp_user,rsp_data,expected[rsp_user],rsp_error);
  got=got+1;
 end
 task read;input [31:0] addr;begin
  @(negedge clk);req_valid=1;req_addr=addr;req_user=sent;
  expected[sent]=inject_error?0:value(addr);expected_error[sent]=inject_error;sent=sent+1;
  @(posedge clk);while(!req_ready)@(posedge clk);
  @(negedge clk);req_valid=0;
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst_n=1;
  read(32'h10004);wait(idle);
  for(n=0;n<12;n=n+1)read(32'h10000+(n%4)*4);
  wait(idle);if(bursts!=1)$fatal(1,"hit path refilled");
  // Hold a response, then miss at a conflicting address: no response may disappear.
  rsp_ready=0;read(32'h10008);repeat(7)@(negedge clk);rsp_ready=1;
  for(n=0;n<12;n=n+1)read(32'h10000+n*1024);
  wait(idle);invalidate=1;@(negedge clk);invalidate=0;
  read(32'h10004);wait(idle);
  prefetch_addr=32'h20000;prefetch_valid=1;
  @(posedge clk);while(!prefetch_ready)@(posedge clk);
  @(negedge clk);prefetch_valid=0;wait(idle);
  read(32'h20004);wait(idle);
  inject_error=1;read(32'h30000);wait(idle);
  inject_error=0;read(32'h30000);wait(idle);
  if(got!=sent)$fatal(1,"lost response got=%d sent=%d",got,sent);
  $display("CACHE_LINE_PASS line=%d width=%d ways=%d hits=%d misses=%d bursts=%d",LINE_BYTES,AXI_DATA_WIDTH,WAYS,hits,misses,bursts);
  $finish;
 end
 initial begin #500000;$fatal(1,"timeout");end
endmodule
module tb_cache_line_128;
 tb_cache_line #(.LINE_BYTES(128),.AXI_DATA_WIDTH(64)) test();
endmodule
module tb_cache_line_256;
 tb_cache_line #(.LINE_BYTES(256),.AXI_DATA_WIDTH(128),.WAYS(1)) test();
endmodule
module tb_cache_line_min;
 tb_cache_line #(.LINE_BYTES(16),.AXI_DATA_WIDTH(128),.WAYS(4)) test();
endmodule
