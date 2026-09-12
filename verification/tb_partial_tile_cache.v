`timescale 1ns/1ps
module tb_partial_tile_cache;
 reg clk=0,rst_n=0,request=0,patch_request=0,frame_start=0;
 reg [31:0] base=32'h10000000,word_count=0;
 wire scene_valid,pending,busy,error;wire [31:0] received,araddr;wire [7:0] arlen;wire arvalid;reg arready=0;
 reg [31:0] rdata=0;reg [1:0] rresp=0;reg rlast=0,rvalid=0;wire rready;
 reg sample_valid=0;reg [12:0] directory_addr=0;reg [9:0] cell_addr=0;wire pixel_valid;wire [23:0] pixel;
 reg [31:0] mem[0:53999];integer i,idx;
 always #5 clk=~clk;
 bosio_v2_cache dut(.clk(clk),.rst_n(rst_n),.request(request),.patch_request(patch_request),.base(base),.word_count(word_count),
  .frame_start(frame_start),.scene_valid(scene_valid),.pending(pending),.busy(busy),.error(error),.received(received),
  .araddr(araddr),.arlen(arlen),.arvalid(arvalid),.arready(arready),.rdata(rdata),.rresp(rresp),.rlast(rlast),.rvalid(rvalid),.rready(rready),
  .sample_valid(sample_valid),.directory_addr(directory_addr),.cell_addr(cell_addr),.pixel_valid(pixel_valid),.pixel(pixel));
 initial begin
  forever begin
   wait(arvalid);@(negedge clk);arready=1;@(negedge clk);arready=0;idx=(araddr-base)>>2;
   for(i=0;i<16;i=i+1)begin rvalid=1;rdata=mem[idx+i];rlast=(i==15);@(negedge clk);end
   rvalid=0;rlast=0;
  end
 end
 task pulse_frame;begin wait(pending);@(negedge clk);frame_start=1;@(negedge clk);frame_start=0;end endtask
 initial begin
  for(i=0;i<54000;i=i+1)mem[i]=32'h0;
  mem[0]=24'h000000;mem[1]=24'h112233;mem[256]=0;mem[4476]=32'h01020304;mem[4477]=32'h11121314;
  repeat(5)@(negedge clk);rst_n=1;
  word_count=4480;request=1;@(negedge clk);request=0;pulse_frame;wait(!busy);
  if(error||!scene_valid||dut.data0[1]!==32'h11121314||dut.data1[1]!==32'h11121314)begin $display("FULL_FAIL %h %h",dut.data0[1],dut.data1[1]);$fatal;end
  for(i=0;i<54000;i=i+1)mem[i]=0;mem[0]=32'h42505431;mem[1]=64;mem[2]=1;mem[3]=96;mem[16]=1;
  for(i=0;i<64;i=i+1)mem[32+i]=32'hA5000000+i;
  word_count=96;@(negedge clk);patch_request=1;@(negedge clk);patch_request=0;pulse_frame;wait(!busy);repeat(2)@(negedge clk);
  if(error||dut.data0[1]!==32'hA5000000||dut.data1[1]!==32'hA5000000||received!=9152)begin $display("PATCH_FAIL error=%b data0=%h data1=%h received=%0d",error,dut.data0[1],dut.data1[1],received);$fatal;end
  $display("PARTIAL_TILE_CACHE_PASS received=%0d",received);$finish;
 end
 initial begin #5000000;$display("TIMEOUT state=%0d count=%0d total=%0d busy=%b pending=%b arvalid=%b rvalid=%b received=%0d",dut.state,dut.count,dut.total,busy,pending,arvalid,rvalid,received);$fatal;end
endmodule
