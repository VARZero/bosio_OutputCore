`timescale 1ns/1ps
// Twenty projective face cones. Each of 60 barycentric numerators is an
// affine function of screen x/y, evaluated using add-only DDA at pixel rate.
module bosio_v2_projector(
 input wire clk,rst_n,enable,ready,
 input wire cfg_we,input wire [7:0] cfg_idx,input wire [31:0] cfg_data,
 input wire commit,output reg commit_ack,
 input wire scene_valid,scene_pending,output wire frame_start,
 input wire [1:0] resolution,
 output reg valid,output reg [4:0] face,
 output reg [31:0] n0,n2,den,
 output reg [1:0] cell_resolution
);
 reg signed [31:0] shadow[0:179];
 reg signed [31:0] starts[0:59],dx[0:59],dy[0:59],row[0:59],cur[0:59];
 reg running,initialized;reg [6:0] gap;reg [10:0] x;reg [9:0] y;
 reg [1:0] res;
 integer i;
 wire start=enable&&!running&&(gap==0)&&(scene_valid||scene_pending)&&(initialized||commit);
 assign frame_start=start;
 always @(posedge clk) if(cfg_we && cfg_idx<180)shadow[cfg_idx]<=cfg_data;
 wire step=running&&ready&&enable;
 always @(posedge clk)begin
  if(!rst_n)begin running<=0;initialized<=0;gap<=0;x<=0;y<=0;commit_ack<=0;res<=1;end
  else begin
   commit_ack<=0;
   if(gap!=0)gap<=gap-1'b1;
   if(!enable)begin running<=0;gap<=64;end
   if(start)begin
    running<=1;x<=0;y<=0;res<=resolution;
    if(commit)begin initialized<=1;commit_ack<=1;end
   end
   if(step)begin
    if(x==1279)begin x<=0;if(y==719)begin running<=0;gap<=64;y<=0;end else y<=y+1'b1;end
    else x<=x+1'b1;
   end
  end
 end
 genvar g;
 generate for(g=0;g<60;g=g+1)begin:dda
  always @(posedge clk)begin
   if(start)begin
    if(commit)begin
     starts[g]<=shadow[g*3];dx[g]<=shadow[g*3+1];dy[g]<=shadow[g*3+2];
     cur[g]<=shadow[g*3];row[g]<=shadow[g*3];
    end else begin cur[g]<=starts[g];row[g]<=starts[g];end
   end else if(step)begin
    if(x==1279)begin cur[g]<=row[g]+dy[g];row[g]<=row[g]+dy[g];end
    else cur[g]<=cur[g]+dx[g];
   end
  end
 end endgenerate
 // Two-stage selection: four independent groups, followed by one mux.
 reg [4:0] gf[0:3];reg [31:0] ga[0:3],gb[0:3],gc[0:3];
 reg svalid;reg [1:0] sr;integer h,j;reg found;
 always @(posedge clk)begin
  svalid<=step&&rst_n;sr<=res;
  for(h=0;h<4;h=h+1)begin
   gf[h]<=31;ga[h]<=0;gb[h]<=0;gc[h]<=0;found=0;
   for(j=0;j<5;j=j+1)begin
    if(!found&&!cur[(h*5+j)*3][31]&&!cur[(h*5+j)*3+1][31]&&!cur[(h*5+j)*3+2][31])begin
     found=1;gf[h]<=h*5+j;ga[h]<=cur[(h*5+j)*3];gb[h]<=cur[(h*5+j)*3+1];gc[h]<=cur[(h*5+j)*3+2];
    end
   end
  end
 end
 reg [31:0] a,b,c;reg [4:0] fid;
 always @*begin
  fid=31;a=0;b=0;c=1;
  if(gf[0]!=31)begin fid=gf[0];a=ga[0];b=gb[0];c=gc[0];end
  else if(gf[1]!=31)begin fid=gf[1];a=ga[1];b=gb[1];c=gc[1];end
  else if(gf[2]!=31)begin fid=gf[2];a=ga[2];b=gb[2];c=gc[2];end
  else if(gf[3]!=31)begin fid=gf[3];a=ga[3];b=gb[3];c=gc[3];end
 end
 reg v2;reg [4:0] f2;reg [31:0] a2,c2,ab2;reg [1:0] r2;
 always @(posedge clk)begin
  v2<=svalid&&rst_n;f2<=fid;a2<=a;c2<=c;ab2<=a+b;r2<=sr;
  valid<=v2&&rst_n;face<=f2;n0<=a2;n2<=c2;den<=ab2+c2;cell_resolution<=r2;
 end
endmodule

// 15-stage restoring fractional divider. Numerators are nonnegative and <= D.
module bosio_v2_normalize(
 input wire clk,rst_n,iv,input wire [4:0] iface,input wire [1:0] ires,
 input wire [31:0] a,c,d,
 output wire ov,output wire [4:0] oface,output wire [1:0] ores,
 output wire [15:0] l0,l2
);
 reg [31:0] ra[0:15],rc[0:15],dd[0:15];
 reg [14:0] qa[0:15],qc[0:15];reg ea[0:15],ec[0:15];
 reg vv[0:15];reg [4:0] ff[0:15];reg [1:0] rr[0:15];
 always @(posedge clk)begin
  ra[0]<=a>=d?0:a;rc[0]<=c>=d?0:c;dd[0]<=d==0?1:d;
  ea[0]<=a>=d;ec[0]<=c>=d;qa[0]<=0;qc[0]<=0;
  vv[0]<=iv&&rst_n;ff[0]<=iface;rr[0]<=ires;
 end
 genvar k;
 generate for(k=0;k<15;k=k+1)begin:divide
  wire [32:0] sa={ra[k],1'b0},sc={rc[k],1'b0};
  wire [32:0] da=sa-{1'b0,dd[k]},dc=sc-{1'b0,dd[k]};
  always @(posedge clk)begin
   ra[k+1]<=da[32]?sa[31:0]:da[31:0];rc[k+1]<=dc[32]?sc[31:0]:dc[31:0];
   qa[k+1]<={qa[k][13:0],!da[32]};qc[k+1]<={qc[k][13:0],!dc[32]};
   dd[k+1]<=dd[k];ea[k+1]<=ea[k];ec[k+1]<=ec[k];vv[k+1]<=vv[k]&&rst_n;ff[k+1]<=ff[k];rr[k+1]<=rr[k];
  end
 end endgenerate
 assign ov=vv[15];assign oface=ff[15];assign ores=rr[15];
 assign l0=ea[15]?16'd32768:{1'b0,qa[15]};assign l2=ec[15]?16'd32768:{1'b0,qc[15]};
endmodule

module bosio_v2_tile_address(
 input wire clk,rst_n,iv,input wire [4:0] iface,input wire [1:0] ires,
 input wire [15:0] l0,l2,
 output reg ov,output reg [12:0] directory_addr,output reg [9:0] cell_addr
);
 wire [15:0] l1=16'd32768-l0-l2;
 reg [1:0] region,r1,r2,r3,r4;reg v1,v2,v3,v4;
 reg [4:0] f1,f2,f3,f4;
 reg [15:0] mu0,mu1,mu2;
 reg [18:0] s0,s1,s2;
 reg [7:0] tid3,tid4;reg [14:0] frac0,frac1,frac2;
 reg [19:0] t0,t1,t2;
 reg [3:0] a,b,c,row,col;reg inv;reg [7:0] tid;
 always @(posedge clk)begin
  v1<=iv&&rst_n;f1<=iface;r1<=ires;
  if(l0>=16384)begin region<=0;mu0<=(l0<<1)-32768;mu1<=l1<<1;mu2<=l2<<1;end
  else if(l1>=16384)begin region<=1;mu0<=l0<<1;mu1<=(l1<<1)-32768;mu2<=l2<<1;end
  else if(l2>=16384)begin region<=2;mu0<=l0<<1;mu1<=l1<<1;mu2<=(l2<<1)-32768;end
  else begin region<=3;mu0<=32768-(l0<<1);mu1<=32768-(l1<<1);mu2<=32768-(l2<<1);end
 end
 reg [1:0] region2;
 // Saturate exact vertices just below N; use 19-bit expressions throughout.
 always @(posedge clk)begin
  v2<=v1&&rst_n;f2<=f1;r2<=r1;region2<=region;
  if(region==3)begin
   s0<=mu0==32768?19'd262143:{mu0,3'b0};s1<=mu1==32768?19'd262143:{mu1,3'b0};s2<=mu2==32768?19'd262143:{mu2,3'b0};
  end else begin
   s0<=mu0==32768?19'd229375:({mu0,3'b0}-{3'b0,mu0});
   s1<=mu1==32768?19'd229375:({mu1,3'b0}-{3'b0,mu1});
   s2<=mu2==32768?19'd229375:({mu2,3'b0}-{3'b0,mu2});
  end
 end
 always @(posedge clk)begin
  a=s0[18:15];b=s1[18:15];c=s2[18:15];
  row=(region2==3?7:6)-a;col=c>row?row:c;
  inv=({2'b0,a}+{2'b0,b}+{2'b0,c}==(region2==3?6:5));
  tid=(region2==3?147:region2*49)+row*row+2*col+inv;
  tid3<=tid;v3<=v2&&rst_n;f3<=f2;r3<=r2;
  frac0<=inv?15'd32767-s0[14:0]:s0[14:0];
  frac1<=inv?15'd32767-s1[14:0]:s1[14:0];
  frac2<=inv?15'd32767-s2[14:0]:s2[14:0];
 end
 always @(posedge clk)begin
  v4<=v3&&rst_n;f4<=f3;r4<=r3;tid4<=tid3;
  case(r3)
   0:begin t0<={2'b0,frac0,3'b0};t1<={2'b0,frac1,3'b0};t2<={2'b0,frac2,3'b0};end
   2:begin t0<={frac0,5'b0};t1<={frac1,5'b0};t2<={frac2,5'b0};end
   default:begin t0<={1'b0,frac0,4'b0};t1<={1'b0,frac1,4'b0};t2<={1'b0,frac2,4'b0};end
  endcase
 end
 reg [5:0] aa,bb,cc,rr,co,nn;reg ii;reg [10:0] ci;
 always @(posedge clk)begin
  nn=r4==0?8:r4==2?32:16;
  aa=t0[19:15];bb=t1[19:15];cc=t2[19:15];rr=nn-1-aa;co=cc>rr?rr:cc;
  ii=({1'b0,aa}+{1'b0,bb}+{1'b0,cc}==nn-2);
  ci=rr*rr+2*co+ii;
  ov<=v4&&rst_n;directory_addr<=f4==31?13'd8191:f4*13'd211+tid4;
  cell_addr<=ci[9:0];
 end
endmodule
