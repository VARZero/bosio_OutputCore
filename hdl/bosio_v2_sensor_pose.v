`timescale 1ns/1ps
// Sensor mrad pose to the 180 affine Q24 coefficients consumed by the v2
// icosahedral projector. The engine is intentionally sequential: sensor
// updates are slow relative to the 100 MHz pixel pipeline, while atomic
// completion matters more than single-cycle latency.
module bosio_v2_sensor_pose(
    input  wire clk,
    input  wire rst_n,
    input  wire enable,
    input  wire sensor_active,
    input  wire [31:0] packet_count,
    input  wire signed [31:0] yaw_mrad,
    input  wire signed [31:0] pitch_mrad,
    input  wire signed [31:0] roll_mrad,
    input  wire commit_ack,
    output reg cfg_we,
    output reg [7:0] cfg_idx,
    output reg signed [31:0] cfg_data,
    output reg commit,
    output wire busy,
    output reg [31:0] applied_count
);
    localparam signed [31:0] SX_Q30 = 32'sd968633;  // 2*tan(30 deg)/1280
    localparam signed [31:0] SY_Q30 = 32'sd1235437; // 2*tan(22.5 deg)/720

    // Every multiplier and wide addition is separated by a register boundary.
    // This keeps the infrequent pose update path comfortably below the same
    // 10 ns period used by the pixel pipeline.
    localparam IDLE=5'd0, WAIT_TRIG=5'd1,
               BASIS_MUL1=5'd2, BASIS_ROUND1=5'd3,
               BASIS_MUL2=5'd4, BASIS_ROUND2=5'd5,
               DOT_MUL=5'd6, DOT_ADD1=5'd7, DOT_ADD2=5'd8,
               SCALE=5'd9, PRODUCTS=5'd10, ROUND_PRODUCTS=5'd11,
               START_COEFF=5'd12, OUT_START=5'd13, OUT_DX=5'd14,
               OUT_DY=5'd15, COMMIT_DELAY=5'd16, WAIT_COMMIT=5'd17;
    reg [4:0] state;
    assign busy = state != IDLE;

    reg signed [31:0] captured_yaw, captured_pitch, captured_roll;
    reg [31:0] accepted_count;
    reg cordic_start;
    wire yaw_done, pitch_done, roll_done;
    wire signed [31:0] syaw, cyaw, spitch, cpitch, sroll, croll;

    bosio_v2_cordic_sincos u_yaw(
        .clk(clk),.rst_n(rst_n),.start(cordic_start),.angle_mrad(captured_yaw),
        .done(yaw_done),.sin_q30(syaw),.cos_q30(cyaw));
    bosio_v2_cordic_sincos u_pitch(
        .clk(clk),.rst_n(rst_n),.start(cordic_start),.angle_mrad(captured_pitch),
        .done(pitch_done),.sin_q30(spitch),.cos_q30(cpitch));
    bosio_v2_cordic_sincos u_roll(
        .clk(clk),.rst_n(rst_n),.start(cordic_start),.angle_mrad(captured_roll),
        .done(roll_done),.sin_q30(sroll),.cos_q30(croll));

    function signed [31:0] round_q30;
        input signed [63:0] value;
        reg signed [63:0] magnitude;
        begin
            if (value >= 0)
                round_q30 = (value + 64'sd536870912) >>> 30;
            else begin
                magnitude = -value;
                round_q30 = -((magnitude + 64'sd536870912) >>> 30);
            end
        end
    endfunction
    function signed [63:0] round20;
        input signed [63:0] value;
        begin round20 = value >= 0 ? (value + 64'sd524288) >>> 20
                                  : -(((-value) + 64'sd524288) >>> 20); end
    endfunction
    function signed [63:0] round26;
        input signed [63:0] value;
        begin round26 = value >= 0 ? (value + 64'sd33554432) >>> 26
                                  : -(((-value) + 64'sd33554432) >>> 26); end
    endfunction
    function signed [63:0] round36;
        input signed [63:0] value;
        begin round36 = value >= 0 ? (value + 64'sd34359738368) >>> 36
                                  : -(((-value) + 64'sd34359738368) >>> 36); end
    endfunction
    function signed [63:0] round37;
        input signed [63:0] value;
        begin round37 = value >= 0 ? (value + 64'sd68719476736) >>> 37
                                  : -(((-value) + 64'sd68719476736) >>> 37); end
    endfunction

    // Q20 rows of the canonical 20 face inverse matrices. Distributed ROM is
    // used because both BRAM banks are reserved for immutable scene snapshots.
    (* rom_style="distributed" *) reg signed [21:0] inv_x[0:59];
    (* rom_style="distributed" *) reg signed [21:0] inv_y[0:59];
    (* rom_style="distributed" *) reg signed [21:0] inv_z[0:59];
    initial begin
        inv_x[0] = 22'sd200260; inv_y[0] = 22'sd1048576; inv_z[0] = -22'sd616338;
        inv_x[1] = -22'sd848316; inv_y[1] = -22'sd648056; inv_z[1] = -22'sd616338;
        inv_x[2] = 22'sd1048576; inv_y[2] = -22'sd648056; inv_z[2] = 22'sd0;
        inv_x[3] = -22'sd200260; inv_y[3] = -22'sd1048576; inv_z[3] = 22'sd616338;
        inv_x[4] = -22'sd724548; inv_y[4] = 22'sd0; inv_z[4] = -22'sd997255;
        inv_x[5] = 22'sd1172344; inv_y[5] = 22'sd0; inv_z[5] = -22'sd380918;
        inv_x[6] = 22'sd648056; inv_y[6] = 22'sd1048576; inv_z[6] = 22'sd0;
        inv_x[7] = 22'sd324028; inv_y[7] = -22'sd648056; inv_z[7] = -22'sd997255;
        inv_x[8] = 22'sd324028; inv_y[8] = -22'sd648056; inv_z[8] = 22'sd997255;
        inv_x[9] = -22'sd648056; inv_y[9] = -22'sd1048576; inv_z[9] = 22'sd0;
        inv_x[10] = 22'sd724548; inv_y[10] = 22'sd0; inv_z[10] = -22'sd997255;
        inv_x[11] = 22'sd724548; inv_y[11] = 22'sd0; inv_z[11] = 22'sd997255;
        inv_x[12] = 22'sd200260; inv_y[12] = 22'sd1048576; inv_z[12] = 22'sd616338;
        inv_x[13] = 22'sd1048576; inv_y[13] = -22'sd648056; inv_z[13] = 22'sd0;
        inv_x[14] = -22'sd848316; inv_y[14] = -22'sd648056; inv_z[14] = 22'sd616338;
        inv_x[15] = -22'sd200260; inv_y[15] = -22'sd1048576; inv_z[15] = -22'sd616338;
        inv_x[16] = 22'sd1172344; inv_y[16] = 22'sd0; inv_z[16] = 22'sd380918;
        inv_x[17] = -22'sd724548; inv_y[17] = 22'sd0; inv_z[17] = 22'sd997255;
        inv_x[18] = -22'sd524288; inv_y[18] = 22'sd1048576; inv_z[18] = 22'sd380918;
        inv_x[19] = 22'sd324028; inv_y[19] = -22'sd648056; inv_z[19] = 22'sd997255;
        inv_x[20] = -22'sd848316; inv_y[20] = -22'sd648056; inv_z[20] = -22'sd616338;
        inv_x[21] = 22'sd524288; inv_y[21] = -22'sd1048576; inv_z[21] = -22'sd380918;
        inv_x[22] = 22'sd0; inv_y[22] = 22'sd0; inv_z[22] = 22'sd1232675;
        inv_x[23] = -22'sd1172344; inv_y[23] = 22'sd0; inv_z[23] = -22'sd380918;
        inv_x[24] = -22'sd524288; inv_y[24] = 22'sd1048576; inv_z[24] = -22'sd380918;
        inv_x[25] = -22'sd848316; inv_y[25] = -22'sd648056; inv_z[25] = 22'sd616338;
        inv_x[26] = 22'sd324028; inv_y[26] = -22'sd648056; inv_z[26] = -22'sd997255;
        inv_x[27] = 22'sd524288; inv_y[27] = -22'sd1048576; inv_z[27] = 22'sd380918;
        inv_x[28] = -22'sd1172344; inv_y[28] = 22'sd0; inv_z[28] = 22'sd380918;
        inv_x[29] = 22'sd0; inv_y[29] = 22'sd0; inv_z[29] = -22'sd1232675;
        inv_x[30] = 22'sd524288; inv_y[30] = -22'sd1048576; inv_z[30] = -22'sd380918;
        inv_x[31] = -22'sd324028; inv_y[31] = 22'sd648056; inv_z[31] = -22'sd997255;
        inv_x[32] = 22'sd848316; inv_y[32] = 22'sd648056; inv_z[32] = 22'sd616338;
        inv_x[33] = 22'sd200260; inv_y[33] = 22'sd1048576; inv_z[33] = 22'sd616338;
        inv_x[34] = -22'sd1172344; inv_y[34] = 22'sd0; inv_z[34] = -22'sd380918;
        inv_x[35] = 22'sd724548; inv_y[35] = 22'sd0; inv_z[35] = -22'sd997255;
        inv_x[36] = 22'sd524288; inv_y[36] = -22'sd1048576; inv_z[36] = 22'sd380918;
        inv_x[37] = 22'sd848316; inv_y[37] = 22'sd648056; inv_z[37] = -22'sd616338;
        inv_x[38] = -22'sd324028; inv_y[38] = 22'sd648056; inv_z[38] = 22'sd997255;
        inv_x[39] = -22'sd524288; inv_y[39] = 22'sd1048576; inv_z[39] = 22'sd380918;
        inv_x[40] = 22'sd0; inv_y[40] = 22'sd0; inv_z[40] = -22'sd1232675;
        inv_x[41] = 22'sd1172344; inv_y[41] = 22'sd0; inv_z[41] = 22'sd380918;
        inv_x[42] = -22'sd200260; inv_y[42] = -22'sd1048576; inv_z[42] = 22'sd616338;
        inv_x[43] = 22'sd848316; inv_y[43] = 22'sd648056; inv_z[43] = 22'sd616338;
        inv_x[44] = -22'sd1048576; inv_y[44] = 22'sd648056; inv_z[44] = 22'sd0;
        inv_x[45] = -22'sd524288; inv_y[45] = 22'sd1048576; inv_z[45] = -22'sd380918;
        inv_x[46] = 22'sd1172344; inv_y[46] = 22'sd0; inv_z[46] = -22'sd380918;
        inv_x[47] = 22'sd0; inv_y[47] = 22'sd0; inv_z[47] = 22'sd1232675;
        inv_x[48] = -22'sd648056; inv_y[48] = -22'sd1048576; inv_z[48] = 22'sd0;
        inv_x[49] = -22'sd324028; inv_y[49] = 22'sd648056; inv_z[49] = 22'sd997255;
        inv_x[50] = -22'sd324028; inv_y[50] = 22'sd648056; inv_z[50] = -22'sd997255;
        inv_x[51] = 22'sd200260; inv_y[51] = 22'sd1048576; inv_z[51] = -22'sd616338;
        inv_x[52] = 22'sd724548; inv_y[52] = 22'sd0; inv_z[52] = 22'sd997255;
        inv_x[53] = -22'sd1172344; inv_y[53] = 22'sd0; inv_z[53] = 22'sd380918;
        inv_x[54] = -22'sd200260; inv_y[54] = -22'sd1048576; inv_z[54] = -22'sd616338;
        inv_x[55] = -22'sd1048576; inv_y[55] = 22'sd648056; inv_z[55] = 22'sd0;
        inv_x[56] = 22'sd848316; inv_y[56] = 22'sd648056; inv_z[56] = -22'sd616338;
        inv_x[57] = 22'sd648056; inv_y[57] = 22'sd1048576; inv_z[57] = 22'sd0;
        inv_x[58] = -22'sd724548; inv_y[58] = 22'sd0; inv_z[58] = 22'sd997255;
        inv_x[59] = -22'sd724548; inv_y[59] = 22'sd0; inv_z[59] = -22'sd997255;
    end

    reg signed [31:0] ty_s,ty_c,tp_s,tp_c,tr_s,tr_c;
    reg signed [31:0] cx,cy,cz;
    reg signed [31:0] a_sr_sy,b_sr_cy,c_cr_sy,d_cr_cy;
    reg signed [32:0] rx,ry,rz,ux,uy,uz;
    reg signed [63:0] p_cx,p_cz,p_a,p_b,p_c,p_d,p_ry,p_uy;
    reg signed [63:0] p_ar,p_bp,p_cp,p_dp;
    reg [5:0] row_index;
    reg signed [63:0] mul_cx,mul_cy,mul_cz;
    reg signed [63:0] mul_rx,mul_ry,mul_rz;
    reg signed [63:0] mul_ux,mul_uy,mul_uz;
    reg signed [63:0] add_c,add_r,add_u;
    reg signed [63:0] dot_c,dot_r,dot_u;
    reg signed [31:0] dc_q24,dr_q30,du_q30;
    reg signed [63:0] prod_x,prod_y;
    reg signed [63:0] center_prod_x,center_prod_y;
    reg signed [31:0] center_corr_x,center_corr_y;
    reg signed [31:0] start_q24,dx_q24,dy_q24;


    always @(posedge clk) begin
        if (!rst_n) begin
            state<=IDLE; cfg_we<=0; cfg_idx<=0; cfg_data<=0; commit<=0;
            applied_count<=0; accepted_count<=0; cordic_start<=0; row_index<=0;
            captured_yaw<=0; captured_pitch<=0; captured_roll<=0;
        end else begin
            cfg_we<=0;
            cordic_start<=0;
            if (!enable) begin
                state<=IDLE;
                commit<=0;
                accepted_count<=packet_count;
            end else case (state)
                IDLE: begin
                    commit<=0;
                    if (sensor_active && packet_count!=accepted_count) begin
                        captured_yaw<=yaw_mrad;
                        captured_pitch<=pitch_mrad;
                        captured_roll<=roll_mrad;
                        accepted_count<=packet_count;
                        cordic_start<=1;
                        state<=WAIT_TRIG;
                    end
                end
                WAIT_TRIG: if (yaw_done && pitch_done && roll_done) begin
                    ty_s<=syaw; ty_c<=cyaw; tp_s<=spitch; tp_c<=cpitch;
                    tr_s<=sroll; tr_c<=croll;
                    state<=BASIS_MUL1;
                end
                BASIS_MUL1: begin
                    p_cx<=$signed(ty_s)*$signed(tp_c);
                    p_cz<=$signed(ty_c)*$signed(tp_c);
                    p_a<=$signed(tr_s)*$signed(ty_s);
                    p_b<=$signed(tr_s)*$signed(ty_c);
                    p_c<=$signed(tr_c)*$signed(ty_s);
                    p_d<=$signed(tr_c)*$signed(ty_c);
                    p_ry<=$signed(tr_s)*$signed(tp_c);
                    p_uy<=$signed(tr_c)*$signed(tp_c);
                    state<=BASIS_ROUND1;
                end
                BASIS_ROUND1: begin
                    cx<=round_q30(p_cx);
                    cy<=tp_s;
                    cz<=-round_q30(p_cz);
                    a_sr_sy<=round_q30(p_a);
                    b_sr_cy<=round_q30(p_b);
                    c_cr_sy<=round_q30(p_c);
                    d_cr_cy<=round_q30(p_d);
                    ry<=round_q30(p_ry);
                    uy<=round_q30(p_uy);
                    state<=BASIS_MUL2;
                end
                BASIS_MUL2: begin
                    p_ar<=$signed(a_sr_sy)*$signed(tp_s);
                    p_bp<=$signed(b_sr_cy)*$signed(tp_s);
                    p_cp<=$signed(c_cr_sy)*$signed(tp_s);
                    p_dp<=$signed(d_cr_cy)*$signed(tp_s);
                    state<=BASIS_ROUND2;
                end
                BASIS_ROUND2: begin
                    rx<=$signed(d_cr_cy)-$signed(round_q30(p_ar));
                    rz<=$signed(c_cr_sy)+$signed(round_q30(p_bp));
                    ux<=-$signed(round_q30(p_cp))-$signed(b_sr_cy);
                    uz<=$signed(round_q30(p_dp))-$signed(a_sr_sy);
                    row_index<=0;
                    state<=DOT_MUL;
                end
                DOT_MUL: begin
                    mul_cx<=$signed(inv_x[row_index])*$signed(cx);
                    mul_cy<=$signed(inv_y[row_index])*$signed(cy);
                    mul_cz<=$signed(inv_z[row_index])*$signed(cz);
                    mul_rx<=$signed(inv_x[row_index])*$signed(rx);
                    mul_ry<=$signed(inv_y[row_index])*$signed(ry);
                    mul_rz<=$signed(inv_z[row_index])*$signed(rz);
                    mul_ux<=$signed(inv_x[row_index])*$signed(ux);
                    mul_uy<=$signed(inv_y[row_index])*$signed(uy);
                    mul_uz<=$signed(inv_z[row_index])*$signed(uz);
                    state<=DOT_ADD1;
                end
                DOT_ADD1: begin
                    add_c<=mul_cx+mul_cy;
                    add_r<=mul_rx+mul_ry;
                    add_u<=mul_ux+mul_uy;
                    state<=DOT_ADD2;
                end
                DOT_ADD2: begin
                    dot_c<=add_c+mul_cz;
                    dot_r<=add_r+mul_rz;
                    dot_u<=add_u+mul_uz;
                    state<=SCALE;
                end
                SCALE: begin
                    dc_q24<=round26(dot_c);
                    dr_q30<=round20(dot_r);
                    du_q30<=round20(dot_u);
                    state<=PRODUCTS;
                end
                PRODUCTS: begin
                    prod_x<=$signed(dr_q30)*$signed(SX_Q30);
                    prod_y<=$signed(du_q30)*$signed(SY_Q30);
                    center_prod_x<=$signed(dr_q30)*32'sd1238881607;
                    center_prod_y<=$signed(du_q30)*32'sd888279203;
                    state<=ROUND_PRODUCTS;
                end
                ROUND_PRODUCTS: begin
                    dx_q24<=round36(prod_x);
                    dy_q24<=round36(-prod_y);
                    center_corr_x<=round37(center_prod_x);
                    center_corr_y<=round37(center_prod_y);
                    state<=START_COEFF;
                end
                START_COEFF: begin
                    start_q24<=$signed(dc_q24)-$signed(center_corr_x)+
                                               $signed(center_corr_y);
                    state<=OUT_START;
                end
                OUT_START: begin
                    cfg_we<=1; cfg_idx<=row_index*3; cfg_data<=start_q24;
                    state<=OUT_DX;
                end
                OUT_DX: begin
                    cfg_we<=1; cfg_idx<=row_index*3+1'b1; cfg_data<=dx_q24;
                    state<=OUT_DY;
                end
                OUT_DY: begin
                    cfg_we<=1; cfg_idx<=row_index*3+2'd2; cfg_data<=dy_q24;
                    if (row_index==59) state<=COMMIT_DELAY;
                    else begin row_index<=row_index+1'b1; state<=DOT_MUL; end
                end
                COMMIT_DELAY: begin commit<=1; state<=WAIT_COMMIT; end
                WAIT_COMMIT: if (commit_ack) begin
                    commit<=0;
                    applied_count<=accepted_count;
                    state<=IDLE;
                end
                default: state<=IDLE;
            endcase
        end
    end
endmodule

// Twenty-iteration circular CORDIC. The angle is signed integer milliradians;
// internal phase uses Q16 milliradians and outputs Q2.30 sine/cosine.
module bosio_v2_cordic_sincos(
    input wire clk,input wire rst_n,input wire start,
    input wire signed [31:0] angle_mrad,
    output reg done,output reg signed [31:0] sin_q30,cos_q30
);
    localparam signed [47:0] PI_Q16=48'sd205887416;
    localparam signed [47:0] TWO_PI_Q16=48'sd411774832;
    localparam signed [47:0] HALF_PI_Q16=48'sd102943708;
    reg running,cos_negative;
    reg [4:0] iteration;
    reg signed [33:0] x,y;
    reg signed [47:0] z;
    reg signed [47:0] normalized,reduced;
    reg reduce_negative;
    function signed [31:0] atan_q16;
        input [4:0] index;
        begin case(index)
            0:atan_q16=51471854; 1:atan_q16=30385610; 2:atan_q16=16054922;
            3:atan_q16=8149729; 4:atan_q16=4090679; 5:atan_q16=2047334;
            6:atan_q16=1023917; 7:atan_q16=511990; 8:atan_q16=255999;
            9:atan_q16=128000; 10:atan_q16=64000; 11:atan_q16=32000;
            12:atan_q16=16000; 13:atan_q16=8000; 14:atan_q16=4000;
            15:atan_q16=2000; 16:atan_q16=1000; 17:atan_q16=500;
            18:atan_q16=250; default:atan_q16=125;
        endcase end
    endfunction

    wire signed [33:0] x_shift=y>>>iteration;
    wire signed [33:0] y_shift=x>>>iteration;
    wire signed [33:0] next_x=z>=0 ? x-x_shift : x+x_shift;
    wire signed [33:0] next_y=z>=0 ? y+y_shift : y-y_shift;
    wire signed [31:0] atan_word=atan_q16(iteration);
    wire signed [47:0] atan_value={{16{atan_word[31]}},atan_word};
    wire signed [47:0] next_z=z>=0 ? z-atan_value : z+atan_value;

    always @* begin
        normalized=$signed(angle_mrad)<<<16;
        if (normalized>PI_Q16) normalized=normalized-TWO_PI_Q16;
        else if (normalized<-PI_Q16) normalized=normalized+TWO_PI_Q16;
        reduced=normalized;
        reduce_negative=0;
        if (normalized>HALF_PI_Q16) begin
            reduced=PI_Q16-normalized; reduce_negative=1;
        end else if (normalized<-HALF_PI_Q16) begin
            reduced=-PI_Q16-normalized; reduce_negative=1;
        end
    end

    always @(posedge clk) begin
        if(!rst_n)begin
            running<=0;done<=0;iteration<=0;x<=0;y<=0;z<=0;
            sin_q30<=0;cos_q30<=32'sd1073741824;cos_negative<=0;
        end else begin
            done<=0;
            if(start&&!running)begin
                running<=1;iteration<=0;x<=34'sd652032874;y<=0;z<=reduced;
                cos_negative<=reduce_negative;
            end else if(running)begin
                x<=next_x;y<=next_y;z<=next_z;
                if(iteration==19)begin
                    running<=0;done<=1;
                    sin_q30<=next_y[31:0];
                    cos_q30<=cos_negative ? -$signed(next_x[31:0]) : next_x[31:0];
                end else iteration<=iteration+1'b1;
            end
        end
    end
endmodule
