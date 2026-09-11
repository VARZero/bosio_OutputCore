`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_pose_proc
// Project: Bosio 3DoF Display Output Core (Clean-Slate Architecture)
// Description:
//   3DoF Orientation Processor:
//   - Multiplexes between Sensor Stream (mrad) and AXI-Lite registers (mrad).
//   - Fast divider-free angle wrapping and 256-entry Quadrant Sine/Cosine LUT.
//   - Computes 3D Forward Look Vector (Fx, Fy, Fz) in Q1.15.
//   - Pipelined 20-Face Dot-Product Tournament to identify center_face_id (1~20).
// ============================================================================

module bosio_out_pose_proc (
    input  wire                             clk,
    input  wire                             rst_n,

    // Pose Source Select
    input  wire                             pose_src_sel,         // 0: Reg, 1: Sensor
    input  wire                             sensor_stream_active,

    // Sensor Pose (Milliradians)
    input  wire signed [31:0]               sensor_yaw_mrad,
    input  wire signed [31:0]               sensor_pitch_mrad,
    input  wire signed [31:0]               sensor_roll_mrad,

    // Register Pose (Milliradians in 16-bit signed)
    input  wire signed [15:0]               reg_yaw_deg_q8,       // Reg Yaw mrad
    input  wire signed [15:0]               reg_pitch_deg_q8,     // Reg Pitch mrad
    input  wire signed [15:0]               reg_roll_deg_q8,      // Reg Roll mrad

    // Processed Outputs
    output reg  [4:0]                       center_face_id,       // 1 ~ 20
    output reg  signed [15:0]               look_vec_x,           // Q1.15
    output reg  signed [15:0]               look_vec_y,           // Q1.15
    output reg  signed [15:0]               look_vec_z,           // Q1.15
    output reg  signed [15:0]               roll_sin,             // Q1.15
    output reg  signed [15:0]               roll_cos,             // Q1.15
    output reg  signed [15:0]               cam_offset_u,         // Q8.0 / DDA accum units
    output reg  signed [15:0]               cam_offset_v          // Q8.0 / DDA accum units
);

    // ------------------------------------------------------------------------
    // 1. Pose Selection (Stage 1)
    // ------------------------------------------------------------------------
    wire use_sensor = pose_src_sel && sensor_stream_active;

    reg signed [31:0] active_yaw_r;
    reg signed [31:0] active_pitch_r;
    reg signed [31:0] active_roll_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            active_yaw_r   <= 32'sd0;
            active_pitch_r <= 32'sd0;
            active_roll_r  <= 32'sd0;
        end else begin
            active_yaw_r   <= use_sensor ? sensor_yaw_mrad   : {{16{reg_yaw_deg_q8[15]}}, reg_yaw_deg_q8};
            active_pitch_r <= use_sensor ? sensor_pitch_mrad : {{16{reg_pitch_deg_q8[15]}}, reg_pitch_deg_q8};
            active_roll_r  <= use_sensor ? sensor_roll_mrad  : {{16{reg_roll_deg_q8[15]}}, reg_roll_deg_q8};
        end
    end

    // ------------------------------------------------------------------------
    // 2. Modulo Angle Wrapping & Phase Conversion (Stage 2)
    // ------------------------------------------------------------------------
    // Wrap to 0 ~ 6283 range (2*PI radians in mrad)
    wire signed [31:0] yaw_p_mod   = active_yaw_r + 32'sd6283;
    wire signed [31:0] yaw_m_mod   = active_yaw_r - 32'sd6283;
    wire signed [31:0] pitch_p_mod = active_pitch_r + 32'sd6283;
    wire signed [31:0] pitch_m_mod = active_pitch_r - 32'sd6283;
    wire signed [31:0] roll_p_mod  = active_roll_r + 32'sd6283;
    wire signed [31:0] roll_m_mod  = active_roll_r - 32'sd6283;

    reg [15:0] wrap_yaw_r;
    reg [15:0] wrap_pitch_r;
    reg [15:0] wrap_roll_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wrap_yaw_r   <= 16'd0;
            wrap_pitch_r <= 16'd0;
            wrap_roll_r  <= 16'd0;
        end else begin
            wrap_yaw_r   <= (active_yaw_r < 0) ? yaw_p_mod[15:0] : (active_yaw_r >= 32'sd6283) ? yaw_m_mod[15:0] : active_yaw_r[15:0];
            wrap_pitch_r <= (active_pitch_r < 0) ? pitch_p_mod[15:0] : (active_pitch_r >= 32'sd6283) ? pitch_m_mod[15:0] : active_pitch_r[15:0];
            wrap_roll_r  <= (active_roll_r < 0) ? roll_p_mod[15:0] : (active_roll_r >= 32'sd6283) ? roll_m_mod[15:0] : active_roll_r[15:0];
        end
    end

    // Map 0 ~ 6283 mrad to 10-bit Phase index (0 ~ 1023):
    // phase = (wrap_mrad * 167) >> 10
    reg [9:0] angle_yaw_idx;
    reg [9:0] angle_pitch_idx;
    reg [9:0] angle_roll_idx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            angle_yaw_idx   <= 10'd0;
            angle_pitch_idx <= 10'd0;
            angle_roll_idx  <= 10'd0;
        end else begin
            angle_yaw_idx   <= (({16'd0, wrap_yaw_r}   * 32'd167) >> 10) & 10'd1023;
            angle_pitch_idx <= (({16'd0, wrap_pitch_r} * 32'd167) >> 10) & 10'd1023;
            angle_roll_idx  <= (({16'd0, wrap_roll_r}  * 32'd167) >> 10) & 10'd1023;
        end
    end

    // ------------------------------------------------------------------------
    // 3. 256-Entry Quarter Sine ROM (Q1.15: 0 to 32767)
    // ------------------------------------------------------------------------
    function signed [15:0] sin_quarter(input [7:0] idx);
        case (idx)
            8'd0: sin_quarter = 16'sd0; 8'd1: sin_quarter = 16'sd201; 8'd2: sin_quarter = 16'sd402; 8'd3: sin_quarter = 16'sd603;
            8'd4: sin_quarter = 16'sd804; 8'd5: sin_quarter = 16'sd1005; 8'd6: sin_quarter = 16'sd1206; 8'd7: sin_quarter = 16'sd1407;
            8'd8: sin_quarter = 16'sd1608; 8'd9: sin_quarter = 16'sd1809; 8'd10: sin_quarter = 16'sd2009; 8'd11: sin_quarter = 16'sd2210;
            8'd12: sin_quarter = 16'sd2410; 8'd13: sin_quarter = 16'sd2611; 8'd14: sin_quarter = 16'sd2811; 8'd15: sin_quarter = 16'sd3012;
            8'd16: sin_quarter = 16'sd3212; 8'd17: sin_quarter = 16'sd3412; 8'd18: sin_quarter = 16'sd3612; 8'd19: sin_quarter = 16'sd3811;
            8'd20: sin_quarter = 16'sd4011; 8'd21: sin_quarter = 16'sd4210; 8'd22: sin_quarter = 16'sd4410; 8'd23: sin_quarter = 16'sd4609;
            8'd24: sin_quarter = 16'sd4808; 8'd25: sin_quarter = 16'sd5007; 8'd26: sin_quarter = 16'sd5205; 8'd27: sin_quarter = 16'sd5404;
            8'd28: sin_quarter = 16'sd5602; 8'd29: sin_quarter = 16'sd5800; 8'd30: sin_quarter = 16'sd5998; 8'd31: sin_quarter = 16'sd6195;
            8'd32: sin_quarter = 16'sd6393; 8'd33: sin_quarter = 16'sd6590; 8'd34: sin_quarter = 16'sd6786; 8'd35: sin_quarter = 16'sd6983;
            8'd36: sin_quarter = 16'sd7179; 8'd37: sin_quarter = 16'sd7375; 8'd38: sin_quarter = 16'sd7571; 8'd39: sin_quarter = 16'sd7767;
            8'd40: sin_quarter = 16'sd7962; 8'd41: sin_quarter = 16'sd8157; 8'd42: sin_quarter = 16'sd8351; 8'd43: sin_quarter = 16'sd8545;
            8'd44: sin_quarter = 16'sd8739; 8'd45: sin_quarter = 16'sd8933; 8'd46: sin_quarter = 16'sd9126; 8'd47: sin_quarter = 16'sd9319;
            8'd48: sin_quarter = 16'sd9512; 8'd49: sin_quarter = 16'sd9704; 8'd50: sin_quarter = 16'sd9896; 8'd51: sin_quarter = 16'sd10087;
            8'd52: sin_quarter = 16'sd10278; 8'd53: sin_quarter = 16'sd10469; 8'd54: sin_quarter = 16'sd10659; 8'd55: sin_quarter = 16'sd10849;
            8'd56: sin_quarter = 16'sd11039; 8'd57: sin_quarter = 16'sd11228; 8'd58: sin_quarter = 16'sd11417; 8'd59: sin_quarter = 16'sd11605;
            8'd60: sin_quarter = 16'sd11793; 8'd61: sin_quarter = 16'sd11980; 8'd62: sin_quarter = 16'sd12167; 8'd63: sin_quarter = 16'sd12353;
            8'd64: sin_quarter = 16'sd12539; 8'd65: sin_quarter = 16'sd12725; 8'd66: sin_quarter = 16'sd12910; 8'd67: sin_quarter = 16'sd13094;
            8'd68: sin_quarter = 16'sd13279; 8'd69: sin_quarter = 16'sd13462; 8'd70: sin_quarter = 16'sd13645; 8'd71: sin_quarter = 16'sd13828;
            8'd72: sin_quarter = 16'sd14010; 8'd73: sin_quarter = 16'sd14191; 8'd74: sin_quarter = 16'sd14372; 8'd75: sin_quarter = 16'sd14553;
            8'd76: sin_quarter = 16'sd14732; 8'd77: sin_quarter = 16'sd14912; 8'd78: sin_quarter = 16'sd15090; 8'd79: sin_quarter = 16'sd15269;
            8'd80: sin_quarter = 16'sd15446; 8'd81: sin_quarter = 16'sd15623; 8'd82: sin_quarter = 16'sd15800; 8'd83: sin_quarter = 16'sd15976;
            8'd84: sin_quarter = 16'sd16151; 8'd85: sin_quarter = 16'sd16325; 8'd86: sin_quarter = 16'sd16499; 8'd87: sin_quarter = 16'sd16673;
            8'd88: sin_quarter = 16'sd16846; 8'd89: sin_quarter = 16'sd17018; 8'd90: sin_quarter = 16'sd17189; 8'd91: sin_quarter = 16'sd17360;
            8'd92: sin_quarter = 16'sd17530; 8'd93: sin_quarter = 16'sd17700; 8'd94: sin_quarter = 16'sd17869; 8'd95: sin_quarter = 16'sd18037;
            8'd96: sin_quarter = 16'sd18204; 8'd97: sin_quarter = 16'sd18371; 8'd98: sin_quarter = 16'sd18537; 8'd99: sin_quarter = 16'sd18703;
            8'd100: sin_quarter = 16'sd18868; 8'd101: sin_quarter = 16'sd19032; 8'd102: sin_quarter = 16'sd19195; 8'd103: sin_quarter = 16'sd19357;
            8'd104: sin_quarter = 16'sd19519; 8'd105: sin_quarter = 16'sd19680; 8'd106: sin_quarter = 16'sd19841; 8'd107: sin_quarter = 16'sd20000;
            8'd108: sin_quarter = 16'sd20159; 8'd109: sin_quarter = 16'sd20317; 8'd110: sin_quarter = 16'sd20475; 8'd111: sin_quarter = 16'sd20631;
            8'd112: sin_quarter = 16'sd20787; 8'd113: sin_quarter = 16'sd20942; 8'd114: sin_quarter = 16'sd21096; 8'd115: sin_quarter = 16'sd21250;
            8'd116: sin_quarter = 16'sd21403; 8'd117: sin_quarter = 16'sd21554; 8'd118: sin_quarter = 16'sd21705; 8'd119: sin_quarter = 16'sd21856;
            8'd120: sin_quarter = 16'sd22005; 8'd121: sin_quarter = 16'sd22154; 8'd122: sin_quarter = 16'sd22301; 8'd123: sin_quarter = 16'sd22448;
            8'd124: sin_quarter = 16'sd22594; 8'd125: sin_quarter = 16'sd22739; 8'd126: sin_quarter = 16'sd22884; 8'd127: sin_quarter = 16'sd23027;
            8'd128: sin_quarter = 16'sd23170; 8'd129: sin_quarter = 16'sd23311; 8'd130: sin_quarter = 16'sd23452; 8'd131: sin_quarter = 16'sd23592;
            8'd132: sin_quarter = 16'sd23731; 8'd133: sin_quarter = 16'sd23870; 8'd134: sin_quarter = 16'sd24007; 8'd135: sin_quarter = 16'sd24143;
            8'd136: sin_quarter = 16'sd24279; 8'd137: sin_quarter = 16'sd24413; 8'd138: sin_quarter = 16'sd24547; 8'd139: sin_quarter = 16'sd24680;
            8'd140: sin_quarter = 16'sd24811; 8'd141: sin_quarter = 16'sd24942; 8'd142: sin_quarter = 16'sd25072; 8'd143: sin_quarter = 16'sd25201;
            8'd144: sin_quarter = 16'sd25329; 8'd145: sin_quarter = 16'sd25456; 8'd146: sin_quarter = 16'sd25582; 8'd147: sin_quarter = 16'sd25708;
            8'd148: sin_quarter = 16'sd25832; 8'd149: sin_quarter = 16'sd25955; 8'd150: sin_quarter = 16'sd26077; 8'd151: sin_quarter = 16'sd26198;
            8'd152: sin_quarter = 16'sd26319; 8'd153: sin_quarter = 16'sd26438; 8'd154: sin_quarter = 16'sd26556; 8'd155: sin_quarter = 16'sd26674;
            8'd156: sin_quarter = 16'sd26790; 8'd157: sin_quarter = 16'sd26905; 8'd158: sin_quarter = 16'sd27019; 8'd159: sin_quarter = 16'sd27133;
            8'd160: sin_quarter = 16'sd27245; 8'd161: sin_quarter = 16'sd27356; 8'd162: sin_quarter = 16'sd27466; 8'd163: sin_quarter = 16'sd27575;
            8'd164: sin_quarter = 16'sd27683; 8'd165: sin_quarter = 16'sd27790; 8'd166: sin_quarter = 16'sd27896; 8'd167: sin_quarter = 16'sd28001;
            8'd168: sin_quarter = 16'sd28105; 8'd169: sin_quarter = 16'sd28208; 8'd170: sin_quarter = 16'sd28310; 8'd171: sin_quarter = 16'sd28411;
            8'd172: sin_quarter = 16'sd28510; 8'd173: sin_quarter = 16'sd28609; 8'd174: sin_quarter = 16'sd28706; 8'd175: sin_quarter = 16'sd28803;
            8'd176: sin_quarter = 16'sd28898; 8'd177: sin_quarter = 16'sd28992; 8'd178: sin_quarter = 16'sd29085; 8'd179: sin_quarter = 16'sd29177;
            8'd180: sin_quarter = 16'sd29268; 8'd181: sin_quarter = 16'sd29358; 8'd182: sin_quarter = 16'sd29447; 8'd183: sin_quarter = 16'sd29534;
            8'd184: sin_quarter = 16'sd29621; 8'd185: sin_quarter = 16'sd29706; 8'd186: sin_quarter = 16'sd29791; 8'd187: sin_quarter = 16'sd29874;
            8'd188: sin_quarter = 16'sd29956; 8'd189: sin_quarter = 16'sd30037; 8'd190: sin_quarter = 16'sd30117; 8'd191: sin_quarter = 16'sd30195;
            8'd192: sin_quarter = 16'sd30273; 8'd193: sin_quarter = 16'sd30349; 8'd194: sin_quarter = 16'sd30424; 8'd195: sin_quarter = 16'sd30498;
            8'd196: sin_quarter = 16'sd30571; 8'd197: sin_quarter = 16'sd30643; 8'd198: sin_quarter = 16'sd30714; 8'd199: sin_quarter = 16'sd30783;
            8'd200: sin_quarter = 16'sd30852; 8'd201: sin_quarter = 16'sd30919; 8'd202: sin_quarter = 16'sd30985; 8'd203: sin_quarter = 16'sd31050;
            8'd204: sin_quarter = 16'sd31113; 8'd205: sin_quarter = 16'sd31176; 8'd206: sin_quarter = 16'sd31237; 8'd207: sin_quarter = 16'sd31297;
            8'd208: sin_quarter = 16'sd31356; 8'd209: sin_quarter = 16'sd31414; 8'd210: sin_quarter = 16'sd31470; 8'd211: sin_quarter = 16'sd31526;
            8'd212: sin_quarter = 16'sd31580; 8'd213: sin_quarter = 16'sd31633; 8'd214: sin_quarter = 16'sd31685; 8'd215: sin_quarter = 16'sd31736;
            8'd216: sin_quarter = 16'sd31785; 8'd217: sin_quarter = 16'sd31833; 8'd218: sin_quarter = 16'sd31880; 8'd219: sin_quarter = 16'sd31926;
            8'd220: sin_quarter = 16'sd31971; 8'd221: sin_quarter = 16'sd32014; 8'd222: sin_quarter = 16'sd32057; 8'd223: sin_quarter = 16'sd32098;
            8'd224: sin_quarter = 16'sd32137; 8'd225: sin_quarter = 16'sd32176; 8'd226: sin_quarter = 16'sd32213; 8'd227: sin_quarter = 16'sd32250;
            8'd228: sin_quarter = 16'sd32285; 8'd229: sin_quarter = 16'sd32318; 8'd230: sin_quarter = 16'sd32351; 8'd231: sin_quarter = 16'sd32382;
            8'd232: sin_quarter = 16'sd32412; 8'd233: sin_quarter = 16'sd32441; 8'd234: sin_quarter = 16'sd32469; 8'd235: sin_quarter = 16'sd32495;
            8'd236: sin_quarter = 16'sd32521; 8'd237: sin_quarter = 16'sd32545; 8'd238: sin_quarter = 16'sd32567; 8'd239: sin_quarter = 16'sd32589;
            8'd240: sin_quarter = 16'sd32609; 8'd241: sin_quarter = 16'sd32628; 8'd242: sin_quarter = 16'sd32646; 8'd243: sin_quarter = 16'sd32663;
            8'd244: sin_quarter = 16'sd32678; 8'd245: sin_quarter = 16'sd32692; 8'd246: sin_quarter = 16'sd32705; 8'd247: sin_quarter = 16'sd32717;
            8'd248: sin_quarter = 16'sd32728; 8'd249: sin_quarter = 16'sd32737; 8'd250: sin_quarter = 16'sd32745; 8'd251: sin_quarter = 16'sd32752;
            8'd252: sin_quarter = 16'sd32757; 8'd253: sin_quarter = 16'sd32761; 8'd254: sin_quarter = 16'sd32765; 8'd255: sin_quarter = 16'sd32766;
            default: sin_quarter = 16'sd0;
        endcase
    endfunction

    function signed [15:0] get_sin(input [9:0] theta);
        reg [1:0] quad;
        reg [7:0] idx;
        begin
            quad = theta[9:8];
            idx  = theta[7:0];
            case (quad)
                2'd0: get_sin =  sin_quarter(idx);
                2'd1: get_sin =  sin_quarter(8'd255 - idx);
                2'd2: get_sin = -sin_quarter(idx);
                2'd3: get_sin = -sin_quarter(8'd255 - idx);
            endcase
        end
    endfunction

    function signed [15:0] get_cos(input [9:0] theta);
        begin
            get_cos = get_sin(theta + 10'd256);
        end
    endfunction

    // ------------------------------------------------------------------------
    // 4. Forward Look Vector Pipeline (Stage 3)
    // ------------------------------------------------------------------------
    reg signed [15:0] az_sin_r, az_cos_r;
    reg signed [15:0] el_sin_r, el_cos_r;
    reg signed [15:0] roll_sin_r, roll_cos_r;

    always @(posedge clk) begin
        az_sin_r   <= get_sin(angle_yaw_idx);
        az_cos_r   <= get_cos(angle_yaw_idx);
        el_sin_r   <= get_sin(angle_pitch_idx);
        el_cos_r   <= get_cos(angle_pitch_idx);
        roll_sin_r <= get_sin(angle_roll_idx);
        roll_cos_r <= get_cos(angle_roll_idx);
    end

    wire signed [31:0] prod_look_x = $signed(az_sin_r) * $signed(el_cos_r);
    wire signed [31:0] prod_look_z = $signed(az_cos_r) * $signed(el_cos_r);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            look_vec_x <= 16'sd0;
            look_vec_y <= 16'sd0;
            look_vec_z <= -16'sd32767;
            roll_sin   <= 16'sd0;
            roll_cos   <= 16'sd32767;
        end else begin
            look_vec_x <= prod_look_x[30:15];
            look_vec_y <= el_sin_r;
            look_vec_z <= -prod_look_z[30:15];
            roll_sin   <= roll_sin_r;
            roll_cos   <= roll_cos_r;
        end
    end

    // ------------------------------------------------------------------------
    // 5. 20 Face Centers & Dot Product Tournament (Q1.15)
    // ------------------------------------------------------------------------
    wire signed [15:0] cx [0:19];
    wire signed [15:0] cy [0:19];
    wire signed [15:0] cz [0:19];

    assign cx[0]  =  16'sd9946;  assign cy[0]  = -16'sd6147;  assign cz[0]  = -16'sd30610; // Face 1
    assign cx[1]  =  16'sd6147;  assign cy[1]  = -16'sd26038; assign cz[1]  = -16'sd18918; // Face 2
    assign cx[2]  =  16'sd32185; assign cy[2]  = -16'sd6147;  assign cz[2]  =  16'sd0;      // Face 3
    assign cx[3]  =  16'sd19892; assign cy[3]  = -16'sd26038; assign cz[3]  =  16'sd0;      // Face 4
    assign cx[4]  =  16'sd9946;  assign cy[4]  = -16'sd6147;  assign cz[4]  =  16'sd30610;  // Face 5
    assign cx[5]  =  16'sd6147;  assign cy[5]  = -16'sd26038; assign cz[5]  =  16'sd18918;  // Face 6
    assign cx[6]  = -16'sd26038; assign cy[6]  = -16'sd6147;  assign cz[6]  =  16'sd18918;  // Face 7
    assign cx[7]  = -16'sd16093; assign cy[7]  = -16'sd26038; assign cz[7]  =  16'sd11692;  // Face 8
    assign cx[8]  = -16'sd26038; assign cy[8]  = -16'sd6147;  assign cz[8]  = -16'sd18918; // Face 9
    assign cx[9]  = -16'sd16093; assign cy[9]  = -16'sd26038; assign cz[9]  = -16'sd11692; // Face 10
    assign cx[10] =  16'sd26038; assign cy[10] =  16'sd6147;  assign cz[10] = -16'sd18918; // Face 11
    assign cx[11] = -16'sd6147;  assign cy[11] =  16'sd26038; assign cz[11] = -16'sd18918; // Face 12
    assign cx[12] =  16'sd26038; assign cy[12] =  16'sd6147;  assign cz[12] =  16'sd18918;  // Face 13
    assign cx[13] =  16'sd16093; assign cy[13] =  16'sd26038; assign cz[13] = -16'sd11692; // Face 14
    assign cx[14] = -16'sd9946;  assign cy[14] =  16'sd6147;  assign cz[14] =  16'sd30610;  // Face 15
    assign cx[15] =  16'sd16093; assign cy[15] =  16'sd26038; assign cz[15] =  16'sd11692;  // Face 16
    assign cx[16] = -16'sd32185; assign cy[16] =  16'sd6147;  assign cz[16] =  16'sd0;      // Face 17
    assign cx[17] = -16'sd6147;  assign cy[17] =  16'sd26038; assign cz[17] =  16'sd18918;  // Face 18
    assign cx[18] = -16'sd9946;  assign cy[18] =  16'sd6147;  assign cz[18] = -16'sd30610; // Face 19
    assign cx[19] = -16'sd19892; assign cy[19] =  16'sd26038; assign cz[19] =  16'sd0;      // Face 20

    // ------------------------------------------------------------------------
    // 5. 20 Face Centers & Pipelined Tournament Tree (Max Dot-Product)
    // ------------------------------------------------------------------------
    // Stage 1: Compute 20 dot products in parallel
    reg signed [31:0] dot_prods [0:19];
    integer i;
    always @(posedge clk) begin
        for (i = 0; i < 20; i = i + 1) begin
            dot_prods[i] <= ($signed(look_vec_x) * cx[i]) + 
                            ($signed(look_vec_y) * cy[i]) + 
                            ($signed(look_vec_z) * cz[i]);
        end
    end

    // Stage 2: Level 1 - 10 parallel 2-to-1 comparisons
    reg signed [31:0] val_l1 [0:9];
    reg [4:0]         id_l1  [0:9];
    integer k;
    always @(posedge clk) begin
        for (k = 0; k < 10; k = k + 1) begin
            if (dot_prods[2*k + 1] > dot_prods[2*k]) begin
                val_l1[k] <= dot_prods[2*k + 1];
                id_l1[k]  <= (2 * k + 2);
            end else begin
                val_l1[k] <= dot_prods[2*k];
                id_l1[k]  <= (2 * k + 1);
            end
        end
    end

    // Stage 3: Level 2 - 5 parallel 2-to-1 comparisons
    reg signed [31:0] val_l2 [0:4];
    reg [4:0]         id_l2  [0:4];
    integer j;
    always @(posedge clk) begin
        for (j = 0; j < 5; j = j + 1) begin
            if (val_l1[2*j + 1] > val_l1[2*j]) begin
                val_l2[j] <= val_l1[2*j + 1];
                id_l2[j]  <= id_l1[2*j + 1];
            end else begin
                val_l2[j] <= val_l1[2*j];
                id_l2[j]  <= id_l1[2*j];
            end
        end
    end

    // Stage 4: Level 3 - Reduce 5 to 3
    reg signed [31:0] val_l3_a, val_l3_b, val_l3_c;
    reg [4:0]         id_l3_a,  id_l3_b,  id_l3_c;
    always @(posedge clk) begin
        // Compare 0 and 1
        if (val_l2[1] > val_l2[0]) begin
            val_l3_a <= val_l2[1];
            id_l3_a  <= id_l2[1];
        end else begin
            val_l3_a <= val_l2[0];
            id_l3_a  <= id_l2[0];
        end

        // Compare 2 and 3
        if (val_l2[3] > val_l2[2]) begin
            val_l3_b <= val_l2[3];
            id_l3_b  <= id_l2[3];
        end else begin
            val_l3_b <= val_l2[2];
            id_l3_b  <= id_l2[2];
        end

        // Pass 4
        val_l3_c <= val_l2[4];
        id_l3_c  <= id_l2[4];
    end

    // Stage 5: Level 4 - Final 3-to-1 comparison to produce center_face_id
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            center_face_id <= 5'd1;
        end else begin
            if (val_l3_a >= val_l3_b && val_l3_a >= val_l3_c) begin
                center_face_id <= id_l3_a;
            end else if (val_l3_b >= val_l3_a && val_l3_b >= val_l3_c) begin
                center_face_id <= id_l3_b;
            end else begin
                center_face_id <= id_l3_c;
            end
        end
    end

    // ------------------------------------------------------------------------
    // 6. Sub-Face Angular Offset Pipeline for Seamless 3DoF Panning
    // ------------------------------------------------------------------------
    function signed [15:0] get_face_center_az_mrad(input [4:0] fid);
        case (fid)
            5'd1:  get_face_center_az_mrad = 16'sd314;
            5'd2:  get_face_center_az_mrad = 16'sd314;
            5'd3:  get_face_center_az_mrad = 16'sd1571;
            5'd4:  get_face_center_az_mrad = 16'sd1571;
            5'd5:  get_face_center_az_mrad = 16'sd2827;
            5'd6:  get_face_center_az_mrad = 16'sd2827;
            5'd7:  get_face_center_az_mrad = 16'sd4084;
            5'd8:  get_face_center_az_mrad = 16'sd4084;
            5'd9:  get_face_center_az_mrad = 16'sd5341;
            5'd10: get_face_center_az_mrad = 16'sd5341;
            5'd11: get_face_center_az_mrad = 16'sd942;
            5'd12: get_face_center_az_mrad = 16'sd5969;
            5'd13: get_face_center_az_mrad = 16'sd2199;
            5'd14: get_face_center_az_mrad = 16'sd942;
            5'd15: get_face_center_az_mrad = 16'sd3456;
            5'd16: get_face_center_az_mrad = 16'sd2199;
            5'd17: get_face_center_az_mrad = 16'sd4712;
            5'd18: get_face_center_az_mrad = 16'sd3456;
            5'd19: get_face_center_az_mrad = 16'sd5969;
            5'd20: get_face_center_az_mrad = 16'sd4712;
            default: get_face_center_az_mrad = 16'sd314;
        endcase
    endfunction

    function signed [15:0] get_face_center_el_mrad(input [4:0] fid);
        case (fid)
            5'd1, 5'd3, 5'd5, 5'd7, 5'd9:       get_face_center_el_mrad = -16'sd189;
            5'd2, 5'd4, 5'd6, 5'd8, 5'd10:      get_face_center_el_mrad = -16'sd918;
            5'd11, 5'd13, 5'd15, 5'd17, 5'd19:  get_face_center_el_mrad =  16'sd189;
            5'd12, 5'd14, 5'd16, 5'd18, 5'd20:  get_face_center_el_mrad =  16'sd918;
            default:                            get_face_center_el_mrad = -16'sd189;
        endcase
    endfunction

    // 7-Stage Delay Line to Synchronize Input Angles with center_face_id
    reg [15:0] yaw_delay_pipe [0:6];
    reg [15:0] pitch_delay_pipe [0:6];
    integer d_idx;
    always @(posedge clk) begin
        yaw_delay_pipe[0]   <= wrap_yaw_r;
        pitch_delay_pipe[0] <= wrap_pitch_r;
        for (d_idx = 0; d_idx < 6; d_idx = d_idx + 1) begin
            yaw_delay_pipe[d_idx + 1]   <= yaw_delay_pipe[d_idx];
            pitch_delay_pipe[d_idx + 1] <= pitch_delay_pipe[d_idx];
        end
    end

    wire [15:0] sync_yaw   = yaw_delay_pipe[6];
    wire [15:0] sync_pitch = pitch_delay_pipe[6];

    wire signed [15:0] face_az_c = get_face_center_az_mrad(center_face_id);
    wire signed [15:0] face_el_c = get_face_center_el_mrad(center_face_id);

    reg signed [15:0] delta_yaw_mrad;
    always @(*) begin
        delta_yaw_mrad = $signed({1'b0, sync_yaw}) - face_az_c;
        if (delta_yaw_mrad > 16'sd3141)
            delta_yaw_mrad = delta_yaw_mrad - 16'sd6283;
        else if (delta_yaw_mrad < -16'sd3141)
            delta_yaw_mrad = delta_yaw_mrad + 16'sd6283;
    end

    wire signed [15:0] s_sync_pitch = (sync_pitch >= 16'd4712) ? $signed(sync_pitch - 16'd6283) : $signed(sync_pitch);
    wire signed [15:0] delta_pitch_mrad = s_sync_pitch - face_el_c;

    reg signed [31:0] prod_cam_u;
    reg signed [31:0] prod_cam_v;
    always @(posedge clk) begin
        prod_cam_u <= delta_yaw_mrad * 16'sd536;
        prod_cam_v <= delta_pitch_mrad * 16'sd536;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cam_offset_u <= 16'sd0;
            cam_offset_v <= 16'sd0;
        end else begin
            cam_offset_u <= prod_cam_u[25:10];
            cam_offset_v <= prod_cam_v[25:10];
        end
    end

endmodule
