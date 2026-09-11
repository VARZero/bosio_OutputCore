`timescale 1ns / 1ps
// ============================================================================
// Module: bosio_out_sensor_rx
// Project: Bosio 3DoF Display Output Core (Clean-Slate Architecture)
// Description:
//   AXI4-Stream Sensor Hub Slave receiver.
//   Receives 96-bit packets containing Yaw, Pitch, Roll in Milliradians (mrad).
//   Non-blocking consumer (tready=1) ensuring lowest-latency pose sampling.
// ============================================================================

module bosio_out_sensor_rx #(
    parameter SENSOR_TDATA_WIDTH = 96
)(
    input  wire                             clk,
    input  wire                             rst_n,

    // AXI4-Stream Sensor Hub Interface
    input  wire [SENSOR_TDATA_WIDTH-1:0]    s_axis_sensor_tdata,
    input  wire                             s_axis_sensor_tvalid,
    output wire                             s_axis_sensor_tready,

    // Unpacked Sensor Pose Outputs (Milliradians)
    output reg  signed [31:0]               out_yaw_mrad,
    output reg  signed [31:0]               out_pitch_mrad,
    output reg  signed [31:0]               out_roll_mrad,

    // Status
    output reg                              sensor_stream_active,
    output reg  [31:0]                      sensor_pkt_count
);

    // Non-blocking consumer: always ready for newest sensor orientation
    assign s_axis_sensor_tready = 1'b1;

    // 24-bit Watchdog counter (~16.7M cycles at 100MHz = ~167ms timeout)
    reg [23:0] watchdog_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_yaw_mrad         <= 32'sd0;
            out_pitch_mrad       <= 32'sd0;
            out_roll_mrad        <= 32'sd0;
            sensor_stream_active <= 1'b0;
            sensor_pkt_count     <= 32'd0;
            watchdog_cnt         <= 24'd0;
        end else begin
            if (s_axis_sensor_tvalid) begin
                sensor_stream_active <= 1'b1;
                sensor_pkt_count     <= sensor_pkt_count + 1'b1;
                watchdog_cnt         <= 24'd0;

                out_yaw_mrad   <= $signed(s_axis_sensor_tdata[31:0]);
                out_pitch_mrad <= $signed(s_axis_sensor_tdata[63:32]);
                out_roll_mrad  <= $signed(s_axis_sensor_tdata[95:64]);
            end else begin
                if (watchdog_cnt < 24'hFFFFFF) begin
                    watchdog_cnt <= watchdog_cnt + 1'b1;
                end else begin
                    sensor_stream_active <= 1'b0; // Sensor timed out
                end
            end
        end
    end

endmodule
