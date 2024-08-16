`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 29.08.2023 00:07:17
// Design Name: 
// Module Name: PID_tb
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module PID_tb;
reg clk;
reg Kp,Kd,Ki;
reg [7:0]e0;
reg [7:0]e1;
reg [7:0]e2;
wire [7:0]u;

PID dut(Kp, Ki, Kd, clk, u, e0, e1, e2);
always #5 clk = ~clk;
initial begin
    clk = 1;
    Kp = 1'd1; Kd = 1'd1; Ki = 1'd1;
    e0 = 8'd100;
    e1 = 8'd0;
    e2 = 8'd0;
    #10
    e0 = 8'd75;
    e1 = 8'd100;
    e2 = 8'd0;
    #10
    e0 = 8'd50;
    e1 = 8'd75;
    e2 = 8'd100;
    #10
    e0 = 8'd25;
    e1 = 8'd50;
    e2 = 8'd75;
    #10
    e0 = 8'd0;
    e1 = 8'd25;
    e2 = 8'd50;
    #10
    $stop;
end
endmodule
