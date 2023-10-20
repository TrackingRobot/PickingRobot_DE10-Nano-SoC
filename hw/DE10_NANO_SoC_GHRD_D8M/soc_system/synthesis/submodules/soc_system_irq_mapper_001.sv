// (C) 2001-2017 Intel Corporation. All rights reserved.
// Your use of Intel Corporation's design tools, logic functions and other 
// software and tools, and its AMPP partner logic functions, and any output 
// files from any of the foregoing (including device programming or simulation 
// files), and any associated documentation or information are expressly subject 
// to the terms and conditions of the Intel Program License Subscription 
// Agreement, Intel FPGA IP License Agreement, or other applicable 
// license agreement, including, without limitation, that your use is for the 
// sole purpose of programming logic devices manufactured by Intel and sold by 
// Intel or its authorized distributors.  Please refer to the applicable 
// agreement for further details.


// $Id: //acds/rel/17.1std/ip/merlin/altera_irq_mapper/altera_irq_mapper.sv.terp#1 $
// $Revision: #1 $
// $Date: 2017/07/30 $
// $Author: swbranch $

// -------------------------------------------------------
// Altera IRQ Mapper
//
// Parameters
//   NUM_RCVRS        : 11
//   SENDER_IRW_WIDTH : 32
//   IRQ_MAP          : 0:5,1:9,2:10,3:2,4:1,5:0,6:3,7:4,8:6,9:7,10:8
//
// -------------------------------------------------------

`timescale 1 ns / 1 ns

module soc_system_irq_mapper_001
(
    // -------------------
    // Clock & Reset
    // -------------------
    input clk,
    input reset,

    // -------------------
    // IRQ Receivers
    // -------------------
    input                receiver0_irq,
    input                receiver1_irq,
    input                receiver2_irq,
    input                receiver3_irq,
    input                receiver4_irq,
    input                receiver5_irq,
    input                receiver6_irq,
    input                receiver7_irq,
    input                receiver8_irq,
    input                receiver9_irq,
    input                receiver10_irq,

    // -------------------
    // Command Source (Output)
    // -------------------
    output reg [31 : 0] sender_irq
);


    always @* begin
	sender_irq = 0;

        sender_irq[5] = receiver0_irq;
        sender_irq[9] = receiver1_irq;
        sender_irq[10] = receiver2_irq;
        sender_irq[2] = receiver3_irq;
        sender_irq[1] = receiver4_irq;
        sender_irq[0] = receiver5_irq;
        sender_irq[3] = receiver6_irq;
        sender_irq[4] = receiver7_irq;
        sender_irq[6] = receiver8_irq;
        sender_irq[7] = receiver9_irq;
        sender_irq[8] = receiver10_irq;
    end

endmodule

