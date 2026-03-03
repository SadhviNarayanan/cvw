// riscvsingle.sv
// RISC-V single-cycle processor
// David_Harris@hmc.edu 2020 kacassidy@hmc.edu 2025

`include "parameters.svh"

module riscvsingle(
    input  logic        clk,
    input  logic        reset,

    // Instruction memory interface
    output logic [31:0] PC,
    input  logic [31:0] Instr,

    // Data memory interface
    output logic [31:0] IEUAdr,
    input  logic [31:0] ReadData,
    output logic [31:0] WriteData,
    output logic        MemEn,
    output logic        WriteEn,
    output logic [3:0]  WriteByteEn
);


    logic [31:0] InstrD;       // full instruction at Decode stage

    // Control signals
    logic [2:0] ResultSrc;
    logic MemWrite, Load, ALUSrc;
    logic RegWrite, Jump, Branch, CSRWrite;
    logic [2:0] ImmSrc;
    logic [3:0] ALUControl;

    // =====================
    // Signals from Memory stage
    // =====================
    logic [31:0] ALUResultM;   // ALU output / address to memory
    logic [31:0] WriteDataM;   // Data to write to memory
    logic        MemWriteM;    // MemWrite from memory stage
    logic        LoadM;        // Load signal from memory stage
    logic [2:0]  Funct3M;      // Funct3 for memory access
    logic [31:0] ReadDataAdjusted; // Adjusted load data from memory

    assign IEUAdr = ALUResultM;
    assign MemEn = MemWriteM | LoadM;
    assign WriteEn = MemWriteM;

    // Your existing datapath - it should output these Memory stage signals
    datapath dp(clk, reset, ResultSrc, MemWrite, Load,
              ALUSrc, RegWrite,
              ImmSrc, ALUControl, CSRWrite, Jump, Branch,
              PC, Instr,
              ALUResultM, WriteDataM, MemWriteM, LoadM, Funct3M, ReadData, InstrD);

    // controller
    controller c(InstrD[30], InstrD[25], InstrD[14:12], InstrD[6:0],
               ResultSrc, MemWrite, Load,
               ALUSrc, RegWrite, Jump, Branch, CSRWrite,
               ImmSrc, ALUControl
    `ifdef DEBUG
,       .insn_debug(InstrD)
    `endif);

    // Store unit: uses Memory stage signals
    storeUnit su(
        .funct3(Funct3M), // Funct3 from Memory stage
        .addr_low(ALUResultM[1:0]),
        .MemWrite(MemWriteM),
        .WriteByteEn(WriteByteEn),
        .WriteDataIn(WriteDataM),
        .WriteDataOut(WriteData)
    );

endmodule


// module riscvsingle(
//         input   logic           clk,
//         input   logic           reset,

//         output  logic [31:0]    PC,  // instruction memory target address
//         input   logic [31:0]    Instr, // instruction memory read data

//         output  logic [31:0]    IEUAdr,  // data memory target address
//         input   logic [31:0]    ReadData, // data memory read data
//         output  logic [31:0]    WriteData, // data memory write data

//         output  logic           MemEn,
//         output  logic           WriteEn,
//         output  logic [3:0]     WriteByteEn  // strobes, 1 hot stating weather a byte should be written on a store
//     );

//     logic [31:0] PCPlus4;
//     logic PCSrc;
//     logic Load;

//     ifu ifu(.clk, .reset, .PCSrc, .IEUAdr, .PC, .PCPlus4);
//     ieu ieu(.clk, .reset, .Instr, .PC, .PCPlus4, .PCSrc, .WriteByteEn,
//             .IEUAdr, .WriteData, .ReadData, .MemEn
//         );

//     assign WriteEn = |WriteByteEn;
// endmodule
