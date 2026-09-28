///////////////////////////////////////////
// ieu.sv
//
// Written: David_Harris@hmc.edu 9 January 2021
// Modified:
//
// Purpose: Integer Execution Unit: datapath and controller
//
// Documentation: RISC-V System on Chip Design
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// https://github.com/openhwgroup/cvw
//
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file
// except in compliance with the License, or, at your option, the Apache License version 2.0. You
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
// either express or implied. See the License for the specific language governing permissions
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

module ieu import cvw::*;  #(parameter cvw_t P) (
  input  logic              clk, reset,
  // Decode stage signals
  input  logic [31:0]       InstrD,                          // Instruction
  input  logic [31:0]       Instr2D,                         // Second instruction (superscalar slot 1), decoded but not issued
  input  logic              Issue2D,                         // Slot 1 is paired with slot 0 this cycle
  input  logic [1:0]        STATUS_FS,                       // is FPU enabled?
  input  logic [3:0]        ENVCFG_CBE,                      // Cache block operation enables
  input  logic              IllegalIEUFPUInstrD,             // Illegal instruction
  output logic              IllegalBaseInstrD,               // Illegal I-type instruction, or illegal RV32 access to upper 16 registers
  // Execute stage signals
  input  logic [P.XLEN-1:0] PCE,                             // PC
  input  logic [P.XLEN-1:0] PCLinkE,                         // PC + 4
  output logic              PCSrcE,                          // Select next PC (between PC+4 and IEUAdrE)
  input  logic              FWriteIntE, FCvtIntE,            // FPU writes to integer register file, FPU converts float to int
  output logic [P.XLEN-1:0] IEUAdrE,                         // Memory address
  output logic              IntDivE, W64E,                   // Integer divide, RV64 W-type instruction
  output logic [2:0]        Funct3E,                         // Funct3 instruction field
  output logic [P.XLEN-1:0] ForwardedSrcAE, ForwardedSrcBE,  // ALU src inputs before the mux choosing between them and PCE to put in srcA/B
  output logic [4:0]        RdE,                             // Destination register
  output logic              MDUActiveE,                      // Mul/Div instruction being executed
  output logic [3:0]        CMOpM,                           // 1: cbo.inval; 2: cbo.clean; 4: cbo.flush; 8: cbo.zero
  output logic              IFUPrefetchE,                    // instruction prefetch
  output logic              LSUPrefetchM,                    // datata prefetch
  // Memory stage signals
  input  logic              SquashSCW,                       // Squash store conditional, from LSU
  output logic [1:0]        MemRWE,                          // Read/write control goes to LSU
  output logic [1:0]        MemRWM,                          // Read/write control goes to LSU
  output logic [1:0]        AtomicM,                         // Atomic control goes to LSU
  output logic [P.XLEN-1:0] WriteDataM,                      // Write data to LSU
  output logic [2:0]        Funct3M,                         // Funct3 (size and signedness) to LSU
  output logic [P.XLEN-1:0] SrcAM,                           // ALU SrcA to Privileged unit and FPU
  output logic [4:0]        RdM,                             // Destination register
  input  logic [P.XLEN-1:0] FIntResM,                        // Integer result from FPU (fmv, fclass, fcmp)
  output logic              InvalidateICacheM, FlushDCacheM, // Invalidate I$, flush D$
  output logic              InstrValidD, InstrValidE, InstrValidM, // Instruction is valid
  output logic              BranchD, BranchE,
  output logic              JumpD, JumpE,
  // Writeback stage signals
  input  logic [P.XLEN-1:0] FIntDivResultW,                  // Integer divide result from FPU fdivsqrt)
  input  logic [P.XLEN-1:0] CSRReadValW,                     // CSR read value,
  input  logic [P.XLEN-1:0] MDUResultW,                      // multiply/divide unit result
  input  logic [P.XLEN-1:0] FCvtIntResW,                     // FPU's float to int conversion result
  input  logic              FCvtIntW,                        // FPU converts float to int
  output logic [4:0]        RdW,                             // Destination register
  input  logic [P.XLEN-1:0] ReadDataW,                       // LSU's read data
  // Hazard unit signals
  input  logic              StallD, StallE, StallM, StallW,  // Stall signals from hazard unit
  input  logic              FlushD, FlushE, FlushM, FlushW,  // Flush signals
  output logic              StructuralStallD,                // IEU detects structural hazard in Decode stage
  output logic              LoadStallD,                      // Structural stalls for load, sent to performance counters
  output logic              StoreStallD,                     // load after store hazard
  output logic              CSRReadM, CSRWriteM, PrivilegedM,// CSR read, CSR write, is privileged instruction
  output logic              CSRWriteFenceM                   // CSR write or fence instruction needs to flush subsequent instructions
);

  logic [2:0] ImmSrcD;                                       // Select type of immediate extension
  logic [1:0] FlagsE;                                        // Comparison flags ({eq, lt})
  logic       ALUSrcAE, ALUSrcBE;                            // ALU source operands
  logic [2:0] ResultSrcW;                                    // Selects result in Writeback stage
  logic       ALUResultSrcE;                                 // Selects ALU result to pass on to Memory stage
  logic [2:0] ALUSelectE;                                    // ALU select mux signal
  logic       FWriteIntM;                                    // FPU writing to integer register file
  logic       IntDivW;                                       // Integer divide instruction
  logic [3:0] BSelectE;                                      // Indicates if ZBA_ZBB_ZBC_ZBS instruction in one-hot encoding
  logic [3:0] ZBBSelectE;                                    // ZBB Result Select Signal in Execute Stage
  logic [2:0] BALUControlE;                                  // ALU Control signals for B instructions in Execute Stage
  logic       SubArithE;                                     // Subtraction or arithmetic shift
  logic       UW64E;                                         // .uw-type instruction

  logic [6:0] Funct7E;

  // Forwarding signals
  logic [4:0] Rs1D, Rs2D;
  logic [4:0] Rs2E;                                          // Source registers

  // Slot 1 (superscalar) Decode-stage outputs: the same signals the controller produces in D for slot 0.
  // Slot 1 is decoded alongside slot 0 but does not issue yet, so these are observed by the testbench only.
  logic [4:0] Rs1_2D, Rs2_2D;                                // Slot 1 source registers
  logic [2:0] ImmSrc2D;                                      // Slot 1 immediate format
  logic       IllegalBaseInstr2D;                            // Slot 1 is an illegal base instruction
  logic       Branch2D, Jump2D;                              // Slot 1 is a branch / jump
  logic       InstrValid2D;                                  // Slot 1 controller valid bit (not yet qualified by the IFU's Instr2ValidD)
  logic       StructuralStall0D;                             // Slot 0's own structural hazards
  logic       StructuralStall2D;                             // Slot 1 depends on a result not ready to forward
  logic       RegWriteM;                                     // Slot 0 writes a register in Memory
  logic       RegWrite2M, RegWrite2W;                        // Slot 1 writes a register in Memory / Writeback
  logic [4:0] Rd2M, Rd2W;                                    // Slot 1 destination register in Memory / Writeback

  // Slot 1 Execute-stage signals.  Slot 1 computes a result but does not commit it yet.
  logic [4:0] Rs1_2E, Rs2_2E;                                // Slot 1 source registers in Execute
  logic [2:0] Forward2AE, Forward2BE;                        // Lane 2 forwarding selects (from controller c2)
  logic       ALUSrcA2E, ALUSrcB2E, ALUResultSrc2E;          // Slot 1 ALU operand and result selects
  logic [2:0] ALUSelect2E;                                   // Slot 1 ALU operation
  logic [2:0] Funct3_2E;                                     // Slot 1 funct3
  logic [6:0] Funct7_2E;                                     // Slot 1 funct7
  logic       W64_2E, UW64_2E, SubArith2E;                   // Slot 1 W-type, .uw-type, subtract/arithmetic-shift
  logic       Jump2E;                                        // Slot 1 is a jump (always 0 under the issue rules)
  logic [3:0] BSelect2E, ZBBSelect2E;                        // Slot 1 bit-manipulation selects
  logic [2:0] BALUControl2E;                                 // Slot 1 bit-manipulation ALU control
  logic       BMUActive2E;                                   // Slot 1 bit-manipulation instruction active
  logic [1:0] CZero2E;                                       // Slot 1 czero.* active
  logic [2:0] ForwardAE, ForwardBE;                          // Select signals for forwarding multiplexers
  logic       RegWriteW;                                     // Register will be written in Writeback stage
  logic       BranchSignedE;                                 // Branch does signed comparison on operands
  logic       BMUActiveE;                                    // Bit manipulation instruction being executed
  logic [1:0] CZeroE;                                        // {czero.nez, czero.eqz} instructions active

  controller #(P) c(
    .clk, .reset, .StallD, .FlushD, .InstrD, .STATUS_FS, .ENVCFG_CBE, .ImmSrcD,
    .IllegalIEUFPUInstrD, .IllegalBaseInstrD,
    .StructuralStallD(StructuralStall0D), .LoadStallD, .StoreStallD, .Rs1D, .Rs2D, .Rs2E,
    .Rs1_2D, .Rs2_2D, .StructuralStall2D,                     // lane 2's hazard, computed here against the real RdE
    .StallE, .FlushE, .FlagsE, .FWriteIntE,
    .PCSrcE, .ALUSrcAE, .ALUSrcBE, .ALUResultSrcE, .ALUSelectE,
    .Funct3E, .Funct7E, .IntDivE, .W64E, .UW64E, .SubArithE, .BranchD, .BranchE, .JumpD, .JumpE,
    .BranchSignedE, .BSelectE, .ZBBSelectE, .BALUControlE, .BMUActiveE, .CZeroE, .MDUActiveE,
    .FCvtIntE, .ForwardAE, .ForwardBE, .CMOpM, .IFUPrefetchE, .LSUPrefetchM,
    .StallM, .FlushM, .MemRWE, .MemRWM, .CSRReadM, .CSRWriteM, .PrivilegedM, .AtomicM, .Funct3M,
    .FlushDCacheM, .InstrValidM, .InstrValidE, .InstrValidD, .FWriteIntM,
    .RegWriteM, .RegWriteOtherM(RegWrite2M),                  // lane 1 forwards from lane 2
    .StallW, .FlushW, .RegWriteW, .IntDivW, .ResultSrcW, .CSRWriteFenceM, .InvalidateICacheM,
    .RegWriteOtherW(RegWrite2W),
    .RdW, .RdE, .RdM, .RdOtherM(Rd2M), .RdOtherW(Rd2W));

  // Slot 1 controller (superscalar).  A second copy of the controller decodes Instr2D in the Decode stage.
  // Only its Decode-stage outputs are used; slot 1 does not issue, so its Execute/Memory/Writeback
  // outputs are left unconnected and its Execute-stage feedback inputs are tied off.
  controller #(P) c2(
    .clk, .reset, .StallD, .FlushD, .InstrD(Instr2D), .STATUS_FS, .ENVCFG_CBE, .ImmSrcD(ImmSrc2D),
    .IllegalIEUFPUInstrD(1'b0), .IllegalBaseInstrD(IllegalBaseInstr2D),
    // The load-use hazard cannot be computed here: it depends on the Execute-stage control of the
    // real pipeline (MemReadE, CSRReadE, MDUE), which only c has.  c computes it for both lanes.
    .StructuralStallD(), .LoadStallD(), .StoreStallD(),
    .Rs1_2D(5'b0), .Rs2_2D(5'b0), .StructuralStall2D(),
    .Rs1D(Rs1_2D), .Rs2D(Rs2_2D), .Rs2E(Rs2_2E),
    // An unpaired slot 1 is flushed on its way into Execute rather than being allowed down the
    // pipeline and suppressed at each consumer.  Its control signals, RegWrite2M/W and Rd2M/W then
    // come out as zero on their own, so nothing downstream needs to know about pairing.
    .StallE, .FlushE(FlushE | ~Issue2D), .FlagsE(2'b00), .FWriteIntE(1'b0),
    .PCSrcE(), .ALUSrcAE(ALUSrcA2E), .ALUSrcBE(ALUSrcB2E), .ALUResultSrcE(ALUResultSrc2E), .ALUSelectE(ALUSelect2E),
    .Funct3E(Funct3_2E), .Funct7E(Funct7_2E), .IntDivE(), .W64E(W64_2E), .UW64E(UW64_2E), .SubArithE(SubArith2E),
    .BranchD(Branch2D), .BranchE(), .JumpD(Jump2D), .JumpE(Jump2E),
    .BranchSignedE(), .BSelectE(BSelect2E), .ZBBSelectE(ZBBSelect2E), .BALUControlE(BALUControl2E),
    .BMUActiveE(BMUActive2E), .CZeroE(CZero2E), .MDUActiveE(),
    .FCvtIntE(1'b0), .ForwardAE(Forward2AE), .ForwardBE(Forward2BE), .CMOpM(), .IFUPrefetchE(), .LSUPrefetchM(),
    .StallM, .FlushM, .MemRWE(), .MemRWM(), .CSRReadM(), .CSRWriteM(), .PrivilegedM(), .AtomicM(), .Funct3M(),
    .FlushDCacheM(), .InstrValidM(), .InstrValidE(), .InstrValidD(InstrValid2D), .FWriteIntM(),
    .RegWriteM(RegWrite2M), .RegWriteOtherM(RegWriteM),       // lane 2 forwards from lane 1
    .StallW, .FlushW, .RegWriteW(RegWrite2W), .IntDivW(), .ResultSrcW(), .CSRWriteFenceM(), .InvalidateICacheM(),
    .RegWriteOtherW(RegWriteW),
    .RdW(Rd2W), .RdE(), .RdM(Rd2M), .RdOtherM(RdM), .RdOtherW(RdW));

  // Stall the bundle when slot 1 depends on a result that cannot be forwarded yet, exactly as slot 0
  // already does.  Gated by Issue2D so a slot 1 hazard costs nothing on cycles where the two slots
  // were never going to issue together.
  assign StructuralStallD = StructuralStall0D | (Issue2D & StructuralStall2D);

  datapath #(P) dp(
    .clk, .reset, .ImmSrcD, .InstrD, .Rs1D, .Rs2D, .Rs2E, .StallE, .FlushE, .ForwardAE, .ForwardBE, .W64E, .UW64E, .SubArithE,
    .ImmSrc2D, .Instr2D, .Rs1_2D, .Rs2_2D, .Rs2_2E, .Forward2AE, .Forward2BE,                       // slot 1
    .ALUSrcA2E, .ALUSrcB2E, .ALUResultSrc2E, .ALUSelect2E, .Funct3_2E, .Funct7_2E,                  // slot 1
    .W64_2E, .UW64_2E, .SubArith2E, .BSelect2E, .ZBBSelect2E, .BALUControl2E, .BMUActive2E, .CZero2E, // slot 1
    .RegWrite2W, .Rd2W,                                                                             // slot 1 write port
    .Funct3E, .Funct7E, .ALUSrcAE, .ALUSrcBE, .ALUResultSrcE, .ALUSelectE, .JumpE, .BranchSignedE,
    .PCE, .PCLinkE, .FlagsE, .IEUAdrE, .ForwardedSrcAE, .ForwardedSrcBE, .BSelectE, .ZBBSelectE, .BALUControlE, .BMUActiveE, .CZeroE,
    .StallM, .FlushM, .FWriteIntM, .FIntResM, .SrcAM, .WriteDataM, .FCvtIntW,
    .StallW, .FlushW, .RegWriteW, .IntDivW, .SquashSCW, .ResultSrcW, .ReadDataW, .FCvtIntResW,
    .CSRReadValW, .MDUResultW, .FIntDivResultW, .RdW);
endmodule
