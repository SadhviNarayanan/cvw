///////////////////////////////////////////
// issue.sv
//
// Written: September 2026
//
// Purpose: Decide whether the two instructions in the fetch window may issue together.
//
//          This runs in the Fetch stage because the PC advance is computed there: when both
//          slots issue the PC must move by both their lengths in the same cycle.  Everything it
//          needs is a fixed bit field of an uncompressed instruction, so no decode is required.
//
//          The rule is a whitelist for pairing, not for execution.  When Issue2F is low the core
//          behaves exactly as the original single-issue Wally: slot 0 issues by itself and slot 1
//          is fetched again next cycle as the next slot 0.  No instruction is ever skipped.
//
//          Slot 1 is restricted to instructions that are legal in every configuration, cannot
//          raise an exception, and need nothing but a second ALU -- no memory, no CSR, no
//          multiply/divide, no control transfer.
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

module issue (
  input  logic [31:0] Instr0F,        // Slot 0 raw instruction (at PCF)
  input  logic [31:0] Instr1F,        // Slot 1 raw instruction, already aligned to bit 0
  input  logic        Instr2ValidF,   // Slot 1 holds a complete instruction from the I$
  output logic        Issue2F         // Both slots may issue together this cycle
);

  // Opcodes
  localparam [6:0] LUI      = 7'b0110111;
  localparam [6:0] AUIPC    = 7'b0010111;
  localparam [6:0] OP       = 7'b0110011;
  localparam [6:0] OP_IMM   = 7'b0010011;
  localparam [6:0] BRANCH   = 7'b1100011;
  localparam [6:0] JAL      = 7'b1101111;
  localparam [6:0] JALR     = 7'b1100111;
  localparam [6:0] SYSTEM   = 7'b1110011;
  localparam [6:0] MISC_MEM = 7'b0001111;

  logic [6:0] Op0F, Op1F, Funct7_1F;
  logic [2:0] Funct3_1F;
  logic [4:0] Rd0F, Rs1_1F, Rs2_1F, Rd1F;
  logic       BothUncompressedF, Slot0CanPairF, Slot1CanPairF, NoDependencyF;

  assign Op0F      = Instr0F[6:0];
  assign Rd0F      = Instr0F[11:7];
  assign Op1F      = Instr1F[6:0];
  assign Funct3_1F = Instr1F[14:12];
  assign Funct7_1F = Instr1F[31:25];
  assign Rs1_1F    = Instr1F[19:15];
  assign Rs2_1F    = Instr1F[24:20];
  assign Rd1F      = Instr1F[11:7];

  // Both instructions must be 32-bit.  A compressed instruction would need its decompressor to
  // know its length and register fields, and those live in the Decode stage.
  assign BothUncompressedF = (&Instr0F[1:0]) & (&Instr1F[1:0]);

  // Which instructions may carry a partner in slot 1.  These all still execute normally on their
  // own when they cannot pair.
  //   branch/jal/jalr: slot 1 is the fall-through, so a taken transfer would leave it on the wrong
  //     path.  Pairing these needs a way to kill slot 1 in Execute; that is a later step.
  //   SYSTEM: mret and sret redirect the PC from the Memory stage without flushing Writeback, so a
  //     slot 1 sharing that stage would commit after the return had already been taken.  This one
  //     cannot be fixed by killing slot 1 later -- it is already past the point of no return.
  //   MISC_MEM: fence.i flushes the pipeline behind it.
  assign Slot0CanPairF = (Op0F != BRANCH) & (Op0F != JAL) & (Op0F != JALR) &
                         (Op0F != SYSTEM) & (Op0F != MISC_MEM);

  // Slot 1 must be an instruction a plain ALU can finish in one cycle and that is legal in every
  // configuration, so Fetch never has to know which extensions are enabled:
  //   lui, auipc
  //   addi, slti, sltiu, xori, ori, andi        (shifts excluded: their immediates depend on XLEN)
  //   add, sll, slt, sltu, xor, srl, or, and    (funct7 = 0000000)
  //   sub, sra                                  (funct7 = 0100000)
  // Requiring these exact funct7 values also excludes mul/div (funct7 = 0000001), clmul, AES, SHA,
  // and every Zba/Zbb/Zbs/Zicond encoding, all of which share the OP opcode.
  always_comb
    case (Op1F)
      LUI, AUIPC: Slot1CanPairF = 1'b1;
      OP_IMM:     Slot1CanPairF = (Funct3_1F != 3'b001) & (Funct3_1F != 3'b101);
      OP:         Slot1CanPairF = (Funct7_1F == 7'b0000000) |
                                  ((Funct7_1F == 7'b0100000) & ((Funct3_1F == 3'b000) | (Funct3_1F == 3'b101)));
      default:    Slot1CanPairF = 1'b0;
    endcase

  // Slot 1 must not depend on slot 0.  Forwarding slot 0's result to slot 1 within the same cycle
  // would chain one ALU into the other and roughly double the critical path, so such pairs simply
  // issue one at a time.  Rd0F is treated as a destination whenever it is nonzero, which is
  // conservative for stores (where those bits are part of the immediate) but never unsafe.
  assign NoDependencyF = (Rd0F == 5'b0) |
                         ((Rs1_1F != Rd0F) & (Rs2_1F != Rd0F) & (Rd1F != Rd0F));

  assign Issue2F = Instr2ValidF & BothUncompressedF & Slot0CanPairF & Slot1CanPairF & NoDependencyF;

endmodule
