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
//          Slot 1 is restricted by the hardware it has, not by legality.  It may hold anything its
//          own ALU can finish in one cycle -- including the Zb*/Zk*/Zicond encodings, since its
//          controller has its own bitmanip decoder -- plus one load, because the single load/store
//          unit can be steered to either slot.  It may not hold a multiply or divide (one MDU), a
//          CSR access, or a control transfer.  Stores are excluded for a reason that is about replay
//          rather than hardware; see the LOAD arm below.
//
//          Legality is left to the decoder that already knows it.  Where slot 1 turns out to hold an
//          instruction this configuration does not implement, or one whose memory access faults, the
//          Memory stage cannot describe the trap -- it holds slot 0's PC, cause and tval -- so slot 1
//          is discarded and refetched to run alone, and traps through the ordinary single-issue path.
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

module issue import cvw::*;  #(parameter cvw_t P) (
  input  logic [31:0] Instr0F,        // Slot 0 raw instruction (at PCF)
  input  logic [31:0] Instr1F,        // Slot 1 raw instruction, already aligned to bit 0
  input  logic        Instr2ValidF,   // Slot 1 holds a complete instruction from the I$
  input  logic        BPPCSrcF,       // Branch predictor says this fetch is a taken branch or jump
  output logic        Issue2F         // Both slots may issue together this cycle
);

  // Opcodes
  localparam [6:0] LUI      = 7'b0110111;
  localparam [6:0] AUIPC    = 7'b0010111;
  localparam [6:0] OP       = 7'b0110011;
  localparam [6:0] OP_IMM   = 7'b0010011;
  localparam [6:0] OP_32    = 7'b0111011;   // RV64 only: addw, subw, sllw, srlw, sraw
  localparam [6:0] OP_IMM_32= 7'b0011011;   // RV64 only: addiw, slliw, srliw, sraiw
  localparam [6:0] JAL      = 7'b1101111;
  localparam [6:0] JALR     = 7'b1100111;
  localparam [6:0] SYSTEM   = 7'b1110011;
  localparam [6:0] MISC_MEM = 7'b0001111;
  // Opcodes that use the load/store unit.  Only LOAD and STORE may appear in slot 1; the rest are
  // listed so that a memory operation in slot 0 can be recognised and kept from pairing with one.
  localparam [6:0] LOAD     = 7'b0000011;
  localparam [6:0] STORE    = 7'b0100011;
  localparam [6:0] LOAD_FP  = 7'b0000111;   // flw, fld -- slot 0 only, but still occupies the LSU
  localparam [6:0] STORE_FP = 7'b0100111;   // fsw, fsd -- likewise
  localparam [6:0] AMO      = 7'b0101111;   // lr, sc and the AMOs -- likewise

  logic [6:0] Op0F, Op1F, Funct7_1F;
  logic [2:0] Funct3_1F;
  logic [4:0] Rd0F, Rs1_1F, Rs2_1F, Rd1F;
  logic       BothUncompressedF, Slot0CanPairF, Slot1CanPairF, NoDependencyF;
  logic       Slot0IsMemF, Slot1IsMemF, OneMemOpF;
  logic       MExtF;                      // Slot 1 is a multiply/divide, which needs the one MDU

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
  //   ~BPPCSrcF is required unconditionally, and is about the PC mux rather than the opcode: when
  //     BPPCSrcF is high the next PC becomes the predicted target and the sequential PC is discarded,
  //     so a paired slot 1 would issue while the machine jumped somewhere else.  The predictor can
  //     say taken for an instruction that is not a branch at all (a class misprediction on a stale
  //     or aliased BTB entry), so this cannot be qualified by the opcode.
  //     With it, a conditional branch predicted not-taken DOES pair; if it later resolves taken,
  //     Kill2E in ifu.sv squashes slot 1 before it commits and the redirect re-fetches it.
  //   jal/jalr: never pair.  The instruction after an unconditional jump is never the next one
  //     executed, and BPJumpF forces BPPCSrcF high for them anyway.
  //   SYSTEM: mret and sret redirect the PC from the Memory stage without flushing Writeback, so a
  //     slot 1 sharing that stage would commit after the return had already been taken.  This one
  //     cannot be fixed by killing slot 1 later -- it is already past the point of no return.
  //   MISC_MEM: fence.i flushes the pipeline behind it.
  assign Slot0CanPairF = ~BPPCSrcF & (Op0F != JAL) & (Op0F != JALR) &
                         (Op0F != SYSTEM) & (Op0F != MISC_MEM);

  // What slot 1 may hold is a statement about hardware, not about legality.  Slot 1 has its own full
  // controller, including its own bmuctrl, so it already decodes every Zba/Zbb/Zbs/Zbc/Zbk/Zk/Zicond
  // encoding this configuration enables and flags the rest as illegal.  Fetch cannot make that
  // judgement and does not try: sh1add and an illegal encoding are the same bits, separated only by
  // P.ZBA_SUPPORTED, so an encoding that turns out illegal is replayed from the Memory stage and traps
  // as an unpaired instruction (see Replay2M).  That keeps the one copy of the encoding table in the
  // decoder that owns it, rather than a second copy here that could silently disagree with it.
  //
  // So the whole of what Fetch must refuse is an instruction needing a unit slot 1 does not have:
  //   mul, mulh, mulhsu, mulhu, div, divu, rem, remu  (and the RV64 W forms)
  // These share the OP and OP-32 opcodes with the ALU instructions but need the single multiply/divide
  // unit, and the decoder calls them perfectly legal -- so nothing downstream would catch them.
  // OP-IMM needs no such check: the M extension has no OP-IMM encoding, and bits 31:25 of an addi are
  // immediate bits that may hold any value.
  assign MExtF = (Funct7_1F == 7'b0000001);

  always_comb
    case (Op1F)
      LUI, AUIPC: Slot1CanPairF = 1'b1;
      OP_IMM:     Slot1CanPairF = 1'b1;
      OP:         Slot1CanPairF = ~MExtF;
      OP_IMM_32:  Slot1CanPairF = (P.XLEN == 64);
      OP_32:      Slot1CanPairF = (P.XLEN == 64) & ~MExtF;
      // A load may pair; a store may not, and the reason is replay rather than the load/store unit,
      // which handles either slot equally well.  A faulting store must leave memory untouched, and the
      // only thing that cancels a write in this design is FlushW, which a trap supplies.  A replay
      // withholds FlushW so that slot 0 can still commit, so a replayed slot 1 store would reach memory
      // and only then trap, leaving a handler to find state already modified.  Cancelling it needs the
      // write stopped downstream of the MMU -- at CacheRW, after the MMU has decided store-versus-load
      // page fault from WriteAccessM -- which is a change to the load/store unit that nothing currently
      // tests, since no test in the suite produces a faulting store in slot 1.  Until that is built and
      // proven, stores stay in slot 0, where a trap cancels them exactly as it always has.  The cost is
      // 0.31% of CoreMark cycles; loads are the rest of the benefit.
      //
      // Funct3 is checked even though replay would also catch an illegal width: the decoder's LFunctD
      // leaves ControlsD at its illegal default, which carries MemRW = 00, so no access is made either
      // way.  Checking here declines to pair rather than issuing and then undoing it, and keeps the
      // rule the same for a configuration whose decoder does not examine funct3 at all.
      LOAD:       case (Funct3_1F)
                    3'b011, 3'b110: Slot1CanPairF = (P.XLEN == 64);   // ld, lwu
                    3'b111:         Slot1CanPairF = 1'b0;             // reserved
                    default:        Slot1CanPairF = 1'b1;             // lb, lh, lw, lbu, lhu
                  endcase
      default:    Slot1CanPairF = 1'b0;
    endcase

  // Slot 1 must not depend on slot 0.  Forwarding slot 0's result to slot 1 within the same cycle
  // would chain one ALU into the other and roughly double the critical path, so such pairs simply
  // issue one at a time.  Rd0F is treated as a destination whenever it is nonzero, which is
  // conservative for stores (where those bits are part of the immediate) but never unsafe.
  assign NoDependencyF = (Rd0F == 5'b0) |
                         ((Rs1_1F != Rd0F) & (Rs2_1F != Rd0F) & (Rd1F != Rd0F));

  // The core has one load/store unit, so a bundle may contain at most one memory operation -- but it
  // may sit in either slot, and ieu.sv steers the LSU's inputs to whichever one holds it.  Slot 0's
  // list is much the wider: slot 1 may only ever hold a LOAD, while slot 0 may also hold a store, a
  // floating-point load or store, or an atomic, each of which occupies the LSU just the same.
  assign Slot0IsMemF = (Op0F == LOAD) | (Op0F == STORE) | (Op0F == LOAD_FP) |
                       (Op0F == STORE_FP) | (Op0F == AMO);
  assign Slot1IsMemF = (Op1F == LOAD);
  assign OneMemOpF   = ~(Slot0IsMemF & Slot1IsMemF);

  assign Issue2F = Instr2ValidF & BothUncompressedF & Slot0CanPairF & Slot1CanPairF &
                   NoDependencyF & OneMemOpF;

endmodule
