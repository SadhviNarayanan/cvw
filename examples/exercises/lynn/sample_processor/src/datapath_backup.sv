// riscvsingle.sv
// RISC-V single-cycle processor
// David_Harris@hmc.edu 2020


module datapath(input  logic clk, reset,
        input  logic [2:0]  ResultSrc,
        input  logic        MemWrite,
        input  logic        Load,
        input  logic        ALUSrc,
        input  logic        RegWrite,
        input  logic [2:0]  ImmSrc,
        input  logic [3:0]  ALUControl,
        input  logic        CSRWrite,
        input  logic        Jump,
        input  logic        Branch,
        output logic [31:0] PC,
        input  logic [31:0] Instr,
        output logic [31:0] ALUResult, WriteData,
        output logic        MemWriteOut,
        output logic        LoadOut,
        output logic [2:0]  Funct3Out,
        input  logic [31:0] ReadData,
        output logic [31:0] InstrD);

  // Branch Prediction
  logic [7:0] GHR;  // Global History Register (8 bits)
  logic [1:0] PHT [255:0];  // Pattern History Table (256 x 2-bit counters)

  // Fetch Stage
  logic [31:0] PCF, InstrF, PCPlus4F, PCNextF;
  logic        isBranchF;
  logic        PredictedTakenF; // if its 10 or 11 then its taken, so essentially we only care about bit 1 to check if taken or not, but 0 is just for certainty

  // Decode Stage
  logic [31:0] PCD, PCPlus4D; // InstrD --> output
  logic [31:0] RD1D, RD2D, ImmExtD, oldCSRReadDataD, CSRSrcDataD;
  logic [4:0]  Rs1D, Rs2D, RdD;
  logic [2:0]  funct3D;
  logic [11:0] CSRAddrD;
  logic        PredictedTakenD;
  // Control signals (from controller - we'll connect these later)
  logic        RegWriteD, ALUSrcD, MemWriteD, LoadD,CSRWriteD, BranchD, JumpD;
  logic [2:0]  ResultSrcD, ImmSrcD;
  logic [3:0]  ALUControlD;

  // Execute Stage
  logic [31:0] RD1E, RD2E, PCE, PCPlus4E, ImmExtE;
  logic [31:0] oldCSRReadDataE, CSRSrcDataE, newCSRWriteDataE;
  logic [31:0] SrcAE, SrcBE, SrcBEIntermediate, ALUResultE, WriteDataE, PCTargetE, MulDivResultE;
  logic [4:0]  Rs1E, Rs2E, RdE;
  logic [2:0]  funct3E;
  logic [11:0] CSRAddrE;
  logic        ZeroE, NegativeE, OverflowE, CarryE;
  logic        BranchTakenE, PredictedTakenE, BranchMispredictE;
  // Control signals
  logic        RegWriteE, ALUSrcE, MemWriteE, LoadE, CSRWriteE, BranchE, JumpE;
  logic [2:0]  ResultSrcE;
  logic [3:0]  ALUControlE;
  logic [1:0]  PCSrcE;

  // Memory Stage
  logic [31:0] ALUResultM, WriteDataM, PCPlus4M, PCTargetM, ImmExtM;
  logic [31:0] MulDivResultM, oldCSRReadDataM, newCSRWriteDataM, CSRSrcDataM;
  logic [31:0] ReadDataM, AdjustedReadDataM;
  logic [4:0]  RdM;
  logic [2:0]  funct3M;
  logic [11:0] CSRAddrM;
  logic [31:0] ResultM;
  // Control signals
  logic        RegWriteM, MemWriteM, LoadM, CSRWriteM;
  logic [2:0]  ResultSrcM;
  logic [3:0]  ALUControlM;

  // Writeback Stage
  logic [31:0] ALUResultW, AdjustedReadDataW, PCPlus4W, PCTargetW, ImmExtW;
  logic [31:0] MulDivResultW, oldCSRReadDataW, newCSRWriteDataW; // do i need writedata? isnt that just for csr reg file --> YES bc i have csr reg file now too
  logic [31:0] ResultW;
  logic [4:0]  RdW;
  logic [11:0] CSRAddrW;
  // Control signals
  logic        RegWriteW, CSRWriteW;
  logic [2:0]  ResultSrcW;
  logic [3:0]  ALUControlW;


    // Hazard Unit signals
    logic        StallF, StallD, FlushD, FlushE;
    logic [1:0]  ForwardAE, ForwardBE;
    HazardUnit hazardunit(
        .Rs1D(Rs1D), .Rs2D(Rs2D),
        .Rs1E(Rs1E), .Rs2E(Rs2E), .RdE(RdE),
        .ResultSrcE_b0(ResultSrcE[0]),
        .RegWriteE(RegWriteE),
        .PCSrcE(PCSrcE),
        .RdM(RdM),
        .RegWriteM(RegWriteM),
        .RdW(RdW),
        .RegWriteW(RegWriteW),
        .StallF(StallF),
        .StallD(StallD),
        .FlushD(FlushD),
        .FlushE(FlushE),
        .ForwardAE(ForwardAE),
        .ForwardBE(ForwardBE)
    );


  // FETCH
  // next PC logic
  logic [31:0] entry_addr;
  initial begin
      // default
      entry_addr = '0;

      // override if provided
      void'($value$plusargs("ENTRY_ADDR=%h", entry_addr));

      $display("[TB] ENTRY_ADDR = 0x%h", entry_addr);
  end

  assign InstrF = Instr;
  flopr_en_reset #(32) pcreg(clk, reset, ~StallF, entry_addr, PCNextF, PCF);
  adder       pcadd4(PCF, 32'd4, PCPlus4F);

  // ADDING BRANCH PREDICTION LOGIC - TODO: check
  logic [31:0] BranchTargetPred, TargetPCBranch;
  logic [7:0] BranchIndex;

  assign isBranchF = (InstrF[6:0] == 7'b1100011);
  assign BranchTargetPred = {{20{InstrF[31]}}, InstrF[7], InstrF[30:25], InstrF[11:8], 1'b0};
  assign BranchIndex = PCF[9:2] ^ GHR;
  assign PredictedTakenF = PHT[BranchIndex][1]; // again, only care about bit 1 for taken, bit 0 - certainty
  // assign TargetPCBranch = PCF + BranchTargetPred;

  // PC selection with priority
  logic [1:0] PCSelect;
  always_comb begin
    TargetPCBranch = PCF + BranchTargetPred;
    if (PCSrcE != 2'b00) begin// this means that we have calcualted we need to branch from execute, so this takes priority bc we went down wrong path essentially, we shdl not be continutin down this path if execxute says we need to branch (this means we did not predict correctly and execute stage caught us)
        if (PCSrcE == 2'b11) begin // if PCSrcE is 11, this means we mispredicted and said take branch, but it should have been no, so we need to go back to PCE + 4 instead of PCTargetE. // if PCSrcE is 11, this means we mispredicted and said take branch, but it should have been no, so we need to go back to PCE + 4 instead of PCTargetE. (Hazard unit will take care of flush if PCSrcE != 0)
          TargetPCBranch = PCPlus4E;
        end
          PCSelect = PCSrcE;          // Execute overrides (misprediction recovery, jumps)
    end else if (isBranchF && PredictedTakenF)
          PCSelect = 2'b11;           // Predicted taken
      else
          PCSelect = 2'b00;           // Normal PC+4 --> same this --> PCSrcE or 2'b00
  end

  mux4 #(32)  pcmux(PCPlus4F, PCTargetE, {ALUResultE[31:1], 1'b0}, TargetPCBranch, PCSelect, PCNextF); // need to clear last bit of addrses for jalr
  // mux3 #(32)  pcmux(PCPlus4F, PCTargetE, {ALUResultE[31:1], 1'b0}, PCSrcE, PCNextF);

  // register Fetch --> Decode
  // if stall = 0 (we dont stall), enable the flops to load next values
  flopr_en_flush #(32) IF_ID_PC(clk, reset, ~StallD, FlushD, PCF, PCD);
  flopr_en_flush #(32) IF_ID_PCPlus4(clk, reset, ~StallD, FlushD, PCPlus4F, PCPlus4D);
  flopr_en_flush #(32) IF_ID_Instr(clk, reset, ~StallD, FlushD, InstrF, InstrD);
  flopr_en_flush #(1) IF_ID_BranchPred(clk, reset, ~StallD, FlushD, PredictedTakenF, PredictedTakenD);


  // DECODE
  // Extract fields from instruction
  assign Rs1D = InstrD[19:15];
  assign Rs2D = InstrD[24:20];
  assign RdD = InstrD[11:7];
  assign funct3D = InstrD[14:12];
  assign CSRAddrD = InstrD[31:20];

  assign RegWriteD = RegWrite;
  assign ImmSrcD = ImmSrc;
  assign ALUSrcD = ALUSrc;
  assign MemWriteD = MemWrite;
  assign LoadD = Load;
  assign ResultSrcD = ResultSrc;
  assign ALUControlD = ALUControl;
  assign CSRWriteD = CSRWrite;
  assign BranchD = Branch;
  assign JumpD = Jump;


  // register file logic
  RegFile     RF(clk, RegWriteW, Rs1D, Rs2D,
                 RdW, ResultW, RD1D, RD2D);
  Extend      Ext(InstrD[31:7], ImmSrcD, ImmExtD);
  mux2 #(32)  CSRSrcMux(RD1D, ImmExtD, InstrD[14], CSRSrcDataD); // use top bit of funct3 to decide if imm or srcA is used

  // TODO: the pnly thing i changed from working to now, was commented this out and added the logiv below for csr
  // CsrRegFile  CsrRegFile(clk, CSRWriteW, CSRAddrD, CSRAddrW, newCSRWriteDataW,
  //                oldCSRReadDataD);

  // register step Decode --> Execute (TODO: rename flops)

  // Data signals
  flopr_en_flush #(32) ID_EX_PC(clk, reset, 1'b1, FlushE, PCD, PCE);
  flopr_en_flush #(32) ID_EX_RD1(clk, reset, 1'b1, FlushE, RD1D, RD1E);
  flopr_en_flush #(32) ID_EX_RD2(clk, reset, 1'b1, FlushE, RD2D, RD2E);
  flopr_en_flush #(5)  ID_EX_Rs1(clk, reset, 1'b1, FlushE, Rs1D, Rs1E);
  flopr_en_flush #(5)  ID_EX_Rs2(clk, reset, 1'b1, FlushE, Rs2D, Rs2E);
  flopr_en_flush #(5)  ID_EX_Rd(clk, reset, 1'b1, FlushE, RdD, RdE);
  flopr_en_flush #(3)  ID_EX_funct3(clk, reset, 1'b1, FlushE, funct3D, funct3E);
  flopr_en_flush #(12) ID_EX_CSRAddr(clk, reset, 1'b1, FlushE, CSRAddrD, CSRAddrE);
  flopr_en_flush #(32) ID_EX_ImmExt(clk, reset, 1'b1, FlushE, ImmExtD, ImmExtE);
  flopr_en_flush #(32) ID_EX_PCPlus4(clk, reset, 1'b1, FlushE, PCPlus4D, PCPlus4E);
  // flopr_en_flush #(32) ID_EX_oldCSRReadData(clk, reset, 1'b1, FlushE, oldCSRReadDataD, oldCSRReadDataE);
  flopr_en_flush #(32) ID_EX_CSRSrcData(clk, reset, 1'b1, FlushE, CSRSrcDataD, CSRSrcDataE);
  flopr_en_flush #(1)  ID_EX_BranchPred(clk, reset, 1'b1, FlushE, PredictedTakenD, PredictedTakenE);

  // Control signals
  flopr_en_flush #(1) ID_EX_RegWrite(clk, reset, 1'b1, FlushE, RegWriteD, RegWriteE);
  flopr_en_flush #(1) ID_EX_ALUSrc(clk, reset, 1'b1, FlushE, ALUSrcD, ALUSrcE);
  flopr_en_flush #(1) ID_EX_MemWrite(clk, reset, 1'b1, FlushE, MemWriteD, MemWriteE);
  flopr_en_flush #(1) ID_EX_Load(clk, reset, 1'b1, FlushE, LoadD, LoadE);
  flopr_en_flush #(1) ID_EX_CSRWrite(clk, reset, 1'b1, FlushE, CSRWriteD, CSRWriteE);
  flopr_en_flush #(1) ID_EX_Branch(clk, reset, 1'b1, FlushE, BranchD, BranchE);
  flopr_en_flush #(1) ID_EX_Jump(clk, reset, 1'b1, FlushE, JumpD, JumpE);
  flopr_en_flush #(3) ID_EX_ResultSrc(clk, reset, 1'b1, FlushE, ResultSrcD, ResultSrcE);
  flopr_en_flush #(4) ID_EX_ALUControl(clk, reset, 1'b1, FlushE, ALUControlD, ALUControlE);


  // EXECUTE (TODO: no hazard unit yet)
  // Extract fields from instruction
  // assign SrcAE = RD1E;
  assign WriteDataE = SrcBEIntermediate;
  // PCSrc logic (moved from controller to Execute stage)
  logic take_branchE;

  adder       pcaddbranch(PCE, ImmExtE, PCTargetE);
  // csrData #(32)  csrData(oldCSRReadDataE, CSRSrcDataE, funct3E, newCSRWriteDataE);
  // ALU logic
  mux3 #(32)  SrcAmux(RD1E, ResultW, ResultM, ForwardAE, SrcAE);
  mux3 #(32)  SrcBmuxPrev(RD2E, ResultW, ResultM, ForwardBE, SrcBEIntermediate);
  mux2 #(32)  SrcBmux(SrcBEIntermediate, ImmExtE, ALUSrcE, SrcBE);
  alu         ALU(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE, NegativeE, OverflowE, CarryE);
  mulDiv      mulDiv(SrcAE, SrcBE, funct3E, MulDivResultE);

  always_comb begin
    case(funct3E)
      3'b000: take_branchE = (ZeroE == 1);
      3'b001: take_branchE = (ZeroE == 0);
      3'b100: take_branchE = (NegativeE ^ OverflowE);
      3'b101: take_branchE = (ZeroE == 1) | ~(NegativeE ^ OverflowE);
      3'b110: take_branchE = (CarryE == 0); // borrow is needed
      3'b111: take_branchE = (CarryE == 1) | (ZeroE == 1); // borrow not needed so have carry
      default: take_branchE = 0;
    endcase
  end

  assign BranchTakenE = BranchE & take_branchE;
  assign BranchMispredictE = BranchE && (PredictedTakenE != BranchTakenE);

  logic [7:0] indexE;

  always_comb begin
    if (BranchMispredictE & BranchTakenE) PCSrcE = 2'b01; // only branch and let fetech stage know if it was a mispredicton, othwewise we woudl have already taken the correct branch if we predicted correctly
    else if (BranchMispredictE & ~BranchTakenE) PCSrcE = 2'b11; // if we predicted take branch, but it it actually not take, then we need to jump to original PCE+4, and flush (Hazard unit will take care of this if PCSrcE != 00)
    else if (JumpE) begin
      if (~ALUSrcE) PCSrcE = 2'b01; // jal changed if from op == 7'b1101111
      else                  PCSrcE = 2'b10; // jalr
    end else begin
      PCSrcE = 2'b00;
    end
  end

  // update GHR with new branch status, check if matched prediction, update table

  // update GHR
  flopr_en #(8) EX_MEM_GHRUpdate(clk, reset, BranchE, {GHR[6:0],BranchTakenE}, GHR); // GHR will reflect new value in memory stage
  // update PHT
  assign indexE = PCE[9:2] ^ GHR;
  always_ff @(posedge clk) begin
    if (reset) begin
        // Initialize on reset
        for (int i = 0; i < 256; i++)
            PHT[i] <= 2'b01;
    end else if (BranchE & !BranchTakenE) begin
      if (PHT[indexE] != 2'b00) PHT[indexE] <= PHT[indexE] - 1;
    end else if (BranchE & BranchTakenE) begin
      if (PHT[indexE] != 2'b11) PHT[indexE] <= PHT[indexE] + 1;
    end
  end

  // register step Execute --> Memory (TODO: rename flops)

  // Data signals
  flopr_en #(32) EX_MEM_PCPlus4(clk, reset, 1'b1, PCPlus4E, PCPlus4M);
  flopr_en #(32) EX_MEM_PCTarget(clk, reset, 1'b1, PCTargetE, PCTargetM);
  flopr_en #(32) EX_MEM_ImmExt(clk, reset, 1'b1, ImmExtE, ImmExtM);
  // flopr_en #(32) EX_MEM_oldCSRReadData(clk, reset, 1'b1, oldCSRReadDataE, oldCSRReadDataM);
  // flopr_en #(32) EX_MEM_newCSRWriteData(clk, reset, 1'b1, newCSRWriteDataE, newCSRWriteDataM);
  flopr_en #(32) EX_MEM_ALUResult(clk, reset, 1'b1, ALUResultE, ALUResultM);
  flopr_en #(32) EX_MEM_WriteData(clk, reset, 1'b1, WriteDataE, WriteDataM);
  flopr_en #(32) EX_MEM_MulDivResult(clk, reset, 1'b1, MulDivResultE, MulDivResultM);
  flopr_en #(5)  EX_MEM_Rd(clk, reset, 1'b1, RdE, RdM);
  flopr_en #(3)  EX_MEM_funct3(clk, reset, 1'b1, funct3E, funct3M);
  flopr_en #(12) EX_MEM_CSRAddr(clk, reset, 1'b1, CSRAddrE, CSRAddrM);
  flopr_en #(32) EX_MEM_CSRSrcData(clk, reset, 1'b1, CSRSrcDataE, CSRSrcDataM);

  // Control signals
  flopr_en #(1) EX_MEM_RegWrite(clk, reset, 1'b1, RegWriteE, RegWriteM);
  flopr_en #(1) EX_MEM_Load(clk, reset, 1'b1, LoadE, LoadM);
  flopr_en #(1) EX_MEM_MemWrite(clk, reset, 1'b1, MemWriteE, MemWriteM);
  flopr_en #(1) EX_MEM_CSRWrite(clk, reset, 1'b1, CSRWriteE, CSRWriteM);
  flopr_en #(3) EX_MEM_ResultSrc(clk, reset, 1'b1, ResultSrcE, ResultSrcM);
  flopr_en #(4) EX_MEM_ALUControl(clk, reset, 1'b1, ALUControlE, ALUControlM);



  // MEMORY
  assign ReadDataM = ReadData;
  loadUnit #(32) loadUnit(ReadDataM, ALUResultM[1:0], funct3M, AdjustedReadDataM);

  // Performance counter increment signals
  logic IncrementInstret, IncrementAdd, IncrementBranch, IncrementBranchTaken;

  // signals for csr reg stuff, might need to come back to check
  assign IncrementInstret = RegWriteM || MemWriteM || LoadM;                          // From Writeback
  assign IncrementAdd = (ALUControlW == 4'b0000) && RegWriteW;  // From Writeback
  assign IncrementBranch = BranchE;                             // From Execute
  assign IncrementBranchTaken = BranchE && (PCSrcE == 2'b01);   // From Execute

  CsrRegFile CsrRegFile(
   .clk(clk),
   .reset(reset),
   .WE3(CSRWriteW),
   .A1(CSRAddrM),
   .A2(CSRAddrW),
   .WD3(newCSRWriteDataW),
   .RD1(oldCSRReadDataM),
   .IncrementCycle(1'b1),
   .IncrementInstret(IncrementInstret),
   .IncrementAdd(IncrementAdd),
   .IncrementBranch(IncrementBranch),
   .IncrementBranchTaken(IncrementBranchTaken)
 );

  // because we are reading on the same cycle, timing should be fine for oldCSRReadDataM
  // CSRSrcDataM is the immediate value for csr instructions from execute
  csrData #(32)  csrData(oldCSRReadDataM, CSRSrcDataM, funct3M, newCSRWriteDataM);

  // ADD THIS:

  mux7 #(32) ResultMmux(
      ALUResultM,        // 000
      AdjustedReadDataM, // 001
      PCPlus4M,          // 010
      ImmExtM,           // 011
      PCTargetM,         // 100 ← AUIPC result
      MulDivResultM,     // 101
      oldCSRReadDataM,   // 110
      ResultSrcM,
      ResultM
  );

  // register step Memory --> Writeback (TODO: rename flops)
  // register step Memory --> Writeback
  // Data signals
  flopr_en #(32) MEM_WB_PCPlus4(clk, reset, 1'b1, PCPlus4M, PCPlus4W);
  flopr_en #(32) MEM_WB_PCTarget(clk, reset, 1'b1, PCTargetM, PCTargetW);
  flopr_en #(32) MEM_WB_ImmExt(clk, reset, 1'b1, ImmExtM, ImmExtW);
  flopr_en #(32) MEM_WB_ALUResult(clk, reset, 1'b1, ALUResultM, ALUResultW);
  flopr_en #(32) MEM_WB_MulDivResult(clk, reset, 1'b1, MulDivResultM, MulDivResultW);
  flopr_en #(32) MEM_WB_oldCSRReadData(clk, reset, 1'b1, oldCSRReadDataM, oldCSRReadDataW);
  flopr_en #(32) MEM_WB_newCSRWriteData(clk, reset, 1'b1, newCSRWriteDataM, newCSRWriteDataW);
  flopr_en #(32) MEM_WB_AdjustedReadData(clk, reset, 1'b1, AdjustedReadDataM, AdjustedReadDataW);
  flopr_en #(5)  MEM_WB_Rd(clk, reset, 1'b1, RdM, RdW);
  flopr_en #(12) MEM_WB_CSRAddr(clk, reset, 1'b1, CSRAddrM, CSRAddrW);

  // Control signals
  flopr_en #(1) MEM_WB_RegWrite(clk, reset, 1'b1, RegWriteM, RegWriteW);
  flopr_en #(1) MEM_WB_CSRWrite(clk, reset, 1'b1, CSRWriteM, CSRWriteW);
  flopr_en #(3) MEM_WB_ResultSrc(clk, reset, 1'b1, ResultSrcM, ResultSrcW);
  flopr_en #(4) MEM_WB_ALUControl(clk, reset, 1'b1, ALUControlM, ALUControlW);


  // WRITEBACK
  mux7 #(32)  Resultmux(ALUResultW, AdjustedReadDataW, PCPlus4W, ImmExtW, PCTargetW, MulDivResultW, oldCSRReadDataW, ResultSrcW, ResultW);


  // update signals
  assign PC = PCF;
  assign ALUResult = ALUResultM;  // For dmem address
  assign WriteData = WriteDataM;  // For dmem write data
  assign MemWriteOut = MemWriteM;  // for dmem
  assign LoadOut = LoadM;
  assign Funct3Out = funct3M;      // for dmem
endmodule


// module datapath(
//         input   logic           clk, reset,
//         input   logic [2:0]     Funct3,
//         input   logic           ALUResultSrc, ResultSrc,
//         input   logic [1:0]     ALUSrc,
//         input   logic           RegWrite,
//         input   logic [1:0]     ImmSrc,
//         input   logic [1:0]     ALUControl,
//         output  logic           Eq,
//         input   logic [31:0]    PC, PCPlus4,
//         input   logic [31:0]    Instr,
//         output  logic [31:0]    IEUAdr, WriteData,
//         input   logic [31:0]    ReadData
//     );

//     logic [31:0] ImmExt;
//     logic [31:0] R1, R2, SrcA, SrcB;
//     logic [31:0] ALUResult, IEUResult, Result;

//     // register file logic
//     regfile rf(.clk, .WE3(RegWrite), .A1(Instr[19:15]), .A2(Instr[24:20]),
//         .A3(Instr[11:7]), .WD3(Result), .RD1(R1), .RD2(R2));

//     extend ext(.Instr(Instr[31:7]), .ImmSrc, .ImmExt);

//     // ALU logic
//     cmp cmp(.R1, .R2, .Eq);

//     mux2 #(32) srcamux(R1, PC, ALUSrc[1], SrcA);
//     mux2 #(32) srcbmux(R2, ImmExt, ALUSrc[0], SrcB);

//     alu alu(.SrcA, .SrcB, .ALUControl, .Funct3, .ALUResult, .IEUAdr);

//     mux2 #(32) ieuresultmux(ALUResult, PCPlus4, ALUResultSrc, IEUResult);
//     mux2 #(32) resultmux(IEUResult, ReadData, ResultSrc, Result);

//     assign WriteData = R2;
// endmodule
