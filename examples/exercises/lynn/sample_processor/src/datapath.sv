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
// 16-entry tagless BTB + 16-entry correlating PHT with 3-bit GHR.
logic [2:0] GHR;
logic [2:0] GHR_snapD;
logic [2:0] GHR_snapE;
logic [1:0] PHT [15:0];   // 16-entry PHT, indexed by PC[5:2]^GHR


// Fetch Stage
logic [31:0] PCF, InstrF, PCPlus4F, PCNextF;
logic        PredictedTakenF;


// Decode Stage
logic [31:0] PCD, PCPlus4D;
logic [31:0] RD1D, RD2D, ImmExtD, CSRSrcDataD;
logic [4:0]  Rs1D, Rs2D, RdD;
logic [2:0]  funct3D;
logic [11:0] CSRAddrD;
logic        PredictedTakenD;
logic        RegWriteD, ALUSrcD, MemWriteD, LoadD, CSRWriteD, BranchD, JumpD;
logic [2:0]  ResultSrcD, ImmSrcD;
logic [3:0]  ALUControlD;

// JAL decode-stage prediction signals
logic        isJALD;           // instruction in decode is JAL
logic [31:0] JALTargetD;       // JAL target = PCD + ImmExtD
logic        JALPredictD;      // asserted when we redirect PC for JAL at decode
logic        JALPredictE;      // pipelined to execute for mispredict check


// Execute Stage
logic [31:0] RD1E, RD2E, PCE, PCPlus4E, ImmExtE;
logic [31:0] CSRSrcDataE;
logic [31:0] SrcAE, SrcBE, SrcBEIntermediate, ALUResultE, WriteDataE, PCTargetE, MulDivResultE;
logic [4:0]  Rs1E, Rs2E, RdE;
logic [2:0]  funct3E;
logic [11:0] CSRAddrE;
logic        ZeroE, NegativeE, OverflowE, CarryE;
logic        BranchTakenE, PredictedTakenE, BranchMispredictE;
logic        RegWriteE, ALUSrcE, MemWriteE, LoadE, CSRWriteE, BranchE, JumpE;
logic [2:0]  ResultSrcE;
logic [3:0]  ALUControlE;
logic [1:0]  PCSrcE;


// Memory Stage
logic [31:0] ALUResultM, WriteDataM, PCPlus4M, MulDivResultM;
logic [31:0] oldCSRReadDataM, newCSRWriteDataM, CSRSrcDataM;
logic [31:0] ReadDataM;
logic [4:0]  RdM;
logic [2:0]  funct3M;
logic [11:0] CSRAddrM;
logic [31:0] ResultM;
logic        RegWriteM, MemWriteM, LoadM, CSRWriteM;
logic [2:0]  ResultSrcM;
logic [3:0]  ALUControlM;


// Writeback Stage
logic [31:0] ALUResultW, AdjustedReadDataW, PCPlus4W, MulDivResultW;
logic [31:0] oldCSRReadDataW, newCSRWriteDataW;
logic [31:0] ReadDataW;
logic [31:0] ResultW;
logic [4:0]  RdW;
logic [11:0] CSRAddrW;
logic [2:0]  funct3W;
logic        RegWriteW, CSRWriteW;
logic [2:0]  ResultSrcW;


// Hazard Unit signals
logic        StallF, StallD, FlushD_exe, FlushE;
logic [1:0]  ForwardAE, ForwardBE;
HazardUnit hazardunit(
   .Rs1D(Rs1D), .Rs2D(Rs2D),
   .Rs1E(Rs1E), .Rs2E(Rs2E), .RdE(RdE),
   .ResultSrcM(ResultSrcM),
   .LoadE(LoadE),
   .RegWriteE(RegWriteE),
   .PCSrcE(PCSrcE),
   .RdM(RdM),
   .RegWriteM(RegWriteM),
   .RdW(RdW),
   .RegWriteW(RegWriteW),
   .StallF(StallF),
   .StallD(StallD),
   .FlushD(FlushD_exe),
   .FlushE(FlushE),
   .ForwardAE(ForwardAE),
   .ForwardBE(ForwardBE)
);

logic FlushD;
// also flush fetch when JAL is predicted at decode
assign FlushD = (PCSrcE != 2'b00) | (JALPredictD & ~StallD);


// ============================================================
// FETCH STAGE
// ============================================================
logic [31:0] entry_addr;
initial begin
   entry_addr = '0;
   void'($value$plusargs("ENTRY_ADDR=%h", entry_addr));
   $display("[TB] ENTRY_ADDR = 0x%h", entry_addr);
end

assign InstrF = Instr;
flopr_en_reset #(32) pcreg(clk, reset, ~StallF, entry_addr, PCNextF, PCF);
adder pcadd4(PCF, 32'd4, PCPlus4F);


// BTB: Branch Target Buffer (16-entry tagless)
logic [11:0] BTB_Tag    [15:0];
logic [31:0] BTB_Target [15:0];
logic        BTB_Valid  [15:0];
logic [3:0]  BTB_Index;
logic        BTB_Hit;
logic [31:0] BTB_PredictedTarget;

assign BTB_Index           = PCF[5:2];
assign BTB_Hit             = ~reset && BTB_Valid[BTB_Index] && (BTB_Tag[BTB_Index] == PCF[17:6]);
assign BTB_PredictedTarget = BTB_Target[BTB_Index];


// PHT: 16-entry (2,1) correlating predictor, indexed by PC[5:2]^GHR
logic [3:0] PHT_Index;
assign PHT_Index = PCF[5:2] ^ {1'b0, GHR};   // 4-bit: 1 pad + 3-bit GHR

assign PredictedTakenF = ~reset && BTB_Hit && PHT[PHT_Index][1];


// PC selection
logic [1:0] PCSelect;
logic [31:0] PCMux3In;

always_comb begin
 if (PCSrcE != 2'b00) begin
   if (PCSrcE == 2'b11)
     PCMux3In = PCPlus4E;
   PCSelect = PCSrcE;
 end else if (JALPredictD & ~StallD) begin
   // redirect to JAL target one cycle earlier than execute resolution
   // JALTargetD = PCD + ImmExtD, computed in decode
   PCMux3In = JALTargetD;
   PCSelect = 2'b11;   // use mux input 3 (PCMux3In)
 end else if (BTB_Hit && PredictedTakenF) begin
   PCMux3In = BTB_PredictedTarget;
   PCSelect = 2'b11;
 end else begin
   PCMux3In = PCPlus4F;
   PCSelect = 2'b00;
 end
end

mux4 #(32) pcmux(PCPlus4F, PCTargetE, {ALUResultE[31:1], 1'b0}, PCMux3In, PCSelect, PCNextF);


// Fetch → Decode pipeline registers
flopr_en_flush #(32) IF_ID_PC(clk, reset, ~StallD, FlushD, PCF, PCD);
flopr_en_flush #(32) IF_ID_PCPlus4(clk, reset, ~StallD, FlushD, PCPlus4F, PCPlus4D);
flopr_en_flush #(32) IF_ID_Instr(clk, reset, ~StallD, FlushD, InstrF, InstrD);
flopr_en_flush #(1)  IF_ID_BranchPred(clk, reset, ~StallD, FlushD, PredictedTakenF, PredictedTakenD);
flopr_en_flush #(3)  IF_ID_GHR(clk, reset, ~StallD, FlushD, GHR, GHR_snapD);


// ============================================================
// DECODE
// ============================================================
assign Rs1D    = InstrD[19:15];
assign Rs2D    = InstrD[24:20];
assign RdD     = InstrD[11:7];
assign funct3D = InstrD[14:12];
assign CSRAddrD = InstrD[31:20];

assign RegWriteD  = RegWrite;
assign ImmSrcD    = ImmSrc;
assign ALUSrcD    = ALUSrc;
assign MemWriteD  = MemWrite;
assign LoadD      = Load;
assign ResultSrcD = ResultSrc;
assign ALUControlD = ALUControl;
assign CSRWriteD  = CSRWrite;
assign BranchD    = Branch;
assign JumpD      = Jump;

// JAL detection and target computation at decode
// need to distinguish JAL from JALR (ALUSrc=1)
assign isJALD    = JumpD & ~ALUSrcD;
assign JALTargetD = PCD + ImmExtD;
// Only predict if not already stalled and not being flushed from execute
assign JALPredictD = isJALD & ~(PCSrcE != 2'b00);


RegFile RF(clk, RegWriteW, Rs1D, Rs2D, RdW, ResultW, RD1D, RD2D);
Extend Ext(InstrD[31:7], ImmSrcD, ImmExtD);
mux2 #(32) CSRSrcMux(RD1D, ImmExtD, InstrD[14], CSRSrcDataD);


// Decode → Execute pipeline registers
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
flopr_en_flush #(32) ID_EX_CSRSrcData(clk, reset, 1'b1, FlushE, CSRSrcDataD, CSRSrcDataE);
flopr_en_flush #(1)  ID_EX_BranchPred(clk, reset, 1'b1, FlushE, PredictedTakenD, PredictedTakenE);
flopr_en_flush #(3)  ID_EX_GHR(clk, reset, 1'b1, FlushE, GHR_snapD, GHR_snapE);
flopr_en_flush #(1)  ID_EX_JALPredict(clk, reset, 1'b1, FlushE, JALPredictD, JALPredictE);

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


// ============================================================
// EXECUTE
// ============================================================
assign WriteDataE = SrcBEIntermediate;
logic take_branchE;

adder      pcaddbranch(PCE, ImmExtE, PCTargetE);
mux3 #(32) SrcAmux(RD1E, ResultW, ResultM, ForwardAE, SrcAE);
mux3 #(32) SrcBmuxPrev(RD2E, ResultW, ResultM, ForwardBE, SrcBEIntermediate);
mux2 #(32) SrcBmux(SrcBEIntermediate, ImmExtE, ALUSrcE, SrcBE);
alu        ALU(SrcAE, SrcBE, ALUControlE, ALUResultE);
mulDiv     mulDiv(SrcAE, SrcBE, funct3E, MulDivResultE);

logic eq_E, lt_signed_E, lt_unsig_E;
comparator comparator(SrcAE, SrcBE, {eq_E, lt_signed_E, lt_unsig_E});

logic [31:0] ALUResultPipeE;
always_comb begin
case (ResultSrcE)
 3'b100:  ALUResultPipeE = PCTargetE;
 3'b011:  ALUResultPipeE = ImmExtE;
 default: ALUResultPipeE = ALUResultE;
endcase
end

always_comb begin
case(funct3E)
 3'b000: take_branchE = eq_E;
 3'b001: take_branchE = ~eq_E;
 3'b100: take_branchE = lt_signed_E;
 3'b101: take_branchE = ~lt_signed_E;
 3'b110: take_branchE = lt_unsig_E;
 3'b111: take_branchE = ~lt_unsig_E;
 default: take_branchE = 0;
endcase
end

assign BranchTakenE      = BranchE & take_branchE;
assign BranchMispredictE = BranchE && (PredictedTakenE != BranchTakenE);

logic [3:0] indexE;
assign indexE = PCE[5:2] ^ {1'b0, GHR_snapE};

always_comb begin
 if (BranchMispredictE & BranchTakenE)   PCSrcE = 2'b01;
 else if (BranchMispredictE & ~BranchTakenE) PCSrcE = 2'b11;
 else if (JumpE) begin
   if (~ALUSrcE & JALPredictE) PCSrcE = 2'b00; // JAL already handled at decode
   else if (~ALUSrcE)          PCSrcE = 2'b01; // JAL not predicted at decode, redirect now
   else                        PCSrcE = 2'b10; // JALR always resolved at execute
 end else begin
   PCSrcE = 2'b00;
 end
end

// GHR is 3 bits
flopr_en #(3) EX_MEM_GHRUpdate(clk, reset, BranchE, {GHR[1:0], BranchTakenE}, GHR);

// Update PHT
always_ff @(posedge clk) begin
 if (reset) begin
   // init to 2'b01 (weakly not-taken)
   for (int i = 0; i < 16; i++)
       PHT[i] <= 2'b01;
 end else if (BranchE & !BranchTakenE) begin
   if (PHT[indexE] != 2'b00) PHT[indexE] <= PHT[indexE] - 1;
 end else if (BranchE & BranchTakenE) begin
   if (PHT[indexE] != 2'b11) PHT[indexE] <= PHT[indexE] + 1;
 end
end

// Update BTB (all branches — needed so not-taken→taken transitions are captured)
always_ff @(posedge clk) begin
 if (reset) begin
   for (int i = 0; i < 16; i++) begin
     BTB_Valid[i]  <= 1'b0;
     BTB_Tag[i]    <= 12'h0;
     BTB_Target[i] <= 32'h0;
   end
 end else if (BranchE) begin
   BTB_Tag[PCE[5:2]]    <= PCE[17:6];
   BTB_Target[PCE[5:2]] <= PCTargetE;
   BTB_Valid[PCE[5:2]]  <= 1'b1;
 end
end


// Execute → Memory pipeline registers
flopr_en #(32) EX_MEM_PCPlus4(clk, reset, 1'b1, PCPlus4E, PCPlus4M);
flopr_en #(32) EX_MEM_ALUResult(clk, reset, 1'b1, ALUResultPipeE, ALUResultM);
flopr_en #(32) EX_MEM_MulDivResult(clk, reset, 1'b1, MulDivResultE, MulDivResultM);
flopr_en #(32) EX_MEM_WriteData(clk, reset, 1'b1, WriteDataE, WriteDataM);
flopr_en #(5)  EX_MEM_Rd(clk, reset, 1'b1, RdE, RdM);
flopr_en #(3)  EX_MEM_funct3(clk, reset, 1'b1, funct3E, funct3M);
flopr_en #(12) EX_MEM_CSRAddr(clk, reset, 1'b1, CSRAddrE, CSRAddrM);
flopr_en #(32) EX_MEM_CSRSrcData(clk, reset, 1'b1, CSRSrcDataE, CSRSrcDataM);

flopr_en #(1) EX_MEM_RegWrite(clk, reset, 1'b1, RegWriteE, RegWriteM);
flopr_en #(1) EX_MEM_Load(clk, reset, 1'b1, LoadE, LoadM);
flopr_en #(1) EX_MEM_MemWrite(clk, reset, 1'b1, MemWriteE, MemWriteM);
flopr_en #(1) EX_MEM_CSRWrite(clk, reset, 1'b1, CSRWriteE, CSRWriteM);
flopr_en #(3) EX_MEM_ResultSrc(clk, reset, 1'b1, ResultSrcE, ResultSrcM);
flopr_en #(4) EX_MEM_ALUControl(clk, reset, 1'b1, ALUControlE, ALUControlM);


// MEMORY
assign ReadDataM = ReadData;

logic IncrementInstret, IncrementAdd, IncrementBranch, IncrementBranchTaken;
logic IncrementLoads, IncrementStores, IncrementStalls, IncrementFlushes, BranchMisPrediction;

assign IncrementInstret    = RegWriteM || MemWriteM || LoadM;
assign IncrementAdd        = (ALUControlM == 4'b0000) && RegWriteM;
assign IncrementBranch     = BranchE;
assign IncrementBranchTaken = BranchTakenE;
assign BranchMisPrediction  = BranchMispredictE;
assign IncrementLoads       = LoadM;
assign IncrementStores      = MemWriteM;
assign IncrementStalls      = StallD;
assign IncrementFlushes     = FlushE;

csrfile csrfile(
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
 .IncrementBranchTaken(IncrementBranchTaken),
 .BranchMisPrediction(BranchMisPrediction),
 .IncrementLoads(IncrementLoads),
 .IncrementStores(IncrementStores),
 .IncrementStalls(IncrementStalls),
 .IncrementFlushes(IncrementFlushes)
);

csrData #(32) csrData(oldCSRReadDataM, CSRSrcDataM, funct3M, newCSRWriteDataM);

// ADD THIS:
logic [31:0] ResultM_Forward;  // For forwarding only
mux7 #(32) ResultMmux(
   ALUResultM,  // 000
   ReadDataM,   // 001 raw load data for M-stage forwarding (load-use stall means consumer is never in EX when load is in M, so raw value is safe)
   PCPlus4M,    // 010
   ALUResultM,  // 011 LUI: folded into ALUResult at E/M boundary
   ALUResultM,  // 100 AUIPC: folded into ALUResult at E/M boundary
   MulDivResultM, // 101
   32'b0, // 110
   ResultSrcM,
   ResultM
);



// Memory → Writeback pipeline registers
flopr_en #(32) MEM_WB_PCPlus4(clk, reset, 1'b1, PCPlus4M, PCPlus4W);
flopr_en #(32) MEM_WB_ALUResult(clk, reset, 1'b1, ALUResultM, ALUResultW);
flopr_en #(32) MEM_WB_MulDivResult(clk, reset, 1'b1, MulDivResultM, MulDivResultW);
flopr_en #(32) MEM_WB_oldCSRReadData(clk, reset, 1'b1, oldCSRReadDataM, oldCSRReadDataW);
flopr_en #(32) MEM_WB_newCSRWriteData(clk, reset, 1'b1, newCSRWriteDataM, newCSRWriteDataW);
flopr_en #(32) MEM_WB_ReadData(clk, reset, 1'b1, ReadDataM, ReadDataW);
flopr_en #(3)  MEM_WB_funct3(clk, reset, 1'b1, funct3M, funct3W);
flopr_en #(5)  MEM_WB_Rd(clk, reset, 1'b1, RdM, RdW);
flopr_en #(12) MEM_WB_CSRAddr(clk, reset, 1'b1, CSRAddrM, CSRAddrW);

flopr_en #(1) MEM_WB_RegWrite(clk, reset, 1'b1, RegWriteM, RegWriteW);
flopr_en #(1) MEM_WB_CSRWrite(clk, reset, 1'b1, CSRWriteM, CSRWriteW);
flopr_en #(3) MEM_WB_ResultSrc(clk, reset, 1'b1, ResultSrcM, ResultSrcW);


// WRITEBACK
loadUnit #(32) loadUnit(ReadDataW, ALUResultW[1:0], funct3W, AdjustedReadDataW);
mux7 #(32) Resultmux(ALUResultW, AdjustedReadDataW, PCPlus4W, ALUResultW, ALUResultW, MulDivResultW, oldCSRReadDataW, ResultSrcW, ResultW);

assign PC         = PCF;
assign ALUResult  = ALUResultM;
assign WriteData  = WriteDataM;
assign MemWriteOut = MemWriteM;
assign LoadOut    = LoadM;
assign Funct3Out  = funct3M;
endmodule
