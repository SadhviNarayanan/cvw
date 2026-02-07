// riscvsingle.sv
// RISC-V single-cycle processor
// David_Harris@hmc.edu 2020


module datapath(input  logic clk, reset,
        input  logic [2:0]  ResultSrc,
        input  logic        MemWrite,
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
        output logic [2:0]  Funct3Out,
        input  logic [31:0] ReadData,
        output logic [31:0] InstrD);

  // Fetch Stage
  logic [31:0] PCF, InstrF, PCPlus4F, PCNextF;

  // Decode Stage
  logic [31:0] PCD, PCPlus4D; // InstrD --> output
  logic [31:0] RD1D, RD2D, ImmExtD, oldCSRReadDataD, CSRSrcDataD;
  logic [4:0]  Rs1D, Rs2D, RdD;
  logic [2:0]  funct3D;
  logic [11:0] CSRAddrD;
  // Control signals (from controller - we'll connect these later)
  logic        RegWriteD, ALUSrcD, MemWriteD, CSRWriteD, BranchD, JumpD;
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
  // Control signals
  logic        RegWriteE, ALUSrcE, MemWriteE, CSRWriteE, BranchE, JumpE;
  logic [2:0]  ResultSrcE;
  logic [3:0]  ALUControlE;
  logic [1:0]  PCSrcE;

  // Memory Stage
  logic [31:0] ALUResultM, WriteDataM, PCPlus4M, PCTargetM, ImmExtM;
  logic [31:0] MulDivResultM, oldCSRReadDataM, newCSRWriteDataM;
  logic [31:0] ReadDataM, AdjustedReadDataM;
  logic [4:0]  RdM;
  logic [2:0]  funct3M;
  logic [11:0] CSRAddrM;
  // Control signals
  logic        RegWriteM, MemWriteM, CSRWriteM;
  logic [2:0]  ResultSrcM;

  // Writeback Stage
  logic [31:0] ALUResultW, AdjustedReadDataW, PCPlus4W, PCTargetW, ImmExtW;
  logic [31:0] MulDivResultW, oldCSRReadDataW, newCSRWriteDataW; // do i need writedata? isnt that just for csr reg file --> YES bc i have csr reg file now too
  logic [31:0] ResultW;
  logic [4:0]  RdW;
  logic [11:0] CSRAddrW;
  // Control signals
  logic        RegWriteW, CSRWriteW;
  logic [2:0]  ResultSrcW;


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
  assign InstrF = Instr;
  flopr_en #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
  adder       pcadd4(PCF, 32'd4, PCPlus4F);
  mux3 #(32)  pcmux(PCPlus4F, PCTargetE, {ALUResultE[31:1], 1'b0}, PCSrcE, PCNextF); // need to clear last bit of addrses for jalr


  // register Fetch --> Decode
  // if stall = 0 (we dont stall), enable the flops to load next values
  flopr_en_flush #(32) IF_ID_PC(clk, reset, ~StallD, FlushD, PCF, PCD);
  flopr_en_flush #(32) IF_ID_PCPlus4(clk, reset, ~StallD, FlushD, PCPlus4F, PCPlus4D);
  flopr_en_flush #(32) IF_ID_Instr(clk, reset, ~StallD, FlushD, InstrF, InstrD);


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
  // TODO: need to make dual ported for a write address coming from writeback stage
  CsrRegFile  CsrRegFile(clk, CSRWriteW, CSRAddrD, CSRAddrW, newCSRWriteDataW,
                 oldCSRReadDataD);

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
  flopr_en_flush #(32) ID_EX_oldCSRReadData(clk, reset, 1'b1, FlushE, oldCSRReadDataD, oldCSRReadDataE);
  flopr_en_flush #(32) ID_EX_CSRSrcData(clk, reset, 1'b1, FlushE, CSRSrcDataD, CSRSrcDataE);

  // Control signals
  flopr_en_flush #(1) ID_EX_RegWrite(clk, reset, 1'b1, FlushE, RegWriteD, RegWriteE);
  flopr_en_flush #(1) ID_EX_ALUSrc(clk, reset, 1'b1, FlushE, ALUSrcD, ALUSrcE);
  flopr_en_flush #(1) ID_EX_MemWrite(clk, reset, 1'b1, FlushE, MemWriteD, MemWriteE);
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
  csrData #(32)  csrData(oldCSRReadDataE, CSRSrcDataE, funct3E, newCSRWriteDataE);
  // ALU logic
  mux3 #(32)  SrcAmux(RD1E, ResultW, ALUResultM, ForwardAE, SrcAE);
  mux3 #(32)  SrcBmuxPrev(RD2E, ResultW, ALUResultM, ForwardBE, SrcBEIntermediate);
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

  always_comb begin
    if (BranchE & take_branchE) PCSrcE = 2'b01;
    else if (JumpE) begin
      if (~ALUSrcE) PCSrcE = 2'b01; // jal changed if from op == 7'b1101111
      else PCSrcE = 2'b10; // jalr
    end else begin
      PCSrcE = 2'b00;
    end
  end


  // register step Execute --> Memory (TODO: rename flops)

  // Data signals
  flopr_en #(32) EX_MEM_PCPlus4(clk, reset, 1'b1, PCPlus4E, PCPlus4M);
  flopr_en #(32) EX_MEM_PCTarget(clk, reset, 1'b1, PCTargetE, PCTargetM);
  flopr_en #(32) EX_MEM_ImmExt(clk, reset, 1'b1, ImmExtE, ImmExtM);
  flopr_en #(32) EX_MEM_oldCSRReadData(clk, reset, 1'b1, oldCSRReadDataE, oldCSRReadDataM);
  flopr_en #(32) EX_MEM_newCSRWriteData(clk, reset, 1'b1, newCSRWriteDataE, newCSRWriteDataM);
  flopr_en #(32) EX_MEM_ALUResult(clk, reset, 1'b1, ALUResultE, ALUResultM);
  flopr_en #(32) EX_MEM_WriteData(clk, reset, 1'b1, WriteDataE, WriteDataM);
  flopr_en #(32) EX_MEM_MulDivResult(clk, reset, 1'b1, MulDivResultE, MulDivResultM);
  flopr_en #(5)  EX_MEM_Rd(clk, reset, 1'b1, RdE, RdM);
  flopr_en #(3)  EX_MEM_funct3(clk, reset, 1'b1, funct3E, funct3M);
  flopr_en #(12) EX_MEM_CSRAddr(clk, reset, 1'b1, CSRAddrE, CSRAddrM);

  // Control signals
  flopr_en #(1) EX_MEM_RegWrite(clk, reset, 1'b1, RegWriteE, RegWriteM);
  flopr_en #(1) EX_MEM_MemWrite(clk, reset, 1'b1, MemWriteE, MemWriteM);
  flopr_en #(1) EX_MEM_CSRWrite(clk, reset, 1'b1, CSRWriteE, CSRWriteM);
  flopr_en #(3) EX_MEM_ResultSrc(clk, reset, 1'b1, ResultSrcE, ResultSrcM);



  // MEMORY
  assign ReadDataM = ReadData;
  loadUnit #(32) loadUnit(ReadDataM, ALUResultM[1:0], funct3M, AdjustedReadDataM);


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


  // WRITEBACK
  mux7 #(32)  Resultmux(ALUResultW, AdjustedReadDataW, PCPlus4W, ImmExtW, PCTargetW, MulDivResultW, oldCSRReadDataW, ResultSrcW, ResultW);


  // update signals
  assign PC = PCF;
  assign ALUResult = ALUResultM;  // For dmem address
  assign WriteData = WriteDataM;  // For dmem write data
  assign MemWriteOut = MemWriteM;  // for dmem
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
