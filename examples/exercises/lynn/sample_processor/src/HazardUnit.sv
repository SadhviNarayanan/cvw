module HazardUnit(
    // Decode stage sources (this lane)
    input  logic [4:0]  Rs1D, Rs2D,

    // Execute stage (this lane)
    input  logic [4:0]  Rs1E, Rs2E, RdE,
    input  logic        ResultSrcE_b0,   // 1 = load (for load-use stall)
    input  logic        Load,
    input  logic        RegWriteE,

    // PC correction from branch/jump unit
    input  logic [1:0]  PCSrcE,
    input  logic        BranchMispredictE,

    // Same-lane M / W stage
    input  logic [4:0]  RdM,
    input  logic        RegWriteM,
    input  logic [4:0]  RdW,
    input  logic        RegWriteW,

    // Cross-lane M / W stage (from the other instruction slot)
    input  logic [4:0]  RdM_other,
    input  logic        RegWriteM_other,
    input  logic [4:0]  RdW_other,
    input  logic        RegWriteW_other,

    // Decode sources of the OTHER lane (for cross-lane load-use stall)
    input  logic [4:0]  Rs1D_other, Rs2D_other,

    output logic        StallF,
    output logic        StallD, FlushD,
    output logic        FlushE,

    // 3-bit forward select encoding:
    //   3'b000 = register file (no forward)
    //   3'b001 = same-lane  W-stage result
    //   3'b010 = same-lane  M-stage result
    //   3'b011 = cross-lane W-stage result
    //   3'b100 = cross-lane M-stage result
    output logic [2:0]  ForwardAE, ForwardBE
);

    // ----------------------------------------------------------------
    // Forwarding (SrcA / Rs1E)
    // Priority: same-lane M > cross-lane M > same-lane W > cross-lane W
    // ----------------------------------------------------------------
    always_comb begin
        if      (Rs1E != 0 && RegWriteM_other && Rs1E == RdM_other) ForwardAE = 3'b100; // cross-lane M
        else if (Rs1E != 0 && RegWriteM       && Rs1E == RdM)       ForwardAE = 3'b010; // same-lane  M
        else if (Rs1E != 0 && RegWriteW_other && Rs1E == RdW_other) ForwardAE = 3'b011; // cross-lane W
        else if (Rs1E != 0 && RegWriteW       && Rs1E == RdW)       ForwardAE = 3'b001; // same-lane  W
        else                                                          ForwardAE = 3'b000; // reg file
    end

    // ----------------------------------------------------------------
    // Forwarding (SrcB / Rs2E)
    // ----------------------------------------------------------------
    always_comb begin
        if      (Rs2E != 0 && RegWriteM_other && Rs2E == RdM_other) ForwardBE = 3'b100;
        else if (Rs2E != 0 && RegWriteM       && Rs2E == RdM)       ForwardBE = 3'b010;
        else if (Rs2E != 0 && RegWriteW_other && Rs2E == RdW_other) ForwardBE = 3'b011;
        else if (Rs2E != 0 && RegWriteW       && Rs2E == RdW)       ForwardBE = 3'b001;
        else                                                          ForwardBE = 3'b000;
    end

    // ----------------------------------------------------------------
    // Load-use stall
    // Covers both same-lane and cross-lane consumers.
    // ----------------------------------------------------------------
    logic lwStall;
    assign lwStall = Load && RegWriteE && (RdE != 5'b0) &&
                     ((RdE == Rs1D)       || (RdE == Rs2D) ||
                      (RdE == Rs1D_other) || (RdE == Rs2D_other));

    assign StallF = lwStall;
    assign StallD = lwStall;

    // ----------------------------------------------------------------
    // Control hazard
    // ----------------------------------------------------------------
    assign FlushD = (PCSrcE != 2'b00);
    assign FlushE = lwStall | (PCSrcE != 2'b00);

endmodule
