module HazardUnit(
    input  logic [4:0]  Rs1D, Rs2D,
    input  logic [4:0]  Rs1E, Rs2E, RdE,
    input  logic [2:0]  ResultSrcM,
    input  logic        LoadE,
    input  logic        RegWriteE,
    input  logic [1:0]  PCSrcE,
    input  logic [4:0]  RdM,
    input  logic        RegWriteM,
    input  logic [4:0]  RdW,
    input  logic        RegWriteW,
    output logic        StallF, StallD, FlushD, FlushE,
    output logic [1:0]  ForwardAE, ForwardBE
);

    // Forwarding logic - SIMPLE is FASTER
    always_comb begin
        if ((Rs1E == RdM) && RegWriteM && (|Rs1E) && (ResultSrcM != 3'b110))
            ForwardAE = 2'b10;
        else if ((Rs1E == RdW) && RegWriteW && (|Rs1E))
            ForwardAE = 2'b01;
        else
            ForwardAE = 2'b00;
    end

    always_comb begin
        if ((Rs2E == RdM) && RegWriteM && (|Rs2E) && (ResultSrcM != 3'b110))
            ForwardBE = 2'b10;
        else if ((Rs2E == RdW) && RegWriteW && (|Rs2E))
            ForwardBE = 2'b01;
        else
            ForwardBE = 2'b00;
    end

    // Stall detection - inline everything
    logic lwStall, csrStall;

    assign lwStall = LoadE && (|RdE) && ((RdE == Rs1D) || (RdE == Rs2D));

    assign csrStall = RegWriteM && (|RdM) && (ResultSrcM == 3'b110) &&
                      ((Rs1D == RdM) || (Rs2D == RdM));

    // Outputs - simple OR gates
    assign StallF = lwStall | csrStall;
    assign StallD = lwStall | csrStall;
    assign FlushD = (PCSrcE != 2'b00);
    assign FlushE = lwStall | csrStall | (PCSrcE != 2'b00);

endmodule
