module HazardUnit(
                input  logic [4:0]  Rs1D, Rs2D,
                input  logic [4:0]  Rs1E, Rs2E, RdE,
                input  logic        ResultSrcE_b0,
                input  logic        RegWriteE,        // DO I NEED? todo check
                input  logic [1:0]  PCSrcE,
                input  logic [4:0]  RdM,
                input  logic        RegWriteM,
                input  logic [4:0]  RdW,
                input  logic        RegWriteW,
                output logic        StallF,
                output logic        StallD, FlushD,
                output logic        FlushE,
                output  logic [1:0] ForwardAE, ForwardBE);

    // Data hazard logic
    always_comb begin
        if (Rs1E == RdM && RegWriteM && (Rs1E != 0)) ForwardAE = 2'b10; // EX hazard
        else if (Rs1E == RdW && RegWriteW && (Rs1E != 0)) ForwardAE = 2'b01; // MEM hazard
        else ForwardAE = 2'b00;
    end
    always_comb begin
        if (Rs2E == RdM && RegWriteM && (Rs2E != 0)) ForwardBE = 2'b10; // EX hazard
        else if (Rs2E == RdW && RegWriteW && (Rs2E != 0)) ForwardBE = 2'b01; // MEM hazard
        else ForwardBE = 2'b00;
    end

    // Load word stall logic
    logic lwStall;
    assign lwStall = (ResultSrcE_b0 && ((RdE == Rs1D) | (RdE == Rs2D)) && RegWriteE && (RdE != 0)); // do i need regwriteE here?
    assign StallF = lwStall;
    assign StallD = lwStall;

    // Control hazard logic
    assign FlushD = (PCSrcE != 2'b00); // if branch or jump taken, flush decode
    assign FlushE = lwStall | (PCSrcE != 2'b00); // if branch or jump taken, flush execute
endmodule
