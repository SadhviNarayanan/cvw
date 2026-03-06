module mulDiv(
    input  logic [31:0] a, b,
    input  logic [2:0]  funct3,
    output logic [31:0] result
);

    logic signed [63:0] product;
    logic signed [32:0] a_ext, b_ext;

    always_comb begin
        // Sign extension based on funct3
        case(funct3)
            3'b000, 3'b001: begin  // MUL, MULH - both signed
                a_ext = {a[31], a};
                b_ext = {b[31], b};
            end
            3'b010: begin  // MULHSU - A signed, B unsigned
                a_ext = {a[31], a};
                b_ext = {1'b0, b};
            end
            3'b011: begin  // MULHU - both unsigned
                a_ext = {1'b0, a};
                b_ext = {1'b0, b};
            end
            default: begin
                a_ext = {a[31], a};
                b_ext = {b[31], b};
            end
        endcase

        // Single multiply
        product = a_ext * b_ext;

        // Output selection
        case(funct3)
            3'b000: result = product[31:0];   // MUL - lower 32 bits
            3'b001: result = product[63:32];  // MULH - upper 32 bits
            3'b010: result = product[63:32];  // MULHSU - upper 32 bits
            3'b011: result = product[63:32];  // MULHU - upper 32 bits
            default: result = 32'h0;
        endcase
    end

endmodule
