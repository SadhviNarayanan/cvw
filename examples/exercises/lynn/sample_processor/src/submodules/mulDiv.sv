module mulDiv(input  logic [31:0] a, b,
              input  logic [2:0]  funct3,
            output logic [31:0] result);

  logic [63:0] product, a_ext, b_ext;

  always_comb begin
    case(funct3)
      3'b000: begin  // MUL
        result = a * b;
      end

      3'b001: begin  // MULH - signed × signed
        a_ext = {{32{a[31]}}, a};
        b_ext = {{32{b[31]}}, b};
        product = $signed(a_ext) * $signed(b_ext);
        result = product[63:32];
      end

      3'b010: begin  // MULHSU - signed × unsigned
        a_ext = {{32{a[31]}}, a};
        b_ext = {32'b0, b};
        product = $signed(a_ext) * $unsigned(b_ext);
        result = product[63:32];
      end

      3'b011: begin  // MULHU - unsigned × unsigned
        a_ext = {32'b0, a};
        b_ext = {32'b0, b};
        product = $unsigned(a_ext) * $unsigned(b_ext);
        result = product[63:32];
      end


      3'b100: begin  // DIV
        result = $signed(a) / $signed(b);
      end

      3'b101: begin  // DIVU - unsigned
        result = a / b;
      end

      3'b110: begin  // REM
        result = $signed(a) % $signed(b);
      end

      3'b111: begin  // REMU - unsigned
        result = a % b;
      end
    endcase
  end

endmodule
