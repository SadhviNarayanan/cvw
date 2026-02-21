module mux4 #(parameter WIDTH = 8)
  (input  logic [WIDTH-1:0] d0, d1, d2, d3,
   input  logic [1:0]       s,
   output logic [WIDTH-1:0] y);

  always_comb
    case(s)
      3'b00: y = d0;
      3'b01: y = d1;
      3'b10: y = d2;
      3'b11: y = d3;
      default: y = d0;
    endcase
endmodule
