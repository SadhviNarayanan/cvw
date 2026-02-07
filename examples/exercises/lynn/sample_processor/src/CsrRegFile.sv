module CsrRegFile(input  logic        clk,
               input  logic        WE3,
               input  logic [11:0] A1,
               input  logic [11:0] A2,
               input  logic [31:0] WD3,
               output logic [31:0] RD1);

  logic [31:0] csrRf[4095:0] = '{default: 32'h0};

  // singlle ported register file
  // read/write same input port combinationally
  // register 0 hardwired to 0

  always_ff @(posedge clk)
    if (WE3) csrRf[A2] <= WD3;

  assign RD1 = csrRf[A1];
endmodule
