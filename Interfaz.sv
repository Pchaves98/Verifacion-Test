interface Bus_Intfz(input bit clk)
  logic reset;
  logic pndng [bits-1:0][drvrs-1:0];
  
  logic push[bits-1:0][drvrs-1:0];
  logic pop[bits-1:0][drvrs-1:0];
  
  logic [pckg_sz-1:0] D_pop[bits-1:0][drvrs-1:0];
  logic [pckg_sz-1:0] D_push[bits-1:0][drvrs-1:0];
 
endinterface

