class Bus_trans
  rand bit [pckg_sz-1:pckg_sz-8] pckg_dest;
  rand bit [pckg_sz-1:0] D_pop;

  bit push [bits-1:0][drvrs-1:0] 
  bit pop [bits-1:0][drvrs-1:0] 
  bit [pckg_sz-1:0] D_push[bits-1:0][drvrs-1:0];
endclass
