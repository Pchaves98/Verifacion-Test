interface fifos_Bus #(parameter pckg_sz=32)(input clk) ;\
\
  		//FIFO IN\
      bit					Pndng_int;\
      bit					D_out_int;\
      logic	[pckg_sz-1:0]	Pop_int;\
      bit					Push;\
      bit					Full_in;\
      logic	[pckg_sz-1:0]	D_push;\
  \
  		//FIFO OUT\
      bit                   Full_out;\
      bit                   Pndng;\
      bit                   Pop;\
  	  logic [pckg_sz-1:0]   D_pop;\
      bit                   Push_int;\
 	  logic [pckg_sz-1:0]   D_in_int;\
  \
  modsport fifo_in( output Pndng_int,\
    				output D_out_int,\
    				output Full_in,\
    				input  Pop_int,\
    				input  D_push,\
    				input  Push);\
  \
  modsport fifo_out(\
  					output Full_out,\
                    output Pndng,\
                    input Pop,\
     				output D_pop,\
                    input Push_int,\
 				    input D_in_int);\
endinterface\
  }