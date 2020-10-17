interface fifo_drv #(int pckg_sz=32)(input clk) ;

  		//FIFO IN\\
      bit					    Pndng_int;
      bit				        Pop_int;
      logic		[pckg_sz-1:0]	D_out_int;
  

  
  modsport fifo_in( output Pndng_int,
    				output D_out_int,
    				input  Pop_int;
  
endinterface
                   
 
                   
interface fifo_moni #(int pckg_sz=32)(input clk) ;

  		//FIFO OUT\\
  	  bit				        Push_int;
      logic		[pckg_sz-1:0]	D_in_int;
  

  
  modsport fifo_out( input	D_in_int,
    				 input  Push_int);
  
endinterface

