class Bus_trans #(parameter pckg_sz=32, drvrs=4);
  rand bit [pckg_sz-1:pckg_sz-8] pckg_dest;
  rand bit [pckg_sz-1:0] D_pop;
  rand bit [4:0] delay;
  rand bit drivers;
  rand bit Fifo_sz;

  bit push;//preguntarle al profe
  bit pop;//igual
  bit [pckg_sz-1:0] D_push[drvrs-1:0];
endclass



class generator;
  
  rand Bus_trans item;   //declarando la clase transaccion
  
  mailbox gen2driv;       //ahora vamos a declarar el mailbox 
  
  //evento
  event ended;

  
  function new (mailbox gent2driv);	//seguidamente vamos a construirlo (es un metodo)
    this.gen2driv=gen2driv;
  endfunction
   
  int num=20;	//variable con limite de transacciones
  
  //ahora vamos a hacer un Maintask de forma que cree y randomice los paquetes y los ponga en el mailbox

  task main();
  for (int i = 0; i < num; i++) begin
      item=new();
    if( !item.randomize() ) $fatal("[generador] no se logro randomizar la transaccion");
    gen2driv.put(item); //estamos poniendo la transaccion en el mailbox
    
    //aqui estamos metiendo un delay 
    for ( int h=0; h<item.delay;h++)//un ciclo para generar el delay aleatorio entre mensajes
          begin
            #1ns;
          end
    $display ("T=%0t [generador] Loop: %0d/%0d cree el siguiente item", $time, i+1, num);
      //drv_mbx.put(item);//manda el item hacia la capa superior
    -> ended; 	//listo el evento desde la capa superior
    end
    $display ("T=%0t [Generator] Done generation of %0d items", $time, num);
    
  endtask
  
endclass