class Generador	
  mailbox gendriv;//buzon entre generador y driver
  event driv_listo;
  int contador = 35;//cantidad de items que vamos a crear
  
  task run();
  for (int i=0; i< num; i++)
    begin
      Bus_trans item=new;//creamos una transaccion nueva
      item.randomize();//generamos el valor aleatorio del delay
      for (int h=0; h<item.delay;h++)//este ciclo genera una espera de los delays
        begin
        #1ns;
        end
      gendriv.put(item);// enviamos los valores al buzon
      $display ("T=%0t [Generador] Loop:%0d de %0d",$time,i+1);
      @(driv_listo);//espera a que el evento termine desde el driver para hacer la siguiente iteración
    end
    $display("T=%0t Completada la generación de %0d items",$time, num);//mensaje de que ya completó todos las iteraciones 
  endtask
endclass
