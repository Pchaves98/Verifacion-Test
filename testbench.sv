//**************DEFINICION DE TIPOS DE TRANSACCION**************************
typedef enum{llenado_aleatorio,llenado_especifico} tipos_trans;//definir los casos de generación del test


//**************Tipos de prueba con casos de esquina***********************
typedef enum {ceros, unos, cerounos, fakeaddress,brdcst,cero_lejos, uno_cerca} cas_esq;//se definen los casos de esquina


bit [7:0]arrayglobal [400000];//se utiliza un array global para guardar los tiempos de envios de los paquetes
//**************TRANSACCION1************************************************
class Bus_trans#(parameter pckg_sz,drivers,tipocaso,esquina);//transacción del mensaje que entra al DUT
   rand bit [pckg_sz-1:pckg_sz-8] destino;
  randc bit [pckg_sz-9:0] payload;
  rand bit [2:0] delay;
  rand int Fifo_id;

  constraint item {0<=Fifo_id;Fifo_id<=drivers-1;destino!=Fifo_id;}
  constraint direc {destino dist {[8'b0:8'b11]:=250,[8'b100:{8{1'b1}}]:=50};}
  constraint direccion { if (tipocaso==llenado_especifico & esquina==fakeaddress) destino>drivers;}
  
  bit [pckg_sz-1:0] D_pop;
  bit envio;

  function print  (string tag);
    $display("[T=%g] %s dato=%b,destino=%b,Fifo_id=%0d, D_pop=%b", 
             $time,
             tag, 
             this.payload,
             this.destino, 
             this.Fifo_id,
             this.D_pop);
  endfunction
  
endclass

/************************************Transaccion2******************************************************/
class TB_trans#(parameter pckg_sz);//transacción que sale del DUT
  int Fifo_id;
  bit [pckg_sz-1:0] D_push;
  int retraso;

  function print  (string tag);
    $display("[T=%g] %s Fifo_id=%0d, D_push=%b, Tiempo de llegada=%0d", 
             $time,
             tag, 
             this.Fifo_id,
             this.D_push,
             this.retraso);
  endfunction
  
endclass


//*************************GENERADOR************************************
class Generador#(parameter iter, pckg_sz,drivers,tipocaso,esquina,broadcast);
  mailbox gen_agen;//el buzon que va hacia el agente
  event agen_listo;
  tipos_trans tipo_llenado=tipocaso;//handler del tipo de generación del testbench
  cas_esq corner=esquina; //handler de casos de esquina
  
  
  task run();//task donde corre el generador
    case(tipo_llenado)//un case para saber si genera una transacción completamente aleatoria o con un caso de esquina
        llenado_aleatorio: //genera una transaccion aleatoria
          begin 
            $display("[T=%0t] [Generador] Se ha escogido la transaccion de llenado aleatorio", $time);
            for (int i = 0; i < iter; i++) begin//en este ciclo se crean las transacciones
              Bus_trans #(.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) item = new;//creamos una nueva transacción
               item.randomize();//generamos los valores aleatorios
                  for ( int h=0; h<item.delay;h++)//un ciclo para generar el delay aleatorio entre mensajes
                      begin
                        #1ns;
                      end
              $display ("[T=%0t] [Generator] Loop:%0d/%0d create next item", $time, i+1, iter);
              gen_agen.put(item);//manda el item hacia la capa inferior
              @(agen_listo);//espera a que el agente avise que ya tomo la transacción
                end
                $display ("[T=%0t] [Generator] Done generation of %0d items", $time, iter);
          end
        
        llenado_especifico: //genera una transaccion con algunos datos especificos
          begin
            $display("[T=%0t] [Generador] Se ha escogido la transaccion de llenado especifico", $time);
            for (int i = 0; i < iter; i++) begin
              Bus_trans #(.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) item = new;//creamos una nueva transacción
              item.randomize();
              case (corner)//en el case se asignan algunos valores que no sean aleatorios, dependiendo del caso elegido
                ceros:
                  begin
                    $display("[T=%0t] [Ambiente] Se ha escogido el caso de esquina con un payload de ceros", $time);
            		item.payload = {8{1'b0}};
                  end

        	    unos:
          		  begin
            		$display("[T=%0t] [Ambiente] Se ha escogido el caso de esquina con un payload de unos", $time);
            		item.payload = 8'b11111111;
          		  end

        		cerounos:
          		  begin
            		$display("[T=%0t] [Ambiente] Se ha escogido el caso de esquina con un payload de ceros-unos", $time);
            		item.payload= 8'b01010101;
          		  end

        		fakeaddress:
          		 begin
            		$display("[T=%0t] [Ambiente] Se ha escogido el caso de esquina con una direccion incorrecta", $time);
          		 end
        
        		brdcst:
          		  begin
            		$display("[T=%0t] [Ambiente] Se ha escogido mandar mensajes con broadcast",$time);
            		item.destino=broadcast;
          		  end
        
        		cero_lejos:
          		 begin
            		$display("[T=%0t] [Ambiente] Se ha escogido mandar mensajes con ceros a dispositivo más lejano",$time);
            		item.destino=drivers-1;
            		item.payload=8'b00000000;
            		item.Fifo_id=0;
          		 end
        
        		uno_cerca:
          		 begin
            		$display("[T=%0t] [Ambiente] Se ha escogido mandar mensajes con unos a un dispositivo adyacente",$time);
            		item.destino=drivers-1;
            		item.payload=8'b11111111;
            		item.Fifo_id=drivers-2;
          		 end
        
      		endcase
              for ( int h=0; h<item.delay;h++)//un ciclo para generar el delay aleatorio entre mensajes
                      begin
                        #1ns;
                      end
                  $display ("[T=%0t] [Generator] Loop:%0d/%0d create next item", $time, i+1, iter);
                  gen_agen.put(item);//manda el item hacia la capa inferior
                  @(agen_listo);//espera a que el agente avise que ya tomo la transacción
              end
              $display ("[T=%0t] [Generator] Done generation of %0d items", $time, iter);
          end
    endcase
  endtask
endclass



//**********************FIFO*************************************************+
class Fifo #(parameter pckg_sz);//esta clase sirve para simular el FIFO
  bit [pckg_sz-1:0]D_pop;
  bit [pckg_sz-1:0]D_push;
  bit [pckg_sz-1:0]q[$];
  bit maximo;
  int tamano;
  int id;
  
    
  function new (bit maximo);//constructor del Fifo
    this.maximo=maximo;//se le ingresa el tamaño que va a determinar el tamaño máximo
  endfunction
  
  task POP(output bit [pckg_sz-1:0] D_pop);//función para hacer pop
    if (q.size()!=0)//si el Fifo no está vacío sí puede hacer la operación
      begin
		D_pop=q.pop_front;
      end
    
    this.tamano=q.size();//debe guardar el tamaño actual en caso de que se ocupe retornar el valor

  endtask
  
  task PUSH (input bit [pckg_sz-1:0]D_push);
    if (q.size>this.maximo)//se analiza si el Fifo ya está lleno
      begin//entonces si está lleno mete otro dato al Fifo pero borra el primero
        q.push_back(D_push);
        q.delete(0);
      end
    else 
      begin
        q.push_back(D_push);//en caso contrario solo mete uno más y ya
      end
    
    this.tamano=q.size;//debe guardar el tamaño actual en caso de que se ocupe retornar el valor
  endtask
endclass

//**********************INTERFACE**************************************************************  
interface Int_fifo#(parameter pckg_sz, drivers,bits)(input bit clk);//interfaz para conectar DUT con TB
  bit pndng[bits-1:0][drivers-1:0];
  bit reset;
  bit [pckg_sz-1:0] D_pop[bits-1:0][drivers-1:0];
  bit [pckg_sz-1:0] D_push[bits-1:0][drivers-1:0];
  bit pop[bits-1:0][drivers-1:0];
  bit push[bits-1:0][drivers-1:0];
  
endinterface

//******************AGENTE**********************************
class Agente#(parameter pckg_sz,drivers,tipocaso,esquina);
  event agen_listo;//declaro un evento
  mailbox gen_agen;//declaro el buzón entre generador y agente
  mailbox age_driv;//declaro el buzón entre driver y agente
  mailbox agen_checker;//declaro el buzón entre agente y checker
  Bus_trans #(.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) mensaje;//declaro la transacción que va hacia el driver: mensaje
  
  task run();//genero el task donde corre el agente
    mensaje=new;
    forever begin//esto corre una y otra vez
      #1gen_agen.get(mensaje);//tomo la transacción que se haya enviado del generador
      $display("[T=%0t][Agente] Transacción ha llegado al agente",$time);
      #1mensaje.D_pop={mensaje.destino,mensaje.payload};//concateno destino y payload
      $display ("[T=%0t][Agente] Contenido de la transacción: ID del Fifo=%0d,Mensaje %b",$time,mensaje.Fifo_id,mensaje.D_pop);//imprimo el contenido ya concatenado
      //assert (mensaje.destino<drivers)
       // $display("[T=%0t] [Agente] Assertion passed!: Dirección de mensaje válida",$time);
      #1age_driv.put(mensaje);//envio la transacción mensaje hacia el driver
      $display("[T=%0t][Agente] Transacción enviada al Driver",$time);
      #1agen_checker.put(mensaje);//envio la transacccion hacia el checker también
      $display("[T=%0t][Agente] Transacción enviada al Checker",$time);
      -> agen_listo;//anuncio que el evento ya está listo
      end
  endtask
endclass
  

//*******************DRIVER***************************************************
class Driver#(parameter drivers,Fifo_sz,bits,pckg_sz,tipocaso,esquina);
  mailbox age_driv;//declaro el mailbox entre agente y  driver
  virtual Int_fifo #(.pckg_sz(pckg_sz),.drivers(drivers),.bits(bits)) inf;//instancio la interfaz
  Fifo #(.pckg_sz(pckg_sz)) f1[drivers-1:0];//declaro el Fifo
  Bus_trans #(.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) item1[drivers-1:0],item2[drivers-1:0];//declaro la transacción
  int t_envio;//esta variable va a ser el tiempo de envio del mensaje
  
  
  task reset();//reseteo el DUT y la interfaz
    for (int i=0;i<drivers;i++)begin
      inf.pndng[0][i]<=0;
      inf.D_pop[0][i]<=0;
      inf.reset<=1'b1;
      #1inf.reset<=1'b0;
    end
  endtask
    
  task run();
    reset();//inicio haciendo un reset
    $display("DUT reseteado");
    for (int q=0; q<drivers;q++)//en el ciclo voy a correr varios forks
      begin
        automatic int w=q;
        fork begin//inicio un proceso
          f1[w]=new(Fifo_sz);//creo un fifo
          item1[w]=new;
          item2[w]=new;
          $display("Fifo [%0d] creado y corriendo",w);
          forever begin//lo hago en un forever para que lo corrar una y otra vez
            @(posedge inf.clk)//en cada flanco del reloj hace lo siguiente
            #1age_driv.peek(item1[w]);//ojeo el buzón y copio el contenido
            if (item1[w].Fifo_id==w)begin//comparo si el id del mensaje es igual al id del Fifo(que es el # de iteración)
              #1age_driv.get(item1[w]);//si se cumple tomo la transacción dl buzón
              #1item1[w].print("[Driver] Transacción qu llegó al Driver");
              #1f1[w].PUSH(item1[w].D_pop);//lo meto al Fifo
              $display("[T=%0t] [Driver] Mensaje (id=[%0d]) ingresado al Fifo [%0d]",$time,item1[w].Fifo_id,w);
              #1f1[w].POP(item2[w].D_pop);//saco el D_pop del fifo y lo pongo en el D_pop del item2
              #1inf.D_pop[0][w]<=item2[w].D_pop;
              #1inf.pndng[0][w]<=1'b1;//levanto la bandera d pending
              $display("[T=%0t] [Driver] Se ha prendido la bandera de pending del Fifo[%0d]",$time,w);
              @(negedge inf.pop[0][w])//en cada flanco negativo de la señal pop
              if (item2[w].D_pop[pckg_sz-1:pckg_sz-8]<drivers)begin//si el mensaje tiene una dirección validad
              t_envio=$time;//t_envio va a ser el tiempo de simulacion actual
                arrayglobal[t_envio]=item2[w].D_pop[pckg_sz-9:0];//en el array global coloco el payload con el t_envio como indice
              end
              inf.pndng[0][w]<=1'b0;//luego apago la señal de pending de este dispositivo
            end
            end
          end
        join_none
      end
  endtask
endclass
       
    

//********************Monitor**********************************************
class Monitor#(parameter drivers,Fifo_sz,bits,pckg_sz);
  virtual Int_fifo #(.pckg_sz(pckg_sz),.drivers(drivers),.bits(bits)) inf;//declaro la interfaz 
  TB_trans #(.pckg_sz(pckg_sz)) item[drivers-1:0];//declaro una transación
  mailbox moni_che;//un buzon entre monitor y checker
  Fifo #(.pckg_sz(pckg_sz)) f2[drivers-1:0];//declaro los FIFOS
  
  task run();//en esta corre el Monitor
    for (int p=0;p<drivers;p++)begin
      automatic int w=p;
      fork begin
        item[w]=new;//corro un proceso
        f2[w]=new(Fifo_sz);//construyo un fifo
        forever begin
          @(posedge inf.push[0][w])//cada vez que encuentro un flanco del push del dispositivo
          #1f2[w].PUSH(inf.D_push[0][w]);//le hago un push al mensaje que viene del bus en el Fifo
          #1f2[w].POP(item[w].D_push);//le hago pop al mensaje desde el FIFO
          item[w].Fifo_id=w;//indico cual fue el FIFO que recibió el mensaje
          item[w].retraso=$time;//indico cual fue el tiempo de simulación en el que se recibió el mensaje
          $display("[T= %0t] [Monitor] Mensaje recibido desde el DUT, ha sido tomado por el Fifo %0d",$time,w);
          item[w].print("[Monitor]");
          moni_che.put(item[w]);//coloco los mensajes en el buzon
        end
      end
        join_none
      end
  endtask
endclass


//*********************CHECKER**********************************************
class Checker#(parameter pckg_sz,iter,broadcast,drivers,tipocaso,esquina);
    mailbox moni_che; //se declaran los mailboxes
    mailbox agen_checker;
  	Bus_trans #(.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) item_score; //se declaran y se crean las transacciones que vienen de monitor y del agente
  	TB_trans  #(.pckg_sz(pckg_sz))item_moni;
  	event agen_listo;
  	bit [pckg_sz-1:pckg_sz-8] todos [iter];//declaro un array que voy a utilizar para guardar los payload
  	bit [pckg_sz-9:0]cajon1[$];//esta queue sirve para averiguar si un payload está entre los payload enviados al DUT
  	int cajon2[$];//esta queue sirve para averiguar si un payload está entre los payload enviados al DUT y obtener su indice que es el tiempo de envio del mensaje
  	int t,t_envio,contador=0,sumat,buffer;//integers que se utilizan para calcular tiempos
  	string mensaje,dspstv,rts,arrvng,sndng;//strings que sirven para generar el csv
  	string outputCVS_line,coma=",";
  	real t_simul;
  	int counter[0:drivers];//estas colas sirven para calcular los retrasos por cada fifo
  int colaDispo [0:drivers-1][0:iter-1];//esta matriz almacena los retrasos en  una matriz de Dispositivosxnumero de mensajes
  
  
  
  task getting();//este task simplemente va almacenando todos los mensajes generados por el TB
    for (int i=0;i<iter;i++)begin
      automatic int u=i;
      @(agen_listo)//cada vez que el agente avisa que ha enviado otro mensae
      agen_checker.get(item_score);//tomo lo que haya mandado el agente
      todos[u]=item_score.payload;//guardo los payload de los mensajes que genera el TB
    end
  endtask
  
  
    task run();
      $display("[T=%0t] [Checker] Se ha iniciado el checker",$time);
      item_moni=new;//creo las transacciones
      item_score=new;
      getting();
      #1$display("[T=%0t] [Checker] Se han obtenido todos los mensajes que haya generado el TB",$time);
      $system("echo Dispositivo,Mensaje,Tiempo de envio[ns],Tiempo de llegada [ns], Retraso del mensaje[ns] > salidaTB.csv");
      forever begin//un ciclo forever para que corra una y otra vez
        moni_che.get(item_moni);//espera hasta que pueda tomar algo que le mande el monitor
        contador++;//este contador acumula el valor de todos los mensajes checkeados
        #1item_moni.print("[Checker] Mensaje a checkear:");
        #1cajon1=todos.find(x) with (x==item_moni.D_push[pckg_sz-9:0]);//busco el payload del mensaje que envia el DUT entre los payload que generó el TB y si está se guarda en el queue cajon1
        cajon2=arrayglobal.find_index with (item==item_moni.D_push[pckg_sz-9:0]);//busco el payload del mensaje que envia el DUT entre los payload que generó el TB y si está guardo su indice(que es el tiempo de envio) en cajon2
        t_envio=cajon2[0];//t_envio va a tener el tiempo de envio del mensaje
        arrayglobal[t_envio]=0;//vacio el dato cuyo indice sea t_envio
        t=item_moni.retraso-t_envio;//t va a ser el retraso del mensaje(tiempo de recibo-tiempo de envio)
       	buffer=sumat;//esto lo utilizo porque no se puede escribir y leer una variable a la vez
        sumat=buffer+t;//voy guardando los retrasos de los mensajes que llegan al checker
        counter[item_moni.Fifo_id]++;//aumento el contador de los mensajes obtenidos por x dispositivo
        colaDispo[item_moni.Fifo_id][counter[item_moni.Fifo_id]]=t;//almaceno el tiempo del mensaje z para el dispositivo x
        $display("[T=%0t] [Checker] Mensaje fue enviado con un retraso de: %0d ns",$time,t);
        #1if (item_moni.D_push[pckg_sz-1:pckg_sz-8]==broadcast)//verifico si el mensaje viene de broadcast
          $display("[T=%0t] [Checker] PASS: Mensaje enviado mediante broadcast",$time);
        else begin
        if(item_moni.Fifo_id!=item_moni.D_push[pckg_sz-1:pckg_sz-8])//verifico que el mensaje haya llegado al dispositivo que debía
          $display("[T=%0t] [Checker] FAIL: El mensaje llegó a un Fifo erroneo", $time);
        else
          $display("[T=%0t] [Checker] PASS: El destino del mensaje es correcto", $time);
        end
        
        if(cajon1.size!=0)//verifico que si el cajon está vacio es porque no encontró ese payload
          begin
         $display("[T=%0t] [Checker] PASS: El payload es correcto", $time);
            append2outputCSV();//si está correcto envio el mensaje y sus valores al csv
       end
       else 
         $display("[T%0t] [Checker] FAIL: El payload del mensaje es incorrecto",$time);
        
        
        #1cajon1={};//vacio nuevamente los cajones
        cajon2={};
        t_simul=$time;
       end
    endtask
  
  
  task append2outputCSV();//con esta tarea almaceno valores en un csv
    mensaje.hextoa(item_moni.D_push);//mensaje va a tener un string del valor hexadecimal del payload del mensaje
    dspstv.itoa(item_moni.Fifo_id);//dspstv va a tener un string del valor del dispositivo receptor
    rts.itoa(t);///mismo que los valores anteriores pero almacena el tiempo de recibo
    arrvng.itoa(item_moni.retraso);//mismo que los valores anteriores pero almacena el tiempo de retraso
    sndng.itoa(t_envio);//mismo que los valores anteriores pero almacena el tiempo de envio
    outputCVS_line = {dspstv,coma,mensaje,coma,sndng,coma,arrvng,coma,rts};//con esto le agrego el orden de las columnas
    $system($sformatf("echo %0s >> salidaTB.csv",outputCVS_line));
  endtask
  
  endclass



//**********************Ambiente********************************

class Ambiente#(parameter pckg_sz, drivers,bits,iter,Fifo_sz,broadcast,tipocaso,esquina);
  Generador #(.iter(iter),.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina),.broadcast(broadcast)) G0;//instancio los manejadores de cada clase del TB
  Agente #(.pckg_sz(pckg_sz),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) A0;
  Monitor#(.drivers(drivers),.Fifo_sz(Fifo_sz),.bits(bits),.pckg_sz(pckg_sz)) M0;
  Driver #(.drivers(drivers),.Fifo_sz(Fifo_sz),.bits(bits),.pckg_sz(pckg_sz),.tipocaso(tipocaso),.esquina(esquina)) D0;
  Checker #(.pckg_sz(pckg_sz),.iter(iter),.broadcast(broadcast),.drivers(drivers),.tipocaso(tipocaso),.esquina(esquina)) C0;
  mailbox gen_agen;//instancio todos los buzones que ocupa el TB
  mailbox age_driv;
  mailbox moni_che;
  mailbox agen_checker;
  mailbox driv_check;
  event agen_listo;
  virtual Int_fifo #(.pckg_sz(pckg_sz),.drivers(drivers),.bits(bits))inf;//instancio la interfaz del TB
 
  
  function new();
    gen_agen=new();
    age_driv=new();
    agen_checker=new();
    moni_che=new();
    G0=new;
    G0.gen_agen=gen_agen;
    A0=new;
    A0.gen_agen=gen_agen;
    A0.age_driv=age_driv;
    A0.agen_checker=agen_checker;
    M0=new;
    M0.moni_che=moni_che;
    G0.agen_listo=agen_listo;
    A0.agen_listo=agen_listo;
    D0=new();
    D0.age_driv=age_driv;
    C0=new();
    C0.moni_che=moni_che;
    C0.agen_checker=agen_checker;
    C0.agen_listo=agen_listo; 
  endfunction
  
  
  task run();
    
    M0.inf=inf;
    D0.inf=inf;
    fork
      G0.run();
      A0.run();
      D0.run();
      M0.run();
      C0.run();
    join_none
  endtask
  
endclass


//*************************Modulo para prueba******************************
module tb#(parameter pckg_sz=16, tipocaso=llenado_aleatorio,esquina=fakeaddress, dispositivos=4,bits=1,iter=600,Fifo_sz=12,broadcast={8{1'b1}});
  reg clk;
  real ab,rprom;
  real resultado[0:dispositivos],buffer [0:dispositivos];
  
  
  always #10 clk=~clk;
  Int_fifo #(.pckg_sz(pckg_sz),.drivers(dispositivos),.bits(bits)) inf(clk);
  bs_gnrtr_n_rbtr dut1(.clk(clk),.reset(inf.reset),.pndng(inf.pndng),.push(inf.push),.pop(inf.pop),.D_pop(inf.D_pop),.D_push(inf.D_push));
  Ambiente #(.pckg_sz(pckg_sz),.drivers(dispositivos),.bits(bits),.iter(iter),.Fifo_sz(Fifo_sz),.broadcast(broadcast),.tipocaso(tipocaso),.esquina(esquina)) A1;
  
  covergroup cg @(posedge clk);//declaro el grupo de cobertura
    cp_dstny : coverpoint A1.A0.mensaje.D_pop[pckg_sz-1:pckg_sz-8];//el punto de cobertura son los payload que toma el agente
      endgroup
  
  initial begin
    cg cg_inst=new();
    {clk,inf.reset}<=0;
    A1=new();
    A1.inf=inf;
    A1.run();
    
    #150000 $display("[Coverage] El coverage obtenido en este prueba es %0.2f %%", cg_inst.get_coverage());
    ab=(iter*pckg_sz*1000)/A1.C0.t_simul;//calculo el ancho de banfa
    rprom=A1.C0.sumat/A1.C0.contador;//calculo el retraso promedio total
    $display("[Ancho de banda] El ancho de banda total promedio fue de %0.1f x10^9 Hz,utilizando %0d dispositivos y una profundidad de Fifos de %0d",ab,dispositivos,Fifo_sz);
    $display("[Retraso] El retraso promedio de los mensajes que fueron recibidos fue de %0.1f ns, utilizando %0d dispositivos y una profundidad de Fifo de %0d",rprom,dispositivos,Fifo_sz);
    for (int i=0;i<dispositivos;i++)begin//estos ciclos for emula un foreach para obtener el retraso promedio por dispositivo
      for (int j=0;j<iter;j++)begin
        buffer[i]=resultado[i];
        resultado[i]=buffer[i]+A1.C0.colaDispo[i][j];
        end
      rprom=resultado[i]/A1.C0.counter[i];
      $display("[Retraso] El retraso promedio de los %0d mensajes dirigidos al dispositivo %0b fue de %0.1f ns, utilizando %0d dispositivos y una profundidad de Fifo de %0d",A1.C0.counter[i],i,rprom,dispositivos,Fifo_sz);
    end    
    
    $finish;
  end
 
  initial begin
    $dumpvars(0,bs_gnrtr_n_rbtr);
    $dumpfile("dump.vcd");
  end
endmodule
