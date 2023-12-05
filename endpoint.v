module endpoint(output[39:0] dout, output[7:0] a, output send, rx_ack, rd, wr, inout[15:0] d, input[39:0] din, input tx_ack, dav, rdy, irq, clk, clr);
  
  wire[39:0] r_dout, c_dout;
  wire out_data, s_dav;
  
  receiver_interface r(.rx_ack(rx_ack), .s_dav(s_dav), .dav(dav), .clk(clk), .clr(clr));
  coprocessor_interface c(.dout(dout), .a(a), .out_data(out_data), .rd(rd), .wr(wr), .d(d), .din(din), .s_dav(s_dav), .rx_ack(rx_ack), .rdy(rdy), .tx_ack(tx_ack), .irq(irq), .clk(clk), .clr(clr));
  transmitter_interface t(.send(send), .tx_ack(tx_ack), .out_data(out_data), .clk(clk), .clr(clr));
  
endmodule

//the receiver interface
module receiver_interface(output reg rx_ack, s_dav, input dav, clk, clr);
  
  //synchronizing data available
  always@(posedge clk, posedge clr) begin
    if(clr)
      s_dav <= 1'b0;
    else
      s_dav <= dav;
  end
  
 //performing handshake
  always@(posedge clk, posedge clr) begin
    if(clr) begin
      rx_ack <= 1'b0;
    end
	//if dav high, rx_ack goes high	
    else if(s_dav) begin
      rx_ack <= 1'b1;
    end
	//if dav low, rx_ack goes low
    else if(~s_dav) begin
      rx_ack <= 1'b0;
    end
    else begin
      rx_ack <= rx_ack;
    end
  end
    
endmodule

//the coprocessor interface
module coprocessor_interface(output reg[39:0] dout, output reg[7:0] a, output reg out_data, rd, wr, inout[15:0] d, input[39:0] din, input s_dav, rx_ack, rdy, tx_ack, irq, clk, clr);
  
  //states
  localparam IDLE = 2'd0, WRITE = 2'd1, READ = 2'd2, INT = 2'd3;
  reg[1:0] state, next_state;
  reg[39:0] data;
  wire[15:0] read_data, write_data;
  reg interr, prev_irq;
  assign read_data = rd ? d : 16'bz;
  assign d = wr ? data[15:0] : 16'bz;
  
  //control when interrupt occurs 
  always@(posedge clk) begin
    if(clr) begin
      prev_irq <= 1'b0;
      interr <= 1'b0;
    end
    else begin	
      prev_irq <= irq;						//regsiter the interrupt
	  //if interrupt dropped low
      if(irq < prev_irq)
        interr <= 1'b1;
	  //if in state INT and interrupt, drop it
      else if((state == INT) && interr)
        interr <= 1'b0;
      else
        interr <= interr;
    end
  end
  
  //control the data being processed and when it's taken in
  always@(posedge clk, posedge clr) begin
    if(clr)
      data <= 40'b0;
	//if state is IDLE and data is available, take it in
    else if((state == IDLE) && s_dav)
      data <= din;
    else
      data <= data;
  end
  
  //control the dout based on read or interrupt
  always@(posedge clk, posedge clr) begin
    if(clr)
      dout <= 40'b0;  
	//if in INT state and there's an interrupt, output interrupt response
    else if((state == INT) && interr) 
      dout <= {8'b00000, 8'd3, data[23:16], data[15:0]};
	//if reading and ready received, ouptut read response
    else if(rd && rdy)
      dout <= {data[39:32], 8'd2, data[23:16], read_data};
    else
      dout <= dout;
  end
  
  //update the state
  always@(posedge clk, posedge clr) begin
    if(clr) begin
      state <= IDLE;
    end
    else
      state <= next_state;
  end
  
  //next state logic
  always@(*) begin
    case(state)
      IDLE:
	    //if IDLE and interrupt, go to INT state
        if(interr)
          next_state = INT;
		//if rx_ack, check commands
        else if(rx_ack) begin
          case(data[31:24])
            //if read comment, go to READ
			8'd0:
              next_state = READ;
            //if write command, go to WRITE
    		8'd1:
              next_state = WRITE;
			//otherwise state in IDLE
            default
              next_state = IDLE;
          endcase
        end
		//otherwise stay in IDLE
        else
          next_state = IDLE;
      WRITE:
	    //if writing and ready, go to IDLE
        if(rdy)
          next_state = IDLE;
		//otherwise stay in WRITE
        else
          next_state = WRITE;
      READ:
	    //if reading and ready, go to IDLE
        if(rdy)
          next_state = IDLE;
		//otherwise stay in READ
        else
          next_state = READ;
	  //otherwise stay in IDLE
      default:
        next_state = IDLE;
    endcase
  end
  
  //output logic
  always@(*) begin
    rd = 1'b0;
    wr = 1'b0;
    a = 8'b0;
    out_data = 1'b0;
    //dout = 40'b0;
    case(state)
      IDLE:
		//if rx_ack, check commands
        if(rx_ack) begin
          case(data[31:24])
			//if reading, set rd high, and output the address
            8'd0: begin
              rd = 1'b1;
              a = data[23:16];
            end
			//if writing, set wr high, and output the address
            8'd1: begin
              wr = 1'b1;
              a = data[23:16];
            end
          endcase
        end
      WRITE:
		//if writing and not ready, wr stays high, and continue outputting address
        if(!rdy) begin
          wr = 1'b1;
          a = data[23:16];
        end  
      READ:
	    //if reading and not ready, rd stays high, and continue outputting address
        if(!rdy) begin
          rd = 1'b1;
          a = data[23:16];      
        end 
		//otherwise, signal data to be output, rd stays high, and continue outputting address
        else begin
          out_data = 1'b1;
          rd = 1'b1;
          a = data[23:16];
        end
      INT:
		//if INT, signal data to be output
        out_data = 1'b1;
      endcase
  end
  
endmodule

//the trasnsmitter interface
module transmitter_interface(output reg send, input tx_ack, out_data, clk, clr);
  
  reg s_tx_ack;
  
  //synchronizing the tx_ack
  always@(posedge clk, posedge clr) begin
    if(clr)
      s_tx_ack <= 1'b0;
    else
      s_tx_ack <= tx_ack;
  end
  
  //peroforming handshaking
  always@(posedge clk, posedge clr) begin
    if(clr)
      send <= 1'b0;
	//if data to be output, send goes high
    else if(out_data)
      send <= 1'b1;
	//if tramitter acknowledges, send goes low
    else if(s_tx_ack)
      send <= 1'b0;
    else
      send <= send;
  end
 
endmodule
