module tx(output sd_out, ack, input [39:0] din, input [7:0] device_id, input send, clk, clr);

  wire [47:0] reg_data;
  wire ready, hold, t_sd_out, past_header;
  send_ack sa(.reg_data(reg_data), .ack(ack), .din(din), .device_id(device_id), .send(send), .clk(clk), .clr(clr));
  transmitter t(.sd_out(t_sd_out), .past_header(past_header), .reg_data(reg_data), .ack(ack), .hold(hold), .clk(clk), .clr(clr));
  bit_stuffer bs(.sd_out(sd_out), .hold(hold), .sd_in(t_sd_out), .past_header(past_header), .clk(clk), .clr(clr));

endmodule

module send_ack(output reg [47:0] reg_data, output reg ack, input [39:0] din, input [7:0] device_id, input send, clk, clr);
  
  localparam IDLE = 1'b0, ACK = 1'b1;
  reg state, next_state;
  reg reg_send; 
  
  always@(posedge clk, posedge clr) begin
    if(clr) begin
      reg_data <= 1'b0;
      reg_send <= 1'b0;
    end
    //Register the data in and the send flag
    else begin
      reg_data <= {device_id, din};
      reg_send <= send;
    end
  end
  
  always@(posedge clk, posedge clr) begin
    if(clr)
      state <= IDLE;
    else
      state <= next_state;
  end
  
  always@(*) begin
    ack = 1'b0;
    case(state) 
      IDLE:
        if(reg_send) begin
          next_state = ACK;
          ack = 1'b1;
        end
        else
          next_state = IDLE;
      ACK:
        if(!reg_send) begin
          next_state = IDLE;
        end
        else begin
          next_state = ACK; 
          ack = 1'b1;
        end
    endcase
  end  

endmodule

module transmitter(output sd_out, past_header, input [47:0] reg_data, input ack, hold, clk, clr);
  
  localparam IDLE = 6'd0, HEADER = 6'd5, STOP = 6'd59;
  reg [5:0] state, next_state;
  reg [58:0] shifter;
  wire [58:0] frame;
  reg pending; //register to notify that a second frame should be sent after the current one
  wire ld_shifter; //signal to nofigy when to load the shifter with data
  reg parity_count, prev_ack; //registers used in an optimized parity detector
  wire parity;
  assign frame = {5'b00000, 1'b0, reg_data[31:0], reg_data[39:32], reg_data[47:40], 5'b11111}; //zero is always assigned in the parity field, it will be written over
  //with the calculated parity count
  assign past_header = (state > HEADER) ? 1'b1 : 1'b0;//Assert this signal for the duration of the transmission. Used to determine state machine in bit
  //unstuffer. Fairly inefficient, but funtional.
  
  assign sd_out = (state == 6'd54) ? parity : shifter[0];//if transmitting the parity bit (state 54), replace the shifter[0] value with the parity
  assign parity = parity_count; //alias to the register parity_count, used above
  
  assign ld_shifter = ((ack || pending) && (state == IDLE)) ? 1'b1 : 1'b0; //Load the shift register if in the idle state and acknowledged new data, or
  //if there is a pending transfer.
  
  //Parity counter
  always@(posedge clk, posedge clr) begin
    if(clr)
      parity_count <= 1'b0;
    //Reset the count if in the header state, we don't want these ones to be accounted for in our parity calculation
    else if(state <= HEADER)
      parity_count <= 1'b0;
    else
    //If not bitstuffing(hold), update the current parity value(xor bit by bit), else hold previous value
      if(!hold)
        parity_count <= (shifter[0] ^ parity_count);
      else
        parity_count <= parity_count;
  end
  
  always@(posedge clk, posedge clr) begin
    if (clr) begin
      shifter <= 59'b0;
      state <= IDLE;
    end
    //Load the shift register if in the idle state and acknowledged new data, or if there is a pending transfer. Advance from Idle state
    else if (ld_shifter) begin
      shifter <= frame;
      state <= 1'b1;
    end
    //If not bitstuffing, shift out the data.
    else if (!hold) begin
      state <= next_state;
      shifter <= {1'b0, shifter[58:1]};
    end
    //Bitstuffing, maintain shift register
    else begin
      shifter <= shifter;
    end
  end
  
  //This block is used to detect a rising edge in the acknowledge signal. When operating in multiple clock domains a signal acknowledge
  //from a low speed clock could be detected as multiple acknowledge signals if we are using level sensitve logic. Therefore we record the
  //previous values of the signal to determine a positive edge, for edge sensitive logic.
  always @(posedge clk, posedge clr) begin
    if(clr)
      prev_ack <= 1'b0;
    else
      prev_ack <= ack;
  end
  
  //Block detects when an acknowledge is recieved and asserts a pending register. This tells the modules to start sending the next packet
  //as soon as it finishes sending the current one.
  always@(posedge clk, posedge clr) begin
    if(clr)
      pending <= 1'b0;
    //If a rising edge of the ack signal is detected, assert the pending register
    else if(prev_ack < ack) 
      if(state != IDLE)
        pending <= 1'b1;
      else
        pending <= 1'b0;
    //Reset the pending register in the IDLE state, at this point the second packet should have already begun to transmit
    else if (state == IDLE)
        pending <= 1'b0;
    else
      pending <= pending;
  end
  
  always@(state) begin
    case(state)
      IDLE:
        if(ack || pending)
          next_state = 6'd1;
        else
          next_state = IDLE;
      HEADER:
        next_state = state + 6'd1;
      STOP:
        next_state = IDLE;
      default
        next_state = state + 6'd1;
    endcase
  end
  
endmodule

module bit_stuffer(output sd_out, output reg hold, input sd_in, past_header, clk, clr);
  
  localparam IDLE = 3'd0, STUFF = 3'd4;
  reg [2:0] state, next_state;
  reg stuff_zero; //register flag to stuff a zero if four ones have been detected
  assign sd_out = (stuff_zero) ? 1'b0 : sd_in;
  
  always@(posedge clk, posedge clr) begin
    if(clr)
      state <= IDLE;
    else
    //Start operation when the tranmitter has pass out the five header ones
      if(past_header)
        state <= next_state;
  end
  
  always@(*) begin
    hold = 1'b0;
    stuff_zero = 1'b0;
    case(state)
      IDLE:
      //Increment the state if sd_in is asserted
        if(!sd_in)
          next_state = IDLE;
        else
          next_state = state + 3'd1;
      STUFF: begin
      //If four ones have been detected, transmit a O and tell the transmitter to stop shifting for a cycle(hold)
        next_state = IDLE;
        stuff_zero = 1'b1;
        hold = 1'b1;
      end
      default
      //Increment the state if sd_in is asserted
        if(!sd_in)
          next_state = IDLE;
        else
          next_state = state + 3'd1;
    endcase
  end
endmodule
