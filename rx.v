module rx(output [39:0] dout, output dav, input [7:0] device_id, input sd_in, ack, clk, clr);

  wire past_header, hold, valid;
  header_detect hd(.past_header(past_header), .sd_in(sd_in), .clk(clk), .clr(clr));
  skip_bit sb(.hold(hold), .sd_in(sd_in), .clk(clk), .clr(clr));
  bit_unstuffer bu(.dout(dout), .valid(valid), .device_id(device_id), .sd_in(sd_in), .past_header(past_header), .hold(hold), .clk(clk), .clr(clr));
  dav_to_ack dta(.dav(dav), .valid(valid), .ack(ack), .clk(clk), .clr(clr));
 
endmodule

module header_detect(output past_header, input sd_in, clk, clr);
  reg [2:0] count; //Counter for first five ones of header
  localparam HEADER = 3'd5;
  assign past_header = (count == HEADER) ? 1'b1 : 1'b0;
 
  always@(posedge clk, posedge clr) begin
    //Clear counter on reset
    if(clr)
      count <= 3'b0;
    else
      //Increment counter if sd_in is asserted
      if (sd_in && (count < HEADER))
        count <= count + 3'b1;
      //Reset counter is sd_in is deasserted
      else
        count <= 3'b0;
  end
 
endmodule


module skip_bit(output hold, input sd_in, clk, clr);
 
  reg [2:0] count; //Counter for four ones
  localparam SKIP = 3'd4;
  assign hold = (count == SKIP) ? 1'b1 : 1'b0;
 
  always@(posedge clk, posedge clr) begin
    //Clear counter on reset
    if(clr)
      count <= 3'b0;
    else
      //Increment counter if sd_in is asserted
      if (sd_in && (count < SKIP))
        count <= count + 3'b1;
      //Reset counter is sd_in is deasserted
      else
        count <= 3'b0;
  end
  
endmodule

module bit_unstuffer(output reg [39:0] dout, output valid, input [7:0] device_id, input sd_in, past_header, hold, clk, clr);
 
  localparam IDLE = 6'd0, STOP = 6'd54, PARITY = 6'd50;
  reg [5:0] state, next_state;
  reg [53:0] shifter;
  //reg parity, parity_count; Used in optimized parity detector
  wire done; 
  wire x, y, z; //variable for 3 validity checks, used in debugging
  assign done = (state == STOP) ? 1'b1 : 1'b0;
  assign x = (device_id == shifter[15:8]);
  assign y = (~^shifter[48:0]);
  assign z = (shifter[53:49] == 5'b0);
  assign valid = (done && x && y && z) ? 1'b1 : 1'b0;
  //assign valid = (done && (device_id == shifter[15:8]) && (~parity) && (shifter[53:49] == 5'b0)) ? 1'b1 : 1'b0; Optimized parity detector
  
  
  always@(posedge clk, posedge clr) begin
    if(clr) begin
      state <= IDLE;
      shifter <= 54'b0;
    end
    //Hold indicates the next bit has been stuffed, so shift in the next bit if this is not asserted
    else if(~hold) begin
      state <= next_state;
      shifter <= {sd_in, shifter[53:1]};
    end
    //The next bit is a stuffed bit and shouldn't be included in the packet. Shifter remains the same
    else begin
      state <= state;
      shifter <= shifter;
    end
  end
 
  always@(posedge clk, posedge clr) begin
    if(clr)
      dout <= 40'b0;
    //If done (state is in stop state) load dout with the shift register in the proper format
    else if(done)
      dout <= {shifter[7:0], shifter[47:16]};
    //If not in the stop state hold the previous data on dout
    else
      dout <= dout;
  end
  
  //Parity detector in attempt to minimize area, critical path. Was buggy so not included final module.
  /*Parity without using a massive exclusive or;
  always@(posedge clk, posedge clr) begin
    if(clr)
      parity <= 1'b0;
    else if(state == PARITY)
      parity <= parity_count;
    else
      parity <= parity;
   end
  
   always@(posedge clk, posedge clr) begin
    if(clr)
      parity_count <= 1'b0;
    else if (past_header)
      parity_count <= 1'b0;
    else if(~hold)
      parity_count <= (sd_in ^ parity_count);
    else
      parity_count <= parity_count;
   end
  */
  
  always@(*) begin
    case(state)
      IDLE:
        if(past_header)
          next_state = state + 6'd1;
        else
          next_state = IDLE;
      STOP: begin
        next_state = IDLE;
      end
      default:
        next_state = state + 6'd1;
    endcase
  end 
 
endmodule

module dav_to_ack(output reg dav, input valid, ack, clk, clr);
  reg reg_ack;//synchronized acknowledge signal
 
  always@(posedge clk, posedge clr) begin
    //Synchronize the acknowledge signal
    if(clr)
      reg_ack <= 1'b0;
    else
      reg_ack <= ack;
  end
 
  always@(posedge clk, posedge clr) begin
    if(clr)
      dav <= 1'b0;
    //If valid set the data available flag
    else if (valid)
      dav <= 1'b1;
    //If endpoint acknowledges it's recieved packet, deassert the data available
    else if(reg_ack)
      dav <= 1'b0;
    else
      dav <= dav;
  end
endmodule
