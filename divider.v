module divider(inout [15:0] d, output irq, rdy, input [1:0] a, input rd, wr, clk, clr);
  
  wire [15:0] numMS, numLS, den, status, remainder, quotient;
  wire ex_div, read;
  
  reg_file r(.d(d), .ex_div(ex_div), .read(read), .rdy(rdy), .numLS(numLS), .numMS(numMS), .den(den), .status(status), .remainder(remainder), .quotient(quotient), .wr(wr), .rd(rd), .clr(clr), .clk(clk), .addr(a));
  division_unit div(.numLS(numLS), .numMS(numMS), .den(den), .ex_div(ex_div), .rd(read), .clr(clr), .clk(clk), .quo(quotient), .rem(remainder), .status(status), .irq(irq));
  
endmodule

//the reg file
module reg_file(inout [15:0] d, output ex_div, read, output reg rdy, output reg [15:0] numLS, numMS, den, input [15:0] status, remainder, quotient, input wr, rd, clr, clk, input [1:0] addr);

  reg [15:0] data;
  wire [15:0] wr_data;
  wire [15:0] reg_status, reg_rem, reg_quo;
  //if writing to address 3, output that divison is to be executed
  assign ex_div = (wr && (addr == 2'd3)) ? 1'b1 : 1'b0;
  //bidirectional tri-states
  assign d = rd ? data : 16'bz;
  assign wr_data = wr ? d : 16'bz;
  assign read = (rd && (addr == 2'd3)) ? 1'b1 : 1'b0;
  
  //handles input and output logic
  always@(posedge clk, posedge clr) begin
    if (clr) begin
      numLS <= 16'b0;
      numMS <= 16'b0;
      den <= 16'b0;
      rdy <= 1'b0;
      data <= 16'b0;
    end
    else begin
      data <= 16'bz;
      rdy <= 1'b0;
	  //if address is 0
      if(addr == 2'd0) begin
		//if writing, store the least significant bits of the numerator and rdy goes high
        if(wr) begin
          numLS <= wr_data;
          rdy <= 1'b1;
        end
		//if reading, output the status and rdy goes high
        else if(rd) begin
          data <= status;
          rdy <= 1'b1;
        end
      end
	  //if address is 1
      else if(addr == 2'd1) begin
	    //if writing, store the least significant bits of the numerator and rdy goes high
        if(wr) begin
          numMS <= wr_data;
          rdy <= 1'b1;
        end
		//if reading, output 0's and rdy goes high
        else if(rd) begin
          data <= 16'd0;
          rdy <= 1'b1;
        end
      end
	  //if address is 2
      else if(addr == 2'd2) begin
		//if writing, store the denominator and rdy goes high
        if(wr) begin
          den <= wr_data;
          rdy <= 1'b1;
        end
		//if reading, output the remainder and ready goes high
        else if(rd) begin
          data <= remainder;
          rdy <= 1'b1;
        end
      end
	  //if address is 3
      else if(addr == 2'd3) begin
		//if reading, output the quotient and ready goes high
        if(rd) begin
          data <= quotient;
          rdy <= 1'b1;
        end
		//if writing, ready goes high
        if(wr) begin
          rdy <= 1'b1;
        end
      end
    end
  end
endmodule

//the divison unit
module division_unit(input [15:0] numLS, numMS, den, input ex_div, rd, clr, clk, output [15:0] status, output reg [15:0] quo, rem, output reg irq);
  
  //states
  localparam IDLE = 5'd0, DIVIDE = 5'd1, STOP = 5'd17;
  reg [4:0] state, next_state;
  reg [32:0] numerator, denominator, tc_denominator;
  reg [32:0] sub_result;
  reg [15:0] quotient, remainder;
  reg overflow, div_zero;
  //the status of the divider/division
  assign status = {13'b0, overflow, div_zero, ~irq};
  reg [4:0] end_early;
  
  //sequential behavior
  always@(posedge clk, posedge clr) begin
    if(clr) begin
      state <= IDLE;
      quo <= 16'b0;
      rem <= 16'b0;
      numerator <= 33'b0;
      denominator <= 33'b0;
      tc_denominator <= 33'b0;
      quotient <= 16'b0;
      remainder <= 16'b0;
      overflow <= 1'b0;
      div_zero <= 1'b0;
      end_early <= 1'b0;
    end
    else begin
      state <= next_state;
	  //if in IDLE, initialize the divider
      if(state == IDLE) begin
          numerator <= {1'b0, numMS, numLS};				//set the numerators
          denominator <= {1'b0, den, 16'b0};				//set the denominator
          tc_denominator <= ~{1'b0, den, 16'b0} + 1'b1;		//two's complement of the denominator
          quotient <= 16'b0;			
          remainder <= 16'b0;
          overflow <= 1'b0;
          div_zero <= 1'b0;
          end_early <= 1'b0;
		  //if executing division
          if(ex_div) begin
			//if most significant of numerator is greater than or equal to denominator, overflow has occurred
            if(numMS >= den) begin
              overflow <= 1'b1;
            end
			//if denominator is zero, divide by zero error
            if(den == 16'b0) begin
              div_zero <= 1'b1;
            end
          end
      end
	  //if not in the STOP state
      else if(state != STOP) begin
		//shift the quotient left with complement of MSB of sub_result (1 if positive, 0 if negative)
        quotient <= {quotient[14:0], ~sub_result[32]};
		//if remainder was negative, restore the numerator
        if(sub_result[32])
          numerator <= sub_result + denominator;
		//if remainder becomes zero, divison is done
        else if(sub_result == 33'b0)
          end_early <= state;
		//otherwise update the numerator register
        else
          numerator <= sub_result;
      end
	  //if in STOP state
      else begin
		//if executing division right away, initialize
        if(ex_div) begin
          numerator <= {1'b0, numMS, numLS};				//set the numerators
          denominator <= {1'b0, den, 16'b0};				//set the denominator
          tc_denominator <= ~{1'b0, den, 16'b0} + 1'b1;		//two's complement of the denominator	
          quotient <= 16'b0;
          remainder <= 16'b0;
          overflow <= 1'b0;
          div_zero <= 1'b0;
          end_early <= 1'b0;
		  //if most significant of numerator is greater than or equal to denominator, overflow has occurred
          if(numMS >= den) begin
            overflow <= 1'b1;
          end
		  //if denominator is zero, divide by zero error
          if(den == 16'b0) begin
            div_zero <= 1'b1;
          end
        end
		//otherwise
        else begin
		  //if end early happpened
          if(end_early != 4'd0) begin
            quo <= (quotient << ((STOP-1'b1) - end_early));		//shift out remainder of quotient at once
            rem <= 16'b0;										//remainder is 0
          end
		  //otherwise output quotient and remainder
          else begin
            quo <= quotient;
            rem <= numerator[31:16];
          end
        end
      end
    end
  end

  //combinational behavior
  always@(*) begin
    irq = 1'b1;
    sub_result = 32'b0;
    case(state)
      IDLE:
		//if IDLE and executing division, go to DIVIDE
        if(ex_div) begin
          next_state = DIVIDE;
        end
		//otherwise stay in IDLE
        else
          next_state = IDLE;
      STOP: begin
        irq = 1'b0;									//interrupt 
		//if reading, go to IDLE state
        if(rd) begin
          next_state = IDLE;
        end
		//if executing division, go straight to DIVIDE
        else if(ex_div) begin
          next_state = DIVIDE;
        end
		//otherwise stay in STOP
        else
          next_state = STOP;
      end
	  //if executing a division
      default: begin
		//subtract the denominator from the numerator
        sub_result = {numerator[31:0], 1'b0} + tc_denominator;
		//if result is 0, or overflow, or divide by zero, go to STOP state
        if(sub_result == 33'b0 || overflow || div_zero)
          next_state = STOP;
		//if result is negative, update state
        else if(sub_result[32]) begin
          next_state = state + 1'b1;
        end
		//otherwise, update state
        else begin
          next_state = state + 1'b1;
        end
      end
    endcase
  end

endmodule


