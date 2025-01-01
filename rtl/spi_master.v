//////////////////////////////////////////////////////////////
//                                                          //
//	SPI_MASTER_CORE             						    //
//															//
//  design 	    : spi_master top module                     //
//  project     : spi_master core                   		//
//				  											//
//  Description :                                           //
//   - A SPI Master module with support for configurable  	//
//     modes, variable data word transfer and variable 		//
//     clock frequency.								        //
//   - WISHBONE interface for integration.          		//
//   - Enables data transmission and reception with robust  //
//     buffer management to prevent "data overrun"   		//
//	   conditions.                   						//
//                                                          //
// Features:                                                //
//   - Variable data word transfer (default: 8 bits).       //
//   - Flexible clock divider for frequency scaling.        //
//   - Supports SPI transmission and reception with 		//
//     LSB/MSB first modes.									//
//   - Full-duplex SPI communication with configurable 		//
//     control registers.									//
//   - SPI communication with all configurable modes		//
//                                                          //
//	  -----------------------------------------				//
//	  |    Modes	|	 cpol 	|	 cpha	  |				//
//	  -----------------------------------------				//
//	  |	   Mode 0 	|  	  0  	|	  0		  |				//
//	  |	   Mode 1 	|     0  	|	  1		  |				//
//	  |	   Mode 2 	|     1  	|	  0		  |				//
//	  |	   Mode 3 	|     1  	|	  1		  |				//
//	  -----------------------------------------				//
//                                                          //
//////////////////////////////////////////////////////////////
//                                                          //
// Copyright (C) 2024 Md Mudassir Ahmed                     //
//                                                          //
// This source file is free for all use and distribution    //
// "as is." No restrictions apply to its use, modification, //
// or redistribution provided that this notice remains 		//
//  included in any derivative work.						//
//															//
// License Terms:                                           //
// 1. You are free to use, modify, and distribute this    	//
//    code for any purpose, including commercial 			//
//	  applications.											//
// 2. You must retain this notice and the disclaimer  		//
//    in all copies or substantial portions of the code.    //
//                                                          //
// Disclaimer:                                      		//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY  		//
// OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT		//
// LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 			//
// FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.	//
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM,	//
// DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF		//
// CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF,		//
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER  	//
// DEALINGS IN THE SOFTWARE.								//
//////////////////////////////////////////////////////////////

module spi_master#(parameter WIDTH = 8)(
    // WISHBONE interface
    input  	        		wb_clk_i,
    input  	        		wb_rst_i,
    input  	          [2:0] wb_adr_i,
    input  	    [WIDTH-1:0] wb_dat_i,
    output reg  [WIDTH-1:0] wb_dat_o,
//  input  	 	      [3:0] wb_sel_i,
    input  	        		wb_we_i,
    input  	        		wb_stb_i,
    input  	        		wb_cyc_i,
    output 	        		wb_ack_o,
    output 	        		wb_err_o,
    output 	        		wb_int_o,

    // SPI external connections
    output reg   	  [2:0] ss_pad_o,	//slave select
    output reg        		sclk_pad_o,	//output clock line for synchronous xmission and reception
    output reg        		mosi_pad_o,	//output xmission line
    input           		miso_pad_i	//input reception line
);

	// Internal Registers
	reg 		 	[7:0] ctrl;		//Control Register
	wire		  	[3:0] spsr;		//Status Register (Read-Only)
	reg 		  	[2:0] divider;	//Divider and
	reg 		  	[2:0] ss_n;		//Slave Select Register
	reg					  cpol;		
	reg					  cpha;

	// WISHBONE interface signals
	reg        			wb_ack;
	reg       			wb_err;
	reg        			wb_int;

	// SPI shift register
	reg   		  [7:0] spi_shift_tx;
	reg   		  [7:0] spi_shift_rx;
	reg     	  [4:0] spi_shift_cnt;	

	// Clock generator
	reg   		  [3:0] clk_div;

	// Internal control signals
	reg         		ass;
	reg         		ie;
	reg         		lsb;
	reg         		tx_neg;
	reg         		rx_neg;
	reg         		go_bsy;
	reg   		  [1:0] char_enc;	
	reg   		  [6:0] char_len;
	
	// Internal status flags of spsr registers
	assign spsr[3]   = wfull;
	assign spsr[2]   = wempty;
	assign spsr[1]   = rfull;
	assign spsr[0]   = rempty;

	// State machine states -> gray-encoding
 	parameter [1:0] IDLE      = 2'b00,
 					START     = 2'b01,
 					TRANSFER  = 2'b11,
 					END       = 2'b10;

	reg [1:0] state;
	
	// declare fifo signals
	wire wfull, wempty;
	reg  wfre, wfwe;
	wire [WIDTH-1:0] wdout;
	reg  [WIDTH-1:0] wdin;
	wire rfull, rempty;
	reg  rfwe, rfre;
	wire [WIDTH-1:0] rdout;
	
	// instantiate 4x8 spi_fifo -> buffer space for data to be xmitted
	spi_fifo w_fifo(
			.clk(wb_clk_i),
			.rst(~wb_rst_i),
			.we(wfwe),
			.re(wfre),
			.din(wdin),
			.dout(wdout),
			.full(wfull),
			.empty(wempty)
	);
	
	// instantiate 4x8 spi_fifo -> buffer space for data received
	spi_fifo r_fifo(
			.clk(wb_clk_i),
			.rst(~wb_rst_i),
			.we(rfwe),
			.re(rfre),
			.din(spi_shift_rx),
			.dout(rdout),
			.full(rfull),
			.empty(rempty)
	);
	
	//decode transfers character lenght
	always @(*)
		begin
		    case(char_enc)
			2'b00	: char_len = 6'd8;			//1-byte
			2'b01	: char_len = 6'd16;			//2-bytes
			2'b10	: char_len = 6'd32;			//3-bytes
			default : char_len = 6'd8;			//default 1-byte
		    endcase
		end
		
	
	// wb access
	wire wb_acc = wb_stb_i & wb_cyc_i;
	
	// WISHBONE interface
	always @(posedge wb_clk_i) 
		begin
		    if (wb_rst_i) begin
			// Reset WISHBONE interface signals
			wb_ack <= 1'b0;
			wb_err <= 1'b0;
			wb_int <= 1'b0;
		    end 
			
		    else if (wb_acc) begin
				// Handle WISHBONE bus cycle
				case (wb_adr_i)
				3'b000 : begin
					// Data receive register 0
					if (wb_we_i) begin
						if(~wfull) begin
							// Write data w_fifo register
							wdin <= wb_dat_i;
							wfwe <= 1'b1;
						end
						else
							wfwe <= 1'b0;
					end 
					else begin
						if(~rempty) begin
							// Read data w_fifo register
							wb_dat_o <= rdout;
							rfre <= 1'b1;
						end
						else 
							rfre <= 1'b0;
					end
				end
							
				3'b001 : begin
					// Divider and slave select register
					if (wb_we_i) begin
						// Write to divider & slave select register
						{cpol, cpha, divider, ss_n} <= wb_dat_i;
					end 
					else begin
						// Read from divider & slave select register
						wb_dat_o <= {cpol, cpha, divider, ss_n};
					end
				end
							
				3'b010 : begin
					// Control register
					if (wb_we_i) begin
						// Write to control register
						ctrl 	<= wb_dat_i;
						ass  	<= wb_dat_i[7];
						ie   	<= wb_dat_i[6];
						lsb  	<= wb_dat_i[5];
						tx_neg 	<= wb_dat_i[4];
						rx_neg 	<= wb_dat_i[3];
						go_bsy 	<= wb_dat_i[2];
						char_enc <= wb_dat_i[1:0];
					end 
					else begin
						// Read from control register
						wb_dat_o <= ctrl;
					end
				end
							
				3'b011 : begin
					// Status register
					if (~wb_we_i) begin
						// Read-only register
						wb_dat_o <= spsr;		
					end
					else begin
						//error!! (cannot write into spsr)
						wb_err <= 1'b1;
					end
				end	
							
				default: begin
					// Invalid address
					wb_err <= 1'b1;
					end
				endcase
				wb_ack <= 1'b1;
			end 
			
			else begin
				wb_ack <= 1'b0;
			end
		end

	// spi state machine
	always @(posedge wb_clk_i) 
		begin
			if (wb_rst_i) begin
				// Reset state machine
				state <= IDLE;
				sclk_pad_o <= cpol;
				mosi_pad_o <= 1'bz;
				ss_pad_o   <= 3'b111;
			end 
			else begin
				case (state)
					IDLE: begin
						if (go_bsy) begin
							state <= START;
							wfre <= 1'b0;
							rfwe <= 1'b0;
						end
					end
					
					START : begin
							// Set slave select
							ss_pad_o <= (ass)? ss_n : 3'b111;
							
							// Set clock and initialize registers
							sclk_pad_o 	<= cpol;
							mosi_pad_o 	<= 1'bz;
							clk_div 	<= 4'd0;
							char_len 	<= (mode)? {char_len << 1} : char_len;
							spi_shift_cnt <= 5'd0;
							spi_shift_rx  <= 8'h00;

							// Transfer fifo data into shift register
							if(~wempty && tx_neg) begin 
								wfre <= 1'b1;
								spi_shift_tx <= wdout;
								state <= TRANSFER;
							end
							else begin
								state <= (rx_neg)? TRANSFER : END;
							end
						end
					
					TRANSFER : begin
							if (clk_div == divider) begin
								// Clock edge
								clk_div <= 16'd0;
								sclk_pad_o <= ~sclk_pad_o;
								
								// xmission at posedge of sclk
								if (tx_neg && sclk_pad_o) begin
									// Transfer byte
									mosi_pad_o 	 <= (lsb)? spi_shift_tx[0] : spi_shift_tx[7];
									spi_shift_tx <= (lsb)? {1'b0, spi_shift_tx[7:1]} : {spi_shift_tx[6:0], 1'b0};
									spi_shift_cnt <= spi_shift_cnt + 1'b1;
								end 
								
								// reception at negedge of sclk
								if (rx_neg && ~sclk_pad_o) begin
									// Receive byte
									spi_shift_rx <= (lsb)? {miso_pad_i, spi_shift_rx[7:1]} : {spi_shift_rx[6:0], miso_pad_i};
									spi_shift_cnt <= spi_shift_cnt + 1'b1;
								end
								
								
								if (spi_shift_cnt == char_len) begin
									state <= END;
								end
							end 
							else begin
								// Clock counter
								clk_div <= clk_div + 1'b1;
							end
						end
					
					END: begin
						// write received data into fifo
						if(~rfull && rx_neg) begin
							rfwe <= 1'b1;
						end
						else begin
							rfwe <= 1'b0;
						end
						
						// Set slave select
						ss_pad_o <= 3'b111;
						wb_int   <= 1'b1;
						state    <= IDLE;
                      	mosi_pad_o <= 1'bz;
					end
				endcase
			end
		end  
	
	//check if both xmission and reception occur during the same bus access
	wire mode = tx_neg & rx_neg;
	
	//output logic for acknowledgement, interrupt and error
	assign wb_ack_o = wb_ack;
	assign wb_err_o = wb_err;
	assign wb_int_o = (ie) ? wb_int : 1'b0;

endmodule
