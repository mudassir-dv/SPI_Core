//////////////////////////////////////////////////////////////////
//                                                          	//
// FIFO MODULE                              			//
//                                                          	//
// design      : parameterized fifo Buffer    			//
// Project     : spi_master core                            	//
//								//
// Description :                                            	//
//   - A configurable FIFO buffer module with parameterized 	//
//     width and and depth for versatile data buffering.    	//
//   - Serves as an efficient buffer for data transmission  	//
//     and reception to avoid the condition that is called  	//
//   "data overrun"                                         	//
// Features:                                                	//
//   - Configurable width and depth (default: 4x8).         	//
//   - Status indicators for full and empty states.        	//
//   - Robust pointer-based implementation for optimal    	//
//     performance.                                        	//
//								//
//////////////////////////////////////////////////////////////////
//                                                          	//
// Copyright (C) 2024 Md Mudassir Ahmed                     	//
//                                                          	//
// This source file is free for all use and distribution   	//
// "as is." No restrictions apply to its use, modification, 	//
// or redistribution provided that this notice remains 		//
//  included in any derivative work.				//
//								//
// License Terms:                                          	//
// 1. You are free to use, modify, and distribute this    	//
//    code for any purpose, including commercial 		//
//	  applications.						//
// 2. You must retain this notice and the disclaimer  		//
//    in all copies or substantial portions of the code.    	//
//                                                          	//
// Disclaimer:                                      		//
// THIS SOFTWARE IS PROVIDED "AS IS" WITHOUT WARRANTY  		//
// OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT		//
// LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 		//
// FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT.	//
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY CLAIM,	//
// DAMAGES, OR OTHER LIABILITY, WHETHER IN AN ACTION OF		//
// CONTRACT, TORT, OR OTHERWISE, ARISING FROM, OUT OF,		//
// OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER  	//
// DEALINGS IN THE SOFTWARE.					//
//////////////////////////////////////////////////////////////////

module spi_fifo #(parameter 	WIDTH = 8, 
				DEPTH = 4) 
	(
		input 			clk, 
		input   		rst, 
		input			we, 
		input			re, 
		input	   [WIDTH-1:0] 	din, 
		output     [WIDTH-1:0] 	dout, 
		output 			full, 
		output 			empty
	);
	
	// Declare memory (Default Configuration of 4x8)
	reg [WIDTH-1:0] mem [DEPTH-1:0]; 
	
	// Declare internal read, write pointers
	reg [$clog2(DEPTH):0] rd_ptr, wr_ptr; 		
	
	// fifo data output
  	assign  dout = (!rst)? 8'b0 : mem[rd_ptr];
	
	// Write Logic
	always @(posedge clk, negedge rst)
		begin
			if(we && (!full)) 
				begin
					mem[wr_ptr] <= din;
				end
		end
		
	// Pointer Logic
	always @(posedge clk, negedge rst)
		begin
			if(!rst)
				begin
					rd_ptr <= 0;
					wr_ptr <= 0;
				end
			else
				begin
					if(re && (!empty))
						rd_ptr <= rd_ptr + 1;
					if(we && (!full))
						wr_ptr <= wr_ptr + 1;
				end
		end
	
	
	// assign outputs full and empty
	assign full  = ((wr_ptr[$clog2(DEPTH)] != rd_ptr[$clog2(DEPTH)]) &&(wr_ptr[$clog2(DEPTH)-1:0] == rd_ptr[$clog2(DEPTH)-1:0]))? 1'b1 : 1'b0;
	assign empty = (wr_ptr == rd_ptr)? 1'b1 : 1'b0;
	
endmodule
