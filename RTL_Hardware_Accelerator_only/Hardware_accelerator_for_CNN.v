

////////////////////////////////////////////////////////////////////////////////////////////////

module improved_logic (
  input  clk,
  input  rst_n,
  
  input  [31:0] in_data,
  input  start,
  
  output reg [31:0] final_out_data,
  output reg out_valid,
  output  reg in_valid_out, 
    
    
   
     input       data_or_wt,

    // Matrix Parameters
    input  [31:0]  input_start_addr,
    input  [31:0]  output_start_addr,
    
    input   [10:0]  IN_SIZE,          // full IMAGE size
    input   [10:0]  IN_CHANNELS,      // Total channels
    input   [10:0]  OUT_SIZE,          // out full IMAGE size
    input   [10:0]  OUT_CHANNELS,      // Total no.of Filters
    

    // Outputs
    output  [31:0] load_addr,
    output  [31:0] store_addr,
    output reg done_out,

    output reg out_valid_2,
    output in_valid_101
    
  
);



  reg rst_n1, in_v ,  c, c11;
 
  wire [31:0] out_data_1, out_data_2, out_data_3, out_data_4;
  wire out_valid1, out_valid3, out_valid4;
  
  
  reg [31:0] in_data1, in_data2, in_data3, in_data4, indata;
  reg [31:0] in1, in2, in3, in4;
  
  
  reg [2:0] count;
  reg [31:0] inw1, inw2, inw3, inw4;
  
  wire wt_valid, done_wt, wt_valid1, input_valid1;

  wire out_valid_101,done_out_temp;
  reg done_out1, done_out2, done_out3,done_out4, done_out5, done_out6, done_out7, done_out8, done_out9, done_out10, done_out11, done_out12, done_out13, done_out14, done_out15, done_out16, done_out17, done_out18, done_out19,  done_out20;
  reg done_out21, done_out22, done_out23, done_out24, done_out25, done_out26, done_out27, done_out28,  temp2;
  
  wire done_wt_1;
  reg [31:0] final_out_data_temp;
  //reg out_valid_2;
  

  
   assign in_valid_101 = input_valid1 | wt_valid1 ;
   assign done_out_temp = (done | (done_wt ));
  
  
  always@(posedge clk) begin
      final_out_data <= final_out_data_temp;
               
     done_out1 <= done_out_temp;
     done_out2 <= done_out1;
     done_out3 <= done_out2;
     done_out4  <= done_out3;
     done_out5 <= done_out4;

     done_out6  <= done_out5;
    done_out7  <= done_out6;
    done_out8  <= done_out7;
    done_out9  <= done_out8;
    done_out10 <= done_out9;
    done_out11 <= done_out10;
    done_out12 <= done_out11;
    done_out13 <= done_out12;
    done_out14 <= done_out13;
    done_out15 <= done_out14;
    done_out16 <= done_out15;
    done_out17 <= done_out16;
    done_out18 <= done_out17;
    done_out19 <= done_out18;
    done_out20 <= done_out19;
    done_out21 <= done_out20;
    done_out22 <= done_out21;
    



    if (start && !data_or_wt)
    done_out   <= done_out20;

    else  if (start && data_or_wt)
    done_out   <= done_out21;

    else
    done_out <= 0;

     
     
     

     
  end
  

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
  
  control_unit cu1(
       .clk(clk),
       .rst_n(rst_n),
       .data_or_wt(data_or_wt),
       .start(start),
       .input_valid(in_valid),
       .input_valid1(input_valid1),
       .wt_valid(wt_valid),
       .wt_valid1(wt_valid1),
       .done_wt(done_wt) ,                
       .done(done),
       .count(count),
       .done_wt_1(done_wt_1 ) );
  
 conv3x3_stream_pipe c1( clk, rst_n, in1, in_v, out_data_1, out_valid1, wt_valid,start, in_valid);
 conv3x3_stream_pipe c2( clk, rst_n, in2, in_v, out_data_2, out_valid2, wt_valid, start, in_valid);
 conv3x3_stream_pipe c3( clk, rst_n, in3, in_v, out_data_3, out_valid3, wt_valid,start, in_valid);
 conv3x3_stream_pipe c4( clk, rst_n, in4, in_v, out_data_4, out_valid4, wt_valid,start, in_valid);
 
 
 
   address_calculator /* #(
    .SUB_ROWS(SUB_ROWS),
    .SUB_COLS(SUB_COLS),
    .SUB_CHANNELS(SUB_CHANNELS),
    .SUB_FILTERS(SUB_FILTERS)
  )*/  acu1 (
    .clk(clk),
    .rst_n(rst_n),
    .input_valid(input_valid1),
    .output_valid(out_valid1),
    .input_start_addr(input_start_addr),
    .output_start_addr(output_start_addr),
    .IN_SIZE(IN_SIZE),
    .IN_CHANNELS(IN_CHANNELS),
    .wt_valid1(wt_valid1),
    
    
    .OUT_SIZE(OUT_SIZE),
    .OUT_CHANNELS(OUT_CHANNELS),
    
    .load_addr(load_addr),
    .store_addr(store_addr),
    
    .done_wt(done_wt),
    .done(done),
    .count(count),
    .done_wt_1(done_wt_1)
  );
  
 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////  
 
  
 always @(posedge clk ) begin
   out_valid_2 <= out_valid1;
   out_valid <= out_valid_2;
   
   in_valid_out <= in_valid_101;
   if (data_or_wt == 0)
   indata <= in_data;
   else if (count[0] == 1)
   indata <= in_data;
     
 end



 

  always @(posedge clk ) begin  // or negegde rst_n
    if (!rst_n ||  (done_out == 1))
     begin
      
      count <= 0;
      temp2 <= 0;
      rst_n1 <= 0;
      c <= 0;
      c11<= 0;
     end
    
    else if (data_or_wt && start && temp2)
      count <= count + 1;

      else if (data_or_wt && start && !temp2)
      temp2 <= 1;
      
     else if (start)
       count <= count + 2;
      
      
      ////////////////
      
      
      ///////////////
      
    // Example computation and control
    final_out_data_temp <= out_data_1 + out_data_2 + out_data_3 + out_data_4;
    //out_valid <= out_valid1;

      in_v <= in_valid && (c == 1);
     //in_valid1 <= in_valid;   
        
    case (count)
      3'b000: begin
        if (data_or_wt == 0 && c == 1)
        in_data1 <= indata;          //
        if (data_or_wt == 1)
        in_data3 <= indata;
        c <= 1;
        end
    
   
      
      
      
      3'b010: begin
      if (data_or_wt == 0)
        in_data2 <= indata;
         else if (data_or_wt == 1)
        in_data4 <= indata;
      end
       
      
      
      
     
      
      
      3'b100: begin
       if (data_or_wt == 0)
        in_data3 <= indata;
        else if (data_or_wt == 1 && c11 == 1)
        in_data1 <= indata; 
        c11 <= 1;
       end
       
       
        
   
      
      
      3'b110: begin
       if (data_or_wt == 0)
        in_data4 <= indata;
        else if (data_or_wt == 1)
        in_data2 <= indata;
      end
      
      
  
        
        
        
    
      
     
    endcase
  


 if (count== 6 && !data_or_wt)
          begin
              
               in1 <= in_data1;
               in2 <= in_data2; 
               in3 <= in_data3;
               in4 <= indata;
               
            end
 else if  (count== 2)
     begin
             
               in1 <= in_data1;
               in2 <= in_data2; 
               in3 <= in_data3;
               in4 <= indata;
               
            end
  
 

end
endmodule


//////////////////////////////////////////////////////////////////////////////////

module control_unit #(
    parameter SUB_ROWS = 8,
    parameter SUB_COLS = 8,
    parameter SUB_CHANNELS = 4,
    parameter SUB_FILTERS = 4,
    parameter KERNEL_ROWS = 3,
    parameter KERNEL_COLS = 3,
    parameter KERNEL_FILTERS = 4,
    parameter IN_COUNT = SUB_ROWS * SUB_COLS * SUB_CHANNELS
    
) (
    input   clk,
    input   rst_n,
    input data_or_wt,
    input   start,
    output reg  input_valid,
    output reg  input_valid1,
    output reg wt_valid,
    output reg wt_valid1,
    input done_wt,
    input done_wt_1,
    input done,
    input [2:0] count
);

reg  wt_valid2, wt_valid3, wt_valid4, wt_valid5, wt_valid6;
reg  input_valid2, input_valid3, input_valid4, input_valid5, input_valid6, input_valid7,  input_valid8,  input_valid9, input_valid10,  input_valid11, input_valid12, input_valid13, input_valid14, input_valid15, input_valid16, input_valid17, input_valid18, input_valid19, input_valid20;

reg [IN_COUNT-1 :0 ] in_count;

always @(posedge clk ) 
begin
   wt_valid2 <= wt_valid1;
   wt_valid3 <= wt_valid2;
   wt_valid4<= wt_valid3;
   wt_valid  <= wt_valid4;
  // wt_valid6 <= wt_valid5;
  // wt_valid <= wt_valid6;
   
   
   input_valid2 <= input_valid1;
   input_valid3<= input_valid2;

   input_valid4 <= input_valid3;
   input_valid5 <= input_valid4;

   
   input_valid6 <= input_valid5;
   input_valid7 <= input_valid6;
   
   input_valid8 <= input_valid7;
   input_valid9 <= input_valid8;

   
   input_valid10 <= input_valid9;
   input_valid11 <= input_valid10;
   input_valid12 <= input_valid11;
   input_valid13  <= input_valid12;
 
 input_valid14 <= input_valid13;
   input_valid15 <= input_valid14;
   input_valid16 <= input_valid15;
   input_valid  <= input_valid16;

   

end

reg done_t1, done_t2;

always@(posedge clk ) begin
    if (!rst_n)
        begin
          done_t1 <= 0;
          done_t2<= 0;
        end
     else 
        begin
          done_t1 <=  done;
          done_t2<=  done_t1;
        end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        begin
          input_valid1 <= 1'b0;
          wt_valid1 <= 0;
        end
        
     else if (start && data_or_wt && done_t2)
             input_valid1 <= 0;    
        
    else if (start && data_or_wt )
         begin
             input_valid1 <= ~input_valid1;  // Toggle when start is high
             wt_valid1 <=  0; 
          end
          
    else if (start && (!data_or_wt) && (!done_wt_1))
              wt_valid1 <=  1;                   //~wt_valid;
        
        
    else
       begin
        input_valid1 <= 1'b0;          // Reset when start is low
        wt_valid1 <= 0;
       end
end

endmodule

/////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////


module address_calculator #(
    parameter SUB_ROWS = 8,
    parameter SUB_COLS = 8,
    parameter SUB_CHANNELS = 4,
    parameter SUB_FILTERS = 4,
    parameter KERNEL_ROWS = 3,
    parameter KERNEL_COLS = 3,
    parameter KERNEL_FILTERS = 4
    
)(
    input         clk,
    input         rst_n,

    // Triggers
    input       input_valid,
    input       output_valid,
    input       wt_valid1,

    // Matrix Parameters
    input  [31:0]  input_start_addr,
    input  [31:0]  output_start_addr,
    input   [10:0]  IN_SIZE,          // full IMAGE size
    input   [10:0]  IN_CHANNELS,      // Total channels
    input   [10:0]  OUT_SIZE,          // full IMAGE size
    input   [10:0]  OUT_CHANNELS,
    

    // Outputs
    output reg [31:0] load_addr,
    output reg [31:0] store_addr,
    
    output reg done_wt,
    output reg done,
    input [2:0] count,
    output reg done_wt_1
);

    reg done_wt_temp, done_wt_1_temp;
    reg done_temp;

    // Internal counters for input
    reg [2:0] in_row, in_col;
    reg [1:0] in_ch;

    // Internal counters for output
    reg [2:0] out_row, out_col;
    reg [1:0] out_ch, k_col, k_row, k_ch, k_filter;

    always @(posedge clk ) begin
     done_wt <= done_wt_temp;
     done_wt_1 <= done_wt_1_temp;
    end  


    always @(posedge clk or negedge rst_n) begin                       // negedge fix
        if (!rst_n) begin
            in_row      <= 0;
            in_col      <= 0;
            in_ch       <= 0;
            load_addr  <= input_start_addr;

            out_row     <= 0;
            out_col     <= 0;
            out_ch      <= 0;
            store_addr <= output_start_addr;
            
            k_col     <= 0;
            k_row     <= 0;
            k_ch      <= 0;
            k_filter  <= 0;
            done_wt_1_temp <= 0;
            done_wt_temp <= 0;
            done <=0;
            done_temp <=0;

        end else begin
             ////////////////
             if (k_filter == 3 && k_ch == 2 && k_row == 2 && k_col == 2 && count == 6 )      //
             begin
                done_wt_temp <= 1;
                done_wt_1_temp <= 1;
              end
             else
               done_wt_temp <= 0;
            ///////////////////             
            // weight address stepping  // 4 factors
            if (wt_valid1 )
              begin 
               load_addr <= ((input_start_addr >> 2 )+(k_filter  * KERNEL_ROWS * KERNEL_COLS *  IN_CHANNELS) + (k_row * KERNEL_COLS *  IN_CHANNELS )   +     (k_col * IN_CHANNELS)    +    k_ch)*4;
                      ////  kernel [filter][row][column][channel]
              
                if (k_ch < SUB_CHANNELS -1)
                    k_ch <= k_ch + 1;
                else begin
                    k_ch <= 0;
                    if (k_col < KERNEL_COLS - 1)
                        k_col <= k_col + 1;
                    else begin
                        k_col <= 0;
                        if (k_row < KERNEL_ROWS - 1)
                            k_row <= k_row + 1;
                        else begin
                            k_row <= 0;
                            if (k_filter < KERNEL_FILTERS - 1)
                            k_filter <= k_filter + 1;
                        end
                    end
                end
            end
              
              
              
              
              
            
            // input[row][column][channel]
            // Input address stepping
            else if (input_valid ) begin
            
                 if (in_ch == SUB_CHANNELS-1  && in_col == SUB_COLS -1 && in_row  == SUB_ROWS-1  )
                 done <= 1;
            
                load_addr <= ((input_start_addr >> 2) + (in_row * IN_SIZE *  IN_CHANNELS )   +     (in_col * IN_CHANNELS)    +    in_ch)*4;

                if (in_ch < SUB_CHANNELS - 1)
                    in_ch <= in_ch + 1;
                else begin
                    in_ch <= 0;
                    if (in_col < SUB_COLS - 1)
                        in_col <= in_col + 1;
                    else begin
                        in_col <= 0;
                        if (in_row < SUB_ROWS - 1)
                            in_row <= in_row + 1;
                        else
                            in_row <= 0;
                    end
                end
            end

            // Output address stepping
            if (output_valid) begin
                store_addr <= ( (output_start_addr >> 2) + (out_row * OUT_SIZE * OUT_CHANNELS) +     (out_col * OUT_CHANNELS)     + out_ch)*4;

                if (out_ch < SUB_CHANNELS - 1)
                    out_ch <= out_ch + 1;
                else begin
                    out_ch <= 0;
                    if (out_col < (SUB_COLS-2) - 1)
                        out_col <= out_col + 1;
                    else begin
                        out_col <= 0;
                        if (out_row < (SUB_ROWS-2) - 1)
                            out_row <= out_row + 1;
                        else
                            out_row <= 0;
                    end
                end
            end
        end
    end
endmodule


/////////////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////////////////

module conv3x3_stream_pipe
 #( parameter IMG_W    = 8
  , parameter IMG_H    = 8
  , parameter DATA_W   = 32
  , parameter K        = 2
  ,  parameter SUB_ROWS = 8,
    parameter SUB_COLS = 8,
    parameter SUB_CHANNELS = 4,
    parameter SUB_FILTERS = 4,
    parameter KERNEL_ROWS = 3,
    parameter KERNEL_COLS = 3,
    parameter KERNEL_FILTERS = 4
  
  )
 (
  input                      clk,
  input                      rst_n,
  input      [DATA_W-1:0]    in_data,
  input                      in_valid,
  output reg [31:0]          out_data,
  output reg                 out_valid,
 // output reg [31:0]          out_address,
  input                       wt_valid,
  input                       start,
  //output reg                  done_wt
  input in_valid_1000
 );


localparam BANKS   = K+1 ;            //  banks
   localparam AW      =   6;// $clog2(IMG_W*IMG_H)
   localparam HAW     =  3; //  $clog2(IMG_H)


  //=====================================================================
  // 1) WRITE STAGE: rotating write bank + row pointer
  //=====================================================================
  reg [31:0]          out_address;
  
  reg out_valid_1, out_valid_2, out_valid_3, out_valid_4, out_valid_5,out_valid_6,out_valid_7, out_valid_8, out_valid_9, out_valid_10, out_valid_11, out_valid_12, out_valid_13, out_valid_14, out_valid_15, out_valid_16, out_valid_17, out_valid_18, out_valid_19 , out_valid_20;
  reg count1 ;
  reg[4:0] count2 ; 
  
  reg signed [DATA_W-1:0] kernel [0:3][0:2][0:2];
  
  
  reg [2:0] k;

  reg [HAW-1:0]           row_ptr, row_ptr_temp, row_ptr1, row_ptr2, row_ptr3,  row_ptr4;
  reg [1:0] wr_bank, wr_bank1 ;
  // rotate banks 0→1→2→3→0...
  
  wire [1:0] a,b;
  wire [2*DATA_W-1:0] temp_mul_product[0:2][0:2];
  
  reg [6:0]  c1,c2,c3;
  reg [1:0]  c0;
  
  reg temp_001;
  reg [3:0] count_row;
  reg flag2;
  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  assign a = k[2:1];
  assign b =  a;  // syc
  

  
  ////////////////////////////////////////////////////////////////////////////////////////////////////
  
  
  
  
  always @(posedge clk ) begin  // or negedge rst_n
  if (!rst_n) begin
  
    
  integer i1, j1, k1;

  // initializing kernel buffer values to zero 

    for (i = 0; i < 4; i = i + 1) begin
      for (j = 0; j < 3; j = j + 1) begin
        for (k = 0; k < 3; k = k + 1) begin
          kernel[i][j][k] <= 0;
        end
      end
    end


    c0 <= 0;
    c1 <= 0;
    c2 <= 0;
    c3 <= 0;
    //done_wt <= 0;
  
     end
     
     



      else if (wt_valid == 1  )
      begin
         if (c0 == 3)
          begin
            kernel[c3][c1][c2] = in_data; 
          end
                                                           //
      
      c0 <= c0 + 1;
     if (c0 == 3)
     begin
     kernel[c3][c1][c2] = in_data;                    ///////////////////-----------------------------/////////////////

     if (c1 <  KERNEL_COLS  - 1 )
                    c1 <= c1 + 1;
                else begin
                    c1 <= 0;
                    if (c2 <  KERNEL_ROWS - 1)
                          c2 <= c2 + 1;
                    else begin
                        c2 <= 0;
                        if (c3 < KERNEL_FILTERS - 1)
                            c3 <= c3 + 1;
                        else
                            c3 <= 0;
                    end
                end
            end
      end
        
       
     
    //  if (c1 == 3 && c2 == 2 && c3 ==2  )
    //    done_wt <= 1;
      
      
    
   
   
      
 
end
  
 
  
  always @(posedge clk or negedge rst_n) begin
   
  
    if (!rst_n || !start)
     begin
      row_ptr <= 0;
      wr_bank <= 0;
      count1 <=0;
      k <= 0;     
      count_row <= 0;                         //
      flag2 <= 0;
     end 
   
     
   else begin
     k <= k + 1;
     if (in_valid_1000  && (k[2:0] == 0) )                                    //
      begin
        row_ptr <= (row_ptr == IMG_H-1) ? 0 : row_ptr + 1;
        if (row_ptr == IMG_H-1) 
          begin
            count_row <= count_row +1 ;
           if (wr_bank < 2)
             wr_bank <= wr_bank + 1;
           else 
             wr_bank <= 0;
          end
      end
    
     if(row_ptr>=2 && wr_bank>=2)
       begin
       count1 <= 1;
       end  

    if(count_row ==8 && row_ptr == 0)
     flag2 <= 1;
    else
      flag2 <= 0;  
    
    
  end
  end


  // 4 banks of 32 rows × 16-bit
  reg [DATA_W-1:0] linebuf [0:BANKS-1][0:IMG_H-1];
  
  always @(posedge clk)
   begin
    //if (in_valid)
      linebuf[wr_bank][row_ptr] = in_data;                                            //
   end

  //=====================================================================
  // 2) WINDOW-CAPTURE STAGE: fetch 3 banks × 3 rows
  //=====================================================================
  // Precompute bank IDs for the last 3 columns
  wire [1:0] b0 = wr_bank;               // newest col
  wire [1:0] b1 = (wr_bank == 0)?2:b0 - 1;                // prev
  wire [1:0] b2 = (wr_bank != 2)?   (wr_bank == 1)?2:1    :  0;                // prev2



  // Pipeline regs to align with capture cycle
  reg [1:0] b0_r1, b1_r1, b2_r1;
  reg [HAW-1:0] rA, rB, rC;
  
  
  always @(posedge clk) begin
    if (in_valid) begin
//        c0 <= b0;
//        c1 <= b1;
//        c2 <= b2;
        
        b0_r1 <=b0;
        b1_r1 <=b1;
        b2_r1 <=b2;

      // three consecutive rows (for 3×3 window)
      rA    <= row_ptr;
      rB    <= (row_ptr==0)        ? 0 : row_ptr - 1;                          // check
      rC    <= (row_ptr<=1)        ? 0 : row_ptr - 2;
    end
  end

  // Capture the 3×3 window into regs
  reg signed [DATA_W-1:0] Wreg1 [0:2][0:2],Wreg2 [0:2][0:2],Wreg3 [0:2][0:2],Wreg4 [0:2][0:2],Wreg5 [0:2][0:2],Wreg6 [0:2][0:2],Wreg [0:2][0:2];
  integer i,j;
  
    always@(posedge clk) 
    begin
      for (i=0; i<3; i=i+1) begin
        for (j=0; j<3; j=j+1) begin
          
             Wreg2[i][j] <= Wreg1[i][j] ;
             Wreg3[i][j] <= Wreg2[i][j] ;
             Wreg[i][j] <= Wreg3[i][j] ;
            // Wreg5[i][j] <= Wreg4[i][j] ;
            // Wreg[i][j] <= Wreg5[i][j] ;
           //  Wreg[i][j] <= Wreg6[i][j] ;
             
        end
      end
    end
  
  ////////////////////////////////////////////////////////
    
    
/*

    reg [7:0] in_valid_delayed;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            in_valid_delayed <= 8'b0;
        else
            in_valid_delayed <= {in_valid_delayed[6:0], in_valid};
    end

    

  ///////////////////////////////////////////////////////
  */
 
  
  always @(posedge clk) begin                                                                                 
    // only valid once we've filled at least 3 rows & 3 cols
    if (in_valid )  begin                                                            //&& (!(row_ptr==0 ))  && (!(row_ptr==7))) begin
      for (i=0; i<3; i=i+1) begin
        for (j=0; j<3; j=j+1) begin
          case (i)
            0: Wreg1[i][j] <= linebuf[{ (j==0 ? b2_r1 : (j==1 ? b1_r1 : b0_r1)) }]  [rC];
            1: Wreg1[i][j] <= linebuf[{ (j==0 ? b2_r1 : (j==1 ? b1_r1 : b0_r1)) }]  [rB];
            2: Wreg1[i][j] <= linebuf[{ (j==0 ? b2_r1 : (j==1 ? b1_r1 : b0_r1)) }]  [rA];
          endcase
        end
      end
    end
  end

  //=====================================================================
  // 3) MULTIPLY STAGE: 9 parallel products
  //=====================================================================


  // pipeline products
  reg signed [DATA_W-1:0] Mreg [0:2][0:2];

  generate

   genvar i2,j2;

    for (i2=0; i2<3; i2=i2+1) begin
      for (j2=0; j2<3; j2=j2+1) begin
        assign temp_mul_product[i2][j2] = $signed(Wreg[i2][j2]) * $signed(kernel[a][i2][j2]);  // sync.   //
      end
    end
  endgenerate


  always @(posedge clk)
   begin
  
    for (i=0; i<3; i=i+1) begin
      for (j=0; j<3; j=j+1) begin
        Mreg[i][j] <= temp_mul_product[i][j][DATA_W-1 : 0 ];
       // Mreg[i][j] <= temp_mul_product[i][j][2*DATA_W-2 : DATA_W -1 ];  // normalized data format   //
      end
    end


  end

  //=====================================================================
  // 4) ADDER-TREE & OUTPUT STAGE
  //=====================================================================
  reg signed [DATA_W + 1 :0] sum0, sum1, sum2, sum01;
  reg signed [DATA_W + 3 :0]  total;




  /////////////////////////////////////
       
  

  reg [7:0] flag_delay;  // 8-cycle shift register

    always@(posedge clk ) begin
        if (!rst_n) begin
            flag_delay <= 8'b0;
            
        end else begin
            flag_delay <= {flag_delay[6:0], in_valid};  // shift left
            //flag_out   <= ;     // OR with 8-cycle old value
        end
    end
  
  ////////////////////////////////////

reg [2:0] row_ptr_t1, row_ptr_t2, row_ptr_t3, row_ptr_t4;
reg [2:0] row_ptr_t5, row_ptr_t6, row_ptr_t7;
reg [3:0] count_row1, count_row2;

always @(posedge clk) begin
  if (!rst_n) begin
    row_ptr_t1 <= 0;
     row_ptr_t2 <= 0;

    count_row1 <= 0;
    count_row2 <= 1; 
    //row_ptr_t3 <= 0;
    //row_ptr_t4 <= 0;
   // row_ptr_t5 <= 0;
   // row_ptr_t6 <= 0;
    //row_ptr_t7 <= 0;
  end else begin
    row_ptr_t1 <= row_ptr;
    row_ptr_t2 <= row_ptr_t1;

    count_row1 <= count_row;
    count_row2 <= count_row1; 
   // row_ptr_t3 <= row_ptr_t2;
    //row_ptr_t4 <= row_ptr_t3;
   // row_ptr_t5 <= row_ptr_t4;
   // row_ptr_t6 <= row_ptr_t5;
   // row_ptr_t7 <= row_ptr_t6;
  end
end

  ////////////////////////////////////



  always @(posedge clk) begin
    if (!rst_n)
      temp_001 <= 0 ;
    
    else begin
    sum0   <= Mreg[0][0] + Mreg[0][1] + Mreg[0][2];
    sum1   <= Mreg[1][0] + Mreg[1][1] + Mreg[1][2];
    sum2   <= Mreg[2][0] + Mreg[2][1] + Mreg[2][2];
     
    total  <= sum0 + sum1 + sum2;
    // out_valid only when window is fully formed (row≥2)
    // pipeling for syncronization


   // if  ((count_row2 == 2 ) &&  ((in_valid )  && (row_ptr_t2 >=2 || row_ptr >=2  ) && ( count1 == 1 )) )                            //
    //  out_valid_1 <= 1 ;

    if  ((flag2 == 0 ) &&  ((in_valid )  && ( row_ptr_t2 >=2  ) && ( count1 == 1 )) )                            //
      out_valid_1 <= 1 ;
    
   // else if ((flag2 == 1 ) && ((in_valid) ))
     //  out_valid_1 <= 1 ;


    else
      out_valid_1 <= 0 ;
    
    out_valid_2 <= out_valid_1 ;
    out_valid_3 <= out_valid_2;
    out_valid_4 <= out_valid_3;
    out_valid_5 <= out_valid_4;
    out_valid_6 <= out_valid_5;          //
    out_valid_7 <= out_valid_6;
    out_valid_8 <= out_valid_7;
    out_valid_9 <= out_valid_8;
    out_valid_10 <= out_valid_9;          //
    out_valid <= out_valid_10;


    out_valid_11 <= out_valid_10;

    out_valid_12 <= out_valid_11;          //


    //out_valid_13 <= out_valid_12;

    //out_valid_14 <= out_valid_13;          //
    //out_valid_15<= out_valid_14;

    //out_valid_16 <= out_valid_15;          //
    //out_valid_17 <= out_valid_16;

    //out_valid_18 <= out_valid_17;          //
    //out_valid <= out_valid_18;
    if (temp_001 == 0 && out_valid_8 == 1 )
         begin
         out_data  <= total[DATA_W -1 :0];
         temp_001 <= 1;
         end
    else if ((out_valid_10 || out_valid_8 == 1  ) == 1 && rst_n )       //
    begin
    out_data  <= total[DATA_W -1 :0];
  //  out_address <= out_address + 4;
    end
    
   // else 
   //out_data  <= 0;
    end 
  end

endmodule

////////////////////////////////////////////////////////////////////////////////////
