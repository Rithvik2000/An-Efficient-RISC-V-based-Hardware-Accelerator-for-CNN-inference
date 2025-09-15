
  
  
#include <stdio.h>
#include <stdlib.h>

#define WIDTH 14
#define HEIGHT 14
#define CHANNELS 8
#define FILTER_SIZE 3
#define NUM_FILTERS 8
#define OUT_WIDTH (WIDTH - FILTER_SIZE + 1)
#define OUT_HEIGHT (HEIGHT - FILTER_SIZE + 1)

// Read cycle count from RISC-V CSR

#define read_cycle(var) __asm__ __volatile__("rdcycle %0" : "=r"(var))


int main() {
       uint64_t start_cycle, end_cycle, start_cycle_v, end_cycle_v, start_cycle_temp, end_cycle_temp  ;
       uint32_t lo, hi;
       int match_cases, mismatch_cases;
       float speedup , accuracy;
        
    
    
      int i11, j11;
      i11 =0;
      j11 =0;
      match_cases =0;
      mismatch_cases =0;
      
      
   // int dummy;
    // Input tensor
    int input[HEIGHT][WIDTH][CHANNELS];
    // Filters
    int filters[NUM_FILTERS][FILTER_SIZE][FILTER_SIZE][CHANNELS];
    // Output tensor
    int output[CHANNELS/4][OUT_HEIGHT][OUT_WIDTH][NUM_FILTERS];
    
    // Output tensor for verification
    int output_v [OUT_HEIGHT][OUT_WIDTH][NUM_FILTERS];
    
    
      register int rs1 asm("a1");
      register int rs2 asm("a2");
     register int rs3 asm("a3");
      register int rs4 asm("a4");
    


    //
    

    // Randomly initialize input
    for (int h = 0; h < HEIGHT; ++h)
        for (int w = 0; w < WIDTH; ++w)
            for (int c = 0; c < CHANNELS; ++c)
                input[h][w][c] = (i11++*4) % 7 ;
   /*            
        // Print input
    printf("Input Tensor:\n");
    for (int c = 0; c < CHANNELS; ++c) {
        printf("Channel %d:\n", c);
        for (int i = 0; i < HEIGHT; ++i) {
            for (int j = 0; j < WIDTH; ++j)
                printf("%2d ", input[i][j][c]);
            printf("\n");
        }
        printf("\n");
    }
 */ 
    // Randomly initialize filters
    for (int f = 0; f < NUM_FILTERS; ++f)
        for (int i = 0; i < FILTER_SIZE; ++i)
            for (int j = 0; j < FILTER_SIZE; ++j)
                for (int c = 0; c < CHANNELS; ++c)
                    filters[f][i][j][c] =   (j11++*4 ) %11 ;  // values -1 to 1
                    
                   
    // Randomly initialize output
    for (int cc = 0; cc < CHANNELS/4; ++cc)
      for (int h = 0; h < OUT_HEIGHT; ++h)
        for (int w = 0; w < OUT_WIDTH; ++w)
            for (int c = 0; c < NUM_FILTERS; ++c)
                output[cc][h][w][c] = 0;

  /*
    // Print filters
    printf("Filters:\n");
    for (int f = 0; f < NUM_FILTERS; ++f) {
        printf("Filter %d:\n", f);
        for (int c = 0; c < CHANNELS; ++c) {
            printf("Channel %d:\n", c);
            for (int i = 0; i < FILTER_SIZE; ++i) {
                for (int j = 0; j < FILTER_SIZE; ++j)
                    printf("%2d ", filters[f][i][j][c]);
                printf("\n");
            }
            printf("\n");
        }
        printf("\n");
    }

    // Print output
    printf("Output Tensor:\n");
    for (int f = 0; f < NUM_FILTERS; ++f) {
        printf("Filter Output %d:\n", f);
        for (int i = 0; i < OUT_HEIGHT; ++i) {
            for (int j = 0; j < OUT_WIDTH; ++j)
                printf("%4d ", output[0][i][j][f]);
            printf("\n");
        }
        printf("\n");
    }
    
    
  /*      // Print input
    printf("Input Tensor:\n");
    for (int c = 0; c < CHANNELS; ++c) {
        printf("Channel %d:\n", c);
        for (int i = 0; i < HEIGHT; ++i) {
            for (int j = 0; j < WIDTH; ++j)
                printf("%2d ", input[i][j][c]);
            printf("\n");
        }
        printf("\n");
    }
      */       
    
        // read start cycle
        __asm__ __volatile__("rdcycle %0" : "=r"(lo));
        __asm__ __volatile__("rdcycleh %0" : "=r"(hi));
        start_cycle = ((uint64_t)hi << 32) | lo;

         // Step 0: Store base addresses in registers
     

      
     int *input_base   = &input[0][0][0];   // RS2
    
     int *filters_base  = &filters[0][0][0][0];  // RS1
    
     int *output_base   = &output[0][0][0][0];  //  RS1
    
     int sizes = 0x00E02008
;            // input matrix size  // 14, 16,16
     
     asm volatile ("mv a1, %0" :: "r"(filters_base));
     
     asm volatile ("nop");
     asm volatile ("nop");
    
    
     asm volatile ("mv a2, %0" :: "r"(sizes));
     
      asm volatile ("nop");
      asm volatile ("nop");
      
     asm volatile ("mv a3, %0" :: "r"(input_base));
     
     asm volatile ("nop");
     asm volatile ("nop");
     
     __asm__ __volatile__ ("mv a4, %0" :: "r"(output_base));
      
      asm volatile ("nop");                
      asm volatile ("nop");
    
   
   

    // Perform convolution
    for (int f = 0; f < NUM_FILTERS; f = f+4) {
        for (int c = 0; c < (CHANNELS/4); c = c+1) {
        for (int i = 0; i < OUT_HEIGHT; i = i+ 6) {
            for (int j = 0; j < OUT_WIDTH; j = j+ 6) {
                //int sum = 0;
                        
                        
                        
                                                // reset instruction for accelerator only                                        // -- cycles
                                            __asm__ __volatile__(
                                                ".insn r 0x77, 4, 3, x0, %[conv_i1], %[conv_i2]"
                                                :
                                                : [conv_i1] "r"(rs3), [conv_i2] "r"(rs4)
                                            );
                                            
                                            
                               
                                         int *input_base   = &input[i][j][c*4];   // RS2
                                        
                                         int *filters_base  = &filters[f][0][0][c*4];  // RS1
                                        
                                         int *output_base   = &output[c][i][j][f];  //  RS1                   //
                                        
                                         int sizes = 0x00E02008
;            // input matrix size
                                         
                                          asm volatile ("nop");
                                         asm volatile ("nop");
                                          asm volatile ("nop");
                                         asm volatile ("nop");
                                         
                                         asm volatile ("mv a1, %0" :: "r"(filters_base));
                                         
                                         asm volatile ("nop");
                                         asm volatile ("nop");
                                        
                                        
                                         asm volatile ("mv a2, %0" :: "r"(sizes));
                                         
                                          asm volatile ("nop");
                                          asm volatile ("nop");
                                          
                                         asm volatile ("mv a3, %0" :: "r"(input_base));
                                         
                                         asm volatile ("nop");
                                         asm volatile ("nop");
                                         
                                         __asm__ __volatile__ ("mv a4, %0" :: "r"(output_base));
                                          
                                          asm volatile ("nop");                
                                          asm volatile ("nop");
                                          
                                           
                                          
                                          
                                              // Perform convolution
                                                // Step 1: LD_FILTERS                                          // -- cycles
                                            __asm__ __volatile__(
                                                ".insn r 0x77, 4, 1, x0, %[conv_i1], %[conv_i2]"
                                                :
                                                : [conv_i1] "r"(rs1), [conv_i2] "r"(rs2)
                                            );
                                                
                                                
                                              
                                                    // Step 2: CONV_8X8X4X4                                          // -- cycles
                                            __asm__ __volatile__(
                                                ".insn r 0x77, 4, 2, x0, %[conv_i1], %[conv_i2]"
                                                :
                                                : [conv_i1] "r"(rs3), [conv_i2] "r"(rs4)
                                            );
                                            
                                                     // reset instruction                                        // -- cycles
                                            __asm__ __volatile__(
                                                ".insn r 0x77, 4, 3, x0, %[conv_i1], %[conv_i2]"
                                                :
                                                : [conv_i1] "r"(rs3), [conv_i2] "r"(rs4)
                                                                        
                                               );
                       }                       
                    }
                  }
               }   
               
                       // read start cycle
                 __asm__ __volatile__("rdcycle %0" : "=r"(lo));
                 __asm__ __volatile__("rdcycleh %0" : "=r"(hi));
                start_cycle_temp = ((uint64_t)hi << 32) | lo;
                
                //output[i][j][f] = sum;  
                   for (int ccc = 0; ccc < NUM_FILTERS ; ccc = ccc + 1) {
                     for (int i = 0; i < OUT_HEIGHT; ++i) {
                        for (int j = 0; j < OUT_WIDTH; ++j) {
                          
                           for (int k = 1; k < CHANNELS/4; ++k) {
                               output[0][i][j][ccc] += output[k][i][j][ccc];
                               
                               
                     
                           }
                 
                          }
                        }
                      }
            
               
    
                                       // read end cycle
                 __asm__ __volatile__("rdcycle %0" : "=r"(lo));
                 __asm__ __volatile__("rdcycleh %0" : "=r"(hi));
                end_cycle_temp = ((uint64_t)hi << 32) | lo;
    
    
   


    
    
    
    
    

   // dummy = (output[1][1][1]) + (output[2][2][2]) + 2 ;
   
  //read_cycle(end_cycle);
  
        __asm__ __volatile__("rdcycle %0" : "=r"(lo));
        __asm__ __volatile__("rdcycleh %0" : "=r"(hi));
        end_cycle = ((uint64_t)hi << 32) | lo;   // printf("Start Cycle: %lu\n", start_cycle);
        
        
 //printf("Cycle count: %llu\n", start_cycle);
  
 //printf("Cycle count: %llu\n", end_cycle);

 
printf("Total Cycles with acelerator: %llu\n", end_cycle - start_cycle);
printf("Total Cycles with acelerator only channael addition part : %llu\n", end_cycle_temp - start_cycle_temp);




////////////////////////////////////verification part /////////////////////////////////////////////////////

     // read start cycle
       __asm__ __volatile__("rdcycle %0" : "=r"(lo));
        __asm__ __volatile__("rdcycleh %0" : "=r"(hi));
        start_cycle_v = ((uint64_t)hi << 32) | lo;
   

    // Perform convolution
    for (int f = 0; f < NUM_FILTERS; ++f) {
        for (int i = 0; i < OUT_HEIGHT; ++i) {
            for (int j = 0; j < OUT_WIDTH; ++j) {
                int sum = 0;
                for (int fi = 0; fi < FILTER_SIZE; ++fi) {
                    for (int fj = 0; fj < FILTER_SIZE; ++fj) {
                        for (int c = 0; c < CHANNELS; ++c) {
                            sum += (input[i + fi][j + fj][c] * filters[f][fi][fj][c]) ;
                        }
                    }
                }
                output_v[i][j][f] = sum;
            }
        }
    }


  //read end cycle
  
        __asm__ __volatile__("rdcycle %0" : "=r"(lo));
        __asm__ __volatile__("rdcycleh %0" : "=r"(hi));
        end_cycle_v = ((uint64_t)hi << 32) | lo;   // printf("Start Cycle: %lu\n", start_cycle);
        
  // printf("Cycle count: %llu\n", start_cycle_v);
  
// printf("Cycle count: %llu\n", end_cycle_v);

 
printf("Total Cycles without accelerator: %llu\n", end_cycle_v - start_cycle_v);


////////// compare /////////////////

 for (int f = 0; f < NUM_FILTERS; ++f) {
        
        for (int i = 0; i < OUT_HEIGHT; ++i) {
            for (int j = 0; j < OUT_WIDTH; ++j)
              if ( output[0][i][j][f] == output_v[i][j][f])
                  match_cases++;
              else 
                 mismatch_cases++;
           
        }
        
    }

printf("Total matched cases: %d\n",  match_cases);
printf("Total mismatched cases: %d\n",  mismatch_cases);

accuracy = ((float)match_cases / (float)(match_cases + mismatch_cases)) * 100.0f;
speedup = (float)(end_cycle_v - start_cycle_v) / (float)(end_cycle - start_cycle);

printf("Total accuracy: %f percent \n",  accuracy );
printf("Total speed-up: %f x\n", speedup  );
//////////////////////////////////////////////////////////////////////////////////////////
 
    
    
                        
  int *a1 = &input[0][0][0];
     
     int *a2  = &filters[0][0][0][0];  // RS1
    
     int *a3   = &output[0][0][0];  //  RS1
    
    
    
    
  
    
     printf("%p , %p , %p, %d:\n",(void *)a1,(void *)a2,(void *)a3,sizes);
   /*
        // Print output
    printf("Output Tensor:\n");
    for (int f = 0; f < NUM_FILTERS; ++f) {
        printf("Filter Output %d:\n", f);
        for (int i = 0; i < OUT_HEIGHT; ++i) {
            for (int j = 0; j < OUT_WIDTH; ++j)
                printf("%2d ", output[0][i][j][f] );
            printf("\n");
        }
        printf("\n");
    }

       // Print output
    printf("Output Tensor for verification:\n");
    for (int f = 0; f < NUM_FILTERS; ++f) {
        printf("Filter Output %d:\n", f);
        for (int i = 0; i < OUT_HEIGHT; ++i) {
            for (int j = 0; j < OUT_WIDTH; ++j)
                printf("%2d ", output_v[i][j][f] );
            printf("\n");
        }
        printf("\n");
    }
    */
    
    


    
    

    return 0;
    
    
}
 



