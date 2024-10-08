#include <ddp23_libs.h>

// accelerator registers
#define XBOX_BASE_ADDR 0x1A400000

 // accelerator memories
 #define XBOX_MEM_BASE_ADDR (XBOX_BASE_ADDR+0x80000)  

int main() {

  // Test Regs Access
  unsigned int  gpp_test_val ;
  int gpp_reg_idx ;
  volatile unsigned int * gpp_reg_addr_start ;
  volatile unsigned char * gpp_reg_addr_done ;
  volatile unsigned int * gpp_reg_addr_row ;
  volatile unsigned int * gpp_reg_addr_col ;

  bm_printf("C MSG: \n\n\n **** HELLO GPP XBOX **** \n\n\n") ;   // bare-metal output to terminal
 
  // Write gpp reg #5 (currently not in use by accelerators)
  int mem_idx = 0;
  volatile unsigned int * mem_inst_input_base_addr = (unsigned int *)(XBOX_MEM_BASE_ADDR) + (mem_idx*32*1024);
  for (int i = 0; i < 16*4; i+=4) {
    int val = i+1;
    mem_inst_input_base_addr[i/4] =  ((((((i+4) << 8) + i+3) << 8) + i+2) << 8) + i+1 ;
  }
  
  mem_idx = 1;
  volatile unsigned int * mem_inst_wight_base_addr = (unsigned int *)(XBOX_MEM_BASE_ADDR) + (mem_idx*32*1024);
  for (int i = 0; i < 16*4; i+=4) {
    int val = i+2;
    mem_inst_wight_base_addr[i/4] =  ((((((i+4) << 8) + i+3) << 8) + i+2) << 8) + i+1 ;
  }


  gpp_reg_idx = 8 ;
  gpp_reg_addr_start = (volatile unsigned int *) (XBOX_BASE_ADDR+(gpp_reg_idx*4)) ;
  gpp_reg_idx = 0 ;
  gpp_reg_addr_done = (volatile unsigned char *) (XBOX_BASE_ADDR+(gpp_reg_idx)) ;
  gpp_reg_idx = 2 ;
  gpp_reg_addr_col = (volatile unsigned int *) (XBOX_BASE_ADDR+(gpp_reg_idx*4)) ;
  gpp_reg_idx = 3 ;
  gpp_reg_addr_row = (volatile unsigned int *) (XBOX_BASE_ADDR+(gpp_reg_idx*4)) ;

  bm_printf("C MSG: Reading %x from gpp_reg %x at address %x\n",gpp_test_val,gpp_reg_idx,gpp_reg_addr_start) ; 
  
  // setup
  *gpp_reg_addr_col = 16;
  *gpp_reg_addr_row = 11; // OZ&Eliyahu : work only with number 10
  // go signal for the accelerator   
  *gpp_reg_addr_start = 1 ;
  // wait for the accelerator to finish it's work
  while(!*gpp_reg_addr_done);

  // Finish
   
  sim_finish () ;  // flag to trigger simulation termination
  
  return 0;
}
