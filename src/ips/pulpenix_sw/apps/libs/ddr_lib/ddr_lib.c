#include <ddp23_libs.h>

//  Local definitions

#define GPP_FPGNIX_START_ADDR 0x1A300000
 
#define GPP_DDR_CMD_REG_ADDR    (volatile unsigned int *)(GPP_FPGNIX_START_ADDR + 4)    
#define GPP_DDR_STATUS_REG_ADDR (volatile unsigned int *)(GPP_FPGNIX_START_ADDR + 8)      
#define GPP_DDR_BUF_START_ADDR  (volatile unsigned int *)(GPP_FPGNIX_START_ADDR + 0x1000)


// Interface to FPFNIX DDR interface hardware
// Should correspond to .../ips/ddr/ddr2_read_write.v 

enum {SOC_CMD_STORE, SOC_CMD_LOAD} soc_cmd_t ;   
enum {DDR_INIT,DDR_IDLE, DDR_WRITE, DDR_READ} state_t ;
 
typedef union { 
  struct soc_ddr_cmd_fields {
    unsigned int ddr_addr : 26 ;
    unsigned int unused   :  2 ;
    unsigned int opc      :  4 ;     
  } cmd_field ;
  unsigned int cmd_word ;
} soc_ddr_cmd_t ;


//----------------------------------------------------------------------------------------

char init_ddr() {
    
  // Polling till DDR is ready
 int pol_max_itr = 100 ; // Timeout num iterations
 int itr_cnt = 0 ;
 unsigned int ddr_status = DDR_INIT ;
 bm_printf("Polling DDR status reg for checking calibration status\n") ;
 while ((itr_cnt++<pol_max_itr) && (ddr_status==DDR_INIT)) {
     ddr_status = *GPP_DDR_STATUS_REG_ADDR ;
     bm_printf("DDR Calibration status Polling itr#%d : GPP_DDR_STATUS_REG = %08x\n", itr_cnt, ddr_status) ;     
 }
 if (itr_cnt==pol_max_itr) {
     bm_printf("DDR Calibration FAIL, exceeded max poling iterations\n") ;
     return 0 ;
 } 
 else {
     bm_printf("\nCALIBRATION STATUS : DONE , DDR READY\n\n") ; 
     return 1 ;
 }  
 
} 

 
//----------------------------------------------------------------------------------------

    

void store_to_ddr(volatile unsigned int * soc_start_addr,   // Recommended for performance but not required to be word address aligned
                  volatile unsigned int   ddr_start_addr,   // TODO! Need to Check: Recommended/Must be be 4 words (128b) address aligned
                  int                     num_words,        // Must be multiplication of 4, other wise down-rounded with no warning !
                  char                    do_print)         // do print (for debug)                 
{
   unsigned int * gpp_ddr_buf = (unsigned int *)GPP_DDR_BUF_START_ADDR ;
   soc_ddr_cmd_t soc_ddr_cmd ;       
   
   unsigned int ddr_addr = ddr_start_addr ;
   
   for  (int i=0;i<(int)(num_words/4);i++) { 
   
      if (do_print) bm_printf("DDR_LIB Message: Storing ");
      
      // Copy 4 words (128b) from desired soc address to intermediate buffer 
      for (int j=3;j>=0;j--) {
          
            unsigned int val = soc_start_addr[(i*4)+j] ;
            gpp_ddr_buf[(i*4)+j] = val ;             
            if (do_print) bm_printf("%08X ",val) ;
      }
      
      // Issue Store Command per 4 words (128b)         
      soc_ddr_cmd.cmd_field.ddr_addr = ddr_addr+=16 ;
      soc_ddr_cmd.cmd_field.opc  = SOC_CMD_STORE ;  
      *GPP_DDR_CMD_REG_ADDR = soc_ddr_cmd.cmd_word ; 
      
      unsigned int ddr_status = *GPP_DDR_STATUS_REG_ADDR ;  // TODO: Real Polling on operation done (need to add op_done status bit in hardware)
                                                            // However it seems like few cycles are needed , thats why this volatile access is at least needed.            
      
      if (do_print) bm_printf("from SOC address %08X to DDR address %08X\n",&(soc_start_addr[i*4]),ddr_start_addr+(i*16)) ;
   }
       
} 

//----------------------------------------------------------------------------------------


void load_from_ddr(volatile unsigned int * soc_start_addr,   // Recommended for performance but not required to be word address aligned
                   volatile unsigned int   ddr_start_addr,   // TODO! Need to Check: Recommended/Must be be 4 words (128b) address aligned
                   int                     num_words,        // Must be multiplication of 4, other wise down-rounded with no warning !
                   char                    do_print)         // do print (for debug)                 
{        
   soc_ddr_cmd_t soc_ddr_cmd ;  

   unsigned int * gpp_ddr_buf = (unsigned int *)GPP_DDR_BUF_START_ADDR ;

   unsigned int ddr_addr = ddr_start_addr ;
    
   for  (int i=0;i<(int)(num_words/4);i++) { 

      // Issue Load Command per 4 words (128b)           
      soc_ddr_cmd.cmd_field.ddr_addr = ddr_addr+=16 ;
      soc_ddr_cmd.cmd_field.opc  = SOC_CMD_LOAD ;  
      *GPP_DDR_CMD_REG_ADDR = soc_ddr_cmd.cmd_word ; 
      
      unsigned int ddr_status = *GPP_DDR_STATUS_REG_ADDR ;  // TODO: Real Polling on operation done (need to add op_done status bit in hardware)
                                                            // However it seems like few cycles are needed , that's why this volatile access is at least needed.            
   
      if (do_print) bm_printf("DDR_LIB Message: Loading ");
      
      // Copy 4 words (128b) intermediate buffer to desired soc address
      for (int j=3;j>=0;j--) {                             
            unsigned int val = gpp_ddr_buf[(i*4)+j]    ;
            soc_start_addr[(i*4)+j] = val ;             
            if (do_print) bm_printf("%08X ",val) ;
      } 
      if (do_print) bm_printf("from DDR address %08X to SOC address %08X\n", ddr_start_addr+(i*16),&(soc_start_addr[i*4])) ;
   }      
} 

//----------------------------------------------------------------------------------------

void ddr_store_from_hex_file (int file_id, int num_bytes , unsigned int ddr_addr) {
  
  // IMPORTANT NOTICE DDR is written in 4 words (16 bytes , 128 bits) chunks granularity
  // 1. It is assumed that ddr_addr is quad word aligned meaning divides by 16.
  // 2. If num_byte does not divide by 16 then unpredicted garbage round-up values are written to DDR to complete the quad-word chunk  
    
  char do_print = 0 ;
        
  unsigned char buf [FILE_TO_DDR_BUF_NUM_BYTES+4] ; // +4 safety alignment overflow

    
  int num_bytes_loaded = 0 ;  // Number of bytes loaded so far
  int itr_num_bytes_transfer = 0 ; // current iteration number of bytes to transfer
  
  while (1) { // will break on completion 
  
   if ((num_bytes_loaded+FILE_TO_DDR_BUF_NUM_BYTES) <= num_bytes) itr_num_bytes_transfer = FILE_TO_DDR_BUF_NUM_BYTES ;
   else itr_num_bytes_transfer = num_bytes - num_bytes_loaded ;

   bm_start_soc_load_hex_file (file_id, itr_num_bytes_transfer, buf) ;
   
   int num_loaded = 0 ; // loop till transfer to buffer done , non 0 indicates completion.
   while (num_loaded==0){
	  num_loaded = bm_check_soc_load_hex_file () ; // num_loaded!=0 indicates completion.
   }
    
   int itr_num_words_transfer = (itr_num_bytes_transfer%4)==0 ? itr_num_bytes_transfer/4 : (itr_num_bytes_transfer/4)+1 ; // Round up number of words

   store_to_ddr((unsigned int *)buf, ddr_addr, itr_num_words_transfer , do_print) ;  
      
   num_bytes_loaded += itr_num_words_transfer*4 ;
   ddr_addr += itr_num_words_transfer*4 ; 
      
   if (num_bytes_loaded >= num_bytes) break ;  
  
  }
}

//----------------------------------------------------------------------------------------


void ddr_dump_to_hex_file (int file_id, int num_bytes , unsigned int ddr_addr, int num_bytes_per_first_line, int num_bytes_per_non_first_line) {
  
  char do_print = 0 ; 
    
  unsigned char buf [FILE_TO_DDR_BUF_NUM_BYTES] ; 
  
  unsigned int * buf_word_addr = (unsigned int *)buf ;
  
  int is_first_line = 1 ;
  int idx_within_line = 0 ;
  
  int byte_idx=0; // byte index with in current iteration buffer
  int num_bytes_dumped = 0 ;  // Number of total bytes dumped so far
  
  char ms_hex_val, ls_hex_val ;
  char ms_hex_char, ls_hex_char ;

  // First Chunk transfer
  int itr_num_bytes_transfer = num_bytes > (FILE_TO_DDR_BUF_NUM_WORDS*4) ? FILE_TO_DDR_BUF_NUM_WORDS : num_bytes/4 ;   
  load_from_ddr(buf_word_addr, ddr_addr, itr_num_bytes_transfer , do_print) ; // Load from file into intermediate buffer

  bm_access_file(file_id) ;
  while (1) { // will break when done dumping
              
       ms_hex_val = buf[byte_idx]/16  ;
       ms_hex_char = ms_hex_val < 10 ? '0'+ms_hex_val : 'a'+(ms_hex_val-10) ;
       if (ms_hex_char=='0') ms_hex_char=' ' ;
       uart_sendchar(ms_hex_char);    
       
       ls_hex_val = buf[byte_idx]%16   ; 
       ls_hex_char = ls_hex_val < 10 ? '0'+ls_hex_val : 'a'+(ls_hex_val-10) ;
       uart_sendchar(ls_hex_char);    
       
       byte_idx++ ;
       num_bytes_dumped++ ;  
       
       idx_within_line ++ ;
       
       if (num_bytes_dumped == num_bytes) {
          uart_sendchar('\n'); ;
          break ;         
       }
       
       if (is_first_line) {
          if (idx_within_line==num_bytes_per_first_line) {
            uart_sendchar('\n');
            idx_within_line = 0 ;
             is_first_line = 0 ;
          }    
          else uart_sendchar(' ');        
       } else {
           if (idx_within_line==num_bytes_per_non_first_line) {
               uart_sendchar('\n');
               idx_within_line = 0 ;
           }
           else uart_sendchar(' ');             
       }
       
       if (byte_idx==FILE_TO_DDR_BUF_NUM_BYTES) {
          ddr_addr += FILE_TO_DDR_BUF_NUM_BYTES ;
          if (do_print) bm_access_file(-1) ;   // return to default stdio for ddr_load prints
          
          int pending_bytes = num_bytes-num_bytes_dumped ;
          itr_num_bytes_transfer = pending_bytes > (FILE_TO_DDR_BUF_NUM_WORDS*4) ? FILE_TO_DDR_BUF_NUM_WORDS : pending_bytes/4 ; 
          
          load_from_ddr(buf_word_addr, ddr_addr, itr_num_bytes_transfer , do_print) ; 
          if (do_print) bm_access_file(file_id) ;  // return to source access
          byte_idx = 0 ;
       }  
  }
     
  bm_access_file(-1) ;   // return to default stdio  
    
}

//----------------------------------------------------------------------------------------


