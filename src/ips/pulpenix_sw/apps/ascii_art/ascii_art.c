#include <ddp23_libs.h> 

/****************************** MACROS ******************************/

// XBOX SW addressing defines
//#define XBOX_APB_BASE_ADDR 0x1A400000   // APB MAPPED accelerator address space ; Not in use for this app
#define XBOX_TCM_BASE_ADDR 0x00200000     // TCM MAPPED(Tightly Coupled Memory)  This is the base address of the accelerators memories from SW perspective.

//Following C macro can be used to easily address a location in the XBOX acceleration memory.
# define LOG2_LINES_PER_MEM 8
# define LINES_PER_MEM (2<<(LOG2_LINES_PER_MEM-1)) // Same as 2 to the power of LOG2_LINES_PER_MEM
# define XBOX_TCM_ADDR(mem_idx,line_idx,word_idx) (XBOX_TCM_BASE_ADDR+(mem_idx*1024*32)+(line_idx*32)+(word_idx*4))

 // Addresses of the base and Next char 2D array of multiple lines each of 32 bytes
#define BF_SUB_IMG_ADDR ((volatile unsigned int  *)  XBOX_TCM_ADDR(0,0,0))  // base frame sub image
#define NF_SUB_IMG_ADDR ((volatile unsigned int  *)  XBOX_TCM_ADDR(0,32,0)) // next frame sub image

#define DIFF_LINE ((volatile unsigned int  *)  XBOX_TCM_ADDR(0,64,0)) // Intermediate next-base line difference

// SW-HW TCM CMD/STATUS handshake TCM locations addressing
// NOTICE following must be compliant with the relative used address in the accelerator code, this is NOT automated.
#define XLRTR_TCM_CMD_ADDR    ((volatile unsigned int *)   XBOX_TCM_ADDR(0,65,0))  // An agreed memory location to trigger commands to the accelerator. 
#define XLRTR_TCM_CMD_INFO    ((volatile unsigned int *)   XBOX_TCM_ADDR(0,65,1))  // An agreed memory location to transfer command related information between the software and the hardware.
#define XLRTR_TCM_STATUS_ADDR ((volatile unsigned int *)   XBOX_TCM_ADDR(0,65,2))  // An agreed memory location for the hardware to tell the SW status of execution such as 'done'.

// Few more C macros
# define BYTES_PER_LINE 32  // Number of bytes per XBOX accelerator memory line.
# define NUM_LINES 32       // Number lines used per sub image
# define SUB_IMG_NUM_BYTES (NUM_LINES*BYTES_PER_LINE) // Number of bytes in total per image.

//---------------------------------------------------------------------------------

// Enumerated type of different commands code
enum {CMD1_TBD=0,   // Place holder for accelertor command1
      CMD2_TBD=1    // Place holder for accelertor command2
     } xlrtr_cmd ;


/*********************************************************************************/

// This function is used to invoke the SOC level (System Ob Chip) to load the frame sun image from a text file.

void load_sub_img_file(char * dst_addr, char * file_name)  {
    
  bm_printf("C MSG: Loading sub image from file %s to address 0x%08x\n",file_name,dst_addr) ; 
  //unsigned int img_file  = bm_fopen_r("app_src_dir/bf_sub_img.txt") ; << ERROR
  unsigned int img_file  = bm_fopen_r(file_name) ; // Open the image input file  <<< CORRECT  
 
  // Starting a SOC level file to memory copy transfer
  bm_start_soc_load_hex_file (img_file, SUB_IMG_NUM_BYTES, (unsigned char *) dst_addr) ; 

  // Polling till transfer completed (SW may also do other stuff mean while)
  int num_loaded = 0 ;
  while (num_loaded==0) num_loaded = bm_check_soc_load_hex_file () ; // num_loaded!=0 indicates completion.
  
  bm_printf("C MSG: Loaded %d bytes\n",num_loaded) ;

  bm_fclose(img_file); // Close image input file.
}



//---------------------------------------------------------------------------------------------

void save_diff_cmpr_out(char * diff_cmpr_out, int num_bytes, char * out_file_name) {
 // Save the diff compressed output file

 unsigned int out_file  = bm_fopen_w(out_file_name) ; // Open the output file

 // Dumps the 'num_bytes' bytes starting at address of 'diff_cmpr_out' to the file
 bm_dump_hex_file (out_file, diff_cmpr_out, num_bytes); 

 bm_fclose(out_file); // Close the output file.

}

//---------------------------------------------------------------------------------------------

void subtract_line (unsigned char * bf_line, unsigned char * nf_line, 
                    unsigned char * diff_line, int line_idx,
                    int is_xltr_enabled) {
         
       if(is_xltr_enabled) {

           // Accelerated calculation of diff
           
           *XLRTR_TCM_STATUS_ADDR = 0 ; // initialize done polling indication // TODO use a real done
           *XLRTR_TCM_CMD_INFO = line_idx ; //  provide line index
           *XLRTR_TCM_CMD_ADDR = XLRTR_SUB_CMD ; // Trigger accelerator , after providing info            
           char polling_done=0 ;  // Polling to check if accelerator is done  
           // Keep Polling til status returned from accelerator is not zero            
           while (!polling_done) polling_done = ((*XLRTR_TCM_STATUS_ADDR)!=0); 

       } else {  
           // Non Accelerated mode - Simple subtraction loop   
           for (int i=0; i<BYTES_PER_LINE ; i++)  diff_line[i] = nf_line[i] - bf_line[i] ;          
       }
}

//---------------------------------------------------------------------------------------------

int num_cz_from_idx(unsigned char *  diff_line, int start_idx, char is_xltr_enabled) {
   
   // Number of consecutive zeros in line starting from given index within line

   int num_cz = 0 ; // default to zero

   is_xltr_enabled = 1 ; // TEMP!!!! OVERWRITE PARAM, TO BE REMOVED ONCE XLRTR IS SUPPORTED !!!   
   if  (is_xltr_enabled) {
       
       //  SUGGESTED XLRTR INTERFACE 
       // Make sure it is well supported by accelerator hardware code

       *XLRTR_TCM_STATUS_ADDR = 0 ; // initialize done polling indication // TODO use a real done
       *XLRTR_TCM_CMD_INFO = start_idx ;   // provide the start index in the line   
       *XLRTR_TCM_CMD_ADDR = XLRTR_CNZ_CMD ; // Trigger accelerator , after providing info
       char polling_done=0 ;  // Polling to check if accelerator is done         
       while (!polling_done) polling_done = ((*XLRTR_TCM_STATUS_ADDR)!=0); 
       num_cz = *XLRTR_TCM_CMD_INFO  ;  // Use the info address also to get the num_cz result from the accelerator.                  
   } 
   
   else {  // Non Accelerated  simple SW solution. 

      for (int i=start_idx; i<BYTES_PER_LINE ; i++) {
         if (diff_line[i]==0) num_cz+=1 ; 
         else break ;
      }      
   }
   
   return num_cz ;   
}

//----------------------------------------------------------------------------------------------

// This function check the difference per byte between two lines and updates the compressed vector accordingly.

int gen_aa (unsigned char * aa_str_in,      // ascii-art input string
            char is_xltr_enabled )          // Indicates acceleor or non accelerator mode.
{


}


//---------------------------------------------------------------------------------------------


int main() {
    
 bm_printf("\nHELLO DDP24 ASCII ART\n"); 

  int is_xltr_enabled = 0 ; // 1: enabled ; 0: disabled

 // Locate the Base and Next of multiple lines each of 32 bytes,  cast to 2D-array
  unsigned char (* bf_sub_img)[BYTES_PER_LINE] = (unsigned char **) BF_SUB_IMG_ADDR; // Base frame 32X32 bytes sub image
  unsigned char (* nf_sub_img)[BYTES_PER_LINE] = (unsigned char **) NF_SUB_IMG_ADDR; // Next frame 32X32 bytes sub image


  load_sub_img_file((char *)bf_sub_img,"app_src_dir/bf_sub_img.txt"); 
  load_sub_img_file((char *)nf_sub_img,"app_src_dir/nf_sub_img.txt"); 
    
  int start_cycle,end_cycle ;            // For performance checking.  
  ENABLE_CYCLE_COUNT ;                   // Enable the cycle counter
  RESET_CYCLE_COUNT  ;                   // Reset counter to ensure 32 bit counter does not wrap in-between start and end.   
  GET_CYCLE_COUNT_START(start_cycle) ;   // Capture the cycle count before the operation.

  gen_aa(ascii_art_str,is_xltr_enabled) ;

  GET_CYCLE_COUNT_END(end_cycle) ;  // Capture the cycle count after the operation.
  int cycle_cnt = end_cycle-start_cycle ; // Calculate consumed cycles.
  
  
  bm_printf("Ascii-Art generation took %d cycles \n\n",cycle_cnt); // Report

  // Save the ascii_art output file at the sim folder
  save_diff_cmpr_out(aa_img, cmpr_byte_length, "../aa_img_out_out.txt"); 

  sim_finish();  // flag to trigger execution termination 
    
 return 0;
 
 
}
