//////
//#include "../include/cnn_inc.h"
#include "include/cnn_inc.h" // Use this for BARE METAL env.


#if (defined FPGNIX) || (defined SANSA_SIM) || (defined DDP21_SIM)
 int main(){
#else
 int main(int argc, char const *argv[]) {   
#endif


#ifndef DDP21_SIM
#ifdef BARE_METAL
#ifndef SANSA_SIM
  init_pll_rcg();   
  uart_set_cfg(0, UART_CLK);
#else

  int fbdiv_tmp;
  fbdiv_tmp = (int) FBDIV_FACTOR;
  set_pll_div(fbdiv_tmp, 1, 1, 1, 0);
  carp_set_divider(GP_RF_MMSPI_D_CLOCK_CTRL_ADDR,1);
  init_pll_rcg();
  uart_set_cfg(0, UART_CLK);
#endif
#endif // BARE_METAL
#endif // DDP21_SIM

printf("HELLO FASHION MNIST\n");

////

// like-dynamic memory allocation for the program
#ifndef BARE_METAL
#include "include/tensor_allocation_setup.h"
#else
#include "include/tensor_allocation_setup.h"
#endif // !BARE_METAL

printf("============================= test ==============================\n");

// ----------------------------------------------------------------------------------------------
// ----------------------------- mannix staruct declaration -------------------------------------
// ----------------------------------------------------------------------------------------------   
// -------------- SW DATA ---------------------
    // declare 4D tensors
    Tensor4D_uint8 image[1];
    Tensor4D_int8 conv_weight[1];
    Tensor4D_uint8 result_4D_tensor_uint8[1];

    // declare matrix bias [for each matrix there is one bias value, for example for image->matrix[0] we add the same value bias->data[0] to all cells]
    Matrix_int32 conv_bias[2];

// ------------ ACCELERATOR DATA ---------------------
    // declare 4D tensors
    Tensor4D_vol32 vol_image[1];
    Tensor4D_vol32 vol_conv_weight[1];
    Tensor4D_vol32 vol_result_4D_tensor_uint8[1];

    // declare matrix bias [for each matrix there is one bias value, for example for image->matrix[0] we add the same value bias->data[0] to all cells]
    Matrix_vol32 vol_conv_bias[2]; 
 
   // allocate memory for image, conv_weight and bias
    printf("allocating memory\n");

// ----------------------------------------------------------------------------------------------
// ----------------------------- mannix staruct define size and data allocation -------------------------------------
// ----------------------------------------------------------------------------------------------   
    create4DTensor_uint8(&image[0], 28, 28, 1, 1,    (Allocator_uint8*)al, (MatAllocator_uint8*)mat_al, (TensorAllocator_uint8*)tens_alloc);  
    create4DTensor_int8(&conv_weight[0], 5, 5, 1, 1, (Allocator_int8*)al, (MatAllocator_int8*)mat_al, (TensorAllocator_int8*)tens_alloc);
    creatMatrix_int32(1, 1,   &conv_bias[0],   (Allocator_int32*) al);  // Martix_int32 * conv_bias = new Matrix_int32(1,1);
    
    create4DTensor_vol32(&vol_image[0], 28, 28, 1, 1,    vol_al, vol_mat_al, vol_tens_alloc);  
    create4DTensor_vol32(&vol_conv_weight[0], 5, 5, 1, 1,    vol_al, vol_mat_al, vol_tens_alloc);  
    creatMatrix_vol32b(1, 1,   &vol_conv_bias[0], vol_al);

    //vol_conv_bias->data[5]


// ----------------------------------------------------------------------------------------------
// ----------------------------- mannix staruct load data ---------------------------------------
// ----------------------------------------------------------------------------------------------   
#ifndef MEM_LOAD_MODE
// if you want to set data manualy use this section
// in mandef.h mark the line //#define MEM_LOAD_MODE   
#endif

#ifdef MEM_LOAD_MODE // Load the model parameters pre-dumped db        
    // MODEL_PARAMS_FILE is defined in mandef.h this is the path to the model params .txt file 
    load_model_params_mfdb(al,MODEL_PARAMS_FILE);  // load mannix format data base
    load_model_params_mfdb_vol32(vol_al,MODEL_PARAMS_FILE);  // load mannix format data base
#endif

#ifdef  MEM_LOAD_MODE
// DATASET_FILE is defined in mandef.h this is the path to the image .txt file
// NOTICE: it's recommended to see image file format before making changes
  FILE_PTR imageFilePointer = fopen_r(DATASET_FILE);  // @MEM_LOAD_MODE (File name defined at man_def.h)
#endif 

//sasa tech

#ifdef SANSA_SIM
#define NUM_IMG_TO_CHK 10
#else
#define NUM_IMG_TO_CHK 10000
#endif


// ----------------------------------------------------------------------------------------------
// ------------------------------- enics UART (DO NOT TOUCH) ------------------------------------
// ----------------------------------------------------------------------------------------------   
printf("============================================================================\n");
printf("=============== starting test (it could take some time...): ================\n");
printf("============================================================================\n\n");

  int start_cycle, end_cycle ;          
  ENABLE_CYCLE_COUNT  ; // Enable the cycle counter
  RESET_CYCLE_COUNT  ; // Reset counter to ensure 32 bit counter does not wrap in-between start and end.           
  GET_CYCLE_COUNT_START(start_cycle) ; 

  int i=0;
  int num_img_per_cnt_msg = 1 ;
//          while (i<NUM_IMG_TO_CHK) {  // TMP NEED ROUBUST BARE-METAL EOF EQUIVELENT
//      	for (int a = 0; a < 1; a++) {
 if (((i%num_img_per_cnt_msg)==0)&(i>0)) {
   printf("Checked %d images ...\n",i) ;
   GET_CYCLE_COUNT_END(end_cycle) ;             
   bm_printf("%d cycles for last %d images\n", end_cycle-start_cycle,num_img_per_cnt_msg) ;   
   RESET_CYCLE_COUNT  ; // Reset counter to ensure 32 bit counter does not wrap in-between start and end.           
   GET_CYCLE_COUNT_START(start_cycle) ; // start measuring for next num_img_per_cnt_msg images                  
 }   
 i++ ;

if (((i%num_img_per_cnt_msg)==0)&(i>0)) {
    RESET_CYCLE_COUNT  ; // Reset counter to ensure 32 bit counter does not wrap in-between start and end.           
    GET_CYCLE_COUNT_START(start_cycle) ; // start measuring for next num_img_per_cnt_msg images                  
}   

// ----------------------------------------------------------------------------------------------
// -------------------------------- convolution layer -------------------------------------------
// ----------------------------------------------------------------------------------------------   
int sc = LOG2_RELU_FACTOR ;  // 'sc' (extra scale) is determined by LOG2_RELU_FACTOR , consider maling this layer specific;
printf("conv1\n");
print4DTensor_uint8(image);
Tensor4D_uint8 * actResult_4D_tensor = tensor4DConvNActivate(image, &conv_weight[0], &conv_bias[0], result_4D_tensor_uint8, (Allocator_int32 *)al, (MatAllocator_int32 *)mat_al, (TensorAllocator_int32 *)tens_alloc, sc);
print4DTensor_uint8(actResult_4D_tensor);																																							

bm_quit_app();  // uart message to simulation/pyshell to quit
return 0;
}

