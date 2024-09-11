#define PADDING

//---- dubaging defines ----
//
//#define TEST // result example of few pictures
// #define CMP_TEST //
//#define DEBUG // debug variable

#ifdef CMP_TEST
#define IFDEF_CMP_TEST(dump_call) dump_call
#else
#define IFDEF_CMP_TEST(dump_call) 
#endif

//--------------------
//--------- OS -------
//--------------------
#define WINDOWS_MANNIX

//---- environment setups----
//#define VS_MANNIX

// --------------------------------------
// ------------ data setting ------------
// --------------------------------------
//#define MEM_DUMP_MODE // Dumps the model parameters loadable data vector , run once per model configuration.
#define MEM_LOAD_MODE  // Model parameters and data to be actively loaded , skip CSV read

//------------------------------------------------------
// ------- Path to image and model param files ---------
//------------------------------------------------------
#ifdef DDP21_SIM
//bm_printf("ddp21\n");
   #define MODEL_PARAMS_FILE "app_src_dir/fashion_mnist_model/model_params_db/all_ones8.txt"
   #define MODEL_PARAMS_FILE_VOL "app_src_dir/fashion_mnist_model/model_params_db/all_ones_vol32.txt"
   #define DATASET_FILE      "app_src_dir/fashion_mnist_model/test_src/all_ones.txt"
#else
  #ifdef SANSA_SIM
bm_printf("sansana\n");
    #define MODEL_PARAMS_FILE "/space/soc/sansa/udik/simulation/fashion_mnist_model/model_params_db/all_ones8.txt"
    #define MODEL_PARAMS_FILE_VOL "/space/soc/sansa/udik/simulation/fashion_mnist_model/model_params_db/all_ones_vol32.txt"
    #define DATASET_FILE      "/space/soc/sansa/udik/simulation/fashion_mnist_model/test_src/fashion_mnist_V1_mfds.txt"
  #else
bm_printf("else*****************************************\n");
    #define MODEL_PARAMS_FILE "fashion_mnist_model/model_params_db/all_ones8.txt"
    #define MODEL_PARAMS_FILE_VOL "fashion_mnist_model/model_params_db/all_ones_vol32.txt"
    #define DATASET_FILE "fashion_mnist_model/test_src/fashion_mnist_V1_mfds.txt"
  #endif
#endif // !DDP21_SIM

//#define DATASET_FILE "fashion_mnist_model/test_src/fashion_mnist_V1_mfds.txt"


#define ONLY_KERNEL_5X5_USED

// ------------------------------------
// ----- memory size definitions ------
// ------------------------------------
#define MANNIX_DATA_SIZE 53000 
#define MANNIX_MAT_SIZE 2000
#define MANNIX_TEN_SIZE 500
#define PADDING
// activation
#define WB_LOG2_SCALE 7
#define UINT_DATA_WIDTH 8
#define LOG2_RELU_FACTOR 1
#define EXP_UINT_DATA_WIDTH 1 << UINT_DATA_WIDTH
#define MAX_DATA_RANGE (EXP_UINT_DATA_WIDTH) - 1  
// WORK MODE
#define ACCELERATOR_MODE 1
#define SW_MODE !ACCELERATOR_MODE 
// BARE-METAL BARE_METAL MODE

// ----------------------------------------
// ------- PULP ENVIRONMENT SETTING -------
// ----------------------------------------
#ifndef BARE_METAL
   #define FILE_PTR FILE*
   #define fopen_w(fileName) fopen(fileName,"w")
   #define fopen_r(fileName) fopen(fileName,"r")  
   #pragma warning(disable : 4996)
#else
   #define printf bm_printf
   #define fprintf bm_fprintf   
   #define exit(X) sim_finish()
   #define fopen_w(fileName) bm_fopen_w(fileName)
   #define fopen_r(fileName) bm_fopen_r(fileName)  
   #define fclose bm_fclose   
   #define FILE_PTR int
   
// ----------------------------------------
// ------------- UART DEFINES -------------
// ----------------------------------------
   //#define CLK_PERIOD_NS CLK_PERIOD  
   #define NS_PER_BIT ((1000000000.0)/BAUD_RATE)
   #define UART_CLK (((NS_PER_BIT/CLK_PERIOD))-0.5)
   #define FBDIV_FACTOR 2*(250/(CLK_PERIOD*10))

   
   #define NULL -1 // TMP, TODO - CHECK
#endif


