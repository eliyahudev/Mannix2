#ifndef CNN_INC_H
#define CNN_INC_H

////
#define BARE_METAL
//#define SANSA_SIM
#define DDP21_SIM

#ifndef BARE_METAL
  #include "stdio.h"
  #include <stdlib.h>
  #include <windows.h>
  #pragma warning(disable : 4996)
#else
  #include <ddp23_libs.h>

  #ifdef SANSA_SIM 
   #include <carp.h>
  #endif
      
#endif
#include "mannix_regs_define.h"
#include "man_def.h"

#ifdef BARE_METAL
 #define  PULP_EXT
 #include "conv_5x5.h"
#endif

#include "man_struct.h"

#ifndef MEM_LOAD_MODE
   #include "read_csv.h"
#endif

#include "mannixlib.h"
#include "mannix_matrix.h"
 
#include "mannix_tensor.h"
#include "mannix_4dtensor.h"
#include "mannix_accelerator.h" 
//#include "cnn.h"  // not needed for now

#endif 
