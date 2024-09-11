#ifndef MANNIX_LIB
#define MANNIX_LIB


// ================= data allocation ============================
Allocator_int32* createAllocator_int32(Allocator_int32* alloc, int * data, int max_size) {
    alloc[0].index = 0;
    alloc[0].max_size = max_size-1;
    alloc[0].data = data;
    return alloc;
}

Allocator_int8* createAllocator_int8(Allocator_int8* alloc, signed char* data, int max_size) {
    alloc[0].index = 0;
    alloc[0].max_size = max_size-1;
    alloc[0].data = (signed char*)data;
    return alloc;
}

Allocator_uint8* createAllocator_uint8(Allocator_uint8* alloc, unsigned char* data, int max_size) {
    alloc[0].index = 0;
    alloc[0].max_size = max_size-1;
    alloc[0].data = data;
    return alloc;
}

static int data_buf_req_byte_size = 0 ;

int* mannixDataMalloc_int32(Allocator_int32* alloc, int length) {
    
    Allocator_uint8 * byte_alloc = (Allocator_uint8 *) alloc ;
    
    int byte_length = length*4 ;
    if (byte_alloc[0].index + byte_length < byte_alloc[0].max_size)
        byte_alloc[0].index += byte_length;
    else {
        bm_printf("ERROR-out of range allocation");
        exit(-1);
    }
    if (byte_alloc[0].index > data_buf_req_byte_size) {
     printf("model weights/bias/workspace allocated data memory %d\n",byte_alloc[0].index) ;
     data_buf_req_byte_size = byte_alloc[0].index ;
    }
    
    return (int *)&(byte_alloc[0].data[byte_alloc[0].index - byte_length]);
}
    
 
signed char* mannixDataMalloc_int8(Allocator_int8* alloc, int length) {
    if (alloc[0].index + length < alloc[0].max_size)
        alloc[0].index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    printf("model weights/bias allocated data memory %d\n",alloc[0].index) ;
    if (alloc[0].index > data_buf_req_byte_size) {
     printf("model weights/bias allocated data memory %d\n",alloc[0].index) ;
     data_buf_req_byte_size = alloc[0].index ;
    }
    return (signed char*)&(alloc[0].data[alloc[0].index -length]) ; 
}

unsigned char* mannixDataMalloc_uint8(Allocator_uint8* alloc, int length) {
    if (alloc[0].index + length < alloc[0].max_size)
        alloc[0].index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    if (alloc[0].index > data_buf_req_byte_size) {
     printf("model weights/bias allocated data memory %d\n",alloc[0].index) ;
     data_buf_req_byte_size = alloc[0].index ;
    }    
    return (unsigned char *)&(alloc[0].data[alloc[0].index -length]) ; 
}
 
 
int mannixDataFree_uint8(Allocator_uint8* alloc, unsigned char* data, int length) {
    
    Allocator_uint8 * byte_alloc = alloc ;
    
    int byte_length = length ;
    if ((byte_alloc->data + byte_alloc->index - byte_length) == (unsigned char *)data)
        byte_alloc->index -= byte_length;
    else {
        bm_printf("ERROR- cannot free memory due to incorrect size  [%p][%p]\n",(alloc->data - length),data);
        exit(-1);
    }
    return 0;
}


int mannixDataFree_int32(Allocator_int32* alloc, int* data, int length) {
    
    Allocator_uint8 * byte_alloc = (Allocator_uint8 *) alloc ;
    
    int byte_length = length*4 ;
    if ((byte_alloc->data + byte_alloc->index - byte_length) == (unsigned char *)data)
        byte_alloc->index -= byte_length;
    else {
        printf("ERROR- cannot free memory due to incorrect size  [%p][%p]\n",(alloc->data - length),data);
        exit(-1);
    }
    return 0;
}


// ================= Matrix allocation ============================

MatAllocator_int32  * createMatrixAllocator_int32 ( MatAllocator_int32 * mat_alloc, Matrix_int32* matrix, int max_size) {
    mat_alloc->index = 0;
    mat_alloc->max_size = max_size-1;
    mat_alloc->matrix = matrix;
    return mat_alloc;
}

MatAllocator_uint8  * createMatrixAllocator_uint8 ( MatAllocator_uint8 * mat_alloc, Matrix_uint8* matrix, int max_size) {
    mat_alloc->index = 0;
    mat_alloc->max_size = max_size-1;
    mat_alloc->matrix = matrix;
    return mat_alloc;
}



Matrix_int32* mannixMatrixMalloc_int32(MatAllocator_int32* mat_alloc, int length) {
    if (mat_alloc->index + length < mat_alloc->max_size)  mat_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return mat_alloc->matrix + mat_alloc->index -length; 
}

Matrix_int8* mannixMatrixMalloc_int8(MatAllocator_int8* mat_alloc, int length) {
    if (mat_alloc->index + length < mat_alloc->max_size)  mat_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return mat_alloc->matrix + mat_alloc->index -length; 
}

Matrix_uint8* mannixMatrixMalloc_uint8(MatAllocator_uint8* mat_alloc, int length) {
    if (mat_alloc->index + length < mat_alloc->max_size)  mat_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return mat_alloc->matrix + mat_alloc->index - length;
}


// ================= Tensor allocation ============================
TensorAllocator_int32* createTensorAllocator_int32(TensorAllocator_int32* tens_alloc, Tensor_int32* tens, int max_size) {
    tens_alloc->index = 0;
    tens_alloc->max_size = max_size-1;
    tens_alloc->tensor = tens;
    return tens_alloc;
}

TensorAllocator_uint8* createTensorAllocator_uint8(TensorAllocator_uint8* tens_alloc, Tensor_uint8* tens, int max_size) {
    tens_alloc->index = 0;
    tens_alloc->max_size = max_size-1;
    tens_alloc->tensor = tens;
    return tens_alloc;
}



Tensor_int32* mannixTensorMalloc_int32(TensorAllocator_int32* tens_alloc, int length) {
    if (tens_alloc->index + length < tens_alloc->max_size)
        tens_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return tens_alloc->tensor + tens_alloc->index -length; 
}



Tensor_uint8* mannixTensorMalloc_uint8(TensorAllocator_uint8* tens_alloc, int length) {
    if (tens_alloc->index + length < tens_alloc->max_size)
        tens_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return tens_alloc->tensor + tens_alloc->index -length; 
}


Tensor_int8* mannixTensorMalloc_int8(TensorAllocator_int8* tens_alloc, int length) {
    if (tens_alloc->index + length < tens_alloc->max_size)
        tens_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return tens_alloc->tensor + tens_alloc->index -length; 
}

//--------------------------------------
// HW ALLOCATORS
//-------------------------------------
/*Allocator_vol_uint32* createAllocator_vol_uint32(Allocator_vol_uint32* alloc, volatile unsigned int * data, int max_size) {
    alloc[0].index = 0;
    alloc[0].max_size = max_size-1;
    alloc[0].data = data;
    return alloc;
}

Allocator_vol_int32* createAllocator_vol_int32(Allocator_vol_int32* alloc, volatile signed int * data, int max_size) {
    alloc[0].index = 0;
    alloc[0].max_size = max_size-1;
    alloc[0].data = data;
    return alloc;
}

//--------------------------------------
// HW MALLOC
//-------------------------------------
/*
volatile unsigned int* mannixDataMalloc_vol_uint32(Allocator_vol_uint32* alloc, int length) {
    
    Allocator_vol_uint32 * byte_alloc = alloc ;
    
    int byte_length = length*4 ;
    if (byte_alloc->index + byte_length < byte_alloc->max_size)
        byte_alloc->index += byte_length;
    else {
        bm_printf("ERROR-out of range allocation");
        exit(-1);
    }
    if (byte_alloc->index > data_buf_req_byte_size) {
     printf("ACCELERATOR: model weights/bias/workspace allocated data memory %d\n",byte_alloc->index) ;
     data_buf_req_byte_size = byte_alloc->index ;
    }
    
    return byte_alloc->data + byte_alloc->index - byte_length;
}
 
/*volatile signed int* mannixDataMalloc_vol_int32(Allocator_vol_int32* alloc, int length) {
    
    Allocator_vol_int32 * byte_alloc = alloc ;
    
    int byte_length = length*4 ;
    if (byte_alloc->index + byte_length < byte_alloc->max_size)
        byte_alloc->index += byte_length;
    else {
        bm_printf("ERROR-out of range allocation");
        exit(-1);
    }
    if (byte_alloc->index > data_buf_req_byte_size) {
     printf("ACCELERATOR: model weights/bias/workspace allocated data memory %d\n",byte_alloc->index) ;
     data_buf_req_byte_size = byte_alloc->index ;
    }
    
    return byte_alloc->data + byte_alloc->index - byte_length;
}
 
/* 
Matrix_vol_uint32* mannixMatrixMalloc_vol_uint32(MatAllocator_vol_uint32* mat_alloc, int length) {
    if (mat_alloc->index + length < mat_alloc->max_size)  mat_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return mat_alloc->matrix + mat_alloc->index -length; 
}

Matrix_vol_int32* mannixMatrixMalloc_vol_int32(MatAllocator_vol_int32* mat_alloc, int length) {
    if (mat_alloc->index + length < mat_alloc->max_size)  mat_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return mat_alloc->matrix + mat_alloc->index -length; 
}


Tensor_vol_uint32* mannixTensorMalloc_vol_uint32(TensorAllocator_vol_uint32* tens_alloc, int length) {
    if (tens_alloc->index + length < tens_alloc->max_size)
        tens_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return tens_alloc->tensor + tens_alloc->index -length; 
}


Tensor_vol_int32* mannixTensorMalloc_vol_int32(TensorAllocator_vol_int32* tens_alloc, int length) {
    if (tens_alloc->index + length < tens_alloc->max_size)
        tens_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return tens_alloc->tensor + tens_alloc->index -length; 
}

*/
//======================================================================================

// Mannif Format (mfdb , mfds) Access functions

//--------------------------------------------------------------------------------------


void bm_fget_hex_val(FILE_PTR filePtr,unsigned int * val_ptr) { // Get from from hex file next unsigned char value 

   #ifndef BARE_METAL
   fscanf(filePtr,"%x",val_ptr);
   #else
   char str_buf[10] ;
   char retVal = bm_fgets (filePtr,str_buf) ;
   *val_ptr = hex_str_to_uint(str_buf) ;
   #endif
}

//-------------------------------------------------------------------


void dump_model_params_mfdb(Allocator_uint8 *alloc, char *dumpFileName) {
    
    FILE_PTR dumpFile = fopen_w(dumpFileName);

    printf("Dumping model parameters to %s\n",dumpFileName); 
    for (int i=0; i<alloc->index; i++) fprintf(dumpFile,"%02x\n",alloc->data[i]);    
    fclose(dumpFile) ;
    printf("\n\nDONE Dumping , exiting\n\n");
    exit(0);
}

//
// load_model_params_mfdb(al,MODEL_PARAMS_FILE);  // load mannix format data base "../model_params_db/model_params_mfdb.txt"
void load_model_params_mfdb(Allocator_uint8 *alloc, char *dumpFileName) {
    	printf("++++++++++++++++++++++++++++++++++++++++\n");
//asdf;lasdkf;laskdf;lasdkf;lasdfk;lasdfk;lasdkf;adslkfsad;lkdas;lf
    FILE_PTR dumpFile = fopen_r(dumpFileName);
    int num_loaded ;
        
    printf("Loading %d model parameters from %s to alloc->data array at addr %08x\n",alloc->index,dumpFileName, (unsigned int)alloc->data); 
		printf("++++++++++++++++++++++++++++++++++++++++\n");
    #ifndef BARE_METAL
    for (int i=0; i<alloc->index; i++) {
        unsigned int val ;
  			printf("11111111111111111111111111111111111111111111111111111111111111\n");
	bm_fget_hex_val(dumpFile,&val);
        alloc->data[i] = (unsigned char)val ;
        if ((i%1000)==0) printf("Loaded %d ...\n",i);         
    } 
    #else
      #ifdef DDP21_SIM // Fast soc level load
        printf("%d alloc index",alloc->index);
	bm_start_soc_load_hex_file (dumpFile, alloc->index,  (char *) alloc->data) ;      
	
	while (num_loaded==0){
	   //printf("num_loaded(%d)", num_loaded );
	 
	   num_loaded = bm_check_soc_load_hex_file () ; // num_loaded!=0 indicates completion.
 	}
	printf("num_loaded (%d)",num_loaded);
      #else // SOC File access currently supported only in simulation.
	printf("2222222222222222222222222222222222222222222222222222222222222222\n");
        num_loaded = bm_load_hex_file (dumpFile, alloc->index,  (char *) alloc->data) ;
      #endif
    #endif
    
    fclose(dumpFile) ;
    printf("\n\nDONE Loading, proceeding.\n\n");
}

//-------------------------------------------------------------------

void getData_mfds_uint8(FILE_PTR mfds_filePointer, int num_data_to_read, unsigned int* matrix_idx_ptr , unsigned int* label_ptr, unsigned char* matrix_flat_vec, int op) {
#ifdef SANSA_SIM
     bm_printf("C message: Loading %d bytes from file index %d\n" , num_data_to_read,mfds_filePointer) ;
#endif
     bm_fget_hex_val(mfds_filePointer,matrix_idx_ptr);
     bm_fget_hex_val(mfds_filePointer,label_ptr); 

     #ifdef DDP21_SIM
     bm_printf("C msg: Reading matrix ID : %d of label %d\n",*matrix_idx_ptr,*label_ptr);
     #endif

#ifndef BARE_METAL
     for (int i=0; i<num_data_to_read-2; i++) {  // already taken 2 items
         unsigned int val ;
         bm_fget_hex_val(mfds_filePointer,&val);
         matrix_flat_vec[i] = (signed char)val ;
     }  
#else
      int num_loaded ;
      #ifdef DDP21_SIM // Fast soc level load
        bm_start_soc_load_hex_file (mfds_filePointer, num_data_to_read-2, (char *)matrix_flat_vec) ;      
        while (num_loaded==0) num_loaded = bm_check_soc_load_hex_file () ; // num_loaded!=0 indicates completion.
      #else // SOC File access currently supported only in simulation.
        num_loaded = bm_load_hex_file (mfds_filePointer, num_data_to_read-2 ,(char *)matrix_flat_vec) ;      
      #endif

#endif
     
   
    
}


//=======================================================================================

#endif // MANNIX_LIB
