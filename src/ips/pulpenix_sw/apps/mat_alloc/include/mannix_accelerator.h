#ifndef MANNIX_ACCELERATOR
#define MANNIX_ACCELERATOR


typedef struct Matrix_vol32 {
    int rows;
    int cols;
    int size;
    int pad_size;
    volatile char* data;    
}Matrix_vol32;

typedef struct Tensor_vol32 {
    int rows;
    int cols;
    int depth;
    Matrix_vol32* matrix;  
}Tensor_vol32;

typedef struct Tensor4D_vol32 {
    int rows;
    int cols;
    int depth;
    int dim;
    Tensor_vol32* tensor;
} Tensor4D_vol32;

// -------- allocators ----------

typedef struct Allocator_vol32 {
    int index;
    int max_size;
    char* data;
} Allocator_vol32 ;

typedef struct MatAllocator_vol32 {
    int index;
    int max_size;
    Matrix_vol32 * matrix;
} MatAllocator_vol32 ;

typedef struct TensorAllocator_vol32 {
    int index;
    int max_size;
    Tensor_vol32* tensor;
} TensorAllocator_vol32 ;

// ================== matrix functions ========================
// ================= int allocation ============================
Allocator_vol32* createAllocator_vol32(Allocator_vol32* alloc, void * data, int max_size) {
    alloc[0].index = 0;
    alloc[0].max_size = max_size-1;
    alloc[0].data = data;
    return alloc;
}

MatAllocator_vol32  * createMatrixAllocator_vol32 ( MatAllocator_vol32 * mat_alloc, Matrix_vol32* matrix, int max_size) {
    mat_alloc->index = 0;
    mat_alloc->max_size = max_size-1;
    mat_alloc->matrix = matrix;
    return mat_alloc;
}

TensorAllocator_vol32* createTensorAllocator_vol32(TensorAllocator_vol32* tens_alloc, Tensor_vol32* tens, int max_size) {
    tens_alloc->index = 0;
    tens_alloc->max_size = max_size-1;
    tens_alloc->tensor = tens;
    return tens_alloc;
}

void* mannixDataMalloc_vol32(Allocator_vol32* alloc, int length) {
        
    int byte_length = length ;
    if (alloc[0].index + byte_length < alloc[0].max_size)
        alloc->index += byte_length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return alloc->data + alloc->index - byte_length; 
}

Matrix_vol32* mannixMatrixMalloc_vol32(MatAllocator_vol32* mat_alloc, int length) {
    if (mat_alloc->index + length < mat_alloc->max_size)  mat_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return mat_alloc->matrix + mat_alloc->index -length; 
}

Tensor_vol32* mannixTensorMalloc_vol32(TensorAllocator_vol32* tens_alloc, int length) {
    if (tens_alloc->index + length < tens_alloc->max_size)
        tens_alloc->index += length;
    else {
        printf("ERROR-out of range allocation");
        exit(-1);
    }
    return tens_alloc->tensor + tens_alloc->index -length; 
}

    // ------------------matrix creation -------------------------
    // cearte matrix with the value of zero 
Matrix_vol32* creatMatrix_vol32(int rows, int cols, Matrix_vol32* m1, Allocator_vol32* al) {
	m1[0].cols = cols;
        m1[0].rows = rows;
        m1->size = cols * rows;
        m1->pad_size = m1->size + (4 - m1->size%4)%4;
        m1[0].data = mannixDataMalloc_vol32(al, m1->pad_size);

        return m1;
    }


Matrix_vol32* creatMatrix_vol32b(int rows, int cols, Matrix_vol32* m1, Allocator_vol32* al) {
	m1[0].cols = cols;
        m1[0].rows = rows;
        m1->size = cols * rows;
        m1->pad_size = m1->size;
        m1[0].data = mannixDataMalloc_vol32(al, m1->pad_size * 4);

        return m1;
    }


Tensor_vol32* createTensor_vol32(int rows, int cols, int depth, Tensor_vol32* tens, Allocator_vol32* al, MatAllocator_vol32* mat_alloc) {

    tens->rows = rows;
    tens->cols = cols;
    tens->depth = depth;
    tens->matrix = mannixMatrixMalloc_vol32(mat_alloc, depth);
    for(int i = 0; i < depth; i++) {
        creatMatrix_vol32(rows, cols, &tens->matrix[i], al);
    }
    return  tens;
}

Tensor_vol32* createTensor_vol32b(int rows, int cols, int depth, Tensor_vol32* tens, Allocator_vol32* al, MatAllocator_vol32* mat_alloc) {

    tens->rows = rows;
    tens->cols = cols;
    tens->depth = depth;
    tens->matrix = mannixMatrixMalloc_vol32(mat_alloc, depth);
    for(int i = 0; i < depth; i++) {
        creatMatrix_vol32b(rows, cols, &tens->matrix[i], al);
    }
    return  tens;
}

void create4DTensor_vol32(Tensor4D_vol32* tens_4d, int rows, int cols, int depth, int dim, Allocator_vol32* al, MatAllocator_vol32* mat_alloc, TensorAllocator_vol32* tens_alloc) {
    tens_4d->rows = rows;
    tens_4d->cols = cols;
    tens_4d->depth = depth;
    tens_4d->dim = dim;
    tens_4d->tensor = mannixTensorMalloc_vol32(tens_alloc, dim);

    for(int i = 0; i < tens_4d->dim; i++)
        createTensor_vol32(rows, cols, depth, &tens_4d->tensor[i], al, mat_alloc);
}


void create4DTensor_vol32b(Tensor4D_vol32* tens_4d, int rows, int cols, int depth, int dim, Allocator_vol32* al, MatAllocator_vol32* mat_alloc, TensorAllocator_vol32* tens_alloc) {
    tens_4d->rows = rows;
    tens_4d->cols = cols;
    tens_4d->depth = depth;
    tens_4d->dim = dim;
    tens_4d->tensor = mannixTensorMalloc_vol32(tens_alloc, dim);

    for(int i = 0; i < tens_4d->dim; i++)
        createTensor_vol32b(rows, cols, depth, &tens_4d->tensor[i], al, mat_alloc);
}


void printMatrix_vol32(Matrix_vol32* m1) {
    if(m1->rows <= 0 || m1->cols <= 0) {
        printf("Dimension ERRER - non positive hight or width  [%d][%d]\n",m1-> cols, m1->rows);
        exit(-1);
    }
    int i=0;
    while(i<m1[0].rows * m1[0].cols) {
        if (i % m1[0].cols == 0)
            printf(" [");
        printf("%d ", *((int*)m1->data + i));
	i = i + 1;
        if (i % m1[0].cols == 0 && i<m1[0].rows * m1[0].cols)
            printf("]\n");
        else if (i % m1[0].cols == 0)
            printf("]");
    }
    // printf("\n row size = %d, col size = %d\n\n", m1[0].rows, m1[0].cols);
}

void printMatrix_vol32b(Matrix_vol32* m1) {
    if(m1->rows <= 0 || m1->cols <= 0) {
        printf("Dimension ERRER - non positive hight or width  [%d][%d]\n",m1-> cols, m1->rows);
        exit(-1);
    }
    int i=0;
    while(i<m1[0].rows * m1[0].cols) {
        if (i % m1[0].cols == 0)
            printf(" [");
        printf("%d ", *((int*)m1->data + i));
	i = i + 1;
        if (i % m1[0].cols == 0 && i<m1[0].rows * m1[0].cols)
            printf("]\n");
        else if (i % m1[0].cols == 0)
            printf("]");
    }
    // printf("\n row size = %d, col size = %d\n\n", m1[0].rows, m1[0].cols);
}


void printTensor_vol32(Tensor_vol32* tens) {

    for(int i = 0; i < tens->depth; i++) {
        printMatrix_vol32(&tens->matrix[i]);
        if(i!= tens->depth-1) 
            printf("\n,\n");        
    }
}


void printTensor_vol32b(Tensor_vol32* tens) {

    for(int i = 0; i < tens->depth; i++) {
        printMatrix_vol32b(&tens->matrix[i]);
        if(i!= tens->depth-1) 
            printf("\n,\n");        
    }
}


void print4DTensor_vol32(Tensor4D_vol32* tens_4d) {

    printf("tensor{\n");    
    for(int i = 0; i < tens_4d->dim; i++) {
        printf("\n[");
        printTensor_vol32(&tens_4d->tensor[i]);
        printf(" ]\n");
    }
    printf("}\n");
    printf("rows size: %d columns size: %d depth size: %d dim: %d \n\n",tens_4d->rows,tens_4d->cols,tens_4d->depth,tens_4d->dim);

}


void print4DTensor_vol32b(Tensor4D_vol32* tens_4d) {

    printf("tensor{\n");    
    for(int i = 0; i < tens_4d->dim; i++) {
        printf("\n[");
        printTensor_vol32b(&tens_4d->tensor[i]);
        printf(" ]\n");
    }
    printf("}\n");
    printf("rows size: %d columns size: %d depth size: %d dim: %d \n\n",tens_4d->rows,tens_4d->cols,tens_4d->depth,tens_4d->dim);

}

// Mannif Format (mfdb , mfds) Access functions
void dump_model_params_mfdb_vol32(Allocator_vol32 * alloc, char *dumpFileName) {
    
    FILE_PTR dumpFile = fopen(dumpFileName,"w");

    printf("Dumping model parameters to %s\n",dumpFileName); 
    for (int i=0; i<alloc->index; i = i+4) {
        int word32 = 0x00000000; 
        word32 = word32 | alloc->data[i];
        word32 = word32 | (int)(alloc->data[i+1]) << 8*1;
        word32 = word32 | (int)(alloc->data[i+2]) << 8*2;
        word32 = word32 | (int)(alloc->data[i+3]) << 8*3;
        printf("-----------\n");
        printf("%08x - i=%d\n",alloc->data[i],i);
        printf("%08x - i=%d\n",alloc->data[i+1],i+1);
        printf("%08x - i=%d\n",alloc->data[i+2],i+2);
        printf("%08x - i=%d\n",alloc->data[i+3],i+3);
        printf("-----------\n");
        printf("%08x - i=%d\n",word32,i);
        printf("-----------\n");
        fprintf(dumpFile,"%08x\n",word32);
    }
        
    fclose(dumpFile) ;
    printf("\n\nDONE Dumping vol32\n\n");
    // exit(0);
}


Matrix_vol32 * vol_ACCELERATOR_matrixFCNActivate(Matrix_vol32* input_matrix, Matrix_vol32* weight_matrix, Matrix_vol32* bias_vector, Matrix_vol32* result_matrix, Allocator_vol32* vol_al ) {

    //  I create a pointer int* because the accelerator getting data and not address
    volatile unsigned int * fc_addrx = (volatile unsigned int *)(FC_ADDRX);
    volatile signed   int * fc_addry = (volatile signed   int *)(FC_ADDRY);
    volatile unsigned int * fc_addrz = (volatile unsigned int *)(FC_ADDRZ);
    volatile unsigned int * fc_addrb = (volatile unsigned int * )(FC_ADDRB);
    volatile unsigned int * fc_xm    = (volatile unsigned int * )(FC_XM);
    volatile unsigned int * fc_xn    = (volatile unsigned int * )(FC_XN);
    volatile unsigned int * fc_ym    = (volatile unsigned int * )(FC_YM);
    volatile unsigned int * fc_yn    = (volatile unsigned int * )(FC_YN);
    volatile unsigned int * fc_start = (volatile unsigned int * )(FC_START);
    volatile unsigned int * fc_done  = (volatile unsigned int * )(FC_DONE);
	
    creatMatrix_vol32(weight_matrix->rows ,input_matrix->cols , result_matrix, vol_al);
    
    * fc_addrx = input_matrix->data  - GPP_BASE_MEM;
    * fc_addry = weight_matrix->data - GPP_BASE_MEM;
    * fc_addrz = result_matrix->data - GPP_BASE_MEM;
    * fc_addrb = bias_vector->data   - GPP_BASE_MEM;
    * fc_xm = input_matrix->rows;
    * fc_ym = weight_matrix->rows;
    * fc_yn = weight_matrix->cols; 
	
    * fc_start = 1;
    while(!fc_done);
    
    return result_matrix;
       
}




Matrix_vol32 * vol_matrixMaxPool(Matrix_vol32* m1, Matrix_vol32* m2, int p_m, int p_n, int stride){

    //  I create a pointer int* because the accelerator getting data and not address
    volatile unsigned int * p_addrx = (volatile unsigned int *)(POOL_ADDRX);
    volatile unsigned int * p_addrz = (volatile unsigned int *)(POOL_ADDRZ);
    volatile unsigned int * p_xm    = (volatile unsigned int * )(POOL_XM);
    volatile unsigned int * p_xn    = (volatile unsigned int * )(POOL_XN);
    volatile unsigned int * p_pm    = (volatile unsigned int * )(POOL_PM);
    volatile unsigned int * p_pn    = (volatile unsigned int * )(POOL_PN);
    volatile unsigned int * p_stride = (volatile unsigned int * )(POOL_STRIDE);
    volatile unsigned int * p_start = (volatile unsigned int * )(POOL_START);
    volatile unsigned int * p_done  = (volatile unsigned int * )(POOL_DONE);
    
    * p_addrx = m1->data  - GPP_BASE_MEM;
    * p_addrz = m2->data - GPP_BASE_MEM;
    * p_xm = m1->rows;
    * p_xn = m1->rows;
    * p_pm = m2->rows;
    * p_pn = m2->cols; 
    * p_stride = stride;
    * p_start = 1;
    while(!p_done);
    
    return m2;
       
}


Tensor_vol32* tensorMaxPool_vol32(Tensor_vol32 *tens, Tensor_vol32 *tens_result, int p_m, int p_n, int stride){
      for(int d = 0; d < tens->depth; d++) {
        vol_matrixMaxPool(&tens->matrix[d], &tens_result->matrix[d], p_m, p_n, stride);
        volatile unsigned int * p_done  = (volatile unsigned int * )(POOL_DONE);    	
	//while(p_done);
      }
    return tens_result;
}



Tensor4D_vol32* vol_tensor4DMaxPool(Tensor4D_vol32* tens_4d, Tensor4D_vol32* tens_4d_result, int p_m, int p_n, int stride, Allocator_vol32* al, MatAllocator_vol32* mat_alloc, TensorAllocator_vol32* tens_alloc) {
    
    int new_rows = (tens_4d->rows - p_m) / stride + 1;
    int new_cols = (tens_4d->cols - p_m) / stride + 1;

    create4DTensor_vol32(tens_4d_result, new_rows, new_cols, tens_4d->depth, tens_4d->dim, al, mat_alloc, tens_alloc);

    for (int i = 0; i < tens_4d->dim; i++) {
        tensorMaxPool_vol32(&tens_4d->tensor[i], &tens_4d_result->tensor[i], p_m, p_n, stride);
    }

    return tens_4d_result;
}




// load_model_params_mfdb(al,MODEL_PARAMS_FILE);  // load mannix format data base "../model_params_db/model_params_mfdb.txt"
void load_model_params_mfdb_vol32(Allocator_vol32 *alloc, char *dumpFileName) {
    printf("++++++++++++++++++++++++++++++++++++++++\n");
    FILE_PTR dumpFile = fopen_r(dumpFileName);        
    printf("Loading volotile memory %d model parameters from %s to alloc->data array at addr %08x\n",alloc->index,dumpFileName, (unsigned int)alloc->data); 
    printf("++++++++++++++++++++++++++++++++++++++++\n");
    int num_loaded ;
    #ifndef BARE_METAL
    for (int i=0; i<alloc->index; i++) {
        unsigned int val ;
	bm_fget_hex_val(dumpFile,&val);
        alloc->data[i] = (unsigned char)val ;
        if ((i%1000)==0) printf("Loaded %d ...\n",i);         
    } 
    #else
      #ifdef DDP21_SIM // Fast soc level load
        printf("%d alloc index",alloc->index);
	unsigned char bytes_vec1[alloc->index] ;
	//bm_load_hex_file (dumpFile,alloc->index,bytes_vec1); // load vec1 byte from hex file
	bm_start_soc_load_hex_file (dumpFile, alloc->index,  bytes_vec1) ;      
	while (num_loaded==0) num_loaded = bm_check_soc_load_hex_file () ; // num_loaded!=0 indicates completion.

	for(int i = 0; i < alloc->index; i=i+4) {
		 alloc->data[i] = *((unsigned int*)bytes_vec1 + i/4);
	}

	printf("num_loaded (%d)",num_loaded);
      #else // SOC File access currently supported only in simulation.
        num_loaded = bm_load_hex_file(dumpFile, alloc->index, (char*) alloc->data) ;
      #endif
    #endif
    
    fclose(dumpFile) ;
    printf("\n\nDONE Loading, proceeding.\n\n");
}
#endif
