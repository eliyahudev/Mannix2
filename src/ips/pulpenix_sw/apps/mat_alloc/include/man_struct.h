#ifndef MAN_STRUCT
#define MAN_STRUCT

// sw structures
typedef struct Matrix_int32 {
    int rows;
    int cols;
    int size;
    int pad_size;
    int* data;    
} Matrix_int32 ;


typedef struct Matrix_int8 {
    int rows;
    int cols;
    int size;
    int pad_size;
    signed char* data;    
} Matrix_int8 ;

typedef struct Matrix_uint8 {
    int rows;
    int cols;
    int size;
    int pad_size;
    unsigned char* data;    
} Matrix_uint8 ;

typedef struct Tensor_int32 {
    int rows;
    int cols;
    int depth;
    Matrix_int32* matrix;    
} Tensor_int32;

typedef struct Tensor_int8 {
    int rows;
    int cols;
    int depth;
    Matrix_int8* matrix;   
} Tensor_int8;

typedef struct Tensor_uint8 {
    int rows;
    int cols;
    int depth;
    Matrix_uint8* matrix;  
} Tensor_uint8;

typedef struct Tensor4D_int32 {
    int rows;
    int cols;
    int depth;
    int dim;
    Tensor_int32* tensor;
} Tensor4D_int32;

typedef struct Tensor4D_int8 {
    int rows;
    int cols;
    int depth;
    int dim;
    Tensor_int8* tensor;
} Tensor4D_int8;

typedef struct Tensor4D_uint8 {
    int rows;
    int cols;
    int depth;
    int dim;
    Tensor_uint8* tensor;
} Tensor4D_uint8;

//------------------------------
// ALLOCATORS
//-----------------------------
typedef struct Allocator_int32 {
    int index;
    int max_size;
    int* data;
} Allocator_int32 ;

typedef struct Allocator_int8 {
    int index;
    int max_size;
    signed char * data;
} Allocator_int8 ;

typedef struct Allocator_uint8 {
    int index;
    int max_size;
    unsigned char * data;
} Allocator_uint8 ;

typedef struct MatAllocator_int32 {
    int index;
    int max_size;
    Matrix_int32 * matrix;
} MatAllocator_int32 ;

typedef struct MatAllocator_int8 {
    int index;
    int max_size;
    Matrix_int8* matrix;
} MatAllocator_int8 ;

typedef struct MatAllocator_uint8 {
    int index;
    int max_size;
    Matrix_uint8 * matrix;
} MatAllocator_uint8 ;

typedef struct TensorAllocator_int32 {
    int index;
    int max_size;
    Tensor_int32* tensor;
} TensorAllocator_int32 ;

typedef struct TensorAllocator_int8 {
    int index;
    int max_size;
    Tensor_int8* tensor;
} TensorAllocator_int8 ;

typedef struct TensorAllocator_uint8 {
    int index;
    int max_size;
    Tensor_uint8* tensor;
} TensorAllocator_uint8 ;

//------------------------------------------
// hw structures
//------------------------------------------
typedef struct Matrix_vol_uint32 {
    int rows;
    int cols;
    int size;
    int pad_size;
    volatile unsigned int* data;    
} Matrix_vol_uint32 ;

typedef struct Matrix_vol_int32 {
    int rows;
    int cols;
    int size;
    int pad_size;
    volatile signed int* data;    
} Matrix_vol_int32 ;

typedef struct Tensor_vol_uint32 {
    int rows;
    int cols;
    int depth;
    Matrix_vol_uint32* matrix;    
} Tensor_vol_uint32;

typedef struct Tensor_vol_int32 {
    int rows;
    int cols;
    int depth;
    Matrix_vol_int32* matrix;    
} Tensor_vol_int32;

typedef struct Tensor4D_vol_uint32 {
    int rows;
    int cols;
    int depth;
    int dim;
    Tensor_vol_uint32* tensor;
} Tensor4D_vol_uint32;

typedef struct Tensor4D_vol_int32 {
    int rows;
    int cols;
    int depth;
    int dim;
    Tensor_vol_int32* tensor;
} Tensor4D_vol_int32;
//------------------------------
// ALLOCATORS
//------------------------------

typedef struct Allocator_vol_uint32 {
    int index;
    int max_size;
    volatile unsigned int* data;
} Allocator_vol_uint32 ;


typedef struct Allocator_vol_int32 {
    int index;
    int max_size;
    volatile signed int* data;
} Allocator_vol_int32 ;

typedef struct MatAllocator_vol_uint32 {
    int index;
    int max_size;
    Matrix_vol_uint32 * matrix;
} MatAllocator_vol_uint32 ;

typedef struct MatAllocator_vol_int32 {
    int index;
    int max_size;
    Matrix_vol_int32 * matrix;
} MatAllocator_vol_int32 ;

typedef struct TensorAllocator_vol_uint32 {
    int index;
    int max_size;
    Tensor_vol_uint32* tensor;
} TensorAllocator_vol_uint32 ;

typedef struct TensorAllocator_vol_int32 {
    int index;
    int max_size;
    Tensor_vol_int32* tensor;
} TensorAllocator_vol_int32 ;


#endif //  MAN_STRUCT
