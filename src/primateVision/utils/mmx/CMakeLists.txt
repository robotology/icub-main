
# defines
ADD_DEFINITIONS( -O3 -D_PIII -D_PIV -fPIC )

# create a shared library from the source code in this directory
ADD_LIBRARY( 
mmx
#STATIC
MMXConvert.c 
MMXConvolve.c
MMXCorrel.c
MMXFlow.c
MMXKernel.c
MMXMatrix.c
MMXShrink.c
MMXUtils.c
MMXyuv.c
MMXOp.c
)



# install targets
INSTALL_TARGETS(${LIB_DIR} 
  mmx
  )
