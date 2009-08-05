ADD_DEFINITIONS(-DQT_THREAD_SUPPORT -o3)

# required paths
INCLUDE_DIRECTORIES( ../mmx )

# create a shared library from the source code in this directory
ADD_LIBRARY( 
sptmpmmx
#SHARED 
#STATIC
alloc.c timer.c fast_sqrt.c dmap_util.c dmap_sad.c dmap_ncc.c
dmap_flow.c dmap_filt.c matrices_cache.c
)


# target link libraries
TARGET_LINK_LIBRARIES(
sptmpmmx 
mmx
 )



# install targets
INSTALL_TARGETS(${LIB_DIR} 
  sptmpmmx
  )
