ADD_DEFINITIONS(-DQT_THREAD_SUPPORT -o3)

SUBDIRS(display  
	multiFrameViewer
	dog
	canny
	timing
	mosaic  
	convert_rgb  
	convert_yuv
	convert_bitdepth
	centsur
	rectify
	depth
	flow
	depthflow
	lgphase
        multiclass
        mmx
        sptmp_MMX
	mydraw
	norm
	)
