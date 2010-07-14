project(iCubGui)

set(QT_USE_QTOPENGL TRUE)

find_package(QtOpenGL)

if(QTOPENGL_FOUND)
	subdirs(src)
else(QTOPENGL_FOUND)
	if(NOT QT_FOUND)
		message("Qt3 not found, won't compile iCubGui")
	endif(NOT QT_FOUND)	
	if(NOT GLUT_FOUND)
		message("Glut not found, won't compile iCubGui")
	endif(NOT GLUT_FOUND)
	if(NOT OPENGL_FOUND)
		message("Qt3 not found, won't compile iCubGui")
	endif(NOT OPENGL_FOUND)
endif(QTOPENGL_FOUND)
