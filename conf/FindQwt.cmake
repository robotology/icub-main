# Try to find qwt
# Once done this will define
#
#  QWT_FOUND - system has qwt
#  QWT_INCLUDE_DIR - ~ the qwt include directory (needs <qwt/qwt_header_xy.h>)
#  QWT_LIBRARY - Link these to use qwt
#
#  2007, Jonas Ruesch
#

FIND_PATH(QWT_INCLUDE_DIR 
NAMES   qwt/qwt_plot.h
PATHS   /usr/local/include
        /usr/include
)


FIND_LIBRARY(QWT_LIBRARY
NAMES qwt
PATHS /usr/lib
      /usr/local/lib
)


if (QWT_LIBRARY)
    set(QWT_FOUND TRUE)
else (QWT_LIBRARY)
    set(QWT_FOUND FALSE)    
endif (QWT_LIBRARY)

set(QWT_INCLUDE_DIR
    ${QWT_INCLUDE_DIR}
)

#MESSAGE("----------qwt: ${QWT_INCLUDE_DIR} qwtlib: ${QWT_LIBRARY}")

