RM =		rm -rf
CC =		g++

INCLUDE =	-I.
LDFLAGS =	-lrt
FLAGS =		-O3 -funroll-all-loops
CFLAGS =	$(FLAGS) $(INCLUDE)
PRECMD =	echo ''

BLAS ?=		refblas

PROJECT =	vector_blas


ifeq ($(BLAS),gsl)
CFLAGS +=	-DGSLBLAS
LDFLAGS +=	-L/usr/local/lib -lm -lgslcblas
endif

ifeq ($(BLAS),atlas)
CFLAGS +=	-DATLAS
LDFLAGS +=	-L/usr/local/lib -I/usr/local/atlas/include -lm -lcblas -latlas
endif

ifeq ($(BLAS),ptatlas)
CFLAGS +=	-DATLAS
LDFLAGS +=	-L/usr/local/lib -L/usr/local/lib/atlas -I/usr/local/atlas/include -lm -lcblas -latlas
endif

ifeq ($(BLAS),refblas)
CFLAGS +=	-DREFBLAS
LDFLAGS +=	-lm -L/usr/lib -lblas
endif

ifeq ($(BLAS),scalapack)
# to do
CFLAGS +=	-DSCALAPACK
#LDFLAGS +=	-lm -lcblas
endif



$(PROJECT):	$(PROJECT).cpp Makefile
		$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $<

run:	$(PROJECT)
		./$(PROJECT)

clean:		
		$(RM) $(PROJECT)
	
