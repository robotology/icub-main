IF (WIN32)
	FIND_PATH( BML_INCLUDE_PATH foo/foo.h
		$ENV{ICUB_ROOT}/src/BoltzmannMachine/BML
		DOC "The directory where foo/foo.h resides")
	FIND_LIBRARY( BML_LIBRARY
		NAMES bml
		PATHS
		$ENV{ICUB_ROOT}/src/BoltzmannMachine/BML
		DOC "The Foo library")
ELSE (WIN32)
	FIND_PATH( BML_INCLUDE_PATH foo/foo.h
		DOC "The directory where foo/foo.h resides")
	FIND_LIBRARY( FOO_LIBRARY
		NAMES foo
		PATHS
		DOC "The Foo library")
ENDIF (WIN32)

IF (BML_INCLUDE_PATH)
	SET( FOO_FOUND 1 CACHE STRING "Set to 1 if Foo is found, 0 otherwise")
ELSE (BML_INCLUDE_PATH)
	SET( FOO_FOUND 0 CACHE STRING "Set to 1 if Foo is found, 0 otherwise")
ENDIF (BML_INCLUDE_PATH)

MARK_AS_ADVANCED( FOO_FOUND )