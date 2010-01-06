MESSAGE(STATUS "GaussianMixtureModelModule Library")

#FIND_LIBRARY(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule ${GaussianMixtureModelModule_DIR}/lib)

#IF (NOT GaussianMixtureModelModule_LIBRARIES)
  SET(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule ${GaussianMixtureModelModule_DIR}/lib)
#ENDIF (NOT GaussianMixtureModelModule_LIBRARIES)

#IF (NESTED_BUILD)
#  SET(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule)
#ELSE (NESTED_BUILD)
#  FIND_LIBRARY(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule ${GaussianMixtureModelModule_DIR}/lib)
#ENDIF (NESTED_BUILD)

SET(GaussianMixtureModelModule_INCLUDE_DIR ${GaussianMixtureModelModule_DIR}/include)

