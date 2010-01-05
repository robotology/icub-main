MESSAGE(STATUS "GaussianMixtureModelModule Library")

IF (NESTED_BUILD)
  SET(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule)
ELSE (NESTED_BUILD)
  FIND_LIBRARY(GaussianMixtureModelModule_LIBRARIES GaussianMixtureModelModule ${GaussianMixtureModelModule_DIR}/lib)
ENDIF (NESTED_BUILD)

SET(GaussianMixtureModelModule_INCLUDE_DIR ${GaussianMixtureModelModule_DIR}/include)

