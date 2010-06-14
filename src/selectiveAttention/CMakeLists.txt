PROJECT (selectiveAttention)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.

FIND_PACKAGE(YARP REQUIRED)

ADD_SUBDIRECTORY(selectiveAttentionInterface)
ADD_SUBDIRECTORY(selectiveAttentionEngine)
