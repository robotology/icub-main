# 1. Prerequisite

- Compile as static library `ICUB_SHARED_LIBRARY              OFF`
- Select unittest with flag  ` BUILD_TESTING                 ON`

# 2. Test execution

The test will be executed with the following:

```bash
export YARP_FORWARD_LOG_ENABLE=0
cd build
make test
bin/unittest
```

It can be useful:
```bash
export YARP_VERBOSE_OUTPUT=1
export YARP_TRACE_ENABLE=1
export YARP_DEBUG_ENABLE=1
export YARP_COLORED_OUTPUT=1
```

# 3. Test topics

## 3.1. Multiple FT sensors
- XML parser for multiple ft sensor
- Multiple FT sensors device methods

## 3.2. Can battery

- XML parser for can battery sensor
