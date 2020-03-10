This is a stub for python/java/... bindings to ICUB libraries.
To build:
 * Build repository as usual.   On a 64-bit machine, consider 
   rebuilding the ICUB repository with `ICUB_SHARED_LIBRARY` set 
   to TRUE/ON.
 * Set ICUB_DIR environment variable to your build directory.
 * Create a new build directory for e.g. python binding, and enter it.
 * Run cmake with the directory this README.txt is in as its source
   directory.
 * Set one of `CREATE_PYTHON`, `CREATE_JAVA`, ... options to TRUE/ON.
 * Set *only* *one* option on.  For multiple languages, use multiple
   build directories.
 * Go ahead and compile.  If you have trouble, consider rebuilding
   the ICUB repository with `ICUB_SHARED_LIBRARY` set to TRUE/ON.

Note, for a lot of purposes you'll also need the YARP bindings. 
See $YARP_ROOT/example/swig

---

Python example:
```python
  import icub
  import yarp
  yarp.Network.init()
  icub.init()
  d = yarp.Drivers.factory()
  print d.toString().c_str()
  # list should include cartesiancontrollerclient etc, available
  # from PolyDriver
```

To run for testing, be in the directory you built the icub bindings,
place the above test in test.py, and do (in a bash shell):
```sh
$ export YARP_PYTHON=/path/to/yarp/python/bindings/
$ PYTHONPATH=$YARP_PYTHON python test.py
```

---

Java example:
```java
  import yarp.icub;
  import yarp.Network;
  import yarp.Drivers;

  class Test {
    public static void main(String[] args) {
	System.loadLibrary("jyarp");
	System.loadLibrary("icub");
	Network.init();
	icub.init();
	System.out.println(Drivers.factory().toString_c());
    }
  };
```

To run for testing, be in the directory you built the icub bindings,
place the above test in Test.java, and do (in a bash shell):
```sh
$ export YARP_JAVA=/path/to/yarp/python/bindings/
$ javac -cp $YARP_JAVA:$PWD Test.java
$ LD_LIBRARY_PATH=$YARP_JAVA:$PWD java -cp $YARP_JAVA:$PWD Test 
```
