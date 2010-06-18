test2-update-repository.sh
Update repository using svn

test2-compile-doc.sh
Run doxygen to build documentation

test2-update-doc
Upload documentation to web

test2-compile.sh
Compile code.



compile-config:

common.sh --> common parameters
helpers.sh --> common functions

compile-config/doxygen
--> doxygen compilation config file (WEB_DOC_SUFFIX)

compile-config/{os_directory}
--> config.sh os specific parameters

