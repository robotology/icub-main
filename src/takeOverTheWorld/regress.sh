
standard_args=""

if [ "k$ICUB_ROOT" = "k" ]; then
    standard_args="$standard_args --ICUB_ROOT $PWD/../.."
fi

cmd="../../bin/takeOverTheWorld $standard_args"

function comment {
    echo "==========================================================="
    echo $*
}

function run {
    full_cmd="$cmd $*"
    echo Running $full_cmd
    $full_cmd
}

comment "use default context"
run --from takeOver.ini

comment "use specific context, give priority to local files in this context"
run --context takeOverWithSeaBass --from takeOver.ini

comment "use specific from file (respect 'tab' rule), give priority to local files #1"
run --from ./conf/example1/takeOver.ini

comment "use specific from file (respect 'tab' rule), give priority to local files #2"
run --from ./conf/example2/takeOver.ini

comment "use specific from file (respect 'tab' rule), give priority to local files #3"
run --from ./conf/example3/takeOver.ini

comment "use default file and default context"
run

comment "use default file and specific context"
run --context takeOverWithSeaBass

