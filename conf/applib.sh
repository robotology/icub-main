########################################################################
########################################################################
########################################################################
####
#### Helper functions
####


# Helper function for starting applications.
app_run() {
    arg_on=${1}
    arg_as=${2}
    arg_cmd=${3}
    (
	cat<<XXX
if yarp exists $arg_on; then
    yarp run --on $arg_on --as $arg_as --cmd "$arg_cmd"
else 
  echo Server $arg_on is not running.  We need it for $arg_as.  Start it with:
  echo "    yarp run --server $arg_on"
  exit 1;
fi
XXX
    )  > helper_start_$arg_as.sh
    chmod u+x helper_start_$arg_as.sh
#    bash ./helper_start_$arg_as.sh
    echo "yarp run --on $arg_on --kill $arg_as" > helper_stop_$arg_as.sh
    chmod u+x helper_stop_$arg_as.sh
    echo "bash ./helper_start_${arg_as}.sh" >> helper_start_all.sh
    echo "bash ./helper_stop_${arg_as}.sh" >> helper_stop_all.sh
    echo $arg_as >> helper_module_list.txt
}

app_run_begin() {
    echo -n > helper_start_all.sh
    chmod u+x helper_start_all.sh
    echo -n > helper_stop_all.sh
    chmod u+x helper_stop_all.sh
    echo -n > helper_module_list.txt
}

app_run_end() {
    echo -n
}

app_connect() {
    arg_src=${1}
    arg_dest=${2}
    arg_proto=${3}
    echo "yarp connect $arg_src $arg_dest $arg_proto" >> helper_connect_all.sh
    echo "yarp disconnect $arg_src $arg_dest" >> helper_disconnect_all.sh
}

app_connect_begin() {
    echo -n > helper_connect_all.sh
    chmod u+x helper_connect_all.sh
    echo -n > helper_disconnect_all.sh
    chmod u+x helper_disconnect_all.sh
}

app_connect_end() {
    echo -n
#    bash ./helper_connect_all.sh
}


app_command() {

    CMD=$1
    shift
    MOD="$*"
    if [ "k$MOD" = "k" ]; then
	MOD=all
    fi

    case "$CMD" in
	'restart')
	    for mod1 in $MOD; do
		if [ -e helper_stop_$mod1.sh ]; then
		    bash ./helper_stop_$mod1.sh
		else
		    echo "Do not know how to stop $mod1"
		fi
	    done
	    for mod1 in $MOD; do
		if [ -e helper_start_$mod1.sh ]; then
		    bash ./helper_start_$mod1.sh
		else
		    echo "Do not know how to start $mod1"
		fi
	    done
	    ;;
	'start')
	    for mod1 in $MOD; do
		if [ -e helper_start_$mod1.sh ]; then
		    bash ./helper_start_$mod1.sh
		else
		    echo "Do not know how to start $mod1"
		fi
	    done
	    ;;
	'stop')
	    for mod1 in $MOD; do
		if [ -e helper_stop_$mod1.sh ]; then
		    bash ./helper_stop_$mod1.sh
		else
		    echo "Do not know how to stop $mod1"
		fi
	    done
	    ;;
	'connect')
	    MOD=all
	    if [ -e helper_connect_$MOD.sh ]; then
		bash ./helper_connect_$MOD.sh
	    else
		echo "Do not know how to connect $MOD"
	    fi
	    ;;
	'disconnect')
	    MOD=all
	    if [ -e helper_disconnect_$MOD.sh ]; then
		bash ./helper_disconnect_$MOD.sh
	    else
		echo "Do not know how to disconnect $MOD"
	    fi
	    ;;
	*)
	    echo "Usage: $0 {start|stop|connect|disconnect} [MODULE]" >&2
	    echo "Examples:" >&2
	    echo "  $0 start" >&2
	    echo "  $0 stop" >&2
	    echo "  $0 connect" >&2
	    echo "  $0 disconnect" >&2
	    if [ -e helper_module_list.txt ]; then
		for f in `cat helper_module_list.txt`; do
		    echo "  $0 start $f" >&2
		done
		for f in `cat helper_module_list.txt`; do
		    echo "  $0 stop $f" >&2
		done
		for f in `cat helper_module_list.txt`; do
		    echo "  $0 restart $f" >&2
		done
	    fi
	    exit 3
	    ;;
    esac
}


