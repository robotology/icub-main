
# where are ppm sequences 
WORK=/scratch/paul/grab/auto3
mkdir -p $WORK
if [ ! "k$1" = "k" ]; then
    mv /tmp/shake_*.ppm $WORK
fi

cp Makefile.pano $WORK/Makefile
cp panoplot.m $WORK
cp panoview.sh $WORK
cd $WORK

for f in `ls -1 | grep shake_ | grep -v view | grep ppm | sed "s/_[+\-].*//" | uniq`; do
	tilt=`ls -1 $f*.ppm | head -n1| sed "s/^[^_]*_[^_]*_//" | sed "s/_.*//"`
	echo There is a sequence $f with tilt $tilt
	if [ ! -e $f.seq ]; then
	    echo $tilt > $f.tilt
	    echo -n > $f.seq
	    for g in `ls $f*.ppm`; do
		base=`basename $g .ppm`
		echo $base.key >> $f.seq
	    done
	fi
done

make

