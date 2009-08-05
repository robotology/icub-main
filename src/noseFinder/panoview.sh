
rm -rf result
mkdir -p result
for f in `ls *view*.jpg`; do
    base=`echo $f | sed "s/.view.*//"`
    tilt=`cat $base.tilt | perl -pe "s/([\+\-])0+/\\$1/"`
    let tilt=500000+$tilt
    echo tilt is $tilt
    cp $f result/$tilt.jpg
done

gthumb result

