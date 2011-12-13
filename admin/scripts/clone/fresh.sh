
# make sure all the repositories are freshly checked out in the directory
# called "state"

#rm -rf state
#mkdir -p state
BASE=$PWD
for f in `cat source.txt target.txt`; do
    echo $f
    mkdir -p state/$f
    cp -R sources/$f state/$f/CVS
    cd state/$f
    cvs update -d
    cd $BASE
done

