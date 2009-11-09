
#cmake --graphviz=foo.dot .

cp foo.dot junk.dot
for f in `cat foo.dot | egrep "([-/][^>])|(mymod)" | sed "s/ . label.*//" | sed "s/[^n]*//" | sed "s/[^a-z0-9]//g"`; do
    echo remove $f
    egrep -v "[^0-9a-z]$f[^0-9a-z]" < junk.dot > junk2.dot
    cp junk2.dot junk.dot
done




