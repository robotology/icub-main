
for target in `make help | grep -v provided  | sed "s/.* //" | grep -v clean | grep -v cache`; do
	echo "Trying $target"
	make $target
done

