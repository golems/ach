cp:
	cp doc/html/* api
	cp doc/man/*.html man
	cp doc/manual/*.html manual
	cp -r doc/images .


sync:
	rsync --progress --recursive         \
	      --exclude 'doc'                \
	      --include 'man'                \
	      --include '*.html'             \
	      --include '*.css'              \
	      --include '*.png'              \
	      --exclude '*'                  \
	      ./ golems@golems.org:code.golems.org/pkg/ach
