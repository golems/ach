cp:
	cp doc/html/* .
	cp doc/*.html man


sync:
	rsync --progress --recursive         \
	      --exclude 'doc'                \
	      --include 'man'                \
	      --include '*.html'             \
	      --include '*.css'              \
	      --include '*.png'              \
	      --exclude '*'                  \
	      ./ golems@golems.org:code.golems.org/pkg/ach
