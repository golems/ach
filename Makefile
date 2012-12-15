cp:
	cp doc/html/* api
	cp doc/man/*.html man
	cp doc/manual/ach-manual.html manual/index.html


sync:
	rsync --progress --recursive         \
	      --exclude 'doc'                \
	      --include 'man'                \
	      --include 'manual'             \
	      --include 'api'                \
	      --include '*.html'             \
	      --include '*.css'              \
	      --include '*.png'              \
	      --include '*.gif'              \
	      --exclude '*'                  \
	      ./ golems@golems.org:code.golems.org/pkg/ach
