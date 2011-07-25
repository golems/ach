
pan: pan.c
	gcc -DSAFETY -DCOLLAPSE -O2 -o $@ pan.c

panhc: pan.c
	gcc -DSAFETY -DCOLLAPSE -DHC -O2 -o $@ pan.c

panbs: pan.c
	gcc -DSAFETY -DCOLLAPSE -DBITSTATE -O2 -o $@ pan.c

pan.c: ach.pml
	spin -a ach.pml

clean:
	rm pan* *.trail

verify: pan
	./pan
