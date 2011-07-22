
pan: pan.c
	gcc -DSAFETY -O2 -o pan pan.c

pan.c: ach.pml
	spin -a ach.pml

clean:
	rm pan* *.trail

verify: pan
	./pan
