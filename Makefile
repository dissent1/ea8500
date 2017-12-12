DIRS = $(patsubst %/Makefile,%,$(shell cd src; echo */Makefile))

all::	$(addprefix .all.,$(DIRS))
clean::	$(addprefix .clean.,$(DIRS))

.all.%:
	make -C src/$*
	touch $@

.clean.%:
	make -C src/$* clean
	rm -f .all.$*
