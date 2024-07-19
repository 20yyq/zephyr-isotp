.PHONY : all distclean hello_world

all: distclean
#	west build -b mimxrt1060_evk hello_world
	west build hello_world

hello_world:
	west build hello_world

distclean:
	rm -rf build
