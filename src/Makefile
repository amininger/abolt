BINPATH=../bin
NAME=kinect

# XXX Requires that the user install libfreenect. See readme
INCLUDES = -I/usr/local/include/libfreenect
LIBS =  -L../lib -L/usr/local/lib64 -lfreenect -lpthread -llcm -lm
FLAGS = -g -O2 -fPIC

# LCM Stuff
LCMTYPES_NAMES := $(shell cd ../lcmtypes && ls *.lcm)
LCMTYPES_C := $(LCMTYPES_NAMES:%.lcm=%.c)
LCMTYPES_C := $(addprefix ./lcmtypes/,$(LCMTYPES_C))

all: $(BINPATH)/$(NAME)

$(BINPATH)/$(NAME): kinect.c $(LCMTYPES_C)
	mkdir -p $(BINPATH)
	g++ kinect.c $(LCMTYPES_C) $(INCLUDES) $(LIBS) $(FLAGS) -o $(BINPATH)/$(NAME)

./lcmtypes/%.c: ../lcmtypes/%.lcm
	mkdir -p lcmtypes
	lcm-gen -c --c-cpath lcmtypes --c-hpath lcmtypes $<

debug:
	echo $(LCMTYPES_NAMES)
	echo $(LCMTYPES_C)

clean:
	rm -rf *.o *.a *~ lcmtypes/*.c lcmtypes/*.h $(BINPATH)/$(NAME)
