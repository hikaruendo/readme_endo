#
# Makefile
#
# slam用Makefile
# with opencv 2.1.0
#
# Ubuntu用
# マクロ定義
ifeq ($(CC),icc)
CC	= icc  #for gcc
CXX	= icpc #for g++ after source /opt/intel/bin/compilervars.sh ia32
LM	=
else
CC	= gcc
CXX	= g++
LM	= -lm
endif
#CC	= gcc-2.95.3
#CFLAGS	= -O2 -Wall
ifdef DEBUG
DEBUG   = -g
else
DEBUG   =
endif
#CC	= gcc 
#CC	= g++ 
#CC	= icc	
OPENCV_INC	= -I/usr/local/include/opencv	
OPENCV_LIB	= -L/usr/local/lib -lcxcore -lcv -lhighgui -lcvaux
#OPENCV_INC	= -I/usr/include/opencv	
#OPENCV_LIB	= -L/usr/include/opencv -lcxcore -lcv -lhighgui -lcvaux
#IPL_LIB		= -L/usr/local/lib -L/usr/lib -liplm6 -lipla6 -liplpx 
#GPG_LIB		= -lgpg5520 
#IEEE_LIB	= -lcam1394 -lraw1394 
#TARGET	= vision
#OPT_FLAG	= -O3 -mcpu=i686
#OPT_FLAG	= -O3 -mcpu=athlon
#OPT_FLAG	= -O3 -mtune=i686
OPT_FLAG	= -mtune=i686
#OPT_FLAG	= -O3 -tpp7 -xK -unroll16 -c99 -prec_div -fno-alias -fno-fnalias -ansi_alias -ip -nobss_init -Zp16 -vec_report5
CPPFLAGS	= -g  $(OPT_FLAG) $(OPENCV_INC)  $(IPL_INC)
LDLIBS		= $(OPENCV_LIB) $(IPL_LIB) $(GPG_LIB) -lm
#TARGET	=slam3 slam4 slam8 slam8+
TARGET	=slam8 slam8+
#TARGET	=slam2 slam3

all:	$(TARGET)

slam2: slam2.c _slam.c _slam.h easyfunc.c init.c pfprocess.c draw.c fileRorW.c calcweight.c LRFfunc.c
#slam2: slam2.c _slam.c structure.h easyfunc.h init.h pfprocess.h draw.h fileRorW.h calcweight.h LRFfunc.h
	$(CC)  -Wall -g -mtune=i686 $(OPENCV_INC) slam2.c  -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam2
#	$(CC)  -Wall -g  -O3 -mtune=i686 $(OPENCV_INC) slam2.c  -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam2
#	gcc  -g  -O3 -mtune=i686 -I/usr/include/opencv -lm  slam2.c  -L/usr/include/opencv -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam2

slam3: slam3.c _slam.c _slam.h easyfunc.c init.c pfprocess.c draw.c fileRorW.c calcweight.c LRFfunc.c
	$(CC)  -Wall -g -mtune=i686 $(OPENCV_INC) slam3.c  -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam3

slam4: slam4.c _slam.c _slam.h easyfunc.c init.c pfprocess.c draw.c fileRorW.c calcweight.c LRFfunc.c
	$(CC)  -Wall -g -mtune=i686 $(OPENCV_INC) slam4.c  -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam4

opencvtest:opencvtest.c
	$(CC)  -Wall -g -mtune=i686 $(OPENCV_INC) opencvtest.c -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o opencvtest

slam8+: slam8+.c _slam.c _slam.h easyfunc.c init.c pfprocess.c draw.c fileRorW5.c calcweight.c LRFfunc.c calcalfa.c
	$(CC)  -Wall -g -mtune=i686 $(OPENCV_INC) slam8+.c  -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam8+

slam8: slam8.c _slam.c _slam.h easyfunc.c init.c pfprocess.c draw.c fileRorW5.c calcweight.c LRFfunc.c  calcalfa.c
	$(CC)  -Wall -g -mtune=i686 $(OPENCV_INC) slam8+.c  -lm  $(OPENCV_LIB) -lcxcore -lcv -lhighgui -lcvaux   -lm -o slam8

clean: 
	rm -f *.o $(TARGET) *.jpg *~

data-clean:
	rm -f *.jpg *.dat

