DIR_TOP = ../..
DIR_INC = $(DIR_TOP)/src
DIR_SRC = $(DIR_TOP)/src
DIR_APP = $(DIR_TOP)/app
DIR_LIB = $(DIR_TOP)/lib
DIR_BIN = $(DIR_TOP)/bin

CC = gcc
AR = ar
LD = ld
CXX = g++

MKDIR = mkdir -p
RM = rm -rf

CFLAGS = -I$(DIR_INC) -g -O2
LDFLAGS = -L$(DIR_LIB) -l360tools -lm
ARFLAGS = r

CSRCS = $(DIR_SRC)/360tools.c \
		$(DIR_SRC)/360tools_img.c \
		$(DIR_SRC)/360tools_erp.c \
		$(DIR_SRC)/360tools_isp.c \
		$(DIR_SRC)/360tools_cmp.c \
		$(DIR_SRC)/360tools_cpp.c \
		$(DIR_SRC)/360tools_ohp.c \
		$(DIR_SRC)/360tools_tsp.c \
		$(DIR_SRC)/360tools_map.c \
		$(DIR_SRC)/360tools_ssp.c \
		$(DIR_SRC)/360tools_tbl.c

CSRCS_APP =	$(DIR_APP)/360tools_conv/360tools_conv.c \
			$(DIR_APP)/360tools_metric/360tools_metric.c 

OBJS = $(CSRCS:.c=.o)

all: RELEASE

# target for release mode
RELEASE: depend $(OBJS)
	$(MKDIR) $(DIR_LIB)
	$(AR) $(ARFLAGS) $(DIR_LIB)/lib360tools.a $(OBJS)
	$(MKDIR) $(DIR_BIN)
	$(CC) -o $(DIR_BIN)/360tools_conv $(CFLAGS) $(DIR_APP)/360tools_conv/360tools_conv.c $(LDFLAGS)
	$(CC) -o $(DIR_BIN)/360tools_metric $(CFLAGS) $(DIR_APP)/360tools_metric/360tools_metric.c $(LDFLAGS)


depend: Makefile $(CSRCS) $(CSRCS_APP) 
	$(CC) -M $(CFLAGS) $(CSRCS) $(CSRCS_APP) > .$@




clean:
	$(RM) $(OBJS) $(DIR_LIB) $(DIR_BIN) .depend

%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

-include .depend

