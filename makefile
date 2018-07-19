CC ?= gcc
CFLAGS ?= -Wall -O3

OUT = libicm20602.a

INC_DIR = ./inc
OBJ_DIR = ./obj
SRC_DIR = ./src

SRC := $(wildcard $(SRC_DIR)/*.c)
OBJ := $(patsubst $(SRC_DIR)/%.c,$(OBJ_DIR)/%.o,$(SRC))

$(OUT): $(OBJ)
	echo $(OBJ)
	echo $(SRC)
	ar -r -o $@ $^

$(OBJ_DIR)/%.o: $(SRC_DIR)/%.c
	$(CC) -c -I $(INC_DIR) $(CFLAGS) -o $@ $<

clean:
	rm -fv $(OUT)
	rm -fv $(OBJ_DIR)/*.o
