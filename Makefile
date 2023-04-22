CC=g++
CFLAGS=-Wall -Wextra -pedantic -std=c++14 -g
LDFLAGS=

SRCS=main.cpp utils.cpp point.cpp kdtree.cpp
OBJS=$(SRCS:.cpp=.o)
DEPS=utils.h point.h kdnode.h kdtree.h
TARGET=a.out

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
