CC=g++
CFLAGS=-Wall -Wextra -pedantic -std=c++14 -O2 -g -fopenmp -lpthread
LDFLAGS=-fopenmp -lpthread

SRCS=main.cpp utils.cpp point.cpp kdtree.cpp dbscan.cpp
OBJS=$(SRCS:.cpp=.o)
DEPS=utils.h point.h kdnode.h kdtree.h random_dataset.h dbscan.h
TARGET=dbscan

all: $(TARGET) test

$(TARGET): $(OBJS)
	$(CC) $(LDFLAGS) $^ -o $@

%.o: %.cpp $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

test: test.cpp point.cpp
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(OBJS) $(TARGET) test
