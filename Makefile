CC=g++
CFLAGS=-Wall -Wextra -pedantic -std=c++14 -O2 -g -fopenmp -lpthread -mavx2
LDFLAGS=-fopenmp -lpthread -mavx2

SRCDIR=src
SRCS=$(wildcard $(SRCDIR)/*.cpp)
OBJS=$(patsubst $(SRCDIR)/%.cpp, %.o, $(SRCS))
DEPS=$(wildcard $(SRCDIR)/*.h)
TARGET=dbscan

all: $(TARGET) test

$(TARGET): $(OBJS) main.o
	$(CC) $(LDFLAGS) $^ -o $@

%.o: $(SRCDIR)/%.cpp $(DEPS)
	$(CC) $(CFLAGS) -c $< -o $@

main.o: main.cpp $(SRCDIR)/utils.cpp $(SRCDIR)/point.cpp $(SRCDIR)/kdtree.cpp $(SRCDIR)/dbscan.cpp
	$(CC) $(CFLAGS) -c $< -o $@

test: test.cpp $(SRCDIR)/point.cpp
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(OBJS) main.o $(TARGET) test
