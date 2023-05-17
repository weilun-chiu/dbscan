CC=g++
CFLAGS=-Wall -Wextra -pedantic -std=c++14 -O2 -g -fopenmp -lpthread -mavx2
LDFLAGS=-fopenmp -lpthread -mavx2

SRCDIR=src
BUILDDIR=build
SRCS=$(wildcard $(SRCDIR)/*.cpp)
OBJS=$(patsubst $(SRCDIR)/%.cpp, $(BUILDDIR)/%.o, $(SRCS))
DEPS=$(wildcard $(SRCDIR)/*.h)
TARGET=dbscan

all: $(TARGET) test

$(TARGET): $(OBJS) $(BUILDDIR)/main.o
	$(CC) $(LDFLAGS) $^ -o $@

$(BUILDDIR)/%.o: $(SRCDIR)/%.cpp $(DEPS)
	mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILDDIR)/main.o: main.cpp $(SRCDIR)/utils.cpp $(SRCDIR)/point.cpp $(SRCDIR)/dbscan.cpp
	mkdir -p $(BUILDDIR)
	$(CC) $(CFLAGS) -c $< -o $@

test: test.cpp $(SRCDIR)/point.cpp
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf $(BUILDDIR) $(TARGET) test
