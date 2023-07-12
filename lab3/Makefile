CXX := g++
CXXFLAGS := -std=c++17 -O3 -DNDEBUG
LDFLAGS := 
SRCS := $(shell find . -name '*.cpp')
OBJS := $(SRCS:.cpp=.o)
DEPS := $(OBJS:.o=.d)
TARGET := Lab3

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^

%.cpp.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

.PHONY: clean
clean:
	$(RM) $(OBJS) $(DEPS) $(TARGET)

-include $(DEPS)
