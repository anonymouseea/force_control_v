TARGET := nrc2.out
CXX := g++
SOURCES := $(wildcard src/*.cpp)
HEADERS := $(wildcard include/*.h)

.PHONY: all clean test

all: $(TARGET)

$(TARGET): $(SOURCES) $(HEADERS)
	$(CXX) -m32 -no-pie -std=c++11 \
		-o $(TARGET) $(SOURCES) \
		-I./include \
		-L./lib \
		-lNexRob -lpthread -lm -ldl -lrt

clean:
	rm -f $(TARGET)

# 离线测试不链接厂家库；可在普通电脑运行，不连接机器人。
TEST_CXX ?= g++
test:
	$(TEST_CXX) -std=c++11 -Wall -Wextra -pedantic -I./include \
		src/Admittance.cpp src/ControlCycle.cpp src/ControlStateMachine.cpp \
		tests/ControlStateMachineTests.cpp -o state_machine_tests
	./state_machine_tests
