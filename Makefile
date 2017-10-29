TARGET = test

CC = g++

CFLAGS = -g -Wall

LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs \
	   -lopencv_videoio -I./include

SRCS = $(wildcard ./*.cpp ./src/*.cpp)

OBJS = $(SRCS:.c=.o)

$(TARGET): $(OBJS)
	$(CC) -g -o $@ $^ $(LIBS)
clean:
	rm -rf $(TARGET) *.o 