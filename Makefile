CC=gcc
CFLAGS=-Wall
LDLIBS=-lportaudio -lmingw32 -lSDL2main -lSDL2 -lSDL2_net
LIBS=-Llib
INCLUDE=-Iinclude/SDL2 -Iinclude/PA

.PHONY: all clean

all: csynth

csynth: csynth.c fft/kiss_fft.c fft/kiss_fftr.c
	$(CC) $^ -o $@ $(INCLUDE) $(LIBS) $(LDLIBS) $(CFLAGS)

clean:
	rm csynth.exe