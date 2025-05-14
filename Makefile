pathfinding.exe : test.c pathfinding.c
	gcc -g -Wall -o $@ $^
	
clean :
	rm pathfinding.exe
	
test : pathfinding.exe
	./$<
