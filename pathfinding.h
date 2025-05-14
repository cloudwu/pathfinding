#ifndef pathfinding_h
#define pathfinding_h

#include <stddef.h>

struct pathfinding_state;

struct pathfinding_neighbor {
	unsigned int pos;
	unsigned int dist;
	unsigned int estimate;
};

// 16 is defined by macro NEIGHBOR_MAX, see pathfinding.c
typedef int (*pathfinding_hfunc)(void *ud, unsigned int pos, struct pathfinding_neighbor result[16]);

struct pathfinding_args {
	unsigned int start;
	unsigned int goal;
	pathfinding_hfunc func;
	void *ud;
};

size_t pathfinding_size(int length);
void pathfinding_init(struct pathfinding_state *, size_t sz);
int pathfinding_find(struct pathfinding_state *, struct pathfinding_args *arg);
int pathfinding_path(struct pathfinding_state *, unsigned int result[], int sz);
int pathfinding_image(struct pathfinding_state *, unsigned char graph[], int x, int y);	// for debug

#endif