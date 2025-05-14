/*
The MIT License (MIT)

Copyright (c) 2025 codingnow.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "pathfinding.h"
#include <string.h>
#include <assert.h>

#define NEIGHBOR_MAX 16

typedef unsigned int uint;

struct node {
	uint version;
	uint coord;
	uint gscore;
	uint fscore;
	uint camefrom;
	int next;
};

struct pathfinding_state {
	uint version;
	int goal_index;
	int path_length;
	int pow2;
	struct node hash[1];
};

struct context {
	uint version;
	int pow2;
	struct node *hash;
	pathfinding_hfunc func;
	void *ud;
};

size_t
pathfinding_size(int length) {
	int area = length * length;
	int size = 1024;
	while (size < area) {
		size *= 2;
	}
	return sizeof(struct pathfinding_state) + (size - 1) * sizeof(struct node);
}

static inline int
highest_bit_idx(uint x) {
	int r = 0;
	if ( x & 0xffff0000 ) {
		x >>= 16;
		r += 16;
	}
	if ( x & 0x0000ff00 ) {
		x >>= 8;
		r += 8;
	}
	if ( x & 0x000000f0 ) {
		x >>= 4;
		r += 4;
	}
	if ( x & 0x0000000c ) {
		x >>= 2;
		r += 2; 
	}
	if ( x & 0x00000002 ) {
		r += 1;
	}
	return r;
}

void
pathfinding_init(struct pathfinding_state *P, size_t sz) {
	int n = (sz - sizeof(*P)) / sizeof(struct node) + 1;
	int pow2 = highest_bit_idx(n);
	assert(pow2 >= 8);
	P->version = 0;
	P->pow2 = pow2;
}

static void
context_init(struct context *ctx, struct pathfinding_state *state, struct pathfinding_args *args) {
	ctx->pow2 = state->pow2;
	ctx->hash = state->hash;
	ctx->func = args->func;
	ctx->ud = args->ud;
	int version = state->version++;
	if (version == 0) {
		int i;
		int cap = 1 << ctx->pow2;
		for (i=0;i<cap;i++) {
			state->hash[i].version = version;
		}
	}
	ctx->version = state->version;
}

static inline int
hash(struct context *ctx, uint v) {
	// knuth multiplicative hash
	int mask = (1 << ctx->pow2) - 1;
	return ((2654435761 * v) >> (32 - ctx->pow2)) & mask;
}

static struct node *
find_node(struct context *ctx, uint pos) {
	int index = hash(ctx, pos);
	struct node * n = &ctx->hash[index];
	for (;;) {
		if (n->coord == pos)
			return n;	// hit
		if (n->version != ctx->version)
			return n;	// empty node
		// hash collide, try next slot
		++index;
		if (index >= (1 << ctx->pow2))
			index = 0;
		n = &ctx->hash[index];
	}
}

static int
insert_node(struct context *ctx, struct node *tmp, int list) {
	int index = tmp - ctx->hash;
	if (list < 0) {
		tmp->next = -1;
		return index;
	}
	struct node *current = &ctx->hash[list];
	if (tmp->fscore <= current->fscore) {
		tmp->next = list;
		return index;
	}
	for (;;) {
		if (current->next < 0) {
			current->next = index;
			tmp->next = -1;
			break;
		}
		struct node * next = &ctx->hash[current->next];
		if (tmp->fscore <= next->fscore) {
			tmp->next = current->next;
			current->next = index;
			break;
		}
		current = next;
	}
	return list;
}

static void
remove_node(struct context *ctx, int from_index, int remove_index, int next_index) {
	struct node *current = &ctx->hash[from_index];
	while (current->next != remove_index) {
		int n = current->next;
		assert(n >= 0);
		current = &ctx->hash[n];
	}
	current->next = next_index;
}

static int
advance_node(struct context *ctx, struct node *tmp, int list) {
	int index = tmp - ctx->hash;
	if (index == list)
		return index;
	int next_node = tmp->next;
	assert(list >= 0);
	struct node *current = &ctx->hash[list];
	if (tmp->fscore <= current->fscore) {
		tmp->next = list;
		remove_node(ctx, list, index, next_node);
		return index;
	}
	for (;;) {
		if (current->next < 0) {
			current->next = index;
			tmp->next = -1;
			return list;
		}
		struct node * next = &ctx->hash[current->next];
		if (next == tmp) {
			return list;
		}
		if (tmp->fscore <= next->fscore) {
			tmp->next = current->next;
			current->next = index;
			remove_node(ctx, tmp->next, index, next_node);
			return list;
		}
		current = next;
	}
}

static int
astar(struct context *ctx, uint start, uint goal) {
	int index = hash(ctx, start);
	struct node * current = &ctx->hash[index];
	int version = ctx->version;
	current->coord = start;
	current->version = version;
	current->gscore = 0;
	current->fscore = 0;
	current->camefrom = index;
	current->next = -1;
	int list = index;
	int size = 1;
	// Use half space to minimalize the hash collide
	int cap = 1 << (ctx->pow2 - 1);
	
	for (;;) {
		// listhead should be lowest fscore
		current = &ctx->hash[list];
		if (current->coord == goal)
			return list;
		struct pathfinding_neighbor neighbor[NEIGHBOR_MAX];
		int n = ctx->func(ctx->ud, current->coord, neighbor);
		if (n == 0) {
			if (current->next < 0) {
				return -1;	// the last open node, can't find goal
			}
		}
		// remove from list (close)
		current->fscore = 0;
		list = current->next;
		int i;
		for (i=0;i<n;i++) {
			struct pathfinding_neighbor *d = &neighbor[i];
			int tentative_gscore = current->gscore + d->dist;
			struct node *tmp = find_node(ctx, d->pos);
			if (tmp->version != version) {
				// new node
				tmp->version = version;
				tmp->coord = d->pos;
				tmp->gscore = tentative_gscore;
				tmp->fscore = tentative_gscore + d->estimate;
				tmp->camefrom = current - ctx->hash;
				list = insert_node(ctx, tmp, list);
				++size;
			} else if (tentative_gscore < tmp->gscore) {
				// remove node and insert again (advance node)
				// tmp must be open if estimate is not smaller than actually (A*)
				int open = tmp->fscore != 0;
				tmp->gscore = tentative_gscore;
				tmp->fscore = tentative_gscore + d->estimate;
				tmp->camefrom = current - ctx->hash;
				if (open)
					list = advance_node(ctx, tmp, list);
				else
					list = insert_node(ctx, tmp, list);
			}
		}
		if (size > cap) {
			// out of memory, return nearest node
			return list;
		}
	}
}

static void
calc_length(struct pathfinding_state *state) {
	int index = state->goal_index;
	assert(index >= 0);
	struct node * n = &state->hash[index];
	int count = 1;
	while (n->gscore != 0) {
		n = &state->hash[n->camefrom];
		++count;
	}
	state->path_length = count;
}

int
pathfinding_find(struct pathfinding_state *state, struct pathfinding_args *args) {
	struct context ctx;
	context_init(&ctx, state, args);
	state->goal_index = astar(&ctx, args->start, args->goal);
	if (state->goal_index < 0) {
		state->path_length = 0;
		return 0;
	}
	struct node * n = &state->hash[state->goal_index];
	calc_length(state);
	if (n->coord != args->goal) {
		return -state->path_length;
	} else {
		return state->path_length;
	}
}

static int
skip_path(struct pathfinding_state *state, int n) {
	int index = state->goal_index;
	struct node *current = &state->hash[index];
	int i;
	for (i=0;i<n;i++) {
		current = &state->hash[current->camefrom];
	}
	return current - state->hash;
}

int
pathfinding_path(struct pathfinding_state *state, unsigned int result[], int sz) {
	int index = state->goal_index;
	if (state->path_length != sz) {
		if (state->path_length > sz) {
			index = skip_path(state, state->path_length - sz);
		} else {
			sz = state->path_length;
		}
	}
	int i;
	for (i=sz-1;i>=0;i--) {
		struct node *current = &state->hash[index];
		result[i] = current->coord;
		index = current->camefrom;
	}
	return state->path_length;
}

int
pathfinding_image(struct pathfinding_state *state, unsigned char graph[], int x, int y) {
	int max_score = 0;
	int cap = 1 << state->pow2;
	int i;
	uint version = state->version;
	for (i=0;i<cap;i++) {
		int gscore = state->hash[i].gscore;
		if (state->hash[i].version == version && gscore > max_score)
			max_score = gscore;
	}
	memset(graph, 0, x * y);
	if (max_score == 0)
		return 0;
	int count = 0;
	for (i=0;i<cap;i++) {
		struct node * n = &state->hash[i];
		if (n->version == version) {
			++count;
			int gscore = n->gscore * 255 / max_score;
			int xx = n->coord >> 16;
			int yy = n->coord & 0xffff;
			if (xx < x || yy < y) {
				int off = yy * x + xx;
				graph[off] = gscore;
			}
		}
	}
	return count;
}
