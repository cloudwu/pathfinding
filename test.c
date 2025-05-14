#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pathfinding.h"

const char * graph =
"###############################################################\n"
"#                                                             #\n"
"#                                                             #\n"
"#                                   G                         #\n"
"#                                                             #\n"
"#                                                             #\n"
"#                                                             #\n"
"#                 ###################                         #\n"
"#                 #                 #                         #\n"
"#                 #        #        #                         #\n"
"#                 #        #        #                         #\n"
"#                 #        #        #                         #\n"
"#                 #        #        #                         #\n"
"#                 #        #        #                         #\n"
"#                 ##########    #####                         #\n"
"#                                                             #\n"
"#                      S                                      #\n"
"#                                                             #\n"
"#                                                             #\n"
"#                                                             #\n"
"#                                                             #\n"
"#                                                             #\n"
"###############################################################\n"
;

struct map {
	int x;
	int y;
	int start_x;
	int start_y;
	int goal_x;
	int goal_y;
	const char * grid;
	char * output;
};

struct neighbor {
	int dx;
	int dy;
	int dist;
};

/*
	7 ~= 5 * sqrt(2)

	dist:
		7 5 7
		5 * 5 
		7 5 7
		
	estimate: S->G
		     G
		     | diff * 5
		+....+
		|   /|
		| /  |
		S----+
		xx * 7 (diagonal line)
 */

#define ABS(v) ((v) < 0 ? -(v) : (v))

static int
calc_estimate(struct map *m, int x, int y) {
	int dx = ABS(m->goal_x - x);
	int dy = ABS(m->goal_y - y);
	int diff;
	if (dx > dy) {
		diff = dx - dy;
		dx = dy;
	} else {
		diff = dy - dx;
	}
	return diff * 5 + dx * 7;
}

static int
heuristic(void *ud, unsigned int pos, struct pathfinding_neighbor result[]) {
	int x = pos >> 16;
	int y = pos & 0xffff;
	const struct neighbor n[8] = {
		{ -1, -1, 7 },
		{ +1, -1, 7 },
		{ -1, +1, 7 },
		{ +1, +1, 7 },
		{ -1,  0, 5 },
		{ +1,  0, 5 },
		{  0, -1, 5 },
		{  0, +1, 5 },
	};
	int i;
	int c = 0;
	struct map *m = (struct map *)ud;
	char slot = m->grid[m->x * y + x];
	if (slot == '#')
		return 0;
	for (i=0;i<8;i++) {
		int dx = x + n[i].dx;
		int dy = y + n[i].dy;
		if (dx >= 0 && dx < m->x && dy >= 0 && dy < m->y) {
			unsigned int npos = pos + (n[i].dx << 16) + n[i].dy;
			result[c].pos = npos;
			result[c].dist = n[i].dist;
			result[c].estimate = calc_estimate(m, dx, dy);
			
			++c;
		}
	}
	return c;
}

static void
map_init(struct map *m, const char *g) {
	const char *lb = strchr(g, '\n');
	m->x = lb - g + 1;
	size_t area = strlen(g);
	m->y = area / m->x;
	const char *start = strchr(g, 'S');
	const char *goal = strchr(g, 'G');
	int off = start - g;
	m->start_y = off / m->x;
	m->start_x = off - m->x * m->start_y;
	off = goal - g;
	m->goal_y = off / m->x;
	m->goal_x = off - m->x * m->goal_y;
	m->grid = g;
	size_t sz = m->x * m->y;
	m->output = malloc(sz);
	memset(m->output, 0, sz);
}

static void
map_deinit(struct map *m) {
	free(m->output);
}

#define MAX_PATH_LENGTH 1024

static void
gen_path(struct map *m, struct pathfinding_state *P) {
	unsigned int path[MAX_PATH_LENGTH];
	int n = pathfinding_path(P, path, MAX_PATH_LENGTH);
	if (n > MAX_PATH_LENGTH)
		n = MAX_PATH_LENGTH;
	int i;
	for (i=0;i<n;i++) {
		int x = path[i] >> 16;
		int y = path[i] & 0xffff;
		m->output[y * m->x + x] = i+1;
	}
}

static void
output_path(struct map *m) {
	int i,j;
	int offset = 0;
	for (i=0;i<m->y;i++) {
		for (j=0;j<m->x;j++) {
			if (m->output[offset] != 0) {
				printf(".");
			} else if (m->grid[offset] == '#') {
				printf("#");
			} else {
				printf(" ");
			}
			++offset;
		}
		printf("\n");
	}
}

static void
output_image(struct map *m, struct pathfinding_state *P) {
	pathfinding_image(P, (unsigned char *)m->output, m->x, m->y);
	const char gray[8] = ".:-=+*O@";
	int offset = 0;
	int i,j;
	for (i=0;i<m->y;i++) {
		for (j=0;j<m->x;j++) {
			char c = m->grid[offset];
			if (c == '#' || c == 'S' || c == 'G') {
				printf("%c", c);
			} else if (m->output[offset] == 0) {
				printf(" ");
			} else {
				unsigned char level = m->output[offset];
				printf("%c", gray[level/32]);
			}
			++offset;
		}
		printf("\n");
	}
}

int
main() {
	struct map m;
	map_init(&m, graph);
	printf("map (%d * %d)\n", m.x, m.y);
	printf("Start (%d , %d) -> Goal (%d , %d)\n", m.start_x, m.start_y, m.goal_x, m.goal_y);
	size_t sz = pathfinding_size(m.x);
	printf("State size = %d\n", (int)sz);
	struct pathfinding_state *S = (struct pathfinding_state *)malloc(sz);
	pathfinding_init(S, sz);
	struct pathfinding_args args = (struct pathfinding_args) {
		.start = m.start_x << 16 | m.start_y,
		.goal = m.goal_x << 16 | m.goal_y,
		.func = heuristic,
		.ud = &m,
	};
	printf("start %x -> goal %x\n", args.start, args.goal);
	int len = pathfinding_find(S, &args);
	printf("path len = %d\n", len);
	gen_path(&m, S);
	output_path(&m);
	output_image(&m, S);
	free(S);
	map_deinit(&m);
	return 0;
}
