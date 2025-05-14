# A simple path finding module

This is a simple C implementation of the A star pathfinding algorithm. 
It features the use of a very simple data structure, so you just need to prepare a fixed-sized memory first, and there are no new memory allocation during the run.

On the maps with complex structure and large scales, this code may have some performance issues because it's not carefully optimized, 
But it works well on smaller maps, and it is easy to maintain because it's simple enough.

## Interface

```C
// The pathfinding_state size depend on the size (length * length) of the map
size_t pathfinding_size(int length);

// Init pathfinding_state once
void pathfinding_init(struct pathfinding_state *, size_t sz);

struct pathfinding_args {
	unsigned int start;
	unsigned int goal;
	pathfinding_hfunc func;
	void *ud;
};

// find a path from start to goal, you should offer a heuristic function, see test.c for an example
// Return 0 : can't find a path
// Return > 0 : Found a path with length
// Return < 0 : pathfinding_state is overflow, found a partial path with -length
int pathfinding_find(struct pathfinding_state *, struct pathfinding_args *arg);

// Fill the path into result[]
int pathfinding_path(struct pathfinding_state *, unsigned int result[], int sz);
```

## About the heuristic function
```C
struct pathfinding_neighbor {
	unsigned int pos;
	unsigned int dist;
	unsigned int estimate;
};

int heuristic(void *ud, unsigned int pos, struct pathfinding_neighbor result[]);
```

The coord should be encoded into a unsigned int, the typical way is `(x << 16) | y`.

The heuristic function should returns a pathfinding_neighbor array, which describes the set of positions connected to the location.
The maximum number of the result[] is 32, usually 8 is enough.

* `.pos` : The position of the connection.
* `.dist` : The distance to the location.
* `.estimate` : The estimate distance to the goal, it should not be larger than actually distance.

## Test

`test.c` is an example. The map is described by a string. `S` is the start point, `G` is the goal, and `#` is the walls.

```C
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
```

If you run the `test.c`, you can see the path (with 8-directions) , the diagonal direction has a weight of 7, 
and the horizontal/ vertical direction has a weight of 5:

```
###############################################################
#                                                             #
#                                                             #
#                                   .                         #
#                                    .                        #
#                                    .                        #
#                                    .                        #
#                 ###################.                        #
#                 #                 #.                        #
#                 #        #        #.                        #
#                 #        #        #.                        #
#                 #        #        #.                        #
#                 #        #        #.                        #
#                 #        #        #.                        #
#                 ##########    #####.                        #
#                       .............                         #
#                      .                                      #
#                                                             #
#                                                             #
#                                                             #
#                                                             #
#                                                             #
###############################################################
```

And the internal state (a flow map) for debug :
```
###############################################################
#                                                             #
#                                                             #
#                                   G@@                       #
#                                   @@@                       #
#                                   @@@                       #
#                                   @O@                       #
#                 ###################OO                       #
#                 #      ++=====++++#OO                       #
#                 #      ++#======++#OO                       #
#                 #      ++#-=====++#**                       #
#                 #        #---====+#**                       #
#                 #        #----===+#**                       #
#                 #        #----===+#++                       #
#                 ##########:---#####++                       #
#                  ::.....::::---===+++                       #
#                  :...S...:::---====+                        #
#                  ::.....::::---===++                        #
#                  ::.....:::----==                           #
#                  ::::.:::::---=                             #
#                   ::::::::---                               #
#                                                             #
###############################################################
```
