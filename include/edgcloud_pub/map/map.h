#ifndef MAP_H
#define MAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

  
typedef struct
{
  double min, max, sum_x, sum_y;
  int visit;
} map_cell_t;

typedef struct
{
  double origin_x, origin_y;
  
  double scale;

  int size_x, size_y;
  
  map_cell_t *cells;

} map_t;

map_t *map_alloc(void);

void map_free(map_t *map);

void map_updata_cell(map_t *map, double gx, double gy, double data);

#define MAP_WXGX(map, i) (map->origin_x + ((i) - map->size_x / 2) * map->scale)
#define MAP_WYGY(map, j) (map->origin_y + ((j) - map->size_y / 2) * map->scale)

#define MAP_GXWX(map, x) (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)
#define MAP_GYWY(map, y) (floor((y - map->origin_y) / map->scale + 0.5) + map->size_y / 2)

#define MAP_VALID(map, i, j) ((i >= 0) && (i < map->size_x) && (j >= 0) && (j < map->size_y))

#define MAP_INDEX(map, i, j) ((i) + (j) * map->size_x)

#ifdef __cplusplus
}
#endif

#endif
