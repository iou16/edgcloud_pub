#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "map.h"


map_t *map_alloc(void)
{
  map_t *map;

  map = (map_t*) malloc(sizeof(map_t));

  map->origin_x = 0;
  map->origin_y = 0;
  
  map->size_x = 0;
  map->size_y = 0;
  map->scale = 0;
  
  map->cells = (map_cell_t*) NULL;
  
  return map;
}


void map_free(map_t *map)
{
  free(map->cells);
  map->cells=0;
  free(map);
  return;
}


void map_updata_cell(map_t *map, double gx, double gy, double data)
{
  int mi = MAP_GXWX(map, gx), mj = MAP_GYWY(map, gy);
  
  if (!MAP_VALID(map, mi, mj))
    return;

  if (map->cells[MAP_INDEX(map,mi,mj)].min == 0.0 && map->cells[MAP_INDEX(map,mi,mj)].max == 0.0) {
    map->cells[MAP_INDEX(map,mi,mj)].min = data;
    map->cells[MAP_INDEX(map,mi,mj)].max = data;
  } else {
    map->cells[MAP_INDEX(map,mi,mj)].min = (map->cells[MAP_INDEX(map,mi,mj)].min > data) ? data: map->cells[MAP_INDEX(map,mi,mj)].min;
    map->cells[MAP_INDEX(map,mi,mj)].max = (map->cells[MAP_INDEX(map,mi,mj)].max < data) ? data: map->cells[MAP_INDEX(map,mi,mj)].max;
  }
  map->cells[MAP_INDEX(map,mi,mj)].diff = map->cells[MAP_INDEX(map,mi,mj)].max - map->cells[MAP_INDEX(map,mi,mj)].min;
}
