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

  int map_index = MAP_INDEX(map,mi,mj);
  if (!map->cells[map_index].visit) {
    map->cells[map_index].min = data;
    map->cells[map_index].max = data;
  } else {
    assert(map->cells[map_index].visit);
    map->cells[map_index].min = (map->cells[map_index].min > data) ? data: map->cells[map_index].min;
    map->cells[map_index].max = (map->cells[map_index].max < data) ? data: map->cells[map_index].max;
  }

  map->cells[map_index].sum_x += gx;
  map->cells[map_index].sum_y += gy;
  map->cells[map_index].visit++;
}
