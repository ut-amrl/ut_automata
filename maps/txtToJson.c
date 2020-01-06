#include <stdio.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    printf("ERROR: Must specify map file!\n");
    return 1;
  }
  FILE* fid = fopen(argv[1], "r");
  float x1, y1, x2, y2;
  int multiple = 0;
  printf("[\n");
  while (fscanf(fid, "%f, %f, %f, %f", &x1, &y1, &x2, &y2) == 4) {
    if (multiple) {
      printf(",\n");
    } 
    printf("{\"p0\":{\"x\":%10f, \"y\":%10f}, \"p1\":{\"x\":%10f, \"y\":%10f}}",
           x1, y1, x2, y2);
    multiple++;
  }
  printf("]\n");
  fclose(fid);
  return 0;
}
