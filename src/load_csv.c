#include "load_csv.h"
#include <stdio.h>
#include <stdlib.h>

#define MAX_ROWS 2000

int load_csv(const char *filename, double Z[][3]) {
  
  FILE *fp = fopen(filename, "r");
  if (!fp) {
    perror("Failed to open launch data\n");
    return -1; 
  } // end if

  char line[256];
  int row = 0;

  while (fgets(line, sizeof(line), fp) && row < MAX_ROWS) {
    sscanf(line, "%lf, %lf, %lf",
	   &Z[row][0],
	   &Z[row][1],
	   &Z[row][2]);
    row++; 
  } // end while

  fclose(fp);
  return row; // number of rows scanned in 

} // end load_csv
