#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "threads/init.h"
#include "threads/malloc.h"
#include "threads/palloc.h"
#include "threads/thread.h"
#include "projects/pa/pa.h"

void run_patest(char **argv)
{   
    /// TODO: make your own test
    palloc_get_status(0);
    char test[5];
    test[0] = malloc(128000); //128K
    palloc_get_status(0);
    test[1] = malloc(256000); //256K
    palloc_get_status(0);
    test[2] = malloc(64000); //64K
    palloc_get_status(0);
    test[3] = malloc(256000); //256K
    palloc_get_status(0);
    free(test[1]); //release 256K
    palloc_get_status(0);
    free(test[0]); //release 128K
    palloc_get_status(0);
    test[4] = malloc(75000); //75K
    palloc_get_status(0);
    free(test[2]); //release 64K
    palloc_get_status(0);
    free(test[4]); //release 75K
    palloc_get_status(0);
    free(test[3]); //release 256K
    palloc_get_status(0);

    while (1) {
        timer_msleep(1000);
    }
}