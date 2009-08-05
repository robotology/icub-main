/*



#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <stdlib.h>
#include <AR/ar.h>
#include "object.h"

static char *get_buff( char *buf, int n, FILE *fp );

ObjectData_T *read_objectdata( const char *name, int *objectnum )
{
    FILE          *fp;
    ObjectData_T  *object;
    char           buf[256], buf1[256];
    int            i;

    if( (fp=fopen(name, "r")) == NULL ) return(0);

    get_buff(buf, 256, fp);
    if( sscanf(buf, "%d", objectnum) != 1 ) {fclose(fp); return(0);}

    object = (ObjectData_T *)malloc( sizeof(ObjectData_T) * *objectnum );
    if( object == NULL ) return(0);

    for( i = 0; i < *objectnum; i++ ) {
        get_buff(buf, 256, fp);
        if( sscanf(buf, "%s", object[i].name) != 1 ) {
          fclose(fp); free(object); return(0);}

        get_buff(buf, 256, fp);
        if( sscanf(buf, "%s", buf1) != 1 ) {
          fclose(fp); free(object); return(0);}
        
        if( (object[i].id = arLoadPatt(buf1)) < 0 )
            {fclose(fp); free(object); return(0);}

        object[i].visible = 0;

        get_buff(buf, 256, fp);
        if( sscanf(buf, "%lf", &object[i].marker_width) != 1 ) {
            fclose(fp); free(object); return(0);
        }

        printf("No.%d: %20s\n", i+1, &(object[i].name[0]) );
    }

    fclose(fp);

    return( object );
}

static char *get_buff( char *buf, int n, FILE *fp )
{
    char *ret;

    for(;;) {
        ret = fgets( buf, n, fp );
        if( ret == NULL ) return(NULL);
        if( buf[0] != '\n' && buf[0] != '#' ) return(ret);
    }
}

*/
