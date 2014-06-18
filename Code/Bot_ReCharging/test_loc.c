#include <stdio.h>
//#include <conio.h>
#include "location.h"
#define ROW_SIZE 4
#define COL_SIZE 4


void main()
{
 int i = 0;
 
 printf("\n0 --> %d\n",east);
 printf("\n1 --> %d\n",north);
 printf("\n2 --> %d\n",west);
 printf("\n3 --> %d\n",south);

 set_grid(ROW_SIZE,COL_SIZE);

 set_direction(north);
 printf("\nInitial Direction is North %d",north);
 printf("\nInitial Position is \nRow - %d , Col - %d\n\n",getRow(),getCol());
 
 
 int path[] = {LT,LT,FR,FR,LT,FR,RT,FR};
 int path_length = 8;
 
 int output = follow_path(path,path_length);
 
 printf("\nFinal status :- %d",output);
 printf("\nCurrent Direction %d",bot_direction);

 int len = reach_origin();

 printf("\nPrinting Path back to Origin %d :- \n",len); 
 
 for(i=0 ; i<len ;i++)
 {
 	printf(" %d ",path_to_origin[i]); 
 }

 printf("Current Direction %d\n",bot_direction);





// printf("Press any key to exit");
// getch();
}
