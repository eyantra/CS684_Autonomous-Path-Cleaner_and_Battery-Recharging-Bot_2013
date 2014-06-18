//#include <stdio.h>

//***************************************************//
//******************* Declarations ******************//
//***************************************************//

// Grid or Matrix dimention
// x co-ordinates are columns
int rows;
// y co-ordinates are rows
int cols;

// To count the dynamic location (row and column)
int rowNum = 0;
int colNum = 0;

// Destination Co-ordinates , i.e. place to be reached for charging
// By default its Origin.
int destRow = 0;
int destCol = 0;

// Array to store shortest path-back-to Origin
int path_to_origin[100];


// Shows the direction in 2D
// like Four Quadrants in 2D-plane
// So 	East 	--> 0
//	North	--> 1
//	West	--> 2
//	South	--> 3
enum bot_direc { east=0 , north=1 , west=2 , south=3 } bot_direction;


// All possible directions at any point for movement
enum grid_direc { FR = 10 , LT = 11 , RT = 12 , ST = 13 } grid_direction;



//***************************************************//
//******************* Getter Methods ****************//
//***************************************************//

// Getter method for rows
int getRow()
{
	return rowNum;
}

// Getter method for cols
int getCol()
{
	return colNum;
}

// Getter method for direction
// So 	East 	--> 0
//	North	--> 1
//	West	--> 2
//	South	--> 3
int getDirection()
{
	return bot_direction;
}


//***************************************************//
//******************* Setter Methods ****************//
//***************************************************//

// Setter method for Grid , Matrix
void set_grid(int row,int col)
{
	rows = row;
	cols = col;
}

// Setter method for Direction
void set_direction(int direction)
{
	bot_direction = direction;
}



//***************************************************//
//******************* Methods ***********************//
//***************************************************//


// Check valadity of location within grid
// 0 for false , i.e. Location is invalid
// 1 for True , i.e. Location is valid
int check_loc(int row,int col)
{
 if (row < (rows-1) && col < (cols-1))
 {
	return 1;
 }
 else
 {
	return 0;
 }
}

// Move bot forward by one unit in the given direction ( set in "bot_direction")
int update_bot_location()
{
	int tempRowNum = rowNum;
	int tempColNum = colNum;

	// Performing "unit motion" based on bot-direction

	if (bot_direction == east)
	{
		tempColNum++;
	}
	else if(bot_direction == north)
	{
		tempRowNum++;
	}
	else if(bot_direction == west)
	{
		tempColNum--;
	}
	else if(bot_direction == south)
	{
		tempRowNum--;
	}

	
	//// printf"\nTempRowNum %d , TempColNum %d\n",tempRowNum,tempColNum);


 /* Following code has been commented as We are not checking for the bounds and keeping the bot in the First Quadrant only
    Now bot will be able to move in any direction changing the any /all co-ordinates to negative values also.
    This will simplify the calculations and we will be able to write simple method for 
	// Checking for the bounds of the Grid/Matrix
	// If fine  return 1;
	// If error return 0;
	
	if (tempRowNum >= rows || tempColNum >= cols)
	{
		return 0;
	}	
	else if(tempRowNum < 0 || tempColNum < 0 )
	{
		return 0;
	}
	else
	{
		rowNum = tempRowNum;
		colNum = tempColNum;
		return 1;
	}
 */
 
 	// code of else block being used directly
 	rowNum = tempRowNum;
	colNum = tempColNum;
	return 1;
}

int move_bot(int direction)
{
	//// printf"\nDirection %d\n",direction);

	// ST = 3 , It refers to "Stop"
	if (direction == ST)
	{
		return 0;
	}
	else if (direction == LT)
	{
		if (bot_direction == east)
		{
			bot_direction = north;
		}
		else if (bot_direction == north)
		{
			bot_direction = west;
		}
		else if (bot_direction == west)
		{
			bot_direction = south;
		}
		else if (bot_direction == south)
		{
			bot_direction = east;
		}	
	}
	else if (direction == RT)
	{
		if (bot_direction == east)
		{
			bot_direction = south;
		}
		else if (bot_direction == south)
		{
			bot_direction = west;
		}
		else if (bot_direction == west)
		{
			bot_direction = north;
		}
		else if (bot_direction == north)
		{
			bot_direction = east;
		}	
	}
	else if(direction == FR)
	{
		int x = update_bot_location();
		//// printf"update_bot_location --> %d",x);
		return x;
	}

	
	return 1;

}

// Tester method for the bot , it calls "int move_bot(int dirction)"
int follow_path(int *path,int count)
{
	int counter = 0;
	
	while(counter < count)
	{
		int status = move_bot(*(path+counter));
		
		if (status == 1)
		{
			// printf"\nDirection is %d\n",*(path+counter));
			// printf"Row - %d , Col - %d\n",getRow(),getCol());
		}
		else
		{
			// printf"Some error occurred");
			return 0;
		}
		
		counter++;
	}
	
	return 1;
}


// Bot will reach the start point and then we can customize the path to reach the actual Charging point from origin which is generally away from the main working grid or matrix.
// Final direction at origin will be west (by default).
// It returns array_len which is Zero if something went wrong , else some non-zero value for length of the Path-Back-To_Origin.
int reach_origin()
{
    // counter to store array-length
    int array_len = 0;
    
	// Step I
	// cover all rows and reduce it to Zero , if applicable

	// It bot is in I or II Quadrant as rowNum>0
	if (rowNum > 0)
	{
		// Turn towards south , if applicable
		if(bot_direction != south)
		{
			if (bot_direction == east)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;			
			}
			else if (bot_direction == north)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;
                
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;
			}
			else if (bot_direction == west)
			{
				move_bot(LT);
				path_to_origin[array_len] = LT ;
		                array_len++;			
			}
		}
		
		// Travel the distance to make rowNum = 0
		while( rowNum > 0 )
		{
        	      move_bot(FR);
        	      path_to_origin[array_len] = FR ;
        	      array_len++;
              
	              //rowNum--;
        	}
        
	}
	// If bot is int III or IV quadrant
	else if (rowNum < 0)
	{
		// Turn towards south , if applicable
		if(bot_direction != north)
		{
			if (bot_direction == east)
			{
				move_bot(LT);
				path_to_origin[array_len] = LT ;
                		array_len++;			
			}
			else if (bot_direction == west)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
		                array_len++;			
			}
			else if (bot_direction == south)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;
                
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;
			}
		}
		
		// Travel the distance to make rowNum = 0
		while( rowNum < 0 )
		{
        	      move_bot(FR);
        	      path_to_origin[array_len] = FR ;
        	      array_len++;
              
	              //rowNum++;
        	}
	}
	
	
	


        // Do the same for Columns

	// Step I
       	// cover all Columns and reduce it to Zero , if applicable
        // If bot is in I or IV Quadrant as colNum>0
        
	if (colNum > 0)
	{
		// Turn towards west , if applicable
		if(bot_direction != west)
		{
			if (bot_direction == east)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;
                		
                		move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;			
			}
			else if (bot_direction == north)
			{
				move_bot(LT);
				path_to_origin[array_len] = LT ;
                		array_len++;
                	}
			else if (bot_direction == south)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
		                array_len++;			
			}
		}

		
		// Travel the distance to make colNum = 0
		while( colNum > 0 )
		{
	              move_bot(FR);
	              path_to_origin[array_len] = FR ;
	              array_len++;
	              
	              //colNum--;
	        }


	}
	// If bot is int II or III quadrant
	else if (colNum < 0)
	{
		// printf"\nColNum initially - %d\n",colNum);
	
		// Turn towards east , if applicable
		if(bot_direction != east)
		{
			if (bot_direction == north)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
                		array_len++;			
			}
			else if (bot_direction == west)
			{
				move_bot(RT);
				path_to_origin[array_len] = RT ;
		                array_len++;			
		                
		                move_bot(RT);
				path_to_origin[array_len] = RT ;
		                array_len++;
			}
			else if (bot_direction == south)
			{
				move_bot(LT);
				path_to_origin[array_len] = LT ;
                		array_len++;
			}
		}
	
	
		// Travel the distance to make colNum = 0
		while( colNum < 0 )
		{
	              move_bot(FR);
	              path_to_origin[array_len] = FR ;
	              array_len++;
	              // printf"\n len - %d , value - %d , colNum - %d",array_len,path_to_origin[array_len],colNum);
	              
	              //colNum++;
	        }	
		
	}
	
	return array_len;	


}





















