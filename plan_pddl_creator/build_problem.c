/**
 * @file   build_problem.c
 * @Author Enrico BORELLO (enrico.borello@yahoo.it)
 * @Author Louise BRASSEUR (louise.brasseur@etu.utc.fr)
 * @date   March, 2017
 * @brief  script that builds the map and the problem for the pddl plan
 *
 * This script takes into account the parameters of the problem and creates the map and the problem files with the pddl format.
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

int main(void)
{
	float globalLength, globalWidth, globalDepth;
	float X, Y, Z;
	int width, length, depth;
	float minSize;
	int drones;
	int area;
	int empty;
	//getting the parameters of the problem
	printf("What is the width of your map in meters : ");
	scanf ("%f",&globalWidth);
	printf("What is the length of your map in meters : ");
   	scanf ("%f",&globalLength);
	printf("What is the depth of your map in meters : ");
	scanf ("%f",&globalDepth);
	printf("What is the diameter of the biggest drone in meter : ");
	scanf ("%f",&minSize);
	printf("How many drones do you want on the map : ");
	scanf ("%d",&drones);
	width=floor(globalWidth/minSize);
	length=floor(globalLength/minSize);
	globalDepth = globalDepth-0.5;
	depth=floor(globalDepth/0.3);
	int start[drones];
	int stop[drones];
	//getting the starting and final point for each drone
	for (int i=0; i<drones; i++)
	{
		do {
			printf("Where should drone %i start : ", i);
			do {
				printf("X : ");
				scanf ("%f",&X);
			} while (fabs(X) >= globalWidth/2);
			do {
				printf("Y : ");
				scanf ("%f",&Y);
			} while (fabs(Y) >= globalLength/2);
			do {
				printf("Z : ");
				scanf ("%f",&Z);
			} while (Z >= globalDepth+0.5 || Z < 0.5);
			Z=floor((Z-0.5)/0.3);
			Y=floor((Y+globalLength/2)/(globalLength/length));
			X=floor((X+globalWidth/2)/(globalWidth/width));
			start[i] = Z+(Y+X*length)*depth;
			empty=0;
			for (int j=0; j<i; j++)
			{
				if (start[i]==start[j]) empty++;
			}
		} while (start[i]<0 || start[i]>=length*width*depth || empty!=0);
		do {
			printf("Where should drone %i stop : ", i);
			do {
				printf("X : ");
				scanf ("%f",&X);
			} while (fabs(X) >= globalWidth/2);
			do {
				printf("Y : ");
				scanf ("%f",&Y);
			} while (fabs(Y) >= globalLength/2);
			do {
				printf("Z : ");
				scanf ("%f",&Z);
			} while (Z >= globalDepth+0.5 || Z < 0.5);
			Z=floor((Z-0.5)/0.3);
			Y=floor((Y+globalLength/2)/(globalLength/length));
			X=floor((X+globalWidth/2)/(globalWidth/width));
			stop[i] = Z+(Y+X*length)*depth;
			empty=0;
			for (int j=0; j<i; j++)
			{
				if (stop[i]==stop[j]) empty++;
			}
		} while (stop[i]<0 || stop[i]>=length*width*depth || empty!=0);
		
	}

	//Writting the problem and map_coordinates files
	FILE *myPlanFile;
	FILE *myMapFile;
	myPlanFile = fopen("script_problem_test.pddl", "w");
	myMapFile = fopen("map_coordinates.pddl", "w");
	if (myPlanFile!=NULL && myMapFile!=NULL)
	{
		fprintf(myPlanFile,"(define\n\t(problem script_problem_test )\n\t(:domain SofAR)\n\t(:objects");
		for (int i=0; i<drones; i++)
		{
			fprintf(myPlanFile, "\n\t\tdrone%i - drone", i);
		}
		//defining the areas
		for (int i=0; i<width; i++)
		{
			for (int j=0; j<length; j++)
			{
				for (int k=0; k<depth; k++)
				{
					area = k+(j+i*length)*depth;
					fprintf(myPlanFile, "\n\t\tarea%i - location", area);
					fprintf(myMapFile, "area%i\t%f\t%f\t%f\n", area, i*globalWidth/width-globalWidth/2+globalWidth/width/2, j*globalLength/length-globalLength/2+globalLength/length/2, k*0.3+0.5);
					
				}
			}
		}
		//starting area of each drone
		fprintf(myPlanFile,"\n\t)\n\t(:init");
		for (int i=0; i<drones; i++)
		{
			fprintf(myPlanFile,"\n\t\t(is-in drone%i area%i)",i, start[i]); 
		}
		//initialising all areas except the ones where the drones are
		for (int i=0; i<width; i++)
		{
			for (int j=0; j<length; j++)
			{
				for (int k=0; k<depth; k++)
				{
					area=k+(j+i*length)*depth;
					empty=0;
					for (int l=0; l<drones; l++)
					{
						if (area==start[l]) empty ++;
					}
					if (empty==0) fprintf(myPlanFile,"\n\t\t(is-free area%i)", area);
				}
			}
		}
		//conecting each area to the adjacent ones
		for (int i=0; i<width; i++)
		{
			for (int j=0; j<length; j++)
			{
				for (int k=0; k<depth; k++)
				{
					area = k+(j+i*length)*depth;
					if (k-1>=0) fprintf(myPlanFile,"\n\t\t(connected area%i area%i)", area, k-1+(j+i*length)*depth);
					if (k+1<depth) fprintf(myPlanFile,"\n\t\t(connected area%i area%i)", area, k+1+(j+i*length)*depth);
					if (j-1>=0) fprintf(myPlanFile,"\n\t\t(connected area%i area%i)", area, k+((j-1)+i*width)*depth);
					if (j+1<length) fprintf(myPlanFile,"\n\t\t(connected area%i area%i)", area, k+((j+1)+i*length)*depth);
					if (i-1>=0) fprintf(myPlanFile,"\n\t\t(connected area%i area%i)", area, k+(j+(i-1)*length)*depth);
					if (i+1<length) fprintf(myPlanFile,"\n\t\t(connected area%i area%i)", area, k+(j+(i+1)*length)*depth);
				}
			}
		}
		//final state of the problem
		fprintf(myPlanFile,"\n\t)\n\t(:goal (and ");
		for (int i=0; i<drones; i++)
		{
			fprintf(myPlanFile,"(is-in drone%i area%i) (not(is-in drone%i area%i)) " ,i, stop[i], i, start[i] );
		}
		fprintf(myPlanFile,"))\n) ");

		fclose(myPlanFile);
		fclose(myMapFile);
	}
}
