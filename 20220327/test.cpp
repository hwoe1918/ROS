#include <stdio.h>
#include <math.h>
#include <iostream>
float datax[] = {2, 4, 6, 8, 10};
float datay[] = {81, 93, 91, 97, 105};

int i = 0;

float avg_array(float array[])
{
	float sum = 0;
	for(i = 0; i < 5; i++)
	{
		sum += array[i];
	}
	printf("%f\n", sum/5);
	return (sum/5);
};

float gradient_x(float array_x[], float array_y[], float avg_x, float avg_y)
{
	float sum_x = 0;
	float sum_xy = 0;
	
	for(i = 0; i < 5; i++)
	{
		sum_x += pow((array_x[i] - avg_x),2);
		sum_xy += (array_x[i] - avg_x) * (array_y[i] - avg_y);
		printf("%f, %f\t",array_x[i],array_y[i]);
		printf("%f %f\n", sum_x, sum_xy);
	}
	
	return sum_xy/sum_x;
}

float intercept(float avg_x, float avg_y, float gradient)
{
	return (avg_y-(avg_x * gradient));
}

int main() {
	float avg_x = avg_array(datax);
	float avg_y = avg_array(datay);
	float gradient = gradient_x(datax, datay, avg_x, avg_y);
	float y_intercept = intercept(avg_x, avg_y, gradient);
	
	printf("%f %f\n", avg_x, avg_y);
	printf("y = %fx+ %f\n", gradient, y_intercept);
	return 0;
}


