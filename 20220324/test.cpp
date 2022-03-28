#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{	
	float a, b;
	a = atof(argv[1]);
	b = atof(argv[2]);
	
	printf("argc = %d\n",argc);
	printf(" %s\n",argv[0]);
	
	printf("%f + %f = %f\n", a, b, a+b);
	printf("%f - %f = %f\n", a, b, a-b);
	printf("%f * %f = %f\n", a, b, a*b);
	printf("%f / %f = %f\n", a, b, a/b);
	
 	return 1;
}
