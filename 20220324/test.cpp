#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char* argv[])
{	
	printf("argc = %d\n",argc);
	printf(" %s\n",argv[0]);
	
	printf("%d\n",atoi(argv[1])+atoi(argv[2]));
	printf("%d\n",atoi(argv[1])-atoi(argv[2]));
	printf("%d\n",atoi(argv[1])*atoi(argv[2]));
	printf("%d\n",atoi(argv[1])/atoi(argv[2]));
	
 	return 1;
}
