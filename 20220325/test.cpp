#include <stdio.h>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <ctime>

//벡터
typedef struct vector
{
	float x;
	float y;
	float z;
}Vector3;

Vector3 plus(Vector3 v1, Vector3 v2)
{
		Vector3 v;
		v.x = v1.x + v2.x;
		v.y = v1.y + v2.y;
		v.z = v1.z + v2.z;
		return v;
}

Vector3 minus(Vector3 v1, Vector3 v2)
{
		Vector3 vt;
		vt.x = v1.x - v2.x;
		vt.y = v1.y - v2.y;
		vt.z = v1.z - v2.z;
		return vt;
}

Vector3 scalar_product(Vector3 v, int n)
{
	v.x = v.x * n;
	v.y = v.y * n;
	v.z = v.z * n;
	return v;
}

//내적
float inner_product(Vector3 v1, Vector3 v2)
{
	float result;
	
	result = v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
 
	return result;
}

//외적
Vector3 outer_product(Vector3 v1, Vector3 v2)
{
	Vector3 result;
	
	result.x = v1.y*v2.z-v1.z*v2.y;
	result.y = v1.z*v2.x-v1.x*v2.z;
	result.z = v1.x*v2.y-v1.y*v2.x;
	
	return result;
}
int main() {
	Vector3 result_v;

	Vector3 v1;
	v1.x = rand()%10+1;
	v1.y = rand()%10+1;
	v1.z = rand()%10+1;
	
	printf("벡터 v1 - x : %f y : %f z : %f \n",v1.x,v1.y,v1.z);
	
	Vector3 v2;
	v2.x = rand()%10+1;
	v2.y = rand()%10+1;
	v2.z = rand()%10+1;
	
	printf("벡터 v2 - x : %f y : %f z : %f \n\n",v2.x,v2.y,v2.z);



	printf("벡터 행렬 연산\n\n");
	
	result_v = plus(v1,v2);
	printf("벡터 합: x : %f, y : %f, z : %f \n",result_v.x,result_v.y,result_v.z);
	
	result_v = minus(v1,v2);
	printf("벡터 차: x : %f, y : %f, z : %f \n",result_v.x,result_v.y,result_v.z);
	
	result_v = scalar_product(v1,2);
	printf("벡터-스칼라 곱: x : %f, y : %f, z : %f \n",result_v.x,result_v.y,result_v.z);

	printf("벡터 내적의 결과: %f \n",inner_product(v1,v2));
	
	result_v = outer_product(v1, v2);
	printf("벡터 외적의 결과: x : %f, y : %f, z : %f \n",result_v.x,result_v.y,result_v.z);
	return 0;
}
