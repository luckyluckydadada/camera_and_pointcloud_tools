#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),  (mode)))==NULL
int main(int argc,char *argv[])
{

	int num = 1000000;

	float *data = (float*)malloc(num * sizeof(float));

	float *px = data + 0;

	float *py = data + 1;

	float *pz = data + 2;

	float *pr = data + 3;

	FILE *stream;
	
    fopen_s(&stream, argv[1], "rb");

	num = fread(data, sizeof(float), num, stream) / 4;

	fclose(stream);

	FILE *writePCDStream;

	fopen_s(&writePCDStream,argv[2], "wb");

	fprintf(writePCDStream, "VERSION 0.7\n");

	fprintf(writePCDStream, "FIELDS x y z\n");

	fprintf(writePCDStream, "SIZE 4 4 4\n");

	fprintf(writePCDStream, "TYPE F F F\n");

	fprintf(writePCDStream, "WIDTH %d\n", num);

	fprintf(writePCDStream, "HEIGHT 1\n");

	fprintf(writePCDStream, "POINTS %d\n", num);

	fprintf(writePCDStream, "DATA ascii\n");

	for (int i = 0; i < num; i++)

	{

		fprintf(writePCDStream, "%f %f %f\n",*px,*py,*pz);

		px += 4; py += 4; pz += 4; pr += 4;

	}

	fclose(writePCDStream);
}

