#include <fstream>
#include <iterator>
#include <algorithm>
#include "string"
using namespace std;

void compileFiletoBinary(const char* filename, const char* output) {

	FILE* file = fopen(filename, "rb");
	FILE* out = fopen(output, "w");

	fprintf(out, 
"#pragma once\n\
\n\
void GenerateEXEFile(const char* path)\n\
{\n\
	static unsigned char const data[] =\n\
	{\n\
		\
"
	);

	size_t count;
	unsigned char buffer[32];
	while (!feof(file)) {
		count = fread(buffer, 1, 32, file);
		for (int n = 0; n < count; ++n) {
			fprintf(out, "0x%02X,", buffer[n]);
		};
	};

	fprintf(out, 
"\n\
	};\n\
	FILE* file = fopen(path, \"wb\");\n\
	fwrite(data, sizeof(char), sizeof(data) / sizeof(char), file);\n\
	fclose(file);\n\
}\n\
float* RunSDFGeneration(const char* exePath,\n\
	const char* inputMesh, int resolution,\n\
	float XMin, float YMin, float ZMin,\n\
	float XMax, float YMax, float ZMax)\n\
{\n\
	system((string(exePath) + \" -i \" + inputMesh + \" -o volume.txt\" + \" -r \" + to_string(resolution)\n\
		+ \" -s \" + to_string(XMin) + \",\" + to_string(YMin) + \",\" + to_string(ZMin)\n\
		+ \" -l \" + to_string(XMax) + \",\" + to_string(YMax) + \",\" + to_string(ZMax)\n\
		).c_str());\n\
	FILE* file = fopen(\"volume.txt\", \"r\");\n\
	int d;\n\
	fscanf(file, \"\%%d %%d %%d\", &d, &d, &d);\n\
	int len = d * d * d;\n\
	float* res = new float[len];\n\
	int idx = 0;\n\
	for (int i = 0; i < d; i++) for (int j = 0; j < d; j++) for (int k = 0; k < d; k++)\n\
	{\n\
		fscanf(file, \"%%f\", &res[idx++]);\n\
	}\n\
	fclose(file);\n\
	remove(\"volume.txt\");\n\
	return res;\n\
}"
	);

	fclose(file);
	fclose(out);
};

int main(int argc, char* argv[])
{
	char* exe_path = argv[1];
	printf("Path: %s", exe_path);

	string out = exe_path;
	out = out.substr(0, out.length() - 4) + ".h";

	compileFiletoBinary(exe_path, out.c_str());
}

// use case:
/*
#include "./SDF-CLI.h"
void Foo()
{
	char ExePath[] = "SDFGenerator.exe";
	GenerateEXEFile(ExePath);

	float* res = RunSDFGeneration(ExePath, "xxx.obj", 64, 0, 0, 0, 1, 1, 1);
	delete(res);
}
*/