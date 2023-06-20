// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include <nls/math/Math.h>

#include <vector>
#include <string>

#include <stdexcept>
#include <cstdio>
#include <cstring>

// Important: this code assumes the machine is using little endian ("<" in NumPy, as opposed to ">" for big endian)

namespace epic::nls::npy {

struct NPYHeader {
	std::string m_dataType; // supported: "<i4" (32-bit int), "<f4" (32-bit float), "<f8" (64-bit double)
	std::vector<int> m_shape;
	bool m_fortranOrder;
};

inline void ErrorNPY(const std::string& error)
{
	CARBON_CRITICAL("Npy error: {}", error);
}

inline const char* magicString = "\x93NUMPY";
inline const char* boolString[2] = { "False", "True" };

template<class T> void SaveNPY(std::string filename, const NPYHeader& header, const T* data, size_t dataSize) {
	std::string htxt = std::string(magicString) + "    {\'descr\': \'" + header.m_dataType + "\', 'fortran_order': " + boolString[header.m_fortranOrder] + ", \'shape\': (";
	for (size_t i = 0; i < header.m_shape.size(); i++)
	{
		htxt += std::to_string(header.m_shape[i]);
		if (i != header.m_shape.size() - 1) htxt += ", ";
	}
	htxt += "), }";
	const size_t htxtLen = htxt.size();
	size_t hbufLen = (htxtLen + 63) & (~63);
	htxt[6] = '\x01';
	htxt[7] = '\x00';
	htxt[8] = (hbufLen-10) & 255; // the -10 is for the 10 bytes of preHeader
	htxt[9] = ((hbufLen-10) >> 8) & 255;

	std::string hbuf; hbuf.insert(0, hbufLen, ' ');
	hbuf.replace(0, htxtLen, htxt);

#ifdef _MSC_VER
	std::FILE* f = nullptr;
	if (fopen_s(&f, filename.c_str(), "wb") != 0) {
		ErrorNPY("Can't open file " + filename);
	}
#else
	std::FILE* f = std::fopen(filename.c_str(), "wb");
	if (!f) ErrorNPY("Can't open file " + filename);
#endif
	if (f)
	{
		if (std::fwrite(&hbuf[0], 1, hbufLen, f) != hbufLen) ErrorNPY("Error writing into file " + filename);
		if (std::fwrite(data, sizeof(T), dataSize, f) != dataSize) ErrorNPY("Error writing into file " + filename);
		std::fclose(f);
	}
}

template<class T> void SaveNPY(std::string filename, const NPYHeader& header, const std::vector<T>& data) {
	SaveNPY(filename, header, data.data(), data.size());
}

inline std::vector<char> readStringInQuotes(const std::vector<char>& htxt, int& cnt, int headerLen)
{
	std::vector<char> result;
	const char openingQuote = htxt[cnt++];
	while (cnt < headerLen && htxt[cnt] != openingQuote)
		result.push_back(htxt[cnt++]);
	result.push_back(0);
	cnt++;
	return result;
}

template<class T> void LoadNPY(std::string filename, NPYHeader& header, std::vector<T>& data) {

#ifdef _MSC_VER
	std::FILE* f = nullptr;
	if (fopen_s(&f, filename.c_str(), "rb") != 0) {
		ErrorNPY("Can't open file " + filename);
	}
#else
	std::FILE* f = std::fopen(filename.c_str(), "rb");
	if (!f) ErrorNPY("Can't open file " + filename);
#endif

	char preHeader[10];
	if (std::fread(preHeader, 1, 10, f) != 10) ErrorNPY("Failed to read NumPy header");
	preHeader[9] = '\0';
	if (std::strncmp(preHeader, magicString, 6) != 0) ErrorNPY("Not a NumPy file");
	if (preHeader[6] != '\x01' || preHeader[7] != '\x00') ErrorNPY("Unsupported NPY version");

	int headerLen = (unsigned char)preHeader[8] + ((unsigned char)preHeader[9] << 8);
	if ((headerLen + 10) % 64 != 0) ErrorNPY("Unaligned NPY header");

	std::vector<char> htxt; htxt.resize(headerLen);
	if (int(std::fread(&htxt[0], 1, headerLen, f)) != headerLen || htxt[0] != '{') ErrorNPY("Failed loading header");

	int cnt = 1;
	unsigned char checks = 0;
	for (;;)
	{
		if (cnt >= headerLen) break;
		if (htxt[cnt] == '}') break;
		if (htxt[cnt] == ' ' || htxt[cnt] == ',') {
			cnt++;
			continue;
		}
		if (htxt[cnt] == '\'' || htxt[cnt] == '\"' || htxt[cnt] == '`') {
			const std::vector<char> str = readStringInQuotes(htxt, cnt, headerLen);

			while (cnt < headerLen && (htxt[cnt] == ':' || htxt[cnt] == ' ')) cnt++;
			if (std::strcmp(str.data(), "descr") == 0)
			{
				const std::vector<char> str2 = readStringInQuotes(htxt, cnt, headerLen);
				header.m_dataType = std::string(&str2[0]);
				checks |= 1;
			}
			else if (std::strcmp(str.data(), "fortran_order") == 0)
			{
				std::vector<char> str3;
				while (cnt < headerLen && htxt[cnt] != ' ' && htxt[cnt] != ',')
					str3.push_back(htxt[cnt++]);
				str3.push_back(0);
				cnt++;
				if (std::strcmp(str3.data(), "True") == 0) {
					header.m_fortranOrder = true;
					checks |= 2;
				}
				else if (std::strcmp(str3.data(), "False") == 0) {
					header.m_fortranOrder = false;
					checks |= 2;
				}
				else ErrorNPY("Unrecognized value for fortran_order");
			}
			else if (std::strcmp(str.data(), "shape") == 0)
			{
				if (cnt >= headerLen || htxt[cnt++] != '(') ErrorNPY("Header parsing error");
				bool numberStarted = false;
				int number = 0;
				for (;;)
				{
					if (cnt >= headerLen) break;
					if (!numberStarted) {
						if (std::isdigit(static_cast<unsigned char>(htxt[cnt]))) {
							numberStarted = true;
							number = htxt[cnt] - '0';
							cnt++;
						}
						else if (htxt[cnt] == ')') break;
						else if (htxt[cnt] == ' ') cnt++;
						else ErrorNPY("Header parsing error");
					}
					else {
						if (std::isdigit(static_cast<unsigned char>(htxt[cnt]))) {
							number = 10 * number + (htxt[cnt] - '0');
							cnt++;
						}
						else if (htxt[cnt] == ',' || htxt[cnt] == ' ' || htxt[cnt] == ')') {
							header.m_shape.push_back(number);
							numberStarted = false;
							checks |= 4;
							if (htxt[cnt++] == ')') break;
						}
						else ErrorNPY("Header parsing error");
					}
				}
			}
			else ErrorNPY("Unrecognized key in header");
		}
		else ErrorNPY("Header parsing error");
	}
	if (checks != 7) ErrorNPY("Header parsing error");
	if (std::strcmp(header.m_dataType.c_str(), "<i4") == 0)
	{	// int
		if constexpr (sizeof(T) != 4) ErrorNPY("Incorrect type T");
	}
	else if (std::strcmp(header.m_dataType.c_str(), "<f4") == 0)
	{	// float
		if constexpr (sizeof(T) != 4) ErrorNPY("Incorrect type T");
	}
	else if (std::strcmp(header.m_dataType.c_str(), "<f8") == 0)
	{	// double
		if constexpr (sizeof(T) != 8) ErrorNPY("Incorrect type T");
	}
	else ErrorNPY("Unsupported dtype");

	size_t scalarsCount = 1;
	for (auto x : header.m_shape) scalarsCount *= x;

	data.resize(scalarsCount);
	size_t numRead = std::fread(&data[0], sizeof(T), scalarsCount, f);
	if (numRead != scalarsCount) ErrorNPY("Data read error");
	std::fclose(f);
}

template <class T, int R, int C, int MatrixOrder>
void SaveMatrixAsNpy(const std::string& filename, const Eigen::Matrix<T, R, C, MatrixOrder>& matrix) {
	npy::NPYHeader vertices_header;
	vertices_header.m_dataType = std::string("<f") + std::to_string(sizeof(T));
	vertices_header.m_shape = {int(matrix.rows()), int(matrix.cols())};
	vertices_header.m_fortranOrder = (MatrixOrder == Eigen::ColMajor) ? true : false;

	npy::SaveNPY(filename, vertices_header, matrix.data(), matrix.size());
}

template <class T, int R, int C, int MatrixOrder>
void LoadMatrixFromNpy(const std::string& filename, Eigen::Matrix<T, R, C, MatrixOrder>& matrix) {
	npy::NPYHeader header;
    std::vector<T> data;
    npy::LoadNPY(filename, header, data);
	if (header.m_shape.size() != 2) {
		CARBON_CRITICAL("Npy error: invalid npy shape of size {}", header.m_shape.size());
	}
	const int rows = header.m_shape[0];
	const int cols = header.m_shape[1];
	if constexpr (R >= 0) {
		if (rows != R) {
			CARBON_CRITICAL("Npy error: number of rows expected to be {}, but got {}", R, rows);
		}
	}
	if constexpr (C >= 0) {
		if (cols != C) {
			CARBON_CRITICAL("Npy error: number of cols expected to be {}, but got {}", C, cols);
		}
	}
	matrix.resize(rows, cols);

	for (int i = 0; i < cols; ++i)
    {
        for (int j = 0; j < rows; ++j)
        {
            if (header.m_fortranOrder) matrix(j, i) = data[j + rows * i];
            else matrix(j, i) = data[i + cols * j];
        }
    }
}

} // namespace epic::nls::npy
