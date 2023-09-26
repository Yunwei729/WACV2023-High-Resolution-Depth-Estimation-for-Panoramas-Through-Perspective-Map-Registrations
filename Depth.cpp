#include <omp.h>  //openmp
#include <list>
#include <string>

#include "Basic.h"
#include "ILMBase.h"
#include "Depth.h"

#include <opencv2/opencv.hpp>

//for loading textures
//#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <ceres/ceres.h>
#include "ceres/problem.h"
#include "ceres/solver.h"

//valid zenith range in degree
Vec2f g_zenith_range(D2R(26), D2R(154));

using namespace DepthNamespace;

//save 16bit png image
bool Save16BitPNG(unsigned short* data, int width, int height, const char* filename)
{
	cv::Mat a = cv::Mat(height, width, CV_16UC1, data);
	imwrite(filename, a);
	return true;
}

Vec3f LinePlaneIntersection(Vec3f p, Vec3f dir, Vec3f normal, Vec3f p0)
{
	//https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection

	//the line is p + t * dir, let's find t
	float t = ((p0 - p).dot(normal)) / (dir.dot(normal));

	return p + t * dir;
}


bool DepthNamespace::PerspectiveMap::Load(std::string& filename)
{
	if (data)
	{
		delete data;
		data = NULL;
	}

	//load a temporary 8bit or 16bit data buffer depending on the image file type
	//then conver to the actual 0~1 float data
	bool is_16bit = false;
	int result = stbi_is_16_bit(filename.c_str());
	if (stbi_is_16_bit(filename.c_str()) > 0)
	{
		is_16bit = true;

		unsigned short* Data = stbi_load_16(filename.c_str(), &width, &height, &channels, 0);
		if (Data == NULL)
		{
			std::cout << "[PerspectiveMap::Load] stbi_load_16 failed! " << filename << std::endl;
			return false;
		}

		data = new float[width * height * channels];
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				for (int c = 0; c < channels; c++)
				{
					data[(y * width + x) * channels + c] = (float)Data[(y * width + x) * channels + c] / 65535.0f;
				}
			}
		}
		delete Data;
	}
	else
	{
		is_16bit = false;


		unsigned char* Data = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (Data == NULL)
		{
			std::cout << "[PerspectiveMap::Load] stbi_load failed! " << filename << std::endl;
			return false;
		}

		data = new float[width * height * channels];
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				for (int c = 0; c < channels; c++)
				{
					data[(y * width + x) * channels + c] = (float)Data[(y * width + x) * channels + c] / 255.0f;
				}
			}
		}
		delete Data;
	}
	//cout << "[PerspectiveMap::Load] success! 16bit?" << is_16bit << " w:" << width << " h:" << height << " c:" << channels << endl;

	return true;
}

float DepthNamespace::PerspectiveMap::Value(float x, float y)
{
	//note: assume the image file's rows are top-to-bottom
	int X = x * (float)(width - 1);
	int Y = y * (float)(height - 1);

	return data[(Y * width + X) * channels];
}

void DepthNamespace::PerspectiveMap::SetWindow(float AzimuthLeft, float AzimuthRight, float ZenithTop, float ZenithDown)
{
	azimuth_left = AzimuthLeft;
	azimuth_right = AzimuthRight;
	zenith_top = ZenithTop;
	zenith_down = ZenithDown;

	////one-time pre-calculate info about the quad viewing window:

	//this is also the looking dir
	middle = SphericalToWorld((azimuth_left + azimuth_right) / 2, (zenith_top + zenith_down) / 2);

	//the left dir is approx. up CROSS look-dir :
	Vec3f left_dir = Vec3f(0, 0, 1).cross(middle).normalize();

	//the real up dir is left cross look-dir:
	Vec3f up_dir = left_dir.cross(middle).normalize();

	//the left-middle and right_middle 3d points: (longer dists)
	Vec3f left_middle = middle + left_dir * tan(abs(azimuth_right - azimuth_left) / 2);
	Vec3f right_middle = middle - left_dir * tan(abs(azimuth_right - azimuth_left) / 2);

	//the up-middle and down-middle 3d points:
	Vec3f up_middle = middle - up_dir * tan(abs(zenith_top - zenith_down) / 2);
	Vec3f down_middle = middle + up_dir * tan(abs(zenith_top - zenith_down) / 2);

	//the 3d position of the "left-up", "left-down", "right-down", and "right-up" corners:
	corner0 = middle + (left_middle - middle) + (up_middle - middle);
	corner1 = middle + (left_middle - middle) + (down_middle - middle);
	corner2 = middle + (right_middle - middle) + (down_middle - middle);
	corner3 = middle + (right_middle - middle) + (up_middle - middle);

	//the two horizontal and vertical edges of the rectangular window in 3d:
	hedge = (right_middle - left_middle);
	vedge = (down_middle - up_middle);
}

Vec2f DepthNamespace::PerspectiveMap::ToSphericalCoord(float x, float y)
{
	//NOTE: assume 0<=x,y<=1

	//calculate the pos of the point @ (x,y) 
	Vec3f pos = corner0 + hedge * x + vedge * y;

	//convert the 3d point to spherical coord!
	return WorldToSpherical(pos);
}

Vec2f DepthNamespace::PerspectiveMap::SphericalTo2D(float azimuth, float zenith)
{
	//the ray's 3d position on the unit sphere (i.e., a dir)
	Vec3f dir = SphericalToWorld(azimuth, zenith);

	//project pos onto the rectangle window
	Vec3f pos = LinePlaneIntersection(Vec3f(0), dir, middle, middle);

	//find x and y using hedge and vedge as two axis
	Vec3f e = pos - corner0;
	float x = (e.dot(hedge) / hedge.length()) / hedge.length();
	float y = (e.dot(vedge) / vedge.length()) / vedge.length();

	return Vec2f(x, y);
}

bool DepthNamespace::PerspectiveMap::Contain(float azimuth, float zenith)
{
	//the ray's 3d position on the unit sphere (i.e., a dir)
	Vec3f dir = SphericalToWorld(azimuth, zenith);

	//project pos onto the rectangle window
	Vec3f pos = LinePlaneIntersection(Vec3f(0), dir, middle, middle);

	//find x and y using hedge and vedge as two axis
	Vec3f e = pos - corner0;
	float x = (e.dot(hedge) / hedge.length()) / hedge.length();
	float y = (e.dot(vedge) / vedge.length()) / vedge.length();

	float threshold = 1e-3;
	bool within = (x >= 0 - threshold && x <= 1 + threshold &&
		y >= 0 - threshold & y <= 1 + threshold);
	if (!within)
	{
		//cout << "Contain NOT: " << x << "," << y << endl;
		return false;
	}
	else
		return true;
}

float DepthNamespace::PerspectiveMap::ValueAtXY(int x, int y)
{
	return data[(y * width + x) * channels];
}

void DepthNamespace::PerspectiveMap::D2DTransform(Vec4f abcd)
{
	float a = abcd[0];
	float b = abcd[1];
	float c = abcd[2];
	float d = abcd[3];

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float X = data[(y * width + x) * channels];  //0~1 "disparity"

			if (X < 1e-4)
				X = 1e-4;
			else if (X > (1 - 1e-4))
				X = 1 - 1e-4;

			//calculate the Y ("depth") according to abcd formula:
			float Y = c / (a * X + b) + d;

			if (Y < 0)
				Y = 0;
			else if (Y > 1)
				Y = 1;

			data[(y * width + x) * channels] = Y;
		}
	}
}

void DepthNamespace::PerspectiveMap::Depth2DepthTransform(Vec4f& abcd)
{
	float a = abcd[0];
	float b = abcd[1];
	float c = abcd[2];
	float d = abcd[3];

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float X = data[(y * width + x) * channels];

			if (X < 1e-4)
				X = 1e-4;
			else if (X > (1 - 1e-4))
				X = 1 - 1e-4;

			//calculate the Y ("depth") according to ab formula:
			float Y = a * X * X * X + b * X * X + c * X + d;

			if (Y < 0)
				Y = 0;
			else if (Y > 1)
				Y = 1;

			data[(y * width + x) * channels] = Y;
		}
	}
}


bool DepthNamespace::EquirectangularMap::Load(std::string& filename, bool mono360)
{
	//end with "pfm"? then use LoadPfm()
	if (filename[filename.size() - 3] == 'p' && filename[filename.size() - 2] == 'f' && filename[filename.size() - 1] == 'm')
	{
		bool flip_vertical = false;
		bool normalize = false;

		//mono360's pfm:
		if (mono360)
		{
			flip_vertical = true;
			normalize = true;
		}

		return LoadPfm(filename, flip_vertical, normalize);
	}

	if (data)
	{
		delete data;
		data = NULL;
	}

	//load a temporary 8bit or 16bit data buffer depending on the image file type
	//then conver to the actual 0~1 float data
	bool is_16bit = false;
	if (stbi_is_16_bit(filename.c_str()) > 0)
	{
		is_16bit = true;

		unsigned short* Data = stbi_load_16(filename.c_str(), &width, &height, &channels, 0);
		if (Data == NULL)
		{
			std::cout << "[EquirectangularMap::Load16bit] stbi_load_16 failed!" << filename << std::endl;
			return false;
		}

		data = new float[width * height * channels];
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				for (int c = 0; c < channels; c++)
				{
					data[(y * width + x) * channels + c] = (float)Data[(y * width + x) * channels + c] / 65535.0f;
				}
			}
		}
		delete Data;
	}
	else
	{
		is_16bit = false;

		unsigned char* Data = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (Data == NULL)
		{
			std::cout << "[EquirectangularMap::Load8bit] stbi_load failed:" << filename << std::endl;
			return false;
		}

		data = new float[width * height * channels];
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				for (int c = 0; c < channels; c++)
				{
					data[(y * width + x) * channels + c] = (float)Data[(y * width + x) * channels + c] / 255.0f;
				}
			}
		}
		delete Data;
	}
	//cout << "[EquirectangularMap::Load] success! 16bit?" << is_16bit << " w:" << width << " h:" << height << " c:" << channels << endl;

	return true;
}

// Load PFM(portable float map) image. float32 pixel data type.
#if defined(__sparcv9)
// Big endian
#else
#if (__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__) || MINIZ_X86_OR_X64_CPU
// Set MINIZ_LITTLE_ENDIAN to 1 if the processor is little endian.
#define MY_LITTLE_ENDIAN 1
#endif
#endif
static void swap4(unsigned int* val) {
	unsigned int tmp = *val;
	unsigned char* dst = (unsigned char*)(val);
	unsigned char* src = (unsigned char*)(&tmp);

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
}
static float* load_pfm(const char* filename, int* w, int* h, int* c) {
	FILE* fp = NULL;
#if defined(_MSC_VER)
	errno_t err = fopen_s(&fp, filename, "rb");
	if (err != 0) {
		fprintf(stderr, "Failed to open file: %s\n", filename);
		return NULL;
	}
#else
	fp = fopen(filename, "rb");
#endif

	if (!fp) {
		fprintf(stderr, "Failed to open file: %s\n", filename);
		return NULL;
	}

	char buf[1024];
	fscanf(fp, "%s\n", buf);

	int channels = 0;
	if (strcmp(buf, "PF") == 0) {
		channels = 3;
	}
	else if (strcmp(buf, "Pf") == 0) {
		channels = 1;
	}
	else {
		fprintf(stderr, "Unsupported type: %s\n", buf);
		return NULL;
	}

	int width, height;
	fscanf(fp, "%d %d\n", &width, &height);

	float endian_flag = 0.0f;
	fscanf(fp, "%f\n", &endian_flag);

	int swap_endian = 0;

	if (endian_flag < 0.0f) {
#if !defined(MY_LITTLE_ENDIAN)
		swap_endian = 1;
#endif
	}
	else {
#if defined(MY_LITTLE_ENDIAN)
		swap_endian = 1;
#endif
	}

	// TODO(syoyo): Consider endianness
	size_t img_size = (size_t)(width * height * channels) * sizeof(float);
	float* img = (float*)malloc(img_size);

	size_t n_read = fread((void*)img, 1, img_size, fp);

	if (n_read != img_size) {
		fprintf(stderr, "Failed to read image data n_read:%d img_size:%d\n", n_read, img_size);
		return NULL;
	}

	fclose(fp);

	if (swap_endian) {
		for (size_t i = 0; i < (size_t)(width * height * channels); i++) {
			swap4((unsigned int*)(&img[i]));
		}
	}

	

	(*w) = width;
	(*h) = height;
	(*c) = channels;

	return img;
}

bool DepthNamespace::EquirectangularMap::LoadPfm(std::string& filename, bool flip_vertical, bool normalize, const char* save_png_filename)
{
	
	//load the float array of the pfm file:
	float* img = NULL;
	img = load_pfm(filename.c_str(), &width, &height, &channels);
	if (img == NULL)
	{
		std::cout << "load_pfm() failed? " << filename << std::endl;
		return false;
	}

	//find min-max of original value range
	Vec2f minmax(FLT_MAX, -FLT_MAX);
	//if (normalize)
	{
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				for (int c = 0; c < channels; c++)
				{
					float val = img[(y * width + x) * channels + c];

					if (val < minmax[0])
						minmax[0] = val;
					if (val > minmax[1])
						minmax[1] = val;
				}
			}
		}
	}

	std::cout << "[LoadPfm] " << filename << " W:" << width << " H:" << height << " C:" << channels << " MMax:" << minmax << std::endl;
	
	//put into data:

	if (data)
	{
		delete data;
		data = NULL;
	}
	data = new float[width * height * channels];
	
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			for (int c = 0; c < channels; c++)
			{
				float val = 0;

				if(!flip_vertical)
					val = img[(y * width + x) * channels + c];
				else
					val = img[((height -1 - y) * width + x) * channels + c];
				
				//normalize to 0~1?
				if(normalize)
					val = (val - minmax[0]) / (minmax[1] - minmax[0]);
				else
				{
					//cap to 0~10
					if (val < 0)
						val = 0;
					val = MIN2(val / 10.0f, 10.0f);
				}

				data[(y * width + x) * channels + c] = val;
			}
		}
	}
	
	//test: output png?
	if(save_png_filename)
	{
		unsigned char *Data = new unsigned char[width * height];
		for (int x = 0; x < width; x++)
		{
			for (int y = 0; y < height; y++)
			{
				float val = data[(y * width + x) * channels];

				Data[y * width + x] = (unsigned char) (val * 255.0f);
			}
		}

		stbi_write_png(save_png_filename, width, height, 1, Data, width * 1/*stride in bytes*/);
		//Save16BitPNG(Data, width, height, save_png_filename);
		delete Data;
	}

	delete img;
	return true;
}

float DepthNamespace::EquirectangularMap::ValueAtCoord(float azi, float zen)
{
	int x = azi / (MYPI * 2) * (float)(width - 1);
	int y = zen / MYPI * (float)(height - 1);
	return data[(y * width + x) * channels];
}

float DepthNamespace::EquirectangularMap::ValueAtXY(int x, int y)
{
	return data[(y * width + x) * channels];
}

double DepthNamespace::EquirectangularMap::Avg()
{
	double avg = 0;
	int count = 0;
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			float val = data[(y * width + x) * channels];
			if (val > 0)
			{
				avg += val;
				count++;
			}
		}
	}

	if (count == 0)
		return 0;

	avg /= (float)count;
	return avg;
}

void DepthNamespace::EquirectangularMap::DispDepthConversion()
{
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			for (int c = 0; c < channels; c++)
			{
				float val = data[(y * width + x) * channels];

				//zero? keep value 
				if (abs(val) < 1e-5)
				{	
				}
				else
				{
					val = 1 / val;  //disp<->depth conversion

					data[(y * width + x) * channels] = val;
				}
			}
		}
	}
}

bool DepthNamespace::EquirectangularMap::Save8bit(std::string& filename)
{
	unsigned char* data = new unsigned char[width * height];	

	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			float val = ValueAtXY(x, y);
			
			//cap to 0~1
			if (val < 0)
				val = 0;
			if (val > 1)
				val = 1;

			data[y * width + x] = (unsigned char)(val * 255.0f);
		}
	}

	stbi_write_png(filename.c_str(), width, height, 1, data, width * 1/*stride in bytes*/);
	delete data;
	return true;
}

bool DepthNamespace::MedianScaling(EquirectangularMap& emap0, EquirectangularMap& emap1, float* emap0_median, float* emap1_median)
{
	std::vector<float> val0s;
	std::vector<float> val1s;
	for (int x = 0; x < emap0.width; x++)
	{
		for (int y = 0; y < emap0.height; y++)
		{
			float val = emap0.ValueAtXY(x, y);
			if (val < 1e-4 || val >= 1 - 1e-4)
				continue;

			val0s.push_back(val);
		}
	}
	for (int x = 0; x < emap1.width; x++)
	{
		for (int y = 0; y < emap1.height; y++)
		{
			float val = emap1.ValueAtXY(x, y);
			if (val < 1e-4 || val >= 1 - 1e-4)
				continue;
			
			val1s.push_back(val);
		}
	}

	auto m0 = val0s.begin() + val0s.size() / 2;
	std::nth_element(val0s.begin(), m0, val0s.end());
	float val0_median = val0s[val0s.size() / 2];

	auto m1 = val1s.begin() + val1s.size() / 2;
	std::nth_element(val1s.begin(), m1, val1s.end());
	float val1_median = val1s[val1s.size() / 2];

	if (emap0_median)
		*emap0_median = val0_median;
	if (emap1_median)
		*emap1_median = val1_median;

	//cout << "val0_median:" << val0_median << " val1_median:" << val1_median << endl;	

	float scaling = 1;
	if (val0_median == 0)
	{
		std::cout << "MedianScaling: zero val0_median ??" << std::endl;
		return false; //??
	}
	scaling = val1_median / val0_median;

	//scale emap0's values!
	for (int x = 0; x < emap0.width; x++)
	{
		for (int y = 0; y < emap0.height; y++)
		{
			float val = emap0.ValueAtXY(x, y);
			if (val < 1e-4 || val >= 1 - 1e-4)
				continue;

			val = val * scaling;
			emap0.data[(y * emap0.width + x) * emap0.channels] = val;
		}
	}
	return true;
}

void DepthNamespace::EquirectangularMap::CopyInvalidPixels(EquirectangularMap& ref_emap)
{
	float ratio_x = (float)ref_emap.width / (float)width;
	float ratio_y = (float)ref_emap.height / (float)height;

	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//given (x,y) in emap0, the corresponding (X,Y) in emap1?
			int X = (float)x * ratio_x;
			int Y = (float)y * ratio_y;

			float val0 = ref_emap.ValueAtXY(X, Y);  //0~1

			//mark masked pixels? (black or white)
			if (val0 < 1e-4 || val0 >= 1 - 1e-4)
			{
				data[(y * width + x) * channels] = val0;
			}
		}
	}
}

//utility function to trivially convert 0~1 disparity value to 0~1 depth value
float DispairtyToDepth(float disparity)
{	
	const float disparity_min = 0.005f;
	if (disparity <= disparity_min)
		disparity = disparity_min;
	
	//trivial conversion:
	return (disparity_min) * 1 / disparity;   //so when disparity = disparity_min, return 1; when disparity = 1, return 0.	
}

//radian spherical coord to (x,y) in equirectangular view
Vec2i CoordToXY(Vec2f& coord, int out_width, int& out_height)
{
	int c = 0;  //#column
	if (coord.x < 0)  //negative azimuth?
		c = (MYPI * 2 + coord.x) / (MYPI * 2) * (out_width - 1);
	else
		c = (coord.x) / (MYPI * 2) * (out_width - 1);
	c = MIN2(c, out_width - 1);

	int r = coord.y / MYPI * (out_height - 1);  //#row
	r = MIN2(r, out_height - 1);

	return Vec2i(c, r);
}

bool DepthNamespace::MergeDepthMaps(std::string& equirectangular_map_filename, std::vector<std::string>& perspective_map_filenames, std::string& out_filename,
	std::vector<Vec4f>& pmap_FOVs, std::vector<Vec4f>& pmap_ranges,
	int out_width, Vec2f& zenith_range, std::string* equirectangular_map_groundtruth, Metrics* metrics, int* time_Reg, int* time_Laplacian)
{
	DWORD time_begin = timeGetTime();

	//the global (baseline) equirectangular map:
	EquirectangularMap emap_baseline;

	//the each perspective maps:
	std::vector<PerspectiveMap> pmaps;

	//load the "baseline" equirectangular map
	if (!emap_baseline.Load(equirectangular_map_filename))
		return false;

	int out_height = out_width / 2;

	//load the perspective_maps
	pmaps.resize(perspective_map_filenames.size());
	for (int i = 0; i < perspective_map_filenames.size(); i++)
	{
		PerspectiveMap& pmap = pmaps[i];
		if (!pmap.Load(perspective_map_filenames[i]))
			return false;

		pmap.SetWindow(pmap_FOVs[i][0], pmap_FOVs[i][1], pmap_FOVs[i][2], pmap_FOVs[i][3]);

		//set the valid (to-use) ranges of each pmap
		pmap.ranges[0] = MIN2(pmap_ranges[i][0], D2R(359.9));
		pmap.ranges[1] = MIN2(pmap_ranges[i][1], D2R(359.9));
		pmap.ranges[2] = pmap_ranges[i][2];
		pmap.ranges[3] = pmap_ranges[i][3];
	}

	//let's solve depth-to-depth register for every pmap first
	if (true)
	{
		DWORD time = timeGetTime();

		for (int p = 0; p < pmaps.size(); p++)
		{
			std::vector<bool> pmaps_actives(pmaps.size(), false);
			pmaps_actives[p] = true;  //only activate this pmap

			Vec4f abcd;
			if (SolveDepthToDepth(emap_baseline, pmaps, pmaps_actives, zenith_range, abcd))
			{
				//now, convert the disparity values to depth values by abcd
				pmaps[p].Depth2DepthTransform(abcd);
			}
		}

		if (time_Reg)
			*time_Reg = timeGetTime() - time;

	}	

	//the output equirectangular map buffer (16bit):
	unsigned short* data = new unsigned short[out_width * out_height];  //rows is top-to-bottom order
	std::memset(data, 0, sizeof(unsigned short) * out_width * out_height);  //default is completely black 

	//debug: save directly copy values from pmaps?
	if (false)
	{
		unsigned short* data2 = new unsigned short[out_width * out_height];
		std::memset(data2, 0, sizeof(unsigned short) * out_width * out_height);

		for (int x = 0; x < out_width; x++)
		{
			for (int y = 0; y < out_height; y++)
			{
				//to spherical coord.
				Vec2f coord((float)x / (float)(out_width - 1) * 2 * MYPI, (float)y / (float)(out_height - 1) * MYPI);

				for (int p = 0; p < pmaps.size(); p++)
				{
					PerspectiveMap& pmap = pmaps[p];

					//determine the bounding box (as x,y pixel coords) of this pamp in the output buffer
					int x0 = round((pmap.ranges[0] / (2 * MYPI)) * (out_width - 1));
					int x1 = round((pmap.ranges[1] / (2 * MYPI)) * (out_width - 1));
					int y0 = round((pmap.ranges[2] / MYPI) * (out_height - 1));
					int y1 = round((pmap.ranges[3] / MYPI) * (out_height - 1));

					if (x >= MIN2(x0, x1) && x <= MAX2(x0, x1) && y >= y0 && y <= y1)
					{
						Vec2f xy = pmap.SphericalTo2D(coord.x, coord.y);
						float val = pmap.Value(xy[0], xy[1]);

						//outline pmap bounding boxes:
						//if(x == x0 || x == x1 || y == y0 || y == y1)
						//	val = 1;  

						data2[y * out_width + x] = (unsigned short)(val * 65535.0f);

						//save to data too?
						data[y * out_width + x] = (unsigned short)(val * 65535.0f);

						break;
					}
				}
			}
		}

		Save16BitPNG(data2, out_width, out_height, (out_filename + ".pmap.png").c_str());
		//Save16BitPNG(data2, out_width, out_height, (out_filename).c_str());

		delete data2;

		//return true;
	}

	//debug: overwrite emap by current pmaps?
	if (false)
	{
		for (int x = 0; x < out_width; x++)
		{
			for (int y = 0; y < out_height; y++)
			{
				//to spherical coord.
				Vec2f coord((float)x / (float)(out_width - 1) * 2 * MYPI, (float)y / (float)(out_height - 1) * MYPI);

				for (int p = 0; p < pmaps.size(); p++)
				{
					PerspectiveMap& pmap = pmaps[p];

					//determine the bounding box (as x,y pixel coords) of this pamp in the output buffer
					int x0 = round((pmap.ranges[0] / (2 * MYPI)) * (out_width - 1));
					int x1 = round((pmap.ranges[1] / (2 * MYPI)) * (out_width - 1));
					int y0 = round((pmap.ranges[2] / MYPI) * (out_height - 1));
					int y1 = round((pmap.ranges[3] / MYPI) * (out_height - 1));

					if (x >= MIN2(x0, x1) && x <= MAX2(x0, x1) && y >= y0 && y <= y1)
					{
						Vec2f xy = pmap.SphericalTo2D(coord.x, coord.y);
						float val = pmap.Value(xy[0], xy[1]);

						//outline pmap bounding boxes:
						//if(x == x0 || x == x1 || y == y0 || y == y1)
						//	val = 1;  

						emap_baseline.data[(y * out_width + x) * emap_baseline.channels] = val;
						break;
					}
				}
			}
		}
	}

	//solve depths globally?
	if (true)
	{
		DWORD time = timeGetTime();

		//to save Laplacian image?
		//SolveDepthAll(emap_baseline, pmaps, data, out_width, out_height, zenith_range,
		//	(out_filename + ".Lap.png").c_str());

		SolveDepthAll(emap_baseline, pmaps, data, out_width, out_height, zenith_range);		

		if (time_Laplacian)
			*time_Laplacian = timeGetTime() - time;
	}
	//or do trivial smoothing?
	if (false)
	{
		SolveDepthBySmoothing(pmaps, data, out_width, out_height, zenith_range);
	}

	//stbi_flip_vertically_on_write(false);  //note: don't need to flip vertically
	//stbi_write_png(out_filename.c_str(), out_width, out_height, 1, data, out_width * 1/*stride in bytes*/);

	//save results!
	Save16BitPNG(data, out_width, out_height, out_filename.c_str());

	std::cout << "...All done! @" << timeGetTime() - time_begin << std::endl;

	//if ground truth (equirectangular map) is given, report the error statistics
	if (equirectangular_map_groundtruth)
	{
		int align_way = 1;
		//load the ground-truth emap:
		EquirectangularMap emap_gt;
		bool cap_depth = true;
		if (emap_gt.Load((*equirectangular_map_groundtruth)))
		{
			ErrorEmap(emap_gt, emap_baseline, (*metrics).mse_given, (*metrics).mae_given, (*metrics).mre_given,
				(*metrics).mselog_given, (*metrics).delta1_given, (*metrics).delta2_given, (*metrics).delta3_given, align_way, cap_depth);

			ErrorData(emap_gt, data, out_width, out_height, (*metrics).mse_result, (*metrics).mae_result, (*metrics).mre_result,
				(*metrics).mselog_result, (*metrics).delta1_result, (*metrics).delta2_result, (*metrics).delta3_result, align_way, cap_depth);

			(*metrics).Print();

			//also save result with invalid pixels of gt marked out (made black)
			{
				int height0 = floor(out_height * zenith_range[0] / MYPI);
				int height1 = ceil(out_height * zenith_range[1] / MYPI);

				//write a new "result" emap
				unsigned short* data2 = new unsigned short[out_width * out_height];
				for (int y = 0; y < out_height; y++)
				{
					for (int x = 0; x < out_width; x++)
					{
						//write black if outside the valid zenith range
						if (y< height0 || y > height1)
						{
							data2[y * out_width + x] = 0;
							continue;
						}

						//given (x,y), the corresponding (X,Y) in emap_gt?
						int X = (float)x * (float)emap_gt.width / (float)out_width;
						int Y = (float)y * (float)emap_gt.height / (float)out_height;

						unsigned short val0 = data[y * out_width + x];
						float val1 = emap_gt.ValueAtXY(X, Y);
						if (val1 == 0)
						{
							//overwrite the corresponding pixel in emap to black also!
							data2[y * out_width + x] = 0;
						}
						else if (val1 >= 1 - 1e-4)
						{
							//overwrite the corresponding pixel in emap to white also!
							data2[y * out_width + x] = 65535;
						}
						else  //normal valid values:
						{
							data2[y * out_width + x] = val0;
						}
					}
				}
				Save16BitPNG(data2, out_width, out_height, (out_filename + ".res.png").c_str());
				delete data2;

				//write a new "given" emap
				//EquirectangularMap emap_given;
				//emap_given.Load(equirectangular_map);
				unsigned short* data3 = new unsigned short[emap_baseline.width * emap_baseline.height];  //stride = 1bytes

				height0 = floor(emap_baseline.height * zenith_range[0] / MYPI);
				height1 = ceil(emap_baseline.height * zenith_range[1] / MYPI);

				for (int yy = 0; yy < emap_baseline.height; yy++)
				{
					for (int xx = 0; xx < emap_baseline.width; xx++)
					{
						//write black if outside the valid zenith range
						if (yy< height0 || yy > height1)
						{
							data3[(yy * emap_baseline.width + xx)] = 0;
							continue;
						}

						//given (x,y) in emap0, the corresponding (X,Y) in emap1?
						int X = (float)xx * (float)emap_gt.width / (float)emap_baseline.width;
						int Y = (float)yy * (float)emap_gt.height / (float)emap_baseline.height;

						float val0 = emap_baseline.ValueAtXY(xx, yy);
						float val1 = emap_gt.ValueAtXY(X, Y);
						if (val1 == 0)
						{
							//overwrite the corresponding pixel in emap to black also!
							data3[(yy * emap_baseline.width + xx)] = 0;
						}
						else if (val1 >= 1 - 1e-4)
						{
							//overwrite the corresponding pixel in emap to white also!
							data3[(yy * emap_baseline.width + xx)] = 65535;
						}
						else
						{
							data3[(yy * emap_baseline.width + xx)] = (unsigned short)(val0 * 65535.0f);
						}
					}
				}
				Save16BitPNG(data3, emap_baseline.width, emap_baseline.height, (out_filename + ".giv.png").c_str());
				delete data3;
			}
		}
	}

	delete data;
	return true;
}

//functor for dispariy-to-depth, x to y
struct FunctorDisparity2Depth
{
	FunctorDisparity2Depth(double weight, double x, double y) : Weight(weight), X(x), Y(y) {}

	//template <typename T> bool operator()(const T* const a, const T* const b, const T* const c, const T* const d, T* residual) const
	//{
	//	//residual: c * (1 / (ax+b)) + d - y
	//	residual[0] = T(Weight) * ( c[0] / (a[0]*T(X) + b[0]) + d[0] - T(Y) );
	//	return true;
	//}

	//template <typename T> bool operator()(const T* const b, const T* const c, const T* const d, T* residual) const
	//{
	//	//residual[0] = T(Weight) * (c[0] / (T(X) + b[0] + T(0.0001)) + d[0] - T(Y));
	//	residual[0] = T(Weight) * (c[0] / (T(X) + b[0]) + d[0] - T(Y));
	//	return true;
	//}

	template <typename T> bool operator()(const T* const a, const T* const b, const T* const d, T* residual) const
	{
		//residual[0] = T(Weight) * (c[0] / (T(X) + b[0] + T(0.0001)) + d[0] - T(Y));
		residual[0] = T(Weight) * (T(1) / (a[0] * T(X) + b[0]) + d[0] - T(Y));
		return true;
	}

private:
	double Weight;
	double X;  //disparity
	double Y;  //depth
};

//functor for depth-to-depth, x to y
struct FunctorDepth2Depth
{
	FunctorDepth2Depth(double weight, double x, double y) : Weight(weight), X(x), Y(y) {}

	template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const
	{
		residual[0] = T(Weight) * (T(X) * a[0] + b[0] - T(Y));
		return true;
	}

private:
	double Weight;
	double X;
	double Y;
};
struct FunctorDepth2Depth2
{
	FunctorDepth2Depth2(double weight, double x, double y) : Weight(weight), X(x), Y(y), X2(x* x) {}

	template <typename T> bool operator()(const T* const a, const T* const b, const T* const c, T* residual) const
	{
		residual[0] = T(Weight) * (T(X2) * a[0] + T(X) * b[0] + c[0] - T(Y));
		return true;
	}

private:
	double Weight;
	double X;
	double X2;
	double Y;
};
struct FunctorDepth2Depth1
{
	FunctorDepth2Depth1(double weight, double x, double y) : Weight(weight), X(x), Y(y) {}

	template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const
	{
		residual[0] = T(Weight) * (T(X) * a[0] + b[0] - T(Y));
		return true;
	}

private:
	double Weight;
	double X;
	double Y;
};
struct FunctorDepth2Depth3
{
	FunctorDepth2Depth3(double weight, double x, double y) : Weight(weight), X(x), Y(y), X2(x* x), X3(x* x* x) {}

	template <typename T> bool operator()(const T* const a, const T* const b, const T* const c, const T* const d, T* residual) const
	{
		residual[0] = T(Weight) * (T(X3) * a[0] + T(X2) * b[0] + T(X) * c[0] + d[0] - T(Y));
		return true;
	}

private:
	double Weight;
	double X;
	double X2;
	double X3;
	double Y;
};
struct FunctorDepth2Depth4
{
	FunctorDepth2Depth4(double weight, double x, double y) : Weight(weight), X(x), Y(y), X2(x* x), X3(x* x* x), X4(x*x*x*x) {}

	template <typename T> bool operator()(const T* const a, const T* const b, const T* const c, const T* const d, const T* const e, T* residual) const
	{
		residual[0] = T(Weight) * (T(X4) * a[0] + T(X3) * b[0] + T(X2) * c[0] + T(X) * d[0] + e[0] - T(Y));
		return true;
	}

private:
	double Weight;
	double X;
	double X2;
	double X3;
	double X4;
	double Y;
};

bool DepthNamespace::SolveDepthToDepth2(EquirectangularMap& emap, unsigned short* data, int width, int height,
	Vec2f& zenith_range, Vec4f& abcd)
{
	//sampling along azimuth and zenith ranges
	const float subd_azi = D2R(1);
	const float subd_zen = D2R(1);

	//the actual height range: 
	int height0 = floor(height * zenith_range[0] / MYPI);
	int height1 = ceil(height * zenith_range[1] / MYPI);

	double* vars = new double[4];  //a,b,c,d
	vars[0] = 0;
	vars[1] = 0;
	vars[2] = 1;
	vars[3] = 0;

	//double* vars = new double[3];  //a,b,c
	//vars[0] = 1;
	//vars[1] = 1;
	//vars[2] = 1;

	//double* vars = new double[2];  //a,b
	//vars[0] = 1;
	//vars[1] = 1;	

	ceres::Problem problem;

	int num_samples = width * (height1 - height0 + 1);

	//crate place holders
	ceres::CostFunction** functorD2Ds = new ceres::CostFunction * [num_samples];

	int index = 0;
	for (int Y = height0; Y <= height1; Y++)
	{
		for (int X = 0; X < width; X++)
		{
			//to spherical coord.:
			Vec2f coord((float)X / (float)(width - 1) * 2 * MYPI, (float)Y / (float)(height - 1) * MYPI);

			double depth0 = (float)data[Y * width + X] / 65535.0f;  //depth value of the result data, 0~1
			if (depth0 < 1e-4)
				depth0 = 1e-4;
			else if (depth0 > (1 - 1e-4))
				depth0 = 1 - 1e-4;

			double depth1 = emap.ValueAtCoord(coord.x, coord.y);  //depth value of the baseline emap, 0~1
			if (depth1 < 1e-4)
				depth1 = 1e-4;
			else if (depth1 > (1 - 1e-4))
				depth1 = 1 - 1e-4;

			//add a functor!

			functorD2Ds[index] = new ceres::AutoDiffCostFunction<FunctorDepth2Depth3, 1/*residual size*/,
				1/*a*/, 1/*b*/, 1/*c*/, 1/*d*/>(new FunctorDepth2Depth3(1.0f, depth0, depth1));

			//functorD2Ds[index] = new ceres::AutoDiffCostFunction<FunctorDepth2Depth2, 1/*residual size*/,
			//	1/*a*/, 1/*b*/, 1/*c*/>(new FunctorDepth2Depth2(1.0f, depth0, depth1));

			//functorD2Ds[index] = new ceres::AutoDiffCostFunction<FunctorDepth2Depth, 1/*residual size*/,
//				1/*a*/, 1/*b*/>(new FunctorDepth2Depth(1.0f, depth0, depth1));

			//functorD2Ds[index] = new ceres::AutoDiffCostFunction<FunctorDepth2Depth1, 1/*residual size*/,
			//	1/*a*/>(new FunctorDepth2Depth1(1.0f, depth0, depth1));

			index++;
		}
	}

	//add residual blocks (one per sample point):
	for (int i = 0; i < num_samples; i++)
	{
		problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/, &vars[1]/*b*/, &vars[2]/*c*/, &vars[3]/*d*/);

		//problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/, &vars[1]/*b*/, &vars[2]/*c*/);

		//problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/, &vars[1]/*b*/);

		//problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/);
	}

	// Run the solver!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	//options.max_num_iterations = 1000;
	//options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//cout << summary.BriefReport() << endl;

	//get results!
	abcd = Vec4f(vars[0], vars[1], vars[2], vars[3]);
	//abc = Vec3f(vars[0], vars[1], vars[2]);
	//abc = Vec3f(0, vars[0], vars[1]);
	//abc = Vec3f(0, vars[0], 0);

	std::cout << "[SolveDepthToDepth2] abcd:" << abcd << " cost:" << summary.initial_cost << "->" << summary.final_cost << std::endl;

	return true;
}

bool DepthNamespace::SolveDepthToDepth(EquirectangularMap& emap, std::vector<PerspectiveMap>& pmaps,
	std::vector<bool>& pmaps_actives, Vec2f& zenith_range, Vec4f& abcd)
{
	//solve least square of y = c * (1 / (ax+b)) + d, for every sampled pixel of dispariy x to depth y

	//sampling along azimuth and zenith ranges
	const float subd_azi = D2R(1);
	const float subd_zen = D2R(1);

	double* vars = new double[4];  //a,b,c,d
	vars[0] = 1;
	vars[1] = 1;
	vars[2] = 1;
	vars[3] = 1;

	//double* vars = new double[3];  //a,b,c
	//vars[0] = 1;
	//vars[1] = 1;
	//vars[2] = 1;

	//double* vars = new double[2];  //a,b
	//vars[0] = 1;
	//vars[1] = 0;

	ceres::Problem problem;

	//one functor per sampled point

	//first count the total number of sampled points = # of functors
	int num_samples = 0;
	for (int p = 0; p < pmaps.size(); p++)
	{
		if (!pmaps_actives[p])
			continue;

		PerspectiveMap& pmap = pmaps[p];

		int cols = round(abs(pmap.ranges[1] - pmap.ranges[0]) / subd_azi);

		//the actual zenith range of this pmap:
		float zenith_top = MAX2(zenith_range[0], pmap.ranges[2]);
		float zenith_down = MIN2(zenith_range[1], pmap.ranges[3]);

		int rows = round(abs(zenith_down - zenith_top) / subd_zen);

		num_samples += (cols + 1) * (rows + 1);
	}

	//so to crate place holders
	ceres::CostFunction** functorD2Ds = new ceres::CostFunction * [num_samples];

	int index = 0;
	for (int p = 0; p < pmaps.size(); p++)
	{
		if (!pmaps_actives[p])
			continue;

		PerspectiveMap& pmap = pmaps[p];

		int cols = round(abs(pmap.ranges[1] - pmap.ranges[0]) / subd_azi);

		//the actual zenith range of this pmap:
		float zenith_top = MAX2(zenith_range[0], pmap.ranges[2]);
		float zenith_down = MIN2(zenith_range[1], pmap.ranges[3]);

		int rows = round(abs(zenith_down - zenith_top) / subd_zen);

		for (int r = 0; r <= rows; r++)
		{
			for (int c = 0; c <= cols; c++)
			{
				//find the spherical coord: (sampled uniformedly in a 2d rectangle in emap/spherical coords)
				Vec2f coord;
				coord.x = pmap.ranges[0] + (pmap.ranges[1] - pmap.ranges[0]) * (float)c / (float)cols;
				coord.y = zenith_top + (zenith_down - zenith_top) * (float)r / (float)rows;

				//the corresponding XY in this pmap's 2D screen?
				Vec2f xy = pmap.SphericalTo2D(coord.x, coord.y);

				//limit x and y to 0~1 range
				if (xy[0] < 0)
					xy[0] = 0;
				if (xy[0] > 1)
					xy[0] = 1;
				if (xy[1] < 0)
					xy[1] = 0;
				if (xy[1] > 1)
					xy[1] = 1;

				//value from the pmap:
				double depth0 = pmap.Value(xy.x, xy.y);

				if (depth0 < 1e-4)
					depth0 = 1e-4;
				else if (depth0 > (1 - 1e-4))
					depth0 = 1 - 1e-4;

				//depth value from the emap:
				double depth1 = emap.ValueAtCoord(coord.x, coord.y);

				if (depth1 < 1e-4)
					depth1 = 1e-4;
				else if (depth1 > (1 - 1e-4))
					depth1 = 1 - 1e-4;

				//weight?
				float weight = 1.0f;
				/*if (r == 0 || r == rows || c == 0 || c == cols)
					weight = 10.0f;*/

				//add a functor!

				//abcd:
				functorD2Ds[index] = new ceres::AutoDiffCostFunction<FunctorDepth2Depth3, 1/*residual size*/,
					1/*a*/, 1/*b*/, 1/*c*/, 1/*d*/>(new FunctorDepth2Depth3(weight, depth0, depth1));

				//abc:
				//functorD2Ds[index] = new ceres::AutoDiffCostFunction<FunctorDepth2Depth2, 1/*residual size*/,
				//	1/*a*/, 1/*b*/, 1/*c*/>(new FunctorDepth2Depth2(weight, depth0, depth1));

				//ab:
				//functorD2Ds[index] = new ceres::AutoDiffCostFunction < FunctorDepth2Depth1, 1/*residual size*/,
				//		1/*a*/, 1/*b*/>(new FunctorDepth2Depth1(weight, depth0, depth1));

				index++;
			}
		}
	}

	//add residual blocks (one per sample point):
	for (int i = 0; i < num_samples; i++)
	{
		problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/, &vars[1]/*b*/, &vars[2]/*c*/, &vars[3]/*d*/);
		//problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/, &vars[1]/*b*/, &vars[2]/*c*/);
		//problem.AddResidualBlock(functorD2Ds[i], NULL, &vars[0]/*a*/, &vars[1]/*b*/);
	}

	// Run the solver!
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::DENSE_SCHUR;
	//options.max_num_iterations = 1000;
	//options.minimizer_progress_to_stdout = true;
	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);
	//cout << summary.BriefReport() << endl;

	//get results!
	abcd = Vec4f(vars[0], vars[1], vars[2], vars[3]);
	//abcd = Vec4f(0, vars[0], vars[1], vars[2]);
	//abcd = Vec4f(0, 0, vars[0], vars[1]);

	//cout << "[SolveDepthToDepthPMaps] sol:" << abcd << " cost:" << summary.initial_cost << "->" << summary.final_cost << endl;
	return true;
}

bool DepthNamespace::SolveDepthAll(EquirectangularMap& emap, std::vector<PerspectiveMap>& pmaps,
	unsigned short* data, int& out_width, int& out_height, Vec2f& zenith_range, const char* Laplacian_filename)
{
	////multi-resolutions:
	int max_level = 3;
	//every level, divide width and height by 2
	//<=2048: level=3, 4096: level=4
	if (out_width >= 4096)
		max_level = 4;

	float* buffer_prev = NULL;  //buffer from the previous level
	for (int level = 0; level < max_level; level++)
	{
		int width = out_width / pow(2, max_level - 1 - level);
		int height = out_height / pow(2, max_level - 1 - level);
		int buffer_size = width * height;
		float* buffer = new float[buffer_size];

		//the valid height range according to zenith_range
		//NOTE: need to be dividable by the multi-resolutions
		int height0 = floor(height * zenith_range[0] / MYPI);
		int height1 = ceil(height * zenith_range[1] / MYPI);

		std::cout << "Level" << level << " width:" << width << " height:" << height << "(" << height0 << "~" << height1 << ") buffer_size:" << buffer_size << std::endl;

		//first level: copy 0~1 values of the given (baseline) global emap sampled in a out_width X out_height buffer
		if (level == 0)
		{
			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					//height out-of-bound: just write zero
					if (y<height0 || y>height1)
					{
						buffer[y * width + x] = 0;
					}
					else
					{
						//spherical coord. of this pixel:
						Vec2f coord((float)x / (float)(width - 1) * 2 * MYPI, (float)y / (float)(height - 1) * MYPI);

						//0~1 value from the equirectangular map:
						float depth_equi = emap.ValueAtCoord(coord[0], coord[1]);

						buffer[y * width + x] = depth_equi;
					}
				}
			}
		}
		//not first level: copy values from the previous buffer directly
		else
		{
			int width_prev = width / 2;

			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					int x_prev = x / 2;
					int y_prev = y / 2;

					buffer[y * width + x] = buffer_prev[y_prev * width_prev + x_prev];
				}
			}

			//can delete buffer_prev's memory now
			delete buffer_prev;
			buffer_prev = NULL;
		}

		std::vector<LaplacianWindow> Laplacians(buffer_size);

		float mask_center = 0;

		//build Laplacian masks for every pmap
#pragma omp parallel for
		for (int p = 0; p < pmaps.size(); p++)
		{
			PerspectiveMap& pmap = pmaps[p];

			//determine the bounding box (as x,y pixel coords) of this pamp in the global output buffer
			int x0 = round((pmap.ranges[0] / (2 * MYPI)) * (width - 1));
			int x1 = round((pmap.ranges[1] / (2 * MYPI)) * (width - 1));
			int y0 = round((pmap.ranges[2] / MYPI) * (height - 1));
			int y1 = round((pmap.ranges[3] / MYPI) * (height - 1));

			//NOTE: x0 not necessarily < x1 ! so we need to make sure "x_step" is +1 or -1
			int xs = 0;  //"x step"
			if (x1 >= x0)
				xs = 1;
			else
				xs = -1;

			//can assume y_step is always +1		
			int ys = 1;

			//test: enlarge a bit?
			if (true)
			{
				//int xs_enlarge = 4 / pow(level - 1, 2);
				int xs_enlarge = 4;
				if (level != max_level - 1)
					xs_enlarge = 0;

				//debug?
				xs_enlarge = 0;

				x0 = x0 - xs * xs_enlarge;
				//x0 = x0 - xs * 2;  //for Midas 5fold
				if (x0 < 0)
					x0 = 0;
				if (x0 >= width)
					x0 = width - 1;

				x1 = x1 + xs * xs_enlarge;
				//x1 = x1 + xs * 2;  //for Midas 5fold
				if (x1 < 0)
					x1 = 0;
				if (x1 >= width)
					x1 = width - 1;

				int ys_enlarge = 1;
				if (level != max_level - 1)
					ys_enlarge = 0;

				//debug?
				ys_enlarge = 0;

				y0 = y0 - ys * ys_enlarge;
				if (y0 < 0)
					y0 = 0;
				if (y0 >= height)
					y0 = height - 1;

				y1 = y1 + ys * ys_enlarge;
				if (y1 < 0)
					y1 = 0;
				if (y1 >= height)
					y1 = height - 1;
			}

			//note: must strictly within the zenith_range
			if (y0 <= height0)
				y0 = height0 + 1;
			if (y1 >= height1)
				y1 = height1 - 1;

			//for every pixel in this pmap's BB, inclusive of all boundary pixels		
			int X = x0;
			while (true)  //iterate X from x0 to x1, inclusive of both ends
			{
				for (int Y = y0; Y <= y1; Y += ys)
				{
					std::map<std::pair<int, int>, float> mask;
					//we assume all points within the pmap's range are interior pixels
					{
						//4-neighbors Laplacian mask:
						mask[std::make_pair(X, Y)] = 1;
						mask[std::make_pair(X - xs, Y)] = -0.25f;
						mask[std::make_pair(X + xs, Y)] = -0.25f;
						mask[std::make_pair(X, Y - ys)] = -0.25f;
						mask[std::make_pair(X, Y + ys)] = -0.25f;

						mask_center = mask[std::make_pair(X, Y)];
					}

					//calculate the "target" Laplacian value:
					float Laplacian = 0;
					for (std::map<std::pair<int, int>, float>::iterator itr = mask.begin(); itr != mask.end(); itr++)
					{
						int xx = (*itr).first.first;
						int yy = (*itr).first.second;

						//to spherical coord.
						Vec2f coord((float)xx / (float)(width - 1) * 2 * MYPI, (float)yy / (float)(height - 1) * MYPI);

						Vec2f xy = pmap.SphericalTo2D(coord.x, coord.y);  //to 2D 0~1 xy in this pmap

						if (xy.x < 0 || xy.x > 1 || xy.y < 0 || xy.y >1)
						{
							int X = xy.x * (float)(pmap.width - 1);
							int Y = xy.y * (float)(pmap.height - 1);

							std::cout << "[SolveDepthAll] oops! p#" << p << " range:" << R2D(pmap.ranges[0]) << "," << R2D(pmap.ranges[1]) << "," <<
								R2D(pmap.ranges[2]) << "," << R2D(pmap.ranges[3]) << " @azi:" << R2D(coord.x) << " zen:" << R2D(coord.y) << " xy:" << xy << " X:" << X << " Y:" << Y << std::endl;
						}

						float val = pmap.Value(xy[0], xy[1]);
						Laplacian += val * (*itr).second/*weight*/;
					}

					//accumulate this mask to the pixel's (global) Laplacian window
#pragma omp critical
					{
						LaplacianWindow& window = Laplacians[Y * width + X];
						for (std::map<std::pair<int, int>, float>::iterator itr = mask.begin(); itr != mask.end(); itr++)
						{
							window.mask[(*itr).first] += (*itr).second;
						}
						window.Laplacian += Laplacian;
					}
				}

				X += xs;  //X would be x0 to x1, inclusive
				if (X == x1)
					break;
			}
		}

		//normalize Laplacian windows so that center weight are the same
		for (int ii = 0; ii < Laplacians.size(); ii++)
		{
			int Y = ii / width;
			int X = ii % width;

			if (Y <= height0 || Y >= height1)
				continue;

			LaplacianWindow& window = Laplacians[Y * width + X];

			if (window.mask[std::make_pair(X, Y)] != 0 && window.mask[std::make_pair(X, Y)] != mask_center)
			{
				float scale = mask_center / window.mask[std::make_pair(X, Y)];
				for (std::map<std::pair<int, int>, float>::iterator itr = window.mask.begin(); itr != window.mask.end(); itr++)
				{
					//note: including the middle mask cell
					(*itr).second *= scale;
				}
				window.Laplacian *= scale;
			}
		}

		{
			const float step_size = 0.5;
			const float regularization_weight = 1e-4;
			const float regularization_weight_ = 1 - regularization_weight;

			int iterations = 500;

			if (max_level == 3)
			{
				if (level == 0)
					iterations = 200;
				else if (level == 1)
					iterations = 100;
				else if (level == 2)
					iterations = 50;
			}
			else if (max_level == 4)
			{
				if (level == 0)
					iterations = 200;
				else if (level == 1)
					iterations = 150;
				else if (level == 2)
					iterations = 100;
				else if (level == 3)
					iterations = 50;
			}

			//this is the amount of pixels we would iterate on:
			int indices_size = width * (height1 - height0 + 1);

			for (int iteration = 0; iteration < iterations; iteration++)
			{
				float* buffer_new = new float[width * height];
				memcpy(buffer_new, buffer, sizeof(float) * width * height);

				//update values by Laplacian masks
#pragma omp parallel for
				for (int ii = 0; ii < indices_size; ii++)
				{
					int Y = ii / width + height0;
					int X = ii % width;

					LaplacianWindow& w = Laplacians[Y * width + X];

					//current Laplacian value:
					float Laplacian_cur = 0;
					for (std::map<std::pair<int, int>, float>::iterator itr = w.mask.begin(); itr != w.mask.end(); itr++)
					{
						int xx = (*itr).first.first;
						int yy = (*itr).first.second;
						float val = buffer[yy * width + xx];
						Laplacian_cur += val * (*itr).second /*weight*/;
					}

					//update this pixel's value toward the baseline buffer
					float target_val = buffer[Y * width + X] + (w.Laplacian - Laplacian_cur) * step_size;
					buffer_new[Y * width + X] = target_val * regularization_weight_ + buffer[Y * width + X] * regularization_weight;

					//note: need to be within 0~1!
					if (buffer_new[Y * width + X] < 0)
						buffer_new[Y * width + X] = 0;
					else if (buffer_new[Y * width + X] > 1)
						buffer_new[Y * width + X] = 1;
				}

				memcpy(buffer, buffer_new, sizeof(float) * width * height);
				delete buffer_new;
			}
		}

		//last level: done! copy result to data buffer
		if (level == max_level - 1)
		{
			for (int y = 0; y < height; y++)
			{
				for (int x = 0; x < width; x++)
				{
					//note: cap floating value to 0~1
					float val = buffer[y * width + x];
					if (val < 0)
						val = 0;
					if (val > 1)
						val = 1;

					data[y * width + x] = (unsigned short)(val * 65535.0f);
				}
			}

			//test: check result buffer's Laplacians vs. target Laplacians
			if (false)
			{
				float err = 0;
				for (int y = height0 + 1; y <= height1 - 1; y++)
				{
					for (int x = 1; x < width - 1; x++)
					{
						float val = buffer[y * width + x];
						float val0 = buffer[y * width + x - 1];
						float val1 = buffer[y * width + x + 1];
						float val2 = buffer[(y - 1) * width + x];
						float val3 = buffer[(y + 1) * width + x];
						float Laplacian_cur = val - (val0 + val1 + val2 + val3) / 4.0f;

						LaplacianWindow& window = Laplacians[y * width + x];
						err += pow(Laplacian_cur - window.Laplacian, 2);
					}
				}
				std::cout << std::endl << "[SolveDepthAll] check Laplacian err:" << err << std::endl;
			}			

			delete buffer;
			buffer = NULL;
		}
		else
		{
			//set buffer_prev pointer to buffer
			buffer_prev = buffer;
		}
	}

	return true;
}

bool DepthNamespace::SolveDepthBySmoothing(std::vector<PerspectiveMap>& pmaps, unsigned short* data, int& width, int& height, Vec2f& zenith_range)
{
	int buffer_size = width * height;
	float* buffer = new float[buffer_size];
	memset(buffer, 0, sizeof(float) * buffer_size);

	//the valid height range according to zenith_range
	//NOTE: need to be dividable by the multi-resolutions
	int height0 = floor(height * zenith_range[0] / MYPI);
	int height1 = ceil(height * zenith_range[1] / MYPI);

	//a map of "to-smooth" pixels (close to pmap boundaries)
	bool* to_smooth_map = new bool[buffer_size];
	memset(to_smooth_map, false, sizeof(bool) * buffer_size);
	const int to_smooth_range = 10;

	//write pmaps' values into the common emap buffer
	for (int p = 0; p < pmaps.size(); p++)
	{
		PerspectiveMap& pmap = pmaps[p];

		//determine the bounding box (as x,y pixel coords) of this pamp in the global output buffer
		int x0 = round((pmap.ranges[0] / (2 * MYPI)) * (width - 1));
		int x1 = round((pmap.ranges[1] / (2 * MYPI)) * (width - 1));
		int y0 = round((pmap.ranges[2] / MYPI) * (height - 1));
		int y1 = round((pmap.ranges[3] / MYPI) * (height - 1));

		//NOTE: x0 not necessarily < x1 ! so we need to make sure "x_step" is +1 or -1
		int xs = 0;  //"x step"
		if (x1 >= x0)
			xs = 1;
		else
			xs = -1;

		//can assume y_step is always +1		
		int ys = 1;

		int X = x0;
		while (true)  //iterate X from x0 to x1, inclusive of both ends
		{
			for (int Y = y0; Y <= y1; Y += ys)
			{
				//(X,Y) is pixel coord. in emap

				//to spherical coord.
				Vec2f coord((float)X / (float)(width - 1) * 2 * MYPI, (float)Y / (float)(height - 1) * MYPI);

				Vec2f xy = pmap.SphericalTo2D(coord.x, coord.y);  //to 2D 0~1 xy in this pmap

				buffer[Y * width + X] = pmap.Value(xy[0], xy[1]);

				//within "to-smooth" range?
				if (abs(X - x0) <= to_smooth_range || abs(X - x1) <= to_smooth_range || abs(Y - y0) <= to_smooth_range || abs(Y - y1) <= to_smooth_range)
				{
					to_smooth_map[Y * width + X] = true;
				}
			}

			X += xs;  //X would be x0 to x1, inclusive
			if (X == x1)
				break;
		}
	}

	//do simple smoothing at to-smooth pixels
	for (int iter = 0; iter < 500; iter++)
	{
		for (int Y = height0; Y <= height1; Y++)
		{
			for (int X = 1; X < width - 1; X++)
			{
				if (!to_smooth_map[Y * width + X])
					continue;

				float val = buffer[Y * width + X];
				float val0 = buffer[Y * width + X - 1];
				float val1 = buffer[Y * width + X + 1];
				float val2 = buffer[(Y - 1) * width + X];
				float val3 = buffer[(Y + 1) * width + X];
				float avg = (val0 + val1 + val2 + val3) / 4;
				buffer[Y * width + X] = val + 0.5 * (avg - val);
			}
		}
	}

	//write back the smoothed result
	for (int y = 0; y < height; y++)
	{
		for (int x = 0; x < width; x++)
		{
			//note: cap floating value to 0~1
			float val = buffer[y * width + x];
			if (val < 0)
				val = 0;
			if (val > 1)
				val = 1;

			data[y * width + x] = (unsigned short)(val * 65535.0f);
		}
	}

	delete buffer;
	delete to_smooth_map;

	return true;
}

//functor for solving base Laplacian weights (a,b)
struct FunctorLaplacianWeight2
{
	FunctorLaplacianWeight2(double gt, double base, double pmap, double dist_weight) :
		GT(gt), Base(base), Pmap(pmap), DistWeight(dist_weight) {}

	//"a" is the pmap Laplacian weight min, "b" is the pmap Laplacian weight max
	//actual pmap weight here, pmap_weight, is (a * (1-dist_weight) + b * (dist_weight))
	//GT = (1 - pmap_weight) * Base + (pmap_weight) * Pmap
	template <typename T> bool operator()(const T* const a, const T* const b, T* residual) const
	{
		residual[0] = T(GT) - (T(1) - a[0] * (T(1) - T(DistWeight)) - b[0] * (T(DistWeight))) * T(Base) -
			(a[0] * (T(1) - T(DistWeight)) + b[0] * (T(DistWeight))) * T(Pmap);

		return true;
	}

private:
	double GT;  //gt Laplacian
	double Base;  //baseline Laplacian
	double Pmap;  //Laplacian from pmaps
	double DistWeight;  //dist-based weight (0~1 farther the higher) for pmap_Laplacian_weight_min to max
};

//functor for solving base Laplacian weights 0~(a)
struct FunctorLaplacianWeight0a
{
	FunctorLaplacianWeight0a(double gt, double base, double pmap, double dist_weight) :
		GT(gt), Base(base), Pmap(pmap), DistWeight(dist_weight) {}

	//0 is the pmap Laplacian weight min, "a" is the pmap Laplacian weight max
	//actual pmap weight here, pmap_weight, is (a * (1-dist_weight) + b * (dist_weight))
	//GT = (1 - pmap_weight) * Base + (pmap_weight) * Pmap
	template <typename T> bool operator()(const T* const a, T* residual) const
	{
		residual[0] = T(GT) - (T(1) - a[0] * T(DistWeight)) * T(Base) -
			(a[0] * T(DistWeight)) * T(Pmap);

		return true;
	}

private:
	double GT;  //gt Laplacian
	double Base;  //baseline Laplacian
	double Pmap;  //Laplacian from pmaps
	double DistWeight;  //dist-based weight (0~1 farther the higher) for pmap_Laplacian_weight_min to max
};

//functor for solving base Laplacian weights a~1
struct FunctorLaplacianWeighta1
{
	FunctorLaplacianWeighta1(double gt, double base, double pmap, double dist_weight) :
		GT(gt), Base(base), Pmap(pmap), DistWeight(dist_weight) {}

	//"a" is the pmap Laplacian weight min, 1 is the pmap Laplacian weight max
	//actual pmap weight here, pmap_weight, is (a * (1-dist_weight) + 1 * (dist_weight))
	//GT = (1 - pmap_weight) * Base + (pmap_weight) * Pmap
	template <typename T> bool operator()(const T* const a, T* residual) const
	{
		residual[0] = T(GT) - (T(1) - a[0] * (T(1) - T(DistWeight)) - (T(DistWeight))) * T(Base) -
			(a[0] * (T(1) - T(DistWeight)) + (T(DistWeight))) * T(Pmap);

		return true;
	}

private:
	double GT;  //gt Laplacian
	double Base;  //baseline Laplacian
	double Pmap;  //Laplacian from pmaps
	double DistWeight;  //dist-based weight (0~1 farther the higher) for pmap_Laplacian_weight_min to max
};

//functor for solving base Laplacian weights (a,b)
struct FunctorLaplacianWeight4
{
	FunctorLaplacianWeight4(double gt, double base, double pmap, double dist_weight, double base_depth, double pmap_depth) :
		GT(gt), Base(base), Pmap(pmap), DistWeight(dist_weight), BaseDepth(base_depth), PmapDepth(pmap_depth) {}

	//"a" is the pmap Laplacian weight min, "b" is the pmap Laplacian weight max
	//"p" is weight for baseline depth, "q" is eight for pmap depth
	//actual pmap weight here, pmap_weight, is (a * (1-dist_weight) + b * (dist_weight))
	//GT = (1 - pmap_weight) * Base + (pmap_weight) * Pmap + p * BaseDepth + q * PmapDepth
	template <typename T> bool operator()(const T* const a, const T* const b, const T* const p, const T* const q,
		T* residual) const
	{
		residual[0] = T(GT) - (T(1) - a[0] * (T(1) - T(DistWeight)) - b[0] * (T(DistWeight))) * T(Base) -
			(a[0] * (T(1) - T(DistWeight)) + b[0] * (T(DistWeight))) * T(Pmap) - p[0] * T(BaseDepth) - q[0] * T(PmapDepth);

		return true;
	}

private:
	double GT;  //gt Laplacian
	double Base;  //baseline Laplacian
	double Pmap;  //Laplacian from pmaps
	double DistWeight;  //dist-based weight (0~1 farther the higher) for pmap_Laplacian_weight_min to max
	double BaseDepth;  //baseline depth
	double PmapDepth;  //pmap depth
};

bool DepthNamespace::ErrorData(EquirectangularMap& emap_gt, unsigned short* data, int data_width, int data_height, float& mse, float& mae, float& mre,
	float &mselog, float& delta1, float& delta2, float& delta3, int align_way, bool cap_depth, Vec2f* least_square_shift, float* median_shift_factor_out)
{
	int height0 = (int)(g_zenith_range[0] / MYPI * data_height);
	int height1 = (int)(g_zenith_range[1] / MYPI * data_height);

	//we will check every pixel in emap0 w.r.t. the corresponding pixel at emap1 (at same (0~1) positions)
	float ratio_x = (float)emap_gt.width / (float)data_width;
	float ratio_y = (float)emap_gt.height / (float)data_height;

	mse = 0;
	mae = 0;
	mre = 0;
	mselog = 0;
	delta1 = 0;
	delta2 = 0;
	delta3 = 0;

	int delta1_fail = 0, delta2_fail = 0, delta3_fail = 0;

	//max depth value?
	const float to_Matterport = 65535.0f / 4000.0f;  //0~1 to meter (0~65535 divide by 4000 to get meter)
	float depth_max = 10.0f / to_Matterport;  //back to 0~1 scale

	float median_shift_factor = 1;
	//least square ax+b align?
	Vec2f least_square(0);  //{s,o}

	//if aligning, we scale the given data by factor of medians
	if (align_way == 1)
	{
		float gt_median = 0;
		float given_median = 0;

		//find the medians of gt and given
		std::list<float> gt_sorted;
		std::list<float> given_sorted;
		for (int y = 0; y < data_height; y++)
		{
			for (int x = 0; x < data_width; x++)
			{
				if (y<height0 || y>height1)
					continue;

				//given (x,y) in emap0, the corresponding (X,Y) in emap1?
				int X = (float)x * ratio_x;
				int Y = (float)y * ratio_y;

				float val0 = emap_gt.ValueAtXY(X, Y);  //gt value, 0~1
				float val1 = (float)data[y * data_width + x] / 65535.0f; //to 0~1

				//skip masked pixels
				if (val0 < 1e-4)
					continue;

				//cap depth?
				if (cap_depth)
				{
					val0 = MIN2(val0, depth_max);
					val1 = MIN2(val1, depth_max);
				}

				gt_sorted.push_back(val0);
				given_sorted.push_back(val1);
			}
		}
		gt_sorted.sort();
		given_sorted.sort();
		//get medians:
		{
			int count = 0;
			for (std::list<float> ::iterator itr = gt_sorted.begin(); itr != gt_sorted.end(); itr++)
			{
				if (count == gt_sorted.size() / 2)
				{
					//gotcha!
					gt_median = *itr;
					break;
				}
				count++;
			}
		}
		{
			int count = 0;
			for (std::list<float> ::iterator itr = given_sorted.begin(); itr != given_sorted.end(); itr++)
			{
				if (count == given_sorted.size() / 2)
				{
					//gotcha!
					given_median = *itr;
					break;
				}
				count++;
			}
		}

		median_shift_factor = gt_median / given_median;
		//cout << "[Error_data] ratio:" << ratio_x << "," << ratio_y << " gt_sorted:" << gt_sorted.size() << " gt_median:" << gt_median << " given_median:" << given_median << " median_shift_factor:" << median_shift_factor << endl;

		if (median_shift_factor_out)
			*median_shift_factor_out = median_shift_factor;
	}
	//least square ax+b align?
	else if (align_way == 2)
	{
		//https://github.com/manurare/360monodepth/blob/main/code/python/src/utility/metrics.py
		// pred2gt_least_squares

		float a_00 = 0;
		float a_01 = 0;
		float a_11 = 0;
		float b_0 = 0;
		float b_1 = 0;

		for (int y = 0; y < data_height; y++)
		{
			for (int x = 0; x < data_width; x++)
			{
				if (y<height0 || y>height1)
					continue;

				//given (x,y) in emap0, the corresponding (X,Y) in emap1?
				int X = (float)x * ratio_x;
				int Y = (float)y * ratio_y;

				float val0 = emap_gt.ValueAtXY(X, Y);  //gt value, 0~1
				float val1 = (float)data[y * data_width + x] / 65535.0f; //to 0~1

				//skip masked pixels
				if (val0 < 1e-4)
					continue;

				//cap depth?
				if (cap_depth)
				{
					val0 = MIN2(val0, depth_max);
					val1 = MIN2(val1, depth_max);
				}

				a_00 += val1 * val1;
				a_01 += val1;
				a_11 += 1;
				b_0 += val0 * val1;
				b_1 += val0;
			}
		}

		float det = a_00 * a_11 - a_01 * a_01;
		if (det == 0)
			std::cout << "[ErrorData] det=0?? a00:" << a_00 << " a01:" << a_01 << "a11:" << a_11 << "b0:" << b_0 << " b_1:" << b_1 << std::endl;

		float s = (a_11 * b_0 - a_01 * b_1) / det;
		float o = (-a_01 * b_0 + a_00 * b_1) / det;

		least_square = Vec2f(s, o);

		std::cout << "[ErrorData] a00:" << a_00 << " a01:" << a_01 << "a11:" << a_11 << "b0:" << b_0 << " b_1:" << b_1 << " least_square:" << least_square << std::endl;

		//pred = s * pred + o
	}

	int num_compares = 0;
	int num_log_compares = 0;
	for (int y = 0; y < data_height; y++)
	{
		for (int x = 0; x < data_width; x++)
		{
			if (y<height0 || y>height1)
				continue;

			//given (x,y) in emap0, the corresponding (X,Y) in emap1?
			int X = (float)x * ratio_x;
			int Y = (float)y * ratio_y;

			float val0 = emap_gt.ValueAtXY(X, Y);  //0~1
			float val1 = (float)data[y * data_width + x] / 65535.0f;

			//skip masked pixels
			if (val0 < 1e-4)
				continue;

			//cap depth?
			if (cap_depth)
			{
				val0 = MIN2(val0, depth_max);
				val1 = MIN2(val1, depth_max);
			}

			//align?
			if (align_way == 1)
			{
				val1 *= median_shift_factor;
			}
			else if (align_way == 2)
			{
				val1 = val1 * least_square[0] + least_square[1];
			}

			mse += pow(val0 - val1, 2);
			mae += abs(val0 - val1);
			mre += abs(val0 - val1) / val0;

			if (val0 > 1e-4 && val1 > 1e-4)
			{
				mselog += pow(log10(val0) - log10(val1), 2);
				num_log_compares++;
			}

			float ratio_max = 0;
			if (val0 > 0 && val1 > 0)
			{
				float ratio01 = val0 / val1;
				float ratio10 = val1 / val0;
				ratio_max = MAX2(ratio01, ratio10);

				if (ratio_max >= 1.25)
					delta1_fail++;
				if (ratio_max >= pow(1.25, 2))
					delta2_fail++;
				if (ratio_max >= pow(1.25, 3))
					delta3_fail++;
			}

			num_compares++;
		}
	}

	mse /= (float)num_compares;
	mae /= (float)num_compares;
	mre /= (float)num_compares;
	mselog /= (float)num_log_compares;
	delta1 = (float)(num_compares - delta1_fail) / (float)num_compares;
	delta2 = (float)(num_compares - delta2_fail) / (float)num_compares;
	delta3 = (float)(num_compares - delta3_fail) / (float)num_compares;

	return true;
}
bool DepthNamespace::ErrorEmap(EquirectangularMap& emap_gt, EquirectangularMap& emap_given,
	float& mse, float& mae, float& mre, float &mselog, float& delta1, float& delta2, float& delta3, int align_way, bool cap_depth,
	Vec2f* least_square_shift, float* median_shift_factor_out)
{
 	//result emap's height range
	int height0 = (int)(g_zenith_range[0] / MYPI * emap_given.height);
	int height1 = (int)(g_zenith_range[1] / MYPI * emap_given.height);

	//we will check every pixel in emap0 w.r.t. the corresponding pixel at emap1 (at same (0~1) positions)
	float ratio_x = (float)emap_gt.width / (float)emap_given.width;
	float ratio_y = (float)emap_gt.height / (float)emap_given.height;

	mse = 0;
	mae = 0;
	mre = 0;
	mselog = 0;
	delta1 = 0;
	delta2 = 0;
	delta3 = 0;

	int delta1_fail = 0, delta2_fail = 0, delta3_fail = 0;

	//max depth value?
	const float to_Matterport = 65535.0f / 4000.0f;  //0~1 to meter (0~65535 divide by 4000 to get meter)
	float depth_max = 10.0f / to_Matterport;  //back to 0~1 scale

	//if median shift, we scale the given data by factor of medians
	float median_shift_factor = 1;
	//least square ax+b align?
	Vec2f least_square(0);  //{s,o}
	if (align_way == 1)
	{
		float gt_median = 0, given_median = 0;

		//find the median of gt and given
		std::list<float> gt_sorted;
		std::list<float> given_sorted;
		for (int y = 0; y < emap_given.height; y++)
		{
			for (int x = 0; x < emap_given.width; x++)
			{
				if (y<height0 || y>height1)
					continue;

				//given (x,y) in emap0, the corresponding (X,Y) in emap1?
				int X = (float)x * ratio_x;
				int Y = (float)y * ratio_y;

				float val0 = emap_gt.ValueAtXY(X, Y);  //0~1
				float val1 = emap_given.ValueAtXY(x, y);  //0~1

				//skip masked pixels
				if (abs(val0) < 1e-4)
					continue;

				//cap depth?
				if (cap_depth)
				{
					val0 = MIN2(val0, depth_max);
					val1 = MIN2(val1, depth_max);
				}

				gt_sorted.push_back(val0);
				given_sorted.push_back(val1);
			}
		}
		gt_sorted.sort();
		given_sorted.sort();
		//get medians:
		{
			int count = 0;
			for (std::list<float> ::iterator itr = gt_sorted.begin(); itr != gt_sorted.end(); itr++)
			{
				if (count == gt_sorted.size() / 2)
				{
					//gotcha!
					gt_median = *itr;
					break;
				}
				count++;
			}
		}
		{
			int count = 0;
			for (std::list<float> ::iterator itr = given_sorted.begin(); itr != given_sorted.end(); itr++)
			{
				if (count == given_sorted.size() / 2)
				{
					//gotcha!
					given_median = *itr;
					break;
				}
				count++;
			}
		}

		median_shift_factor = gt_median / given_median;

		std::cout << "[ErrorEmap] ratio:" << ratio_x << "," << ratio_y << " gt_sorted:" << gt_sorted.size() << " gt_median:" << gt_median << " given_median:" << given_median << " median_shift_factor:" << median_shift_factor << std::endl;

		if (median_shift_factor_out)
		{
			*median_shift_factor_out = median_shift_factor;
		}
	}
	//least square ax+b align?
	else if (align_way == 2)
	{
		//https://github.com/manurare/360monodepth/blob/main/code/python/src/utility/metrics.py
		// pred2gt_least_squares
		
		float a_00 = 0;
		float a_01 = 0;
		float a_11 = 0;
		float b_0 = 0;
		float b_1 = 0;

		for (int y = 0; y < emap_given.height; y++)
		{
			for (int x = 0; x < emap_given.width; x++)
			{
				if (y<height0 || y>height1)
					continue;

				//given (x,y) in emap_given, the corresponding (X,Y) in emap_gt?
				int X = (float)x * ratio_x;
				int Y = (float)y * ratio_y;

				float val0 = emap_gt.ValueAtXY(X, Y);  //gt
				float val1 = emap_given.ValueAtXY(x, y);  //given

				//skip masked pixels
				if (val0 < 1e-4)
					continue;

				//cap depth?
				if (cap_depth)
				{
					val0 = MIN2(val0, depth_max);
					val1 = MIN2(val1, depth_max);
				}

				a_00 += val1 * val1;
				a_01 += val1;
				a_11 += 1;
				b_0 += val0 * val1;
				b_1 += val0;
			}
		}

		float det = a_00 * a_11 - a_01 * a_01;
		if (det == 0)
			std::cout << "[ErrorEmap] det=0?? a00:" << a_00 << " a01:" << a_01 << " a11:" << a_11 << " b0:" << b_0 << " b_1:" << b_1 << std::endl;


		float s = (a_11 * b_0 - a_01 * b_1) / det;
		float o = (-a_01 * b_0 + a_00 * b_1) / det;

		least_square = Vec2f(s, o);

		if (least_square_shift)
			*least_square_shift = least_square;

		std::cout << "[ErrorEmap] a00:" << a_00 << " a01:" << a_01 << "a11:" << a_11 << "b0:" << b_0 << " b_1:" << b_1 << " least_square:" << least_square << std::endl;
	}

	int num_compares = 0;
	int num_log_compares = 0;
	for (int y = 0; y < emap_given.height; y++)
	{
		for (int x = 0; x < emap_given.width; x++)
		{
			if (y<height0 || y>height1)
				continue;

			//given (x,y) in emap0, the corresponding (X,Y) in emap1?
			int X = (float)x * ratio_x;
			int Y = (float)y * ratio_y;

			float val0 = emap_gt.ValueAtXY(X, Y);  //0~1
			float val1 = emap_given.ValueAtXY(x, y);  //0~1

			//skip masked pixels
			if (val0 < 1e-4)
				continue;

			//cap depth?
			if (cap_depth)
			{
				val0 = MIN2(val0, depth_max);
				val1 = MIN2(val1, depth_max);
			}

			//median shift?
			if (align_way == 1)
			{
				val1 *= median_shift_factor;
			}
			//least square align?
			else if (align_way == 2)
			{
				val1 = val1 * least_square[0] + least_square[1];
			}

			mse += pow(val0 - val1, 2);
			mae += abs(val0 - val1);
			mre += abs(val0 - val1) / val0;
			
			if (val0 > 1e-4 && val1 > 1e-4)
			{
				mselog += pow(log10(val0) - log10(val1), 2);
				num_log_compares++;
			}

			if (val0 > 0 && val1 > 0)
			{
				float ratio01 = val0 / val1;
				float ratio10 = val1 / val0;
				float ratio_max = MAX2(ratio01, ratio10);

				if (ratio_max >= 1.25)
					delta1_fail++;
				if (ratio_max >= pow(1.25, 2))
					delta2_fail++;
				if (ratio_max >= pow(1.25, 3))
					delta3_fail++;
			}

			num_compares++;
		}
	}

	mse /= (float)(num_compares);
	mae /= (float)(num_compares);
	mre /= (float)(num_compares);
	mselog /= (float)num_log_compares;
	delta1 = (float)(num_compares - delta1_fail) / (float)num_compares;
	delta2 = (float)(num_compares - delta2_fail) / (float)num_compares;
	delta3 = (float)(num_compares - delta3_fail) / (float)num_compares;

	return true;
}

bool DepthNamespace::ErrorCompare(std::string& gt_filename, std::string& baseline_filename, bool DispDepthCompare, float& mse, float& mae, float& mre,
	float &mselog, float& delta1, float& delta2, float& delta3, int align_way, bool cap_depth, const char* shifted_filename)
{
	EquirectangularMap emap_gt;
	if (!emap_gt.Load(gt_filename))
	{
		std::cout << "cannot load emap_gt? " << gt_filename << std::endl;
		return false;
	}

	EquirectangularMap emap_baseline;
	if (!emap_baseline.Load(baseline_filename, true/*mono360*/))
	{
		std::cout << "cannot load emap_baseline? " << baseline_filename << std::endl;
		return false;
	}

	//(mono360) disp-depth compare: 
	if (DispDepthCompare)
	{
		//first convert the gt depth map to disparity map
		EquirectangularMap emap_gt_disp;  //use a temp copy
		if (!emap_gt_disp.Load(gt_filename))
		{
			std::cout << "cannot load emap_gt? " << gt_filename << std::endl;
			return false;
		}
		emap_gt_disp.DispDepthConversion();

		//compute the least square shift:
		float median_shift_factor = 0;
		Vec2f least_square;
		ErrorEmap(emap_gt_disp, emap_baseline, mse, mae, mre, mselog, delta1, delta2, delta3, 2/*least square align*/, false, &least_square, &median_shift_factor);

		//transform the baseline (disp) emap by the least square shift:
		for (int x = 0; x < emap_baseline.width; x++)
		{
			for (int y = 0; y < emap_baseline.height; y++)
			{
				float val = emap_baseline.ValueAtXY(x, y);

				//if (abs(val) > 1e-4)
				{
					val = val * least_square[0] + least_square[1];
				}

				emap_baseline.data[(y * emap_baseline.width + x) * emap_baseline.channels] = val;				
			}
		}

		//then convert the baseline disp emap to depth emap
		emap_baseline.DispDepthConversion();

		//clip the baseline depth value to 0 ~ max_depth (10):
		float max_depth = 10;
		for (int x = 0; x < emap_baseline.width; x++)
		{
			for (int y = 0; y < emap_baseline.height; y++)
			{
				float val = emap_baseline.ValueAtXY(x, y);
				if (val < 0)
					val = 0;
				if (val > max_depth)
					val = max_depth;

				emap_baseline.data[(y * emap_baseline.width + x) * emap_baseline.channels] = val;
			}
		}

		//actually calculate the error metrics: (depth vs depth)
		ErrorEmap(emap_gt, emap_baseline, mse, mae, mre, mselog, delta1, delta2, delta3, align_way, cap_depth, &least_square, &median_shift_factor);

		//save the baseline depth emap!
		{
			//map the baseline depth map to 0~1
			if (true)
			{
				Vec2f minmax(FLT_MAX, -FLT_MAX);
				for (int x = 0; x < emap_baseline.width; x++)
				{
					for (int y = 0; y < emap_baseline.height; y++)
					{
						float val = emap_baseline.ValueAtXY(x, y);
						if (abs(val) < 1e-4)
							continue;

						if (val < minmax[0])
							minmax[0] = val;
						if (val > minmax[1])
							minmax[1] = val;
					}
				}
				std::cout << "DispDepthCompare emap_baseline minmax:" << minmax << std::endl;
				for (int x = 0; x < emap_baseline.width; x++)
				{
					for (int y = 0; y < emap_baseline.height; y++)
					{
						float val = emap_baseline.ValueAtXY(x, y);
						if (abs(val) < 1e-4)  //black keep black
							continue;

						val = (val - minmax[0]) / (minmax[1] - minmax[0]);

						emap_baseline.data[(y * emap_baseline.width + x) * emap_baseline.channels] = val;
					}
				}
			}

			/*unsigned short* data = new unsigned short[emap_baseline.width * emap_baseline.height];
			for (int x = 0; x < emap_baseline.width; x++)
			{
				for (int y = 0; y < emap_baseline.height; y++)
				{
					float val = emap_baseline.ValueAtXY(x, y);

					if (val < 0)
						val = 0;

					data[y * emap_baseline.width + x] = (unsigned short)(val * 65535.0f);
				}
			}
			Save16BitPNG(data, emap_baseline.width, emap_baseline.height, shifted_filename);*/

			unsigned char* data = new unsigned char[emap_baseline.width * emap_baseline.height];
			for (int x = 0; x < emap_baseline.width; x++)
			{
				for (int y = 0; y < emap_baseline.height; y++)
				{
					float val = emap_baseline.ValueAtXY(x, y);

					if (val < 0)
						val = 0;

					data[y * emap_baseline.width + x] = (unsigned char)(val * 255.0f);
				}
			}
			int ret = stbi_write_png(shifted_filename, emap_baseline.width, emap_baseline.height, 1, data, emap_baseline.width * 1/*stride in bytes*/);


			std::cout << "[ErrorCompare] " << ret << " saved shifted emap:" << shifted_filename << std::endl;
			delete data;
		}
		
	}
	else  //default way:
	{
		float median_shift_factor = 0;
		Vec2f least_square;
		ErrorEmap(emap_gt, emap_baseline, mse, mae, mre, mselog, delta1, delta2, delta3, align_way, cap_depth, &least_square, &median_shift_factor);

		//save shifted baseline emap?
		if (shifted_filename)
		{
			unsigned char* data = new unsigned char[emap_baseline.width * emap_baseline.height];
			for (int x = 0; x < emap_baseline.width; x++)
			{
				for (int y = 0; y < emap_baseline.height; y++)
				{
					float val = emap_baseline.ValueAtXY(x, y);

					if (val < 0)
						val = 0;

					data[y * emap_baseline.width + x] = (unsigned char)(val * 255.0f);
				}
			}
			int ret = stbi_write_png(shifted_filename, emap_baseline.width, emap_baseline.height, 1, data, emap_baseline.width * 1/*stride in bytes*/);
			
			std::cout << "[ErrorCompare] " << ret << " saved shifted emap:" << shifted_filename << std::endl;
			delete data;
		}
	}

	return true;
}

bool DepthNamespace::ErrorLaplacian(std::string& gt_filename, std::string& baseline_filename, double& Laplacian_mse, double& Laplacian_mae,
	double& SobelX_mae, double& SobelY_mae, double& Laplacian5x5_mae,
	std::string* gt_Laplacian_fn, std::string* baseline_Laplacian_fn, std::string* Laplacian_ae_fn)
{
	EquirectangularMap emap_gt;
	if (!emap_gt.Load(gt_filename))
	{
		std::cout << "cannot load emap_gt? " << gt_filename << std::endl;
		return false;
	}

	EquirectangularMap emap_baseline;
	if (!emap_baseline.Load(baseline_filename))
	{
		std::cout << "cannot load emap_baseline? " << baseline_filename << std::endl;
		return false;
	}

	float ratio_x = (float)emap_gt.width / (float)emap_baseline.width;
	float ratio_y = (float)emap_gt.height / (float)emap_baseline.height;

	int num_Laplacian_samples = 0;
	int num_Sobel_samples = 0;
	Laplacian_mse = 0;
	Laplacian_mae = 0;
	SobelX_mae = 0;
	SobelY_mae = 0;
	Laplacian5x5_mae = 0;

	//data buffers for output?
	double* data_gt_Laplacian = NULL;
	if (gt_Laplacian_fn)
	{
		data_gt_Laplacian = new double[emap_baseline.width * emap_baseline.height];
		memset(data_gt_Laplacian, 0, sizeof(double) * emap_baseline.width * emap_baseline.height);
	}
	double* data_baseline_Laplacian = NULL;
	if (baseline_Laplacian_fn)
	{
		data_baseline_Laplacian = new double[emap_baseline.width * emap_baseline.height];
		memset(data_baseline_Laplacian, 0, sizeof(double) * emap_baseline.width * emap_baseline.height);
	}
	double* data_Laplacian_ae = NULL;
	if (Laplacian_ae_fn)
	{
		data_Laplacian_ae = new double[emap_baseline.width * emap_baseline.height];
		memset(data_Laplacian_ae, 0, sizeof(double) * emap_baseline.width * emap_baseline.height);
	}

	//only strictly inner pixels have Laplacians
	//(x,y) for baseline
	for (int x = 1; x < emap_baseline.width - 1; x++)
	{
		for (int y = 1; y < emap_baseline.height - 1; y++)
		{
			//(X,Y) for gt
			int X = x * ratio_x;
			int X0 = (x - 1) * ratio_x;
			int X1 = (x + 1) * ratio_x;
			int Y = y * ratio_y;
			int Y0 = (y - 1) * ratio_y;
			int Y1 = (y + 1) * ratio_y;
			if (X0 < 0 || X1 > emap_gt.width - 1 || Y0 < 0 || Y1 > emap_gt.height - 1)
				continue;

			//values in a 3x3 (0~2 x 0~2) mask, gt:
			double val[3][3];  //(x,y)
			val[0][0] = emap_gt.ValueAtXY(X0, Y0);
			val[1][0] = emap_gt.ValueAtXY(X, Y0);
			val[2][0] = emap_gt.ValueAtXY(X1, Y0);
			val[0][1] = emap_gt.ValueAtXY(X0, Y);
			val[1][1] = emap_gt.ValueAtXY(X, Y);  //center
			val[2][1] = emap_gt.ValueAtXY(X1, Y);
			val[0][2] = emap_gt.ValueAtXY(X0, Y1);
			val[1][2] = emap_gt.ValueAtXY(X, Y1);
			val[2][2] = emap_gt.ValueAtXY(X1, Y1);

			////calculate Laplacian:

			//a Laplacian value is valid only if all gt pixel values in the window are valid (not black)
			if (val[1][1] < 1e-4 || val[0][1] < 1e-4 || val[2][1] < 1e-4 || val[1][0] < 1e-4 || val[1][2] < 1e-4)
			{
			}
			else
			{
				//calculate Laplacian of gt and baseline here:

				double gt_Laplacian = 1 * val[1][1] - (val[0][1] + val[2][1] + val[1][0] + val[1][2]) / 4;

				double val_Baseline = emap_baseline.ValueAtXY(x, y);
				double val0_Baseline = emap_baseline.ValueAtXY(x - 1, y);
				double val1_Baseline = emap_baseline.ValueAtXY(x + 1, y);
				double val2_Baseline = emap_baseline.ValueAtXY(x, y - 1);
				double val3_Baseline = emap_baseline.ValueAtXY(x, y + 1);
				double baseline_Laplacian = 1 * val_Baseline - (val0_Baseline + val1_Baseline + val2_Baseline + val3_Baseline) / 4;

				Laplacian_mse += pow(gt_Laplacian - baseline_Laplacian, 2);
				Laplacian_mae += abs(gt_Laplacian - baseline_Laplacian);

				num_Laplacian_samples++;

				if (data_gt_Laplacian)
					data_gt_Laplacian[y * emap_baseline.width + x] = gt_Laplacian;
				if (data_baseline_Laplacian)
					data_baseline_Laplacian[y * emap_baseline.width + x] = baseline_Laplacian;
				if (data_Laplacian_ae)
					data_Laplacian_ae[y * emap_baseline.width + x] = abs(gt_Laplacian - baseline_Laplacian);
			}

			////calculate sobel x and y:

			if (val[0][0] < 1e-4 || val[0][1] < 1e-4 || val[0][2] < 1e-4 || val[0][1] < 1e-4 || val[1][1] < 1e-4 || val[2][1] < 1e-4 ||
				val[0][2] < 1e-4 || val[1][2] < 1e-4 || val[2][2] < 1e-4)
			{
			}
			else
			{
				double gt_SobelX = val[0][0] - val[2][0] + 2 * val[0][1] - 2 * val[2][1] + val[0][2] - val[2][2];
				double gt_SobelY = val[0][0] + 2 * val[1][0] + val[2][0] - val[0][2] - 2 * val[1][2] - val[2][2];

				//baseline values:
				double Val[3][3];  //(x,y)
				Val[0][0] = emap_baseline.ValueAtXY(x - 1, y - 1);
				Val[1][0] = emap_baseline.ValueAtXY(x, y - 1);
				Val[2][0] = emap_baseline.ValueAtXY(x + 1, y - 1);
				Val[0][1] = emap_baseline.ValueAtXY(x - 1, y);
				Val[1][1] = emap_baseline.ValueAtXY(x, y);
				Val[2][1] = emap_baseline.ValueAtXY(x + 1, y);
				Val[0][2] = emap_baseline.ValueAtXY(x - 1, y + 1);
				Val[1][2] = emap_baseline.ValueAtXY(x, y + 1);
				Val[2][2] = emap_baseline.ValueAtXY(x + 1, y + 1);

				double baseline_SobelX = Val[0][0] - Val[2][0] + 2 * Val[0][1] - 2 * Val[2][1] + Val[0][2] - Val[2][2];
				double baseline_SobelY = Val[0][0] + 2 * Val[1][0] + Val[2][0] - Val[0][2] - 2 * Val[1][2] - Val[2][2];

				SobelX_mae += abs(gt_SobelX - baseline_SobelX);
				SobelY_mae += abs(gt_SobelY - baseline_SobelY);

				num_Sobel_samples++;
			}
		}
	}

	//write Lalpacians to files?
	//if (data_gt_Laplacian && data_baseline_Laplacian)
	//{
	//	//let's use gt or baseline's max Laplacian value as value range!
	//	Vec2f Laplacian_range(FLT_MAX, -FLT_MAX);
	//	for (int x = 0; x < emap_baseline.width; x++)
	//	{
	//		for (int y = 0; y < emap_baseline.height; y++)
	//		{
	//			if (data_gt_Laplacian[y * emap_baseline.width + x] < Laplacian_range[0])
	//				Laplacian_range[0] = data_gt_Laplacian[y * emap_baseline.width + x];
	//			if (data_gt_Laplacian[y * emap_baseline.width + x] > Laplacian_range[1])
	//				Laplacian_range[1] = data_gt_Laplacian[y * emap_baseline.width + x];

	//			if (data_baseline_Laplacian[y * emap_baseline.width + x] < Laplacian_range[0])
	//				Laplacian_range[0] = data_baseline_Laplacian[y * emap_baseline.width + x];
	//			if (data_baseline_Laplacian[y * emap_baseline.width + x] > Laplacian_range[1])
	//				Laplacian_range[1] = data_baseline_Laplacian[y * emap_baseline.width + x];
	//		}
	//	}
	//	
	//	//prepare the 16bit output buffers
	//	unsigned short* data_gt = new unsigned short[emap_baseline.width * emap_baseline.height];
	//	unsigned short* data_baseline = new unsigned short[emap_baseline.width * emap_baseline.height];
	//	const float max_cap = 0.4;
	//	for (int x = 0; x < emap_baseline.width; x++)
	//	{
	//		for (int y = 0; y < emap_baseline.height; y++)
	//		{
	//			double val_gt = data_gt_Laplacian[y * emap_baseline.width + x];
	//			if (val_gt < 0)
	//				val_gt = MAX2(-1, val_gt / (abs(Laplacian_range[0]) / 2));
	//			else if (val_gt >= 0)
	//				val_gt = MIN2(1, val_gt / (abs(Laplacian_range[1]) / 2));
	//			data_gt[y * emap_baseline.width + x] = (unsigned short)(32767 + val_gt * 32767);

	//			double val_baseline = data_baseline_Laplacian[y * emap_baseline.width + x];
	//			if (val_baseline < 0)
	//				val_baseline = MAX2(-1, val_baseline / (abs(Laplacian_range[0]) * max_cap));
	//			else if (val_baseline >= 0)
	//				val_baseline = MIN2(1, val_baseline / (abs(Laplacian_range[1]) * max_cap));
	//			data_baseline[y * emap_baseline.width + x] = (unsigned short)(32767 + val_baseline * 32767);

	//			//if (x % 100 == 0 && y % 100 == 0)
	//			//	cout << val_gt << " " << data_gt[y * emap_baseline.width + x] << " | " << val_baseline << " " << data_baseline[y * emap_baseline.width + x] << endl;
	//		}
	//	}
	//	Save16BitPNG(data_gt, emap_baseline.width, emap_baseline.height, gt_Laplacian_fn->c_str());
	//	Save16BitPNG(data_baseline, emap_baseline.width, emap_baseline.height, baseline_Laplacian_fn->c_str());
	//	delete data_gt;
	//	delete data_baseline;
	//}

	if (data_gt_Laplacian)
		delete data_gt_Laplacian;
	if (data_baseline_Laplacian)
		delete data_baseline_Laplacian;
	if (data_Laplacian_ae)
		delete data_Laplacian_ae;

	//5x5 LoG filters?
	int num_Laplacian5x5_samples = 0;
	for (int x = 2; x < emap_baseline.width - 2; x++)
	{
		for (int y = 2; y < emap_baseline.height - 2; y++)
		{
			//(X,Y) for gt
			int X0 = (x - 2) * ratio_x;
			int X1 = (x - 1) * ratio_x;
			int X2 = (x)*ratio_x;
			int X3 = (x + 1) * ratio_x;
			int X4 = (x + 2) * ratio_x;
			int Y0 = (y - 2) * ratio_y;
			int Y1 = (y - 1) * ratio_y;
			int Y2 = (y)*ratio_y;
			int Y3 = (y + 1) * ratio_y;
			int Y4 = (y + 2) * ratio_y;
			if (X0 < 0 || X2 > emap_gt.width - 1 ||
				Y0 < 0 || Y2 > emap_gt.height - 1)
				continue;

			//values in a 3x3 (0~2 x 0~2) mask, gt:
			double val[5][5];
			val[0][0] = emap_gt.ValueAtXY(X0, Y0);
			val[1][0] = emap_gt.ValueAtXY(X1, Y0);
			val[2][0] = emap_gt.ValueAtXY(X2, Y0);
			val[3][0] = emap_gt.ValueAtXY(X3, Y0);
			val[4][0] = emap_gt.ValueAtXY(X4, Y0);
			val[0][1] = emap_gt.ValueAtXY(X0, Y1);
			val[1][1] = emap_gt.ValueAtXY(X1, Y1);
			val[2][1] = emap_gt.ValueAtXY(X2, Y1);
			val[3][1] = emap_gt.ValueAtXY(X3, Y1);
			val[4][1] = emap_gt.ValueAtXY(X4, Y1);
			val[0][2] = emap_gt.ValueAtXY(X0, Y2);
			val[1][2] = emap_gt.ValueAtXY(X1, Y2);
			val[2][2] = emap_gt.ValueAtXY(X2, Y2);
			val[3][2] = emap_gt.ValueAtXY(X3, Y2);
			val[4][2] = emap_gt.ValueAtXY(X4, Y2);
			val[0][3] = emap_gt.ValueAtXY(X0, Y3);
			val[1][3] = emap_gt.ValueAtXY(X1, Y3);
			val[2][3] = emap_gt.ValueAtXY(X2, Y3);
			val[3][3] = emap_gt.ValueAtXY(X3, Y3);
			val[4][3] = emap_gt.ValueAtXY(X4, Y3);
			val[0][4] = emap_gt.ValueAtXY(X0, Y4);
			val[1][4] = emap_gt.ValueAtXY(X1, Y4);
			val[2][4] = emap_gt.ValueAtXY(X2, Y4);
			val[3][4] = emap_gt.ValueAtXY(X3, Y4);
			val[4][4] = emap_gt.ValueAtXY(X4, Y4);

			//a Laplacian value is valid only if all gt pixel values in the window are valid (not black)
			bool all_valid = true;
			for (int xx = 0; xx < 5; xx++)
			{
				for (int yy = 0; yy < 5; yy++)
				{
					if (val[xx][yy] < 1e-4)
					{
						all_valid = false;
						break;
					}
				}
			}
			if (!all_valid)
				continue;

			double Laplacian_gt = -1 * val[2][0] - 1 * val[1][1] - 2 * val[2][1] - 1 * val[3][1]
				- 1 * val[0][2] - 2 * val[1][2] + 16 * val[2][2] - 2 * val[3][2] - 1 * val[4][2]
				- 1 * val[1][3] - 2 * val[2][3] - 1 * val[3][3] - 1 * val[2][4];

			double Val[5][5];  //baseline
			Val[0][0] = emap_baseline.ValueAtXY(x - 2, y - 2);
			Val[1][0] = emap_baseline.ValueAtXY(x - 1, y - 2);
			Val[2][0] = emap_baseline.ValueAtXY(x, y - 2);
			Val[3][0] = emap_baseline.ValueAtXY(x + 1, y - 2);
			Val[4][0] = emap_baseline.ValueAtXY(x + 2, y - 2);
			Val[0][1] = emap_baseline.ValueAtXY(x - 2, y - 1);
			Val[1][1] = emap_baseline.ValueAtXY(x - 1, y - 1);
			Val[2][1] = emap_baseline.ValueAtXY(x, y - 1);
			Val[3][1] = emap_baseline.ValueAtXY(x + 1, y - 1);
			Val[4][1] = emap_baseline.ValueAtXY(x + 2, y - 1);
			Val[0][2] = emap_baseline.ValueAtXY(x - 2, y);
			Val[1][2] = emap_baseline.ValueAtXY(x - 1, y);
			Val[2][2] = emap_baseline.ValueAtXY(x, y);
			Val[3][2] = emap_baseline.ValueAtXY(x + 1, y);
			Val[4][2] = emap_baseline.ValueAtXY(x + 2, y);
			Val[0][3] = emap_baseline.ValueAtXY(x - 2, y + 1);
			Val[1][3] = emap_baseline.ValueAtXY(x - 1, y + 1);
			Val[2][3] = emap_baseline.ValueAtXY(x, y + 1);
			Val[3][3] = emap_baseline.ValueAtXY(x + 1, y + 1);
			Val[4][3] = emap_baseline.ValueAtXY(x + 2, y + 1);
			Val[0][4] = emap_baseline.ValueAtXY(x - 2, y + 2);
			Val[1][4] = emap_baseline.ValueAtXY(x - 1, y + 2);
			Val[2][4] = emap_baseline.ValueAtXY(x, y + 2);
			Val[3][4] = emap_baseline.ValueAtXY(x + 1, y + 2);
			Val[4][4] = emap_baseline.ValueAtXY(x + 2, y + 2);

			double Laplacian_baseline = -1 * Val[2][0] - 1 * Val[1][1] - 2 * Val[2][1] - 1 * Val[3][1]
				- 1 * Val[0][2] - 2 * Val[1][2] + 16 * Val[2][2] - 2 * Val[3][2] - 1 * Val[4][2]
				- 1 * Val[1][3] - 2 * Val[2][3] - 1 * Val[3][3] - 1 * Val[2][4];

			Laplacian5x5_mae += abs(Laplacian_gt - Laplacian_baseline);
			num_Laplacian5x5_samples++;
		}
	}

	Laplacian_mse /= (double)num_Laplacian_samples;
	Laplacian_mae /= (double)num_Laplacian_samples;
	SobelX_mae /= (double)num_Sobel_samples;
	SobelY_mae /= (double)num_Sobel_samples;
	Laplacian5x5_mae /= (double)num_Laplacian5x5_samples;

	std::cout << "[ErrorLap] Lap_mse:" << Laplacian_mse << " Lap_mae:" << Laplacian_mae <<
		" SobelX:" << SobelX_mae << " SobelY:" << SobelY_mae << " Lap5x5:" << Laplacian5x5_mae << std::endl;
	return true;
}

Vec3f DepthNamespace::SphericalToWorld(float azimuth, float zenith)
{
	return Vec3f(sin(zenith) * cos(azimuth), sin(zenith) * sin(azimuth), cos(zenith));
}

Vec2f DepthNamespace::WorldToSpherical(Vec3f& p)
{
	p = p.normalize();

	float azimuth = fmod(atan2(p.y, p.x), 2 * MYPI);
	if (azimuth < 0)
		azimuth += 2 * MYPI;

	float zenith = atan2(Vec2f(p.x, p.y).length(), p.z);  //0~PI from the north pole

	return Vec2f(azimuth, zenith);
}

void DepthNamespace::WindowCoords(Vec2f& middle_coord, float azi_half, float zen_half,
	Vec2f& left_up_coord, Vec2f& left_down_coord, Vec2f& right_down_coord, Vec2f& right_up_coord)
{
	//this is also the looking dir
	Vec3f middle = SphericalToWorld(middle_coord[0], middle_coord[1]);

	//the left dir is approx. up CROSS look-dir :
	Vec3f left_dir = Vec3f(0, 0, 1).cross(middle).normalize();

	//the real up dir is left cross look-dir:
	Vec3f up_dir = left_dir.cross(middle).normalize();

	//the left-middle and right_middle 3d points: (longer dists)
	Vec3f left_middle = middle + left_dir * tan(azi_half);
	Vec3f right_middle = middle - left_dir * tan(azi_half);

	//the up-middle and down-middle 3d points:
	Vec3f up_middle = middle - up_dir * tan(zen_half);
	Vec3f down_middle = middle + up_dir * tan(zen_half);

	//the 3d position of the "left-up", "left-down", "right-down", and "right-up" corners:
	Vec3f corner0 = middle + (left_middle - middle) + (up_middle - middle);
	Vec3f corner1 = middle + (left_middle - middle) + (down_middle - middle);
	Vec3f corner2 = middle + (right_middle - middle) + (down_middle - middle);
	Vec3f corner3 = middle + (right_middle - middle) + (up_middle - middle);

	//test: output the boundary sampled
	for (int side = 0; side < 4; side++)
	{
		Vec3f start, end;
		if (side == 0)
		{
			start = corner0;
			end = corner1;
		}
		else if (side == 1)
		{
			start = corner1;
			end = corner2;
		}
		else if (side == 2)
		{
			start = corner2;
			end = corner3;
		}
		else if (side == 3)
		{
			start = corner3;
			end = corner0;
		}

		//inclusive of start, exclusive of end
		std::cout << "#side" << side << std::endl;
		const int subds = 50;
		for (int i = 0; i < subds; i++)
		{
			Vec3f p = start + (end - start) * (float)i / (float)subds;
			Vec2f coord = WorldToSpherical(p);
			std::cout << "p" << i << ": " << R2D(coord.x) << "," << R2D(coord.y) << std::endl;
		}
	}

	left_up_coord = WorldToSpherical(corner0);
	left_down_coord = WorldToSpherical(corner1);
	right_down_coord = WorldToSpherical(corner2);
	right_up_coord = WorldToSpherical(corner3);
}