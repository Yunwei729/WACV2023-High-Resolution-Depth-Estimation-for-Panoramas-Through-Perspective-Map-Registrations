#pragma once

#include <vector>
#include <map>

namespace DepthNamespace
{
	//a full equirectangular (azimuth: 0~2PI, zenith: 1~PI) map
	class EquirectangularMap
	{
	public:
		int width;
		int height;
		int channels;

		float* data;  //width by height (row major) of #channel depth or disparity map

	public:

		EquirectangularMap()
		{
			data = NULL;
		}

		~EquirectangularMap()
		{
			if (data)
			{
				delete data;
				data = NULL;
			}
		}

		//load a equirectangular depth/disparity map from a file
		//mono360: dpt/pfm depth from mono360?
		bool Load(std::string& filename, bool mono360=false);

		//load from a pfm (portable float map) file:
		//normalize: normalize minmax to 0~1 range?
		bool LoadPfm(std::string& filename, bool flip_vertical, bool normalize, const char *save_png_filename = NULL);

		//get a depth/disparity value according to (azimuth,zenith)
		//note: return 0 if it is out of this map's domain
		float ValueAtCoord(float azimuth, float zenith);
		//return value at (x,y) (pixel coordinates)
		float ValueAtXY(int x, int y);

		//return the avg of values (depth or disparity)
		double Avg();

		//mark invalid pixels from a reference emap
		void CopyInvalidPixels(EquirectangularMap& ref_emap);		

		//disp<->depth conversion
		void DispDepthConversion();

		//save to a file 8bit
		bool Save8bit(std::string& filename);
	};

	class PerspectiveMap
	{
	public:
		int width;
		int height;
		int channels;
		float* data;  //width by height (row major) of #channel depth or disparity map. 0~1 float values

		//the FOVs of the viewing window
		float azimuth_left;
		float azimuth_right;
		float zenith_top;
		float zenith_down;

		//the valid azimuth and zenith ranges, in radians
		Vec4f ranges;  //{azi_left, azi_right, zen_up, zen_down}

		//triangulation
		std::vector<Vec2f> vertices;  //2D spherical coordinates or original vertices
		std::vector<std::vector<int>> faces;  //faces

		//subdivision
		std::vector<std::map<int, float>> subd_points;  //<vertex index, weight> equations for every Loop subdivided vertex
		std::vector<std::vector<int>> subd_faces;  //subd. faces
		std::map<int, bool> subd_boundary_points;  //indices of boundary subd_points

		////caching the info about the the quadrilateral viewing window:
		Vec3f middle;  //window center
		//the two horizontal and vertical edges of the rectangular window in 3d:
		Vec3f hedge;
		Vec3f vedge;
		//the 4 corners (from lower-left, CCW):
		Vec3f corner0, corner1, corner2, corner3;

		PerspectiveMap()
		{
			data = NULL;
			width = 0;
			height = 0;
			channels = 0;
			azimuth_left = 0;
			azimuth_right = 0;
			zenith_top = 0;
			zenith_down = 0;
		}

		~PerspectiveMap()
		{
			if (data)
			{
				delete data;
				data = NULL;
			}
		}

		//load a perspective depth/disparity map from a file, and setup the viewing window
		bool Load(std::string& filename);

		//setup the viewing window
		void SetWindow(float AzimuthLeft, float AzimuthRight, float ZenithTop, float ZenithDown);

		//get a depth/disparity value according to 2D input (x,y), 0<=x,y<=1
		//note: return 0 if it is out of this map's domain
		float Value(float x, float y);

		//return spherical coord. (azimuth, zenith) by 2D input (x,y), 0<=x,y<=1
		//note: uses azimuth and zenith ranges of this pmap
		Vec2f ToSphericalCoord(float x, float y);
		//return 2D (x,y), 0<=x,y<=1, given a spherical coord
		Vec2f SphericalTo2D(float azimuth, float zenith);

		//just test if this pmap contains a ray (given as a spherical coord.) 
		bool Contain(float azimuth, float zenith);

		float AzimuthMin()
		{
			return MIN2(azimuth_left, azimuth_right);
		}
		float AzimuthMax()
		{
			return MAX2(azimuth_left, azimuth_right);
		}
		float ZenithMin()
		{
			return MIN2(zenith_top, zenith_down);
		}
		float ZenithMax()
		{
			return MAX2(zenith_top, zenith_down);
		}

		float ValueAtXY(int x, int y);

		//apply disparity-to-depth transform w/ abcd
		void D2DTransform(Vec4f abcd);

		void Depth2DepthTransform(Vec4f& abc);		
	};

	//pano depth metrics
	class Metrics
	{
	public:
		float mse_given;  //mean squared error of given and result w.r.t. gt
		float mse_result;
		float mae_given;  //mean absolute error
		float mae_result;
		float mre_given;  //mean relative error
		float mre_result;
		float mselog_given;
		float mselog_result;
		float delta1_given;  //relative error < 1.25 percentage
		float delta1_result;
		float delta2_given;  //relative error < 1.25^2 percentage
		float delta2_result;
		float delta3_given;  //relative error < 1.25^3 percentage
		float delta3_result;

		Metrics()
		{
			mse_given = 0;
			mse_result = 0;
			mae_given = 0;  //mean absolute error
			mae_result = 0;
			mre_given = 0;
			mre_result = 0;
			mselog_given = 0;
			mselog_result = 0;
			delta1_given = 0;
			delta1_result = 0;
			delta2_given = 0;
			delta2_result = 0;
			delta3_given = 0;
			delta3_result = 0;
		}

		bool Save(const char* filename)
		{
			FILE* fp = fopen(filename, "w+");
			if (!fp)
			{
				std::cout << "fopen failed?" << std::endl;
				return false;
			}

			fprintf(fp, "mse_given: %f\n", mse_given);
			fprintf(fp, "mse_result: %f\n", mse_result);
			if (mse_given != 0)
				fprintf(fp, "mse diff: %f\n", (mse_result - mse_given) / mse_given);

			fprintf(fp, "mae_given: %f\n", mae_given);
			fprintf(fp, "mae_result: %f\n", mae_result);
			if (mae_given != 0)
				fprintf(fp, "mae diff: %f\n", (mae_result - mae_given) / mae_given);

			fprintf(fp, "mre_given: %f\n", mre_given);
			fprintf(fp, "mre_result: %f\n", mre_result);
			if (mre_given != 0)
				fprintf(fp, "mre diff: %f\n", (mre_result - mre_given) / mre_given);

			fprintf(fp, "mselog_given: %f\n", mselog_given);
			fprintf(fp, "mselog_result: %f\n", mselog_result);
			if (mselog_given != 0)
				fprintf(fp, "mselog diff: %f\n", (mselog_result - mselog_given) / mselog_given);
			
			fprintf(fp, "delta1_given: %f\n", delta1_given);
			fprintf(fp, "delta1_result: %f\n", delta1_result);
			if (delta1_given != 0)
				fprintf(fp, "delta1 diff: %f\n", (delta1_result - delta1_given) / delta1_given);

			fprintf(fp, "delta2_given: %f\n", delta2_given);
			fprintf(fp, "delta2_result: %f\n", delta2_result);
			if (delta2_given != 0)
				fprintf(fp, "delta2 diff: %f\n", (delta2_result - delta2_given) / delta2_given);

			fprintf(fp, "delta3_given: %f\n", delta3_given);
			fprintf(fp, "delta3_result: %f\n", delta3_result);
			if (delta1_given != 0)
				fprintf(fp, "delta3 diff: %f\n", (delta3_result - delta3_given) / delta3_given);

			fclose(fp);
			return true;
		};

		void Print()
		{
			std::cout << "RMSE " << sqrt(mse_given) << "->" << sqrt(mse_result) <<
				" (" << (sqrt(mse_result) - sqrt(mse_given)) / sqrt(mse_given) <<
				") MAE " << mae_given << "->" << mae_result <<
				" (" << (mae_result - mae_given) / mae_given <<
				") MRE " << mre_given << "->" << mre_result <<
				" (" << (mre_result - mre_given) / mre_given <<
				") RMSElog " << sqrt(mselog_given) << "->" << sqrt(mselog_result) <<
				" (" << (sqrt(mselog_result) - sqrt(mselog_given)) / sqrt(mselog_given) <<
				") deltas:" << delta1_given << "->" << delta1_result << "(" << (delta1_result - delta1_given) <<
				") , " << delta2_given << "->" << delta2_result << "(" << (delta2_result - delta2_given) <<
				") , " << delta3_given << "->" << delta3_result << "(" << (delta3_result - delta3_given) << std::endl;
		}
	};

	//Laplacian windows: each is a mask of weights, e.g., "+1 for center pixel, -1/4 for each of the four neighbors"
	class LaplacianWindow
	{
	public:
		std::map<std::pair<int, int>, float> mask;
		float Laplacian; //supposed Laplacian value

		LaplacianWindow()
		{
			Laplacian = 0;
		}
	};


	//with a "base" depth/disparity map, merge several perspective depth/disparity maps
	//perspective_map_ranges: one or multiple azimuth left-right and zenith top-down subsets of every perspective map
	//output a merged panorama in equirectangular format
	//perspective_map_spans: the horizonontal and vertical FOVs of each viewing window
	//perspective_map_ranges: the valid ranges to use for each pmap
	//out_pano_width: width of the output depth panorama (height = width/2). shall be even
	//zenith_range: the range (top and down) of valid zenith (w.r.t. to given emap)
	//equirectangular_map_groundtruth: (optional) if given, also calculate various error metrics
	//mse: mean squared error 
	//mae: mean absolute error
	//time_total: total time (ms) spent (not including the statistics times)
	bool MergeDepthMaps(std::string& equirectangular_map_filename, std::vector<std::string>& perspective_map_filenames, std::string& out_filename,
		std::vector<Vec4f>& perspective_map_FOVs, std::vector<Vec4f>& perspective_map_ranges,
		int out_width, Vec2f& zenith_range, std::string* equirectangular_map_groundtruth = NULL,
		Metrics* metrics = NULL, int* time_Reg = NULL, int* time_Laplacian = NULL);
	
	//solve disparity-to-depth transform: y = c * (1 / (ax+b)) + d.  x is disparity, y is depth
	//pmaps_actives: active-or-not for every given pmaps
	bool SolveDisparityToDepth(EquirectangularMap& emap, std::vector<PerspectiveMap>& pmaps,
		std::vector<bool>& pmaps_actives, Vec2f& zenith_range, Vec4f& abcd);

	//final global depth emap to depth emap registration
	bool SolveDepthToDepth(EquirectangularMap& emap, std::vector<PerspectiveMap>& pmaps,
		std::vector<bool>& pmaps_actives, Vec2f& zenith_range, Vec4f& abcd);
	//solve emap (data) to emap D2D transform
	bool SolveDepthToDepth2(EquirectangularMap& emap, unsigned short* data, int width, int height,
		Vec2f& zenith_range, Vec4f& abcd);

	
	//solve per-pixel depth values w.r.t a global baseline emap
	//data: the output buffer of 0~255 unsigned char out_width X out_height
	bool SolveDepthAll(EquirectangularMap& emap, std::vector<PerspectiveMap>& pmaps, unsigned short* data,
		int& out_width, int& out_height, Vec2f& zenith_range, const char* Laplacian_filename = NULL);

	bool SolveDepthBySmoothing(std::vector<PerspectiveMap>& pmaps, unsigned short* data, int& out_width, int& out_height, Vec2f& zenith_range);
		
	//mesure pixel-wise errors between two images (given as filenames)
	//align_way: 0=nope, 1=median shift, 2=least square way
	bool ErrorData(EquirectangularMap& emap_gt, unsigned short* data, int data_width, int data_height, float& mse, float& mae, float& mre, float& mse_log,
		float& delta1, float& delta2, float& delta3, int align_way, bool cap_depth, Vec2f *least_square_shift = NULL, float* median_shift_factor = NULL);
	bool ErrorEmap(EquirectangularMap& emap_gt, EquirectangularMap& emap_given, float& mse, float& mae, float& mre, float& mse_log,
		float& delta1, float& delta2, float& delta3, int align_way, bool cap_depth, Vec2f* least_square_shift = NULL, float* median_shift_factor = NULL);

	bool ErrorCompare(std::string& gt_filename, std::string& baseline_filename, bool DispDepthCompare,
		float& mse, float& mae, float& mre, float& mse_log, float& delta1, float& delta2, float& delta3, int align_way, bool cap_depth, const char* shifted_filename = NULL);

	bool ErrorLaplacian(std::string& gt_filename, std::string& baseline_filename, double& Laplacian_mse, double& Laplacian_mae,
		double& SobelX_mae, double& SobelY_mae, double& Laplacian5x5_mae,
		std::string* gt_Laplacian_fn = NULL, std::string* baseline_Laplacian_fn = NULL, std::string* Laplacian_ae_fn = NULL);

	//sphercial coord. (radians) to 3d position
	Vec3f SphericalToWorld(float azimuth, float zenith);

	//3d position (normalized) to spherical coord.
	Vec2f WorldToSpherical(Vec3f& p);

	//utility function to find the spherical coords of the 4 corners of a viewing window w/ center (azi,zen) and 
	//horizontal and vertical FOVs
	void WindowCoords(Vec2f& middle_coord, float azi_half, float zen_half, Vec2f& left_up_coord, Vec2f& left_down_coord,
		Vec2f& right_down_coord, Vec2f& right_up_coord);
		
	//median scaling emap0 toward emap1
	bool MedianScaling(EquirectangularMap& emap0, EquirectangularMap& emap1, float *emap0_median = NULL, float *emap1_median = NULL);
};