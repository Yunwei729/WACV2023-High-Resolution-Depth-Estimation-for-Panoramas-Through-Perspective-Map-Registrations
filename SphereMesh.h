#pragma once

#include <vector>
#include <map>
#include "ILMBase.h"

using namespace std;

namespace SphereNamespace
{	
	//a texture stuff
	class Texture
	{
	public:
		int width;
		int height;
		unsigned int id;  //opengl texture id		
		unsigned char* data;

		Texture()
		{
			data = NULL;
			Reset();
		}

		void Reset();
	};

	//a lightweight, non-halfedge mesh 
	class LiteMesh
	{
	public:
		vector<Vec3f> m_vertices;
		vector<Vec2f> m_texcoords;  //texture coordinates. one per vertex
		std::vector<std::vector<int>> m_faces;

		//the current texture
		Texture m_texture;
		
		//(shader programming) vertex and triangle-face "VBO" arrays for array-based draw
		float* m_varray;  //vertex poss array (size = |vertices|)
		float* m_tarray;  //vertex texcoord array (size = |verices|)
		unsigned int* m_tri_array;
		int m_tri_array_size;

		LiteMesh()
		{
			m_varray = NULL;
			m_tarray = NULL;
			m_tri_array = NULL;

			Reset();
		}

		void Reset()
		{
			m_vertices.clear();
			m_texcoords.clear();
			m_faces.clear();			

			if (m_varray)
			{
				delete m_varray;
				m_varray = NULL;
			}			
			if (m_tarray)
			{
				delete m_tarray;
				m_tarray = NULL;
			}
			if (m_tri_array)
			{
				delete m_tri_array;
				m_tri_array = NULL;
			}
			m_tri_array_size = 0;			
		}		
		
		//rebuild the mesh to be a sphere
		//latitudes and longitudes: resolutions (# of stripes) in latitude (horizontal) and longitude (vertical)
		//note: latitude shall be even (half on north hemisphere, half on south)
		bool InitSphere(int latitudes, int longitudes);

		//load a jpg/png file as the main "picture" texture 
		bool LoadTexture(const char* filename);		

		//setup "VBO" arrays
		void CreateArrays();
		void DrawArray();
	};
}