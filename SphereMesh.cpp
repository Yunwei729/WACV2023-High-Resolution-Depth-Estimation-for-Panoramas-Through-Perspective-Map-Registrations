#include "glsl.h"
#include "glut.h"
#include "Basic.h"
#include "ILMBase.h"
#include "SphereMesh.h"

//for loading textures
//#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
//#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

using namespace std;
using namespace SphereNamespace;

void Texture::Reset()
{
	width = 0;
	height = 0;
	if (id > 0)
	{
		glDeleteTextures(1, &id);
	}
	id = 0;

	if (data)
	{
		stbi_image_free(data);
		data = NULL;
	}
}


void LiteMesh::DrawArray()
{
	glEnable(GL_LIGHTING);

	glEnableClientState(GL_NORMAL_ARRAY);
	if (m_tarray)
		glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glEnableClientState(GL_VERTEX_ARRAY);

	glVertexPointer(3, GL_FLOAT, 0, m_varray);
	
	if (m_tarray)
		glTexCoordPointer(2, GL_FLOAT, 0, m_tarray);
	
	glDrawElements(GL_TRIANGLES, m_tri_array_size, GL_UNSIGNED_INT, m_tri_array);

	glDisableClientState(GL_VERTEX_ARRAY);
	if (m_tarray)
		glDisableClientState(GL_TEXTURE_COORD_ARRAY);
	glDisableClientState(GL_NORMAL_ARRAY);

	glDisable(GL_LIGHTING);
}

bool LiteMesh::LoadTexture(const char* filename)
{
	m_texture.Reset();

	//load texture usinb stb_image library
	int channels = 0;
	m_texture.data = stbi_load(filename, &m_texture.width, &m_texture.height, &channels, 0);
	if (m_texture.data == NULL)
	{
		cout << "[LoadTexture] cannot load texture " << filename << endl;
		return false;
	}

	//generate GL texture
	glGenTextures(1, &m_texture.id);
	glBindTexture(GL_TEXTURE_2D, m_texture.id);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	if (channels == 3)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_texture.width, m_texture.height, 0, GL_RGB, GL_UNSIGNED_BYTE, m_texture.data);
	else if (channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, m_texture.width, m_texture.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, m_texture.data);
	else if (channels == 1)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_LUMINANCE, m_texture.width, m_texture.height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, m_texture.data);
	glBindTexture(GL_TEXTURE_2D, 0);

	cout << "[LoadTexture] ok! m_tex_id:" << m_texture.id << endl;
	return true;
}

void LiteMesh::CreateArrays()
{
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

	//setup vertex poss array	
	m_varray = new float[m_vertices.size() * 3];
	for (int i = 0; i < m_vertices.size(); i++)
	{
		m_varray[i * 3] = m_vertices[i].x;
		m_varray[i * 3 + 1] = m_vertices[i].y;
		m_varray[i * 3 + 2] = m_vertices[i].z;
	}

	//per-vertex texcoords array?
	if (m_texcoords.size() > 0)
	{
		m_tarray = new float[m_texcoords.size() * 2];  //u and v


		for (int i = 0; i < m_texcoords.size(); i++)
		{
			m_tarray[i * 2] = m_texcoords[i].x;
			m_tarray[i * 2 + 1] = m_texcoords[i].y;
		}
	}

	//setup face indices array
	vector<vector<int>> tris;  //to individual *triangles* of vertex indices
	for (int i = 0; i < m_faces.size(); i++)
	{
		for (int offset = 0; offset <= m_faces[i].size() - 2; offset += 2)
		{
			vector<int>& f = m_faces[i];
			vector<int> tri;
			tri.push_back(f[offset]);
			tri.push_back(f[(offset + 1) % f.size()]);
			tri.push_back(f[(offset + 2) % f.size()]);
			tris.push_back(tri);
		}
	}
	m_tri_array = new unsigned int[tris.size() * 3];
	for (int i = 0; i < tris.size(); i++)
	{
		m_tri_array[i * 3] = tris[i][0];
		m_tri_array[i * 3 + 1] = tris[i][1];
		m_tri_array[i * 3 + 2] = tris[i][2];
	}
	m_tri_array_size = tris.size() * 3;
}

bool LiteMesh::InitSphere(int latitudes, int longitudes)
{
	Reset();	

	//create a "latitude-longitude" pure quad mesh (with #latitudes horizontal stripes and #longitudes vertical strips)
	//note: at the north and south poles the quads degenerate to triangles

	float depth_all_min = FLT_MAX, depth_all_max = 0;

	//there are #latitudes horizontal strips of vertices
	for (int t = 0; t < latitudes; t++)  //t==0 is at the north pole, t==latitudes-1 is at the south pole
	{
		//there are #longitudes vertical strips of vertices. go CCW
		for (int p = 0; p < longitudes; p++)
		{
			//calculate the physics spherical coordinate (theta (from north pole), phi (from x-axis))
			//"vertical" is z-axis
			float azimuth = (float)p / (float)(longitudes - 1) * (2 * MYPI);  //0~2PI, inclusive at end
			float zenith = (float)t / (float)(latitudes - 1) * MYPI;  //0~PI, inclusive at end

			float radius = 1;			

			float x = sin(zenith) * cos(azimuth) * radius;
			float y = sin(zenith) * sin(azimuth) * radius;
			float z = cos(zenith) * radius;

			//texcoord of this vertex (using equirectangular formula):
			Vec2f texcoord;
			texcoord.x = (float)p / (float)(longitudes - 1);  //0~1 horizontal, inclusive at end
			texcoord.y = (float)t / (float)(latitudes - 1);  //0~1 vertical, inclusive at end

			m_vertices.push_back(Vec3f(x, y, z));
			m_texcoords.push_back(texcoord);
		}
	}

	cout << "[InitSphere] depth_min:" << depth_all_min << " depth_max:" << depth_all_max << endl;

	//create the quad faces
	for (int t = 0; t < latitudes - 1; t++)  //"lower" vertices
	{
		for (int p = 0; p < longitudes - 1; p++)  //"left" vertices
		{
			vector<int> f;
			f.push_back(t * longitudes + p);
			f.push_back(t * longitudes + (p + 1));
			f.push_back((t + 1) * longitudes + (p + 1));
			f.push_back((t + 1) * longitudes + p);
			m_faces.push_back(f);
		}
	}	

	//recreate the "VBO" arrays:
	CreateArrays();

	return true;
}