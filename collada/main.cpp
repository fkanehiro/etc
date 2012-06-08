//
//		OpenGL��COLLADA�h�L�������g��`��
//		Created by TAKAHASHI Masafumi
//			http://www.shader.jp
//

#include <GL/gl.h>
#include <GL/glut.h>

#include "TGATexture.h"

#include <vector>
using namespace std;

// COLLADA DOM�֘A
#include <dae.h>
#include <dom/domCOLLADA.h>

// �r���[�|�[�g
const unsigned int SCREENX = 640;
const unsigned int SCREENY = 480;

// �O���[�o��
DAE*	g_dae = NULL;
int		g_iGeometryElementCount = 0;		// �W�I���g����
int		g_iImageElementCount = 0;			// �e�N�X�`����

vector<CTGATextureHelper*>			g_Texture;
vector<GLuint>				g_TextureObject;	// �e�N�X�`��

// �I�t�Z�b�g�̍ő�l��Ԃ�
unsigned int getMaxOffset( domInputLocalOffset_Array &input_array ) 
{
	unsigned int maxOffset = 0;
	for ( unsigned int i = 0; i < input_array.getCount(); i++ ) {
		if ( input_array[i]->getOffset() > maxOffset ) {
			maxOffset = (unsigned int)input_array[i]->getOffset();
		}
	}
	return maxOffset;
}

// Triangle�̕`��
void RenderTriangle(domMesh *thisMesh, domTriangles *thisTriangles)
{
	//
	int numberOfInputs = (int)getMaxOffset(thisTriangles->getInput_array()) +1;// �I�t�Z�b�g��
	int numberOfTriangles = (int)(thisTriangles->getP()->getValue().getCount() / numberOfInputs);	// �v�f��

	// �C���f�b�N�X�̃I�t�Z�b�g
	unsigned int offset = 0;
	int texoffset = -255, noroffset = -255;
	for(unsigned int i=0;i<thisTriangles->getInput_array().getCount();i++)
	{
		if(strcmp(thisTriangles->getInput_array()[i]->getSemantic(), "VERTEX")==0)
			offset = thisTriangles->getInput_array()[i]->getOffset();
		if(strcmp(thisTriangles->getInput_array()[i]->getSemantic(), "TEXCOORD")==0)
			texoffset = thisTriangles->getInput_array()[i]->getOffset();
		if(strcmp(thisTriangles->getInput_array()[i]->getSemantic(), "NORMAL")==0)
			noroffset = thisTriangles->getInput_array()[i]->getOffset();
	}

	glBegin(GL_TRIANGLES);

	for(int i=0;i<numberOfTriangles;i++)
	{
		int index = thisTriangles->getP()->getValue().get(i*numberOfInputs+offset);

		// �@��
		if(noroffset==-255)
		{
			// �@���̃C���f�b�N�X�����_���W�Ƌ��ʂ̏ꍇ
			glNormal3f(
					thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(index*3),
					thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(index*3+1),
					thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(index*3+2)
			);
		}
		else
		{
			// <p></p>���ɖ@���̃C���f�b�N�X�����݂���ꍇ
			int norindex = thisTriangles->getP()->getValue().get(i*numberOfInputs+noroffset);
			glNormal3f(
					thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(norindex*3),
					thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(norindex*3+1),
					thisMesh->getSource_array()[1]->getFloat_array()->getValue().get(norindex*3+2)
			);
		}

		// �e�N�X�`�����W
		if(texoffset!=-255)
		{
			int texindex = thisTriangles->getP()->getValue().get(i*numberOfInputs+texoffset);
			glTexCoord2f(
				thisMesh->getSource_array()[2]->getFloat_array()->getValue().get(texindex*2),
				thisMesh->getSource_array()[2]->getFloat_array()->getValue().get(texindex*2+1)
				);
		}
		// ���_���W
		glVertex3f(
			thisMesh->getSource_array()[0]->getFloat_array()->getValue().get(index*3),
			thisMesh->getSource_array()[0]->getFloat_array()->getValue().get(index*3+1),
			thisMesh->getSource_array()[0]->getFloat_array()->getValue().get(index*3+2)
			);
	}

	glEnd();
}

// COLLADA �h�L�������g�̕`��
void RenderDAE()
{
	// g_iGeometryElementCount����`��
	for(int currentGeometry=0;currentGeometry<g_iGeometryElementCount;currentGeometry++)
	{
		// ���݂̃W�I���g���̎擾
		domGeometry *thisGeometry;
		g_dae->getDatabase()->getElement((daeElement**)&thisGeometry,currentGeometry, NULL, "geometry");

		// ���b�V���̎擾
		domMesh *thisMesh = thisGeometry->getMesh();

		// Triangle�̏ꍇ
		int triangleElementCount = (int)(thisMesh->getTriangles_array().getCount());
		for(int currentTriangle=0;currentTriangle<triangleElementCount;currentTriangle++)
		{
			domTriangles* thisTriangles = thisMesh->getTriangles_array().get(0);
			RenderTriangle(thisMesh, thisTriangles);
		}
	}
}

// �`��֐�
void display(void)
{
	//
	glClearColor(0,0,0,1);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();
	gluPerspective(60.0, (double)SCREENX / (double)SCREENY, 0.1, 3000.0);
	gluLookAt(0, 0, 400, 0, 0, 0, 0, 1, 0);

	// �e�N�X�`��
	if(g_TextureObject.size() >= 1)
		glBindTexture(GL_TEXTURE_2D, g_TextureObject[0]);
	else
		glDisable(GL_TEXTURE_2D);

	// COLLADA�h�L�������g�̕`��
	RenderDAE();

	glFlush();
}

// �O�p�`��(Polylist)
void ConvertPolylistToTriangles(domMesh *thisMesh, domPolylist *thisPolylist)
{
	// 4�p�`��3�p�`��
	domTriangles *thisTriangles = (domTriangles *)thisMesh->createAndPlace("triangles");
	unsigned int triangles = 0;
	thisTriangles->setMaterial(thisPolylist->getMaterial());
	domP* p_triangles = (domP*)thisTriangles->createAndPlace("p");

	for(int i=0; i<(int)(thisPolylist->getInput_array().getCount()); i++)
	{
		thisTriangles->placeElement( thisPolylist->getInput_array()[i]->clone() );
	}

	// Get the number of inputs and primitives for the polygons array
	int numberOfInputs = (int)getMaxOffset(thisPolylist->getInput_array()) + 1;
	int numberOfPrimitives = (int)(thisPolylist->getVcount()->getValue().getCount());

	unsigned int offset = 0;

	for(int j = 0; j < numberOfPrimitives; j++)
	{	
		int triangleCount = thisPolylist->getVcount()->getValue()[j] -2;
		// Write out the primitives as triangles, just fan using the first element as the base
		int idx = numberOfInputs;
		for(int k = 0; k < triangleCount; k++)
		{
			// First vertex
			for(int l = 0; l < numberOfInputs; l++)
			{
				int a = thisPolylist->getP()->getValue()[offset + l];
				float b;
				memcpy(&b, &a, sizeof(float));
				p_triangles->getValue().append(thisPolylist->getP()->getValue()[offset + l]);
			}
			// Second vertex
			for(int l = 0; l < numberOfInputs; l++)
			{
				p_triangles->getValue().append(thisPolylist->getP()->getValue()[offset + idx + l]);
			}
			// Third vertex
			idx += numberOfInputs;
			for(int l = 0; l < numberOfInputs; l++)
			{
				p_triangles->getValue().append(thisPolylist->getP()->getValue()[offset + idx + l]);
			}
			//thisTriangles->setCount(thisTriangles->getCount()+1);
			triangles++;
		}
		offset += thisPolylist->getVcount()->getValue()[j] * numberOfInputs;
	}

	thisTriangles->setCount( triangles );
}

// �O�p�`��(Polygons)
void ConvertPolygonsToTriangles(domMesh *thisMesh, domPolygons *thisPolygons)
{
	// 3�p�`��
	domTriangles *thisTriangles = (domTriangles *)thisMesh->createAndPlace("triangles");
	thisTriangles->setCount( 0 );
	thisTriangles->setMaterial(thisPolygons->getMaterial());
	domP* p_triangles = (domP*)thisTriangles->createAndPlace("p");

	// Give the new <triangles> the same <input> and <parameters> as the old <polygons>
	for(int i=0; i<(int)(thisPolygons->getInput_array().getCount()); i++)
	{
		thisTriangles->placeElement( thisPolygons->getInput_array()[i]->clone() );
	}
	// Get the number of inputs and primitives for the polygons array
	int numberOfInputs = (int)getMaxOffset(thisPolygons->getInput_array()) +1;
	int numberOfPrimitives = (int)(thisPolygons->getP_array().getCount());

	// Triangulate all the primitives, this generates all the triangles in a single <p> element
	for(int j = 0; j < numberOfPrimitives; j++)
	{
		// Check the polygons for consistancy (some exported files have had the wrong number of indices)
		domP * thisPrimitive = thisPolygons->getP_array()[j];
		int elementCount = (int)(thisPrimitive->getValue().getCount());
		if((elementCount%numberOfInputs) != 0)
		{
//			cerr<<"Primitive "<<j<<" has an element count "<<elementCount<<" not divisible by the number of inputs "<<numberOfInputs<<"\n";
			continue;
		}
		else
		{
			int triangleCount = (elementCount/numberOfInputs)-2;
			// Write out the primitives as triangles, just fan using the first element as the base
			int idx = numberOfInputs;
			for(int k = 0; k < triangleCount; k++)
			{
				// First vertex
				for(int l = 0; l < numberOfInputs; l++)
				{
					p_triangles->getValue().append(thisPrimitive->getValue()[l]);
				}
				// Second vertex
				for(int l = 0; l < numberOfInputs; l++)
				{
					p_triangles->getValue().append(thisPrimitive->getValue()[idx + l]);
				}
				// Third vertex
				idx += numberOfInputs;
				for(int l = 0; l < numberOfInputs; l++)
				{
					p_triangles->getValue().append(thisPrimitive->getValue()[idx + l]);
				}
				thisTriangles->setCount(thisTriangles->getCount()+1);
			}
		}
	}
}

// COLLADA�֘A�̏�����
bool InitCOLLADA()
{
	int iRet = 0;
	// COLLADA �h�L�������g�̃��[�h
	g_dae = new DAE();
	iRet = g_dae->load("file:///tmp/glSimpleCollada_src/Distilateur.dae");
	if(iRet!=DAE_OK)
		return false;

	// �W�I���g�����̎擾
	g_iGeometryElementCount =  g_dae->getDatabase()->getElementCount(NULL, "geometry", NULL);

	// Triangles�ȊO��������Triangles��
	for(int currentGeometry=0;currentGeometry<g_iGeometryElementCount;currentGeometry++)
	{
		// ���݂̃W�I���g���̎擾
		domGeometry *thisGeometry;
		g_dae->getDatabase()->getElement((daeElement**)&thisGeometry,currentGeometry, NULL, "geometry");

		// ���b�V���̎擾
		domMesh *thisMesh = thisGeometry->getMesh();

		// Polylist�̏ꍇ
		int polylistElemntCount = (int)(thisMesh->getPolylist_array().getCount());
		for(int currentPolylist=0;currentPolylist<polylistElemntCount;currentPolylist++)
		{
			domPolylist* thisPolylist = thisMesh->getPolylist_array().get(0);
			ConvertPolylistToTriangles( thisMesh, thisPolylist );
		}

		// Polygons�̏ꍇ
		int polygonsElementCount = (int)(thisMesh->getPolygons_array().getCount());
		for(int currentPolygons=0;currentPolygons<polygonsElementCount;currentPolygons++)
		{
			domPolygons* thisPolygons = thisMesh->getPolygons_array().get(0);
			ConvertPolygonsToTriangles( thisMesh, thisPolygons );
		}
	}

#if 0
	// �e�N�X�`�����̎擾
	g_iImageElementCount = g_dae->getDatabase()->getElementCount(NULL, "image", NULL);
	for(int i=0;i<g_iImageElementCount;i++)
	{
		g_Texture.resize( g_Texture.size()+1);
		g_Texture[i] = new CTGATextureHelper;
		// Image�̎擾
		domImage* thisImage;
		g_dae->getDatabase()->getElement((daeElement**)&thisImage, i, NULL, "image");

		// �t�@�C�����̎擾
		//daeString name =  thisImage->getInit_from()->getValue().getFile();
                daeString name = "mushroom.tga";
		g_Texture[i]->LoadTGA( (char*)name );
	}
	
	// �e�N�X�`������
	for(unsigned int i=0;i<g_Texture.size();i++)
	{
		GLuint tex;
		glGenTextures(1, &tex);
		glBindTexture(GL_TEXTURE_2D, tex);
		g_TextureObject.push_back( tex );

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

		glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_WRAP_S , GL_CLAMP);
		glTexParameteri(GL_TEXTURE_2D , GL_TEXTURE_WRAP_T , GL_CLAMP);
	
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, g_Texture[i]->GetWidth(), g_Texture[i]->GetHeight(),
			0, GL_RGB, GL_UNSIGNED_BYTE, g_Texture[i]->GetImage() );
	}
#endif
	return true;
}

// COLLADA�֘A�̏I������
void cleanupCOLLADA()
{
	// �������
	for(unsigned int i=0;i<g_TextureObject.size();i++)
	{
		glDeleteTextures(1, &g_TextureObject[i]);
	}
	g_TextureObject.clear();

	for(unsigned int i=0;i<g_Texture.size();i++)
	{
		g_Texture[i]->Release();
		delete g_Texture[i];
	}
	g_Texture.clear();

	if(g_dae)
	{
		g_dae->cleanup();
	}

	// �I�����ɌĂ�
	DAE::cleanup();
}

// 
int main(int argc, char *argv[])
{
	// GLUT �֘A
	glutInit( &argc, argv );
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGBA | GLUT_DEPTH);
	
	glutInitWindowSize(SCREENX, SCREENY);
	glutCreateWindow( "Simple COLLADA Sample" );
	
	glutDisplayFunc( display );

	glEnable(GL_TEXTURE_2D);
	glEnable( GL_DEPTH );
	glEnable(GL_CULL_FACE);
	glCullFace(GL_BACK);
	
	// COLLADA�֘A�̏�����
	if(!InitCOLLADA())
		return 0;
	
	glutMainLoop();

	// �I������
	cleanupCOLLADA();

	return 0;
}
