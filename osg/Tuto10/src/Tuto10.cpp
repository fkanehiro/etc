/******************************************************************************\
* OpenGL based program in OSG                                                               *
* Copyright Franclin Foping 
franclin@netcourrier.com
\******************************************************************************/


#include <osg/Drawable>
#include <osg/ShapeDrawable>
#include <osg/Group>
#include <osg/Material>
#include <osgViewer/Viewer>

#include <osg/Texture2D>
#include <osgDB/ReadFile>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>


void DrawCube()
{	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	 glScalef(0.2f,0.2f,0.2f);
	 glBegin(GL_QUADS);
	//Front face
         glNormal3f (0, 0, 1);

	 glTexCoord2f(0,0);
         glVertex3f (-1,-1, 1);
	 
	 glTexCoord2f(1,0);
         glVertex3f (1,-1, 1);
	 
	 glTexCoord2f(1,1);
         glVertex3f (1,1,1);

	 glTexCoord2f(0,1);
         glVertex3f (-1,1,1);
	
	//Back face 
	 glNormal3f(0,0,-1);
	 
	 glTexCoord2f(1,0);
         glVertex3f (-1,-1, -1);
	 
	 glTexCoord2f(1,1);
         glVertex3f (-1,1,-1);
	 
	 glTexCoord2f(0,1);
         glVertex3f (1,1,-1);

	 glTexCoord2f(0,0);
         glVertex3f (1,-1,-1);
	
	//Top face
	glNormal3f(0,1,0);
	 
	 glTexCoord2f(0,1);
         glVertex3f (-1,1, -1);
	 
	 glTexCoord2f(0,0);
         glVertex3f (-1,1,1);
	 
	 glTexCoord2f(1,0);
         glVertex3f (1,1,1);

	 glTexCoord2f(1,1);
         glVertex3f (1,1,-1);

	//Bottom face 
	glNormal3f(0,-1,0);
	 
	 glTexCoord2f(1,1);
         glVertex3f (-1,-1, -1);
	 
	 glTexCoord2f(0,1);
         glVertex3f (1,-1,-1);
	 
	 glTexCoord2f(0,0);
         glVertex3f (1,-1,1);

	 glTexCoord2f(1,0);
         glVertex3f (-1,-1,1);

	//Right face
	 glNormal3f(1,0,0);
	 
	 glTexCoord2f(1,0);
         glVertex3f (1,-1, -1);
	 
	 glTexCoord2f(1,1);
         glVertex3f (1,1,-1);
	 
	 glTexCoord2f(0,1);
         glVertex3f (1,1,1);

	 glTexCoord2f(0,0);
         glVertex3f (1,-1,1);

	//Left face
	 glNormal3f(-1,0,0);
	 
	 glTexCoord2f(0,0);
         glVertex3f (-1,-1, -1);
	 
	 glTexCoord2f(1,0);
         glVertex3f (-1,-1,1);
	 
	 glTexCoord2f(1,1);
         glVertex3f (-1,1,1);

	 glTexCoord2f(0,1);
         glVertex3f (-1,1,-1);

 	glPopMatrix();
	
};

void DrawQuad()
{
  glBegin(GL_QUADS);
         glNormal3f (0, 1, 0);

	 glTexCoord2f(0,0);
         glVertex3f (-1,0, 0);

	 glTexCoord2f(1, 0);
         glVertex3f (1,0,0);

	 glTexCoord2f(1,1);
         glVertex3f ( 1,0,1);

	 glTexCoord2f(0, 1);
	 glVertex3f(-1,0,1);
       
 glEnd();
	
	
};
//For the bezier curve
GLfloat ctrlPoint[4][3] = {{-2.0f,0.0f,0.0f},
			   {-3.0f,3.0f,0.0f},
			   {3.0f,3.0f,0.0f},
			   {1.0f,0.0f,0.0f}
			  };
GLint numPoints = 4;

//For the bezier surface
GLfloat ctrlSurface [3][3][3] = {{{-2,0,2},
				{-1,2,2},
				{2,0,2}},
				
				{{-2,0,0},
				 {-1,2,0},
				 {2,0,0}},

				{{-2,0,-2},
				 {-1,2,-2},
				 {2,0,-2}}
				};
GLint numPointsSurface = 3;
			

void DrawBezierSurface()
{	
	
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	 glScalef(0.2f,0.2f,0.2f);
	glMap2f(	
		GL_MAP2_VERTEX_3,
		0,
		10,
		3,
		3,
		0,
		10,
		9,
		3,
		&ctrlSurface[0][0][0]
		);
	glEnable(GL_MAP2_VERTEX_3);
	 glMapGrid2f(10,0,10,10,0,10);
	 glEvalMesh2(GL_FILL,0,10,0,10);
		glEnable(GL_AUTO_NORMAL);
		glPointSize(5.0f);
	
	glBegin(GL_POINTS);
		for (int i=0; i< numPoints;++i)
		{
			//glTexCoord1f(i/numPoints);
			glVertex2fv(ctrlPoint[i]);
		};
	glEnd();
      
	

};

void RenderPoints() 
	{
	int i;
	//Set point size
	glEnable(GL_AUTO_NORMAL);
	glPointSize(5.0f);
	
	glBegin(GL_POINTS);
		for (int i=0; i< numPoints;++i)
		//glNormal2f (0, 1, 0);
		glVertex2fv(ctrlPoint[i]);
	glEnd();
	};

void RenderBezier()
 	{
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	 glRotatef(45,0,0,1);
	 glRotatef(45,0,1,0);
	 glRotatef(90,1,0,0);
	 glScalef(0.15f,0.15f,0.15f);
	glMap1f(GL_MAP1_VERTEX_3,
		0.0f,
		100.0f,
		3,
		numPoints,
		&ctrlPoint[0][0]);
	
	glEnable(GL_MAP1_VERTEX_3);
	glBegin(GL_LINE_STRIP);
		for (int i=0; i<= 100;++i)
		  glEvalCoord1f((GLfloat)i);
	glEnd();

	RenderPoints();
	glPopMatrix();

	
 
	};

class CustomDrawable: public osg::Drawable
{
   private:
	
	
	
   public:

      // The constructor here does nothing. One thing that may be necessary is
      // disabling display lists. This can be done by calling
      //    setSupportsDisplayList (false);
      // Display lists should be disabled for 'Drawable's that can change over
      // time (that is, the vertices drawn change from time to time).
      CustomDrawable()
      {
	 //setSupportsDisplayList (false);
	// This contructor intentionally left blank. Duh.
      }

      // I can't say much about the methods below, but OSG seems to expect
      // that we implement them.
      CustomDrawable (const CustomDrawable& pg,
                      const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY)
      { }

      virtual osg::Object* cloneType() const { return new CustomDrawable(); }

      virtual osg::Object* clone(const osg::CopyOp& copyop) const
      { return new CustomDrawable (*this, copyop); }

      // Real work is done here. THERE IS A VERY IMPORTANT THING TO NOTE HERE:
      // the 'drawImplementation()' method receives an 'osg::State' as
      // parameter. This can be used to change the OpenGL state, but changing
      // the OpenGL state here is something to be avoided as much as possible.
      // Do this *only* if it is *absolutely* necessary to make your rendering
      // algorithm work. The "right" (most efficient and flexible) way to change
      // the OpenGL state in OSG is by attaching 'StateSet's to 'Node's and
      // 'Drawable's.
      // That said, the example below shows how to change the OpenGL state in
      // these rare cases in which it is necessary. But always keep in mind:
      // *Change the OpenGL state only if strictly necessary*.
      virtual void drawImplementation (osg::RenderInfo& renderInfo) const
      {
         // Create a 'StateSet' with a "blue diffuse color" attribute.
       //  osg::ref_ptr<osg::Material> blue (new osg::Material);
        // blue->setDiffuse (osg::Material::FRONT_AND_BACK,
          //                 osg::Vec4(0.0, 0.0, 1.0, 1.0));
	osg::ref_ptr<osg::Image> image (osgDB::readImageFile("wood.png"));
	osg::ref_ptr<osg::Texture2D> texture (new osg::Texture2D(image.get()));

	osg::ref_ptr<osg::StateSet> texSS (new osg::StateSet);
	texSS->setTextureAttributeAndModes (0,   // unit
                                          texture.get(),
                                          osg::StateAttribute::ON);
	 //texSS->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	
	//renderInfo.getState()->apply(texSS.get());

         // Change the OpenGL state so that it uses the blue material created
         // above. From now on, everything will be drawn using this new (blue)
         // OpenGL state.
		// renderInfo.getState()->applyAttribute(blue.get());
         // Draw the geometry, using standard OpenGL calls. Note that the
         // 'State' class has methods like 'setVertexPointer()', that can be
         // used when working with vertex arrays (they may be faster than
         // using the OpenGL calls directly).
         // Almost always, the implementation of 'drawImplementation()' will
         // contain just this. (Did I mention that is better to keep the OpenGL
         // state untouched here?)
	
	//RenderBezier();
	DrawBezierSurface();
	//DrawCube();
       // DrawQuad();
         // Since we changed the OpenGL state when calling
         // 'state.applyAttribute()' above, we restore it here. On the OSG
         // mailing list, Robert Osfield himself said that this shouldn't be
         // necessary, but is not a bad idea either ("Being conservative adding
         // an state.apply() would be no bad thing, but its probably not
         // required").
		 renderInfo.getState()->apply();
      }
};



int main (int argc, char* argv[])
{

   osgViewer::Viewer viewer;

   // Create the scene graph. First, add two red spheres.
   osg::ref_ptr<osg::Geode> sgRoot (new osg::Geode);

  
   // Finally, add an instance of our wonderful 'CustomDrawable' to the
   // scene graph.
   osg::ref_ptr<CustomDrawable> drawable (new CustomDrawable);
   sgRoot->addDrawable (drawable.get());

   viewer.addEventHandler(new osgViewer::StatsHandler);

	// add the state manipulator
    	viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

	//Windows size handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
   // Set the data to the viewer
   viewer.setSceneData (sgRoot.get());

   viewer.run();
}
