


/****************
Tutorial 11: A billboarding effect
This tutorial will extend the previous one on lighting by adding two billboard quads. 
These are also textured and shaded.
Copyright (c) Franclin Foping franclin@netcourrier.com
***************/
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osgGA/StateSetManipulator>
#include <osgViewer/ViewerEventHandlers>

#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <iostream>
#include <osg/Fog>

#include<osg/Light>
#include<osg/LightSource>  

#include <osg/Billboard>
#include <osgUtil/Optimizer>



osg::ref_ptr<osg::Drawable> myQuad ()
{
   osg::ref_ptr<osg::Geode> geode (new osg::Geode());
   osg::ref_ptr<osg::Geometry> geometry (new osg::Geometry());

   osg::ref_ptr<osg::Vec3Array> vertices (new osg::Vec3Array());

   vertices->push_back (osg::Vec3 ( 1.0, 0.0, 0.0));
   vertices->push_back (osg::Vec3 ( 1.0, 0.0, 1.0));
   vertices->push_back (osg::Vec3 ( 0.0, 0.0, 1.0));
   vertices->push_back (osg::Vec3 (0.0, 0.0, 0.0));

   geometry->setVertexArray (vertices.get());

   // All vertices are white this time (it's hard to see that we have two
   // textures with all the colors...)
   osg::ref_ptr<osg::Vec4Array> colors (new osg::Vec4Array());
   colors->push_back (osg::Vec4 (1.0f, 1.0f, 1.0f, 1.0f));
   geometry->setColorArray (colors.get());
   geometry->setColorBinding (osg::Geometry::BIND_OVERALL);

   osg::ref_ptr<osg::Vec3Array> normals (new osg::Vec3Array());
   normals->push_back (osg::Vec3 (0.0f, -1.0f, 0.0f));
   geometry->setNormalArray (normals.get());
   geometry->setNormalBinding (osg::Geometry::BIND_OVERALL);


   osg::ref_ptr<osg::Vec2Array> texCoords (new osg::Vec2Array());

   texCoords->push_back (osg::Vec2 (0.0, 0.0));
   texCoords->push_back (osg::Vec2 (0.0, 1.0));
   texCoords->push_back (osg::Vec2 (1.0, 1.0));
   texCoords->push_back (osg::Vec2 (1.0, 0.0));

   // Here, the two texture units (0 and 1) share the same texture coordinates.
   geometry->setTexCoordArray (0, texCoords.get());
   geometry->setTexCoordArray (1, texCoords.get());

   // Back to the usual: setup a primitive set and add the geometry to the geode.
   geometry->addPrimitiveSet(
      new osg::DrawArrays (osg::PrimitiveSet::QUADS, // how to render?
                           0,                            // index of first vertex
                           vertices->size()));           // how many vertices?
  // geode->addDrawable (geometry.get());

   return geometry.get();
}



int main()
{
    	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> root (new osg::Group);

	osg::ref_ptr<osg::PositionAttitudeTransform> objectPat (new osg::PositionAttitudeTransform);
	osg::ref_ptr<osg::PositionAttitudeTransform> quadPat (new osg::PositionAttitudeTransform);
	osg::ref_ptr<osg::MatrixTransform> terrainScaleMAT (new osg::MatrixTransform);
	osg::Matrix terrainScaleMatrix;
 	terrainScaleMatrix.makeScale(0.05f,0.05f,0.03f);

	osg::Vec3f objectPosTrans = osg::Vec3f(-1,3,5);
	osg::Vec3f quadPos = osg::Vec3f(5,0,0.5f);
	osg::Vec3f quadPos2 = osg::Vec3f(-5,0,0);
	//osg::Vec3f terrainScale = osg::Vec3f(0.5f,0.5f,0.5f);

	
	//Tuto9: Lighting code
	osg::ref_ptr<osg::Group> lightGroup (new osg::Group);
	osg::ref_ptr<osg::StateSet> lightSS (root->getOrCreateStateSet());
	osg::ref_ptr<osg::LightSource> lightSource1 = new osg::LightSource;
	osg::ref_ptr<osg::LightSource> lightSource2 = new osg::LightSource;
	
	// create a local light.
	osg::Vec4f lightPosition (osg::Vec4f(-5.0,-2.0,3.0,1.0f));
  	osg::ref_ptr<osg::Light> myLight = new osg::Light;
	myLight->setLightNum(1);
	myLight->setPosition(lightPosition);
        myLight->setAmbient(osg::Vec4(0.2f,0.2f,0.2f,1.0f));
        myLight->setDiffuse(osg::Vec4(0.8f,0.8f,0.8f,1.0f));
        myLight->setConstantAttenuation(1.0f);
	lightSource1->setLight(myLight.get());

	lightSource1->setLocalStateSetModes(osg::StateAttribute::ON); 
	lightSource1->setStateSetModes(*lightSS,osg::StateAttribute::ON);
	//osg::StateSet* lightSS (lightGroup->getOrCreateStateSet());
       
	// create a local light.
	osg::Vec4f lightPosition2 (osg::Vec4f(2.0,-1.0,3.0,1.0f));
  	osg::ref_ptr<osg::Light> myLight2 = new osg::Light;
	myLight2->setLightNum(0);
	myLight2->setPosition(lightPosition2);
        myLight2->setAmbient(osg::Vec4(0.2f,0.2f,0.2f,1.0f));
        myLight2->setDiffuse(osg::Vec4(0.8f,0.1f,0.1f,1.0f));
        myLight2->setConstantAttenuation(1.0f);
	            
        lightSource2->setLight(myLight2.get());
	lightSource2->setLocalStateSetModes(osg::StateAttribute::ON); 
	lightSource2->setStateSetModes(*lightSS,osg::StateAttribute::ON);
        

	
    	lightGroup->addChild(lightSource1.get());
	lightGroup->addChild(lightSource2.get());
	//Light markers: small spheres
	osg::ref_ptr<osg::Geode> lightMarkerGeode (new osg::Geode);
	lightMarkerGeode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(-5.0f,-2.0f,3.0f),0.5f)));
	//lightMarkerGeode->getOrCreateStateSet()->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
	
	//Second light marker
	lightMarkerGeode->addDrawable(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3f(2.0f,-1.0f,3.0f),0.5f)));
	
	


	//The geode of the capsule
	osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);

	

	objectPat->addChild(myshapegeode.get());
	objectPat->setPosition(objectPosTrans);
	
	//quadPat->addChild(myQuad().get());
	quadPat->setPosition(quadPos);
	myshapegeode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(),1,2)));
	
	
	
	//Getting the state set of the geode
	osg::ref_ptr<osg::StateSet> nodess (myshapegeode->getOrCreateStateSet());
	
	//loading texture image object
	osg::ref_ptr<osg::Image> image (osgDB::readImageFile("Fieldstone.png"));

	//Bind the image to a 2D texture object
	osg::ref_ptr<osg::Texture2D> tex (new osg::Texture2D);
	tex->setImage(image.get());

	//Release the image memory on the GPU after using it!
	//tex->setUnRefImageDataAfterApply(true);	

	//Applying texture on the object
	
	nodess->setTextureAttributeAndModes(0,tex.get(),osg::StateAttribute::ON);
	
	
	//Loading the terrain node
	osg::ref_ptr<osg::Node> terrainnode (osgDB::readNodeFile("JoeDirt.flt"));
	//osg::ref_ptr<osg::Node> terrainnode (osgDB::readNodeFile("Terrain2.3ds"));
	terrainScaleMAT->addChild(terrainnode.get());
	terrainScaleMAT->setMatrix(terrainScaleMatrix);


	//Tutorial 11: Billboarding stuff
	osg::ref_ptr<osg::Billboard> quadBillBoard = new osg::Billboard();
	osg::ref_ptr<osg::StateSet> billSS (quadBillBoard->getOrCreateStateSet());

	//Adding texture to the billboards
	osg::ref_ptr<osg::Image> image1 (osgDB::readImageFile("foo.png"));
  	 if (image1.get() == 0)
  	 {
   	   std::cerr << "Error loading 'foo.png'.\n";
    	  exit (EXIT_FAILURE);
  	 }

 	osg::ref_ptr<osg::Texture2D> texture (new osg::Texture2D);
  	texture->setImage (image1.get());
	billSS->setTextureAttributeAndModes (0,   // unit
                                         texture.get(),
                                          osg::StateAttribute::ON);

  	root->addChild(quadBillBoard.get());
	
	quadBillBoard->setMode(osg::Billboard::AXIAL_ROT);
  	quadBillBoard->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
  	quadBillBoard->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));	
	
	quadBillBoard->addDrawable(myQuad().get(),quadPos);
	quadBillBoard->addDrawable(myQuad().get(),quadPos2);

	//adding the terrain node to the root node
	//root->addChild(myQuad().get());
	//root->addChild(quadPat.get());
	
	root->addChild(objectPat.get());
	root->addChild(terrainScaleMAT.get());
	
	//Tuto 9: lighting code
	root->addChild(lightGroup.get());
	//Tuto 9: Adding the light marker geode
	root->addChild(lightMarkerGeode.get());

	//Adding the fog to the root node
	//root->setStateSet(setFogState().get());

	// add the state manipulator
    	viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
	
	//Stats Event Handler s key
	viewer.addEventHandler(new osgViewer::StatsHandler);

	//Windows size handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	//Threading Handler activate with the 'm' key
	viewer.addEventHandler(new osgViewer::ThreadingHandler);

	 // run optimization over the scene graph
   	osgUtil::Optimizer optimzer;
  	optimzer.optimize(root.get());
	
	viewer.setSceneData( root.get() );

	return (viewer.run());
	}
