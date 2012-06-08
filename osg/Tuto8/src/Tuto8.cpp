#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osgGA/StateSetManipulator>

#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/Geometry>
#include <iostream>
#include <osg/Fog>

osg::ref_ptr<osg::Geode> myQuad ()
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

   // Load texture from file and map it to the triangle
   osg::ref_ptr<osg::Image> image1 (osgDB::readImageFile("Fieldstone.png"));
   if (image1.get() == 0)
   {
      std::cerr << "Error loading 'Fieldstone.png'.\n";
      exit (EXIT_FAILURE);
   }

   osg::ref_ptr<osg::Texture2D> texture1 (new osg::Texture2D);
   texture1->setImage (image1.get());

   osg::ref_ptr<osg::StateSet> stateSet (geode->getOrCreateStateSet());
   stateSet->setTextureAttributeAndModes (0,   // unit
                                          texture1.get(),
                                          osg::StateAttribute::ON);

   // And do the same for the second "texture unit".
   osg::ref_ptr<osg::Image> image2 (osgDB::readImageFile("window04smash.bmp"));
   if (image2.get() == 0)
   {
      std::cerr << "Error loading 'window04smash.png'.\n";
      exit (EXIT_FAILURE);
   }

   osg::ref_ptr<osg::Texture2D> texture2 (new osg::Texture2D);
   texture2->setImage (image2.get());

   stateSet->setTextureAttributeAndModes (1,   // unit
                                          texture2.get(),
                                          osg::StateAttribute::ON);

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
   geode->addDrawable (geometry.get());

   return geode;
}

//This is where we defined the fogging effect

osg::ref_ptr< osg::StateSet> setFogState()
{
	osg::ref_ptr<osg::Fog> fog = new osg::Fog();  //The fog object
	fog->setMode(osg::Fog::LINEAR);		  	//Fog type 			
	fog->setColor(osg::Vec4(0.8,0.8,0.8,0));	//Fog color	
	fog->setStart(120);				//Start position of the fog - distance from the camera to the fog
	fog->setEnd(200); // maximum distance - where the fog terminates.
	osg::ref_ptr< osg::StateSet> fogState (new osg::StateSet);
	fogState->setAttributeAndModes(fog.get(),osg::StateAttribute::ON);
	fogState->setMode(GL_FOG,osg::StateAttribute::ON);
	
	return fogState;

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
	osg::Vec3f quadPos = osg::Vec3f(5,0,5);
	osg::Vec3f terrainScale = osg::Vec3f(0.5f,0.5f,0.5f);

	
	//The geode of the capsule
	osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);

	

	objectPat->addChild(myshapegeode.get());
	objectPat->setPosition(objectPosTrans);
	
	quadPat->addChild(myQuad().get());
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
	osg::ref_ptr<osg::Node> terrainnode (osgDB::readNodeFile("Terrain2.3ds"));
	terrainScaleMAT->addChild(terrainnode.get());
	terrainScaleMAT->setMatrix(terrainScaleMatrix);
	
	//adding the terrain node to the root node
	//root->addChild(myQuad().get());
	root->addChild(quadPat.get());
	root->addChild(objectPat.get());
	root->addChild(terrainScaleMAT.get());
	

	//Adding the fog to the root node
	root->setStateSet(setFogState().get());

	// add the state manipulator
    	viewer.addEventHandler( new 		osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

	 // run optimization over the scene graph
   	//osgUtil::Optimizer optimzer;
  	//optimzer.optimize(root.get());
	
	viewer.setSceneData( root.get() );

	return (viewer.run());
	}
