/********************************************************************************************
Tutorial 14: Simple Shader examples: an NPR rendering effect (Gooch Shading) and an RGB cube*
Copyright (c) Franclin Foping franclin@netcourrier.com
*********************************************************************************************/


#include <osgViewer/Viewer>

#include <osg/ShapeDrawable>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osg/Texture2D>
#include <osgGA/StateSetManipulator>
#include <osg/Program>
#include <osg/PolygonMode>



int main()
{
    	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> root (new osg::Group);
	
	
	//The Box
	osg::ref_ptr<osg::Geode> myshapegeode2 (new osg::Geode);
	osg::ref_ptr<osg::PolygonMode> poly (new osg::PolygonMode);
	myshapegeode2->addDrawable(new osg::ShapeDrawable(new osg::Box(osg::Vec3f(),2,2,2)));
	osg::ref_ptr<osg::StateSet> nodess2 (myshapegeode2->getOrCreateStateSet());

								
	
	//Forcing the wireframe mode on this particular node (the Cube)
	nodess2->setAttribute(poly.get());
	poly->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
	//Its shader defn
	
	osg::ref_ptr<osg::Program> boxProg (new osg::Program);
	 osg::ref_ptr<osg::Shader> boxvertexShader(
      osg::Shader::readShaderFile (osg::Shader::VERTEX, "shaders/RGBCube.vert"));

	 osg::ref_ptr<osg::Shader> boxfragShader(
      osg::Shader::readShaderFile (osg::Shader::FRAGMENT, "shaders/RGBCube.frag"));
	
	//Binding the box shaders to its program
	boxProg->addShader(boxvertexShader.get());
	boxProg->addShader(boxfragShader.get());
	
	//Attaching the shader program to the node
 	nodess2->setAttribute(boxProg.get());
	

	//The second capsule (Multitextured)
	osg::ref_ptr<osg::Geode> myshapegeode3 (new osg::Geode);
	myshapegeode3->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(3,0,0),1,2)));
	osg::ref_ptr<osg::StateSet> nodess3 (myshapegeode3->getOrCreateStateSet());
	
	//Its shader objects
	osg::ref_ptr<osg::Program> capsuleProg (new osg::Program);
	 osg::ref_ptr<osg::Shader> capsulevertexShader(
      osg::Shader::readShaderFile (osg::Shader::VERTEX, "shaders/gooch.vert"));

	 osg::ref_ptr<osg::Shader> capsulefragShader(
      osg::Shader::readShaderFile (osg::Shader::FRAGMENT, "shaders/gooch.frag"));
	
	//Binding the box shaders to its program
	capsuleProg->addShader(capsulevertexShader.get());
	capsuleProg->addShader(capsulefragShader.get());
	
	//Attaching the shader program to the node
 	nodess3->setAttribute(capsuleProg.get());
	
	//Passing the uniform variable representing the texture to the shader
	nodess3->addUniform(new osg::Uniform("LightPosition",osg::Vec3(0,0,10)));
	nodess3->addUniform(new osg::Uniform("SurfaceColor",osg::Vec3(0.75,0.75,0.75)));
	nodess3->addUniform(new osg::Uniform("WarmColor",osg::Vec3(0.6,0,0)));
	nodess3->addUniform(new osg::Uniform("CoolColor",osg::Vec3(0,0,0.6)));
	nodess3->addUniform(new osg::Uniform("DiffuseWarm",0.45f));
	nodess3->addUniform(new osg::Uniform("DiffuseCool", 0.45f));

	//loading texture image object
	osg::ref_ptr<osg::Image> image (osgDB::readImageFile("Fieldstone.jpg"));
	//Bind the image to a 2D texture object
	osg::ref_ptr<osg::Texture2D> tex (new osg::Texture2D(image.get()));
	
	osg::ref_ptr<osg::Image> img2 (osgDB::readImageFile("foo.jpg"));
	osg::ref_ptr<osg::Texture2D> tex2 (new osg::Texture2D(img2.get()));
	
	
	
	//Adding all the shape to the root node for rendering 
	root->addChild(myshapegeode2.get());
	root->addChild(myshapegeode3.get());
	
	// add the state manipulator to the viewer
    	viewer.addEventHandler( new 		osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

	//Stats Event Handler s key
	viewer.addEventHandler(new osgViewer::StatsHandler);

	viewer.setSceneData( root.get() );

	return (viewer.run());
	}
