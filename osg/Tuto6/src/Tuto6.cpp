#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/StateSet>

int main()
{
    	osgViewer::Viewer viewer;
	osg::ref_ptr<osg::Group> root (new osg::Group);
	
	osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);
	
	myshapegeode->addDrawable(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3f(),1,2)));
	
	root->addChild(myshapegeode.get());
	
	//Getting the state set of the geode
	osg::ref_ptr<osg::StateSet> nodess (myshapegeode->getOrCreateStateSet());
	//Creating the material object
	osg::ref_ptr<osg::Material> mat (new osg::Material);
	//mat->setShininess(osg::Material::FRONT,96.f);
	
	//Specifying the yellow colour of the object
	mat->setDiffuse(osg::Material::FRONT,osg::Vec4(.9f,.9f,.2f,1.0f));
	//Attaching the newly defined state set object to the node state set
	nodess->setAttribute(mat.get());
	
	viewer.setSceneData( root.get() );

	return (viewer.run());
	}
