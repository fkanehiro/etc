#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>

//#include <osgViewer/ViewerEventHandlers>
//#include <osgGA/StateSetManipulator>



int main()
{
	//Creating the viewer	
    	osgViewer::Viewer viewer ;

	//Creating the root node
	osg::ref_ptr<osg::Group> root (new osg::Group);
	
	//The geode containing our shpae
   	osg::ref_ptr<osg::Geode> myshapegeode (new osg::Geode);
	
	//Our shape: a capsule, it could have been any other geometry (a box, plane, cylinder etc.)
	osg::ref_ptr<osg::Capsule> myCapsule (new osg::Capsule(osg::Vec3f(),1,2));

	//Our shape drawable
	osg::ref_ptr<osg::ShapeDrawable> capsuledrawable (new osg::ShapeDrawable(myCapsule.get()));

	myshapegeode->addDrawable(capsuledrawable.get());

	root->addChild(myshapegeode.get());

	viewer.setSceneData( root.get() ); 
	//viewer.setThreadingModel(osgViewer::Viewer::ThreadingModel::SingleThreaded);
	
	//Stats Event Handler s key
	//viewer.addEventHandler(new osgViewer::StatsHandler);

	//Windows size handler
	//viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	
	// add the state manipulator
    	//viewer.addEventHandler( new 		osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );
	//The viewer.run() method starts the threads and the traversals.
	return (viewer.run());
	}
