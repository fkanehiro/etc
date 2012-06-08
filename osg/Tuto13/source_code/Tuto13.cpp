

/*************************************
Tutorial 13: An example of using the Switch Node
Copyright (c) Franclin Foping franclin@netcourrier.com

*************************************/



#include <osg/Texture2D>

#include <osg/Material>
#include <osg/ShapeDrawable>
#include <osgGA/StateSetManipulator>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgDB/ReadFile>
#include <osg/LightSource>
#include <osg/Switch>
#include <osgGA/GUIEventHandler>
#include <osg/PositionAttitudeTransform>


using namespace osg;


class SwitchHandler : public osgGA::GUIEventHandler
{
 public:

        SwitchHandler():
            _childNum(0) {}

        virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& , osg::Object* object, osg::NodeVisitor* )
        {
            osg::Switch* sw = dynamic_cast<osg::Switch*>(object);
            if (!sw) return false;

            if (ea.getHandled()) return false;

            switch(ea.getEventType())
            {
                case(osgGA::GUIEventAdapter::KEYDOWN):
                {
                    if (ea.getKey()=='n')
                    {
                        ++_childNum;
                        if (_childNum >= sw->getNumChildren()) _childNum = 0;

                        sw->setSingleChildOn(_childNum);
			return true;
                    }                
                    break;
                }
                default:
                    break;
            }
            return false;
        }

    private:

        virtual ~SwitchHandler() {}
        unsigned int    _childNum;
};

namespace World
{
enum Objects
{
	TERRAIN = 1,
	BOX = 2,
	SPHERE = 4,
	CAPSULE = 8
	
};

ref_ptr<Geode> WorldObjects(unsigned int mask)
{
	//Our world consists of four objects (drawables)
	// 1 - The terrain 
	// 2 - The Box
 	// 3 - The Sphere
	// 4 - The capsule
	//NOTE: Adding the Box object will override the previous state set, that is why the texture changes, it is not a bug tough, I am fully aware of it!!!
	// As a way around it, you may consider making use of multitexturing on the terrain node. It can be fun!!
	
	ref_ptr<Geode> objectGeode (new Geode);

	if (mask & TERRAIN)
	{
		objectGeode->addDrawable(new ShapeDrawable(new Box(Vec3f(0,0,-1),5,7,0.05f)));
		objectGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, new 			 osg::Texture2D(osgDB::readImageFile("terrain.jpg")));
	}
	if (mask & BOX)
	{
		objectGeode->addDrawable(new ShapeDrawable(new Box(Vec3(-1,1,0),1,1,1)));
		objectGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, new Texture2D(osgDB::readImageFile("wood.png")));
	}
	if (mask & SPHERE)
	{
		objectGeode->addDrawable(new ShapeDrawable(new Sphere(Vec3(1,2,0),0.5f)));
		objectGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, new Texture2D(osgDB::readImageFile("wood.png")));
	}
	if (mask & CAPSULE)
	{
		objectGeode->getOrCreateStateSet()->setTextureAttributeAndModes(0, new Texture2D(osgDB::readImageFile("wood.png")));
		objectGeode->addDrawable(new ShapeDrawable(new Capsule(Vec3(1,-1,0),0.3f,0.5f)));
	}

   return objectGeode;
};
};

int main()
{
	//Declaration of the objects that will form our scene
	osgViewer::Viewer viewer;
	ref_ptr<Group> scene (new Group);
	ref_ptr<Switch> switchNode (new Switch);
	switchNode->setEventCallback(new SwitchHandler);
	switchNode->setDataVariance(Object::DYNAMIC);
	
	ref_ptr<Geode> lightMarkerGeode (new Geode);
	
	ref_ptr<Node> fishnode (osgDB::readNodeFile("Oyanirami0.3ds"));
	fishnode->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL,StateAttribute::ON);
	ref_ptr<PositionAttitudeTransform> fishPAT (new PositionAttitudeTransform);
	fishPAT->addChild(fishnode.get());
	fishPAT->setPosition(osg::Vec3(0,0,1));
	fishPAT->setScale(osg::Vec3(2,2,2));


	//We add these objects to the switch handler
	switchNode->addChild(World::WorldObjects(World::TERRAIN).get(), true);
	switchNode->addChild(World::WorldObjects(World::TERRAIN|World::BOX).get(),false);
	switchNode->addChild(World::WorldObjects(World::TERRAIN|World::BOX|World::SPHERE).get(),false);
	switchNode->addChild(World::WorldObjects(World::TERRAIN|World::BOX|World::SPHERE|World::CAPSULE).get(),false);
	//Next we define the material property of our objects 
	// material
        ref_ptr<Material> matirial = new Material;
        matirial->setColorMode(Material::DIFFUSE);
        matirial->setAmbient(Material::FRONT_AND_BACK, Vec4(0, 0, 0, 1));
        matirial->setSpecular(Material::FRONT_AND_BACK, Vec4(1, 1, 1, 1));
        matirial->setShininess(Material::FRONT_AND_BACK, 64);
        scene->getOrCreateStateSet()->setAttributeAndModes(matirial.get(), StateAttribute::ON);
	scene->setDataVariance(Object::DYNAMIC);

	
	//Main light source 
	Vec3 lightPosition(-1,-1,2); 
	ref_ptr<LightSource> ls = new LightSource;
	ls->setDataVariance(Object::DYNAMIC);
  	ls->getLight()->setPosition(Vec4(lightPosition,1));
	ls->getLight()->setAmbient(Vec4(0.2,0.2,0.2,1.0));
        ls->getLight()->setDiffuse(Vec4(0.8,0.8,0.8,1.0));

	

	//Adding the terrain and object nodes to the root node
	scene->addChild(switchNode.get());
	scene->addChild(lightMarkerGeode.get());
	scene->addChild(fishPAT.get());
	scene->addChild(ls.get());

	//Light marker sphere
	lightMarkerGeode->addDrawable(new ShapeDrawable(new Sphere(lightPosition + Vec3(0,0,0.3f),0.1f)));

	
	viewer.setSceneData( scene.get() );
	
	//Stats Event Handler s key
	viewer.addEventHandler(new osgViewer::StatsHandler);


	// add the state manipulator
    	viewer.addEventHandler( new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()) );

	//Windows size handler
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);

	
	return (viewer.run());
}


