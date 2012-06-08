#ifndef LINK_EXT_H_INCLUDED
#define LINK_EXT_H_INCLUDED

#include <hrpModel/Link.h>

#ifdef WIN32
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace hrp {

class LinkExt :
    public Link
{
    static int      DRAW_MODE;
    static double   CONSTRAINT_FORCE_SCALE_LENGTH;
public:
    enum enumDrawMode{
        NONE,               
        DRAW_CONSTRAINT_FORCE
    };
    virtual void draw(GLenum mode) = 0;
    static void setDrawMode(int nVal);
    static int getDrawMode();
    static void setScaleLength(double dVal);
    static double getScaleLength();

protected:
    void drawConstraintForce();
};

}

#endif
