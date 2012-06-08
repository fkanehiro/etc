#include "LinkExt.h"

using namespace hrp;

int LinkExt::DRAW_MODE = LinkExt::NONE;
double LinkExt::CONSTRAINT_FORCE_SCALE_LENGTH = 0.001;

void LinkExt::setDrawMode(int nVal)
{
    LinkExt::DRAW_MODE = nVal;
}

int LinkExt::getDrawMode()
{
    return LinkExt::DRAW_MODE;
}

void LinkExt::setScaleLength(double dVal)
{
    LinkExt::CONSTRAINT_FORCE_SCALE_LENGTH = dVal;
}

double LinkExt::getScaleLength()
{
    return LinkExt::CONSTRAINT_FORCE_SCALE_LENGTH;
}

void LinkExt::drawConstraintForce()
{
    if( DRAW_MODE == DRAW_CONSTRAINT_FORCE )
    {
        for( ConstraintForceArray::const_iterator ite = constraintForces.begin();
             ite != constraintForces.end(); ++ite )
        {
            const Vector3& localP = ite->point;
            const Vector3& localF = ite->force;
            glDisable(GL_LIGHTING);
            glColor3d(1,0,1);
            glBegin(GL_LINES);
            glVertex3f(localP[0], localP[1], localP[2]);
            glVertex3f( localP[0] + localF[0] * CONSTRAINT_FORCE_SCALE_LENGTH,
                        localP[1] + localF[1] * CONSTRAINT_FORCE_SCALE_LENGTH,
                        localP[2] + localF[2] * CONSTRAINT_FORCE_SCALE_LENGTH ); 
            glEnd();
            glEnable(GL_LIGHTING);
        }
    }
}
