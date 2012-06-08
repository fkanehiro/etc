#include <hrpModel/World.h>

template <class TConstraintForceSolver> class WorldExt : public hrp::WorldBase
{
public:
    TConstraintForceSolver constraintForceSolver;
    
    WorldExt() : constraintForceSolver(*this) { }
    
    virtual void initialize() {
        WorldBase::initialize();
        constraintForceSolver.initialize();
    }
    
    virtual void calcNextState(const std::vector<CollisionDataSequence>& i_cd){
        constraintForceSolver.solve(i_cd);
        WorldBase::calcNextState();
    }
};

