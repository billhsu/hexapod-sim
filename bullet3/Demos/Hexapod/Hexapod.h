#ifndef HEXAPOD_H
#define HEXAPOD_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class Hexapod : public GlutDemoApplication
{
    float m_Time;
    float m_fCyclePeriod; // in milliseconds
    float m_fMuscleStrength;

    btAlignedObjectArray<class TestRig*> m_rigs;

    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*>    m_collisionShapes;

    btBroadphaseInterface*    m_broadphase;

    btCollisionDispatcher*    m_dispatcher;

    btConstraintSolver*    m_solver;

    btDefaultCollisionConfiguration* m_collisionConfiguration;

public:
    void initPhysics();

    void exitPhysics();

    virtual ~Hexapod()
    {
        exitPhysics();
    }

    void spawnTestRig(const btVector3& startOffset, bool bFixed);

    virtual void clientMoveAndDisplay();

    virtual void displayCallback();

    virtual void keyboardCallback(unsigned char key, int x, int y);

    static DemoApplication* Create()
    {
        Hexapod* demo = new Hexapod();
        demo->myinit();
        demo->initPhysics();
        return demo;
    }
    
    void setMotorTargets(btScalar deltaTime);

};


#endif
