#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <thread>
#include <memory>
#include <functional>
#include <vector>
#include <map>
#include <iostream>
#include <fstream>
#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "zmq.hpp"
#include "zhelpers.hpp"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;
class Hexapod;

#define NUM_LEGS 6
#define BODYPART_COUNT 3 * NUM_LEGS + 1
#define JOINT_COUNT BODYPART_COUNT - 1


class HexapodServer
{
public:
    HexapodServer(Hexapod* _hexapod): ctx(1), serverSocket(ctx, ZMQ_STREAM), hexapod(_hexapod){
        servosMap.clear();
        std::ifstream configFile;
        configFile.open("config.txt");
        int count;
        configFile>>count;
        for(int i=0; i<count; ++i)
        {
            int a,b;
            configFile>>a>>b;
            servosMap[a] = b;
        }
    }
    void run();
private:
    zmq::context_t ctx;
    zmq::socket_t serverSocket;
    Hexapod* hexapod;
    std::map<int, int> servosMap; // first: input servo ID
                                 // second: actual servo ID in this system
};

class Hexapod : public GlutDemoApplication
{
    float m_Time;
    float m_fCyclePeriod; // in milliseconds
    float m_fMuscleStrength;

    btAlignedObjectArray<class HexapodRig*> m_rigs;

    //keep the collision shapes, for deletion/cleanup
    btAlignedObjectArray<btCollisionShape*>    m_collisionShapes;

    btBroadphaseInterface*    m_broadphase;

    btCollisionDispatcher*    m_dispatcher;

    btConstraintSolver*    m_solver;

    btDefaultCollisionConfiguration* m_collisionConfiguration;

    HexapodServer *hexapodServer;

public:
    Hexapod()
    {
        hexapodServer = new HexapodServer(this);
        std::thread t1(std::bind(&HexapodServer::run, hexapodServer));
        t1.detach();
        for(int i=0; i<JOINT_COUNT; ++i)
        {
            servoPercentage[i] = 0.0f;
        }
    }
    void initPhysics();

    void exitPhysics();

    virtual ~Hexapod()
    {
        exitPhysics();
    }

    void spawnHexapodRig(const btVector3& startOffset, bool bFixed);

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
    float servoPercentage[JOINT_COUNT];
    void setServoPercentValue(int rigId, int jointId, btScalar targetPercent);
    void setServoPercent(int rigId, int jointId, btScalar targetPercent, float deltaMs);
    void setMotorTargets(btScalar deltaTime);

};


#endif
