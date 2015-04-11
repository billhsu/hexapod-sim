#ifndef HEXAPOD_H
#define HEXAPOD_H

#include <thread>
#include <memory>
#include <functional>

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

class HexapodServer
{
public:
    HexapodServer(Hexapod* _hexapod): ctx(1), serverSocket(ctx, ZMQ_STREAM), hexapod(_hexapod){}
    void run()
    {
        serverSocket.bind ("tcp://*:5555");
        std::cout<<"serverSocket.bind()"<<std::endl;
        while (true)
        {
            std::cout<<"Waiting for request.."<<std::endl;
            zmq::message_t request;
            serverSocket.recv (&request);
            std::string requestStr = std::string(static_cast<char*>(request.data()), request.size());
            std::cout<<"Request: "<<requestStr<<std::endl;

            serverSocket.send(request, ZMQ_SNDMORE);
        }
    }
private:
    zmq::context_t ctx;
    zmq::socket_t serverSocket;
    Hexapod* hexapod;
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
    void setServoPercent(int rigId, int jointId, btScalar targetPercent, float deltaMs);
    void setMotorTargets(btScalar deltaTime);

};


#endif
