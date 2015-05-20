#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "Hexapod.h"


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

#ifndef M_PI_6
#define M_PI_6     0.52359877559
#endif

#ifndef M_PI_8
#define M_PI_8     0.5 * M_PI_4
#endif
static GLDebugDrawer gDebugDrawer;


// LOCAL FUNCTIONS

void vertex(btVector3 &v)
{
    glVertex3d(v.getX(), v.getY(), v.getZ());
}

void drawFrame(btTransform &tr)
{
    const float fSize = 1.f;

    glBegin(GL_LINES);

    // x
    glColor3f(255.f,0,0);
    btVector3 vX = tr*btVector3(fSize,0,0);
    vertex(tr.getOrigin());    vertex(vX);

    // y
    glColor3f(0,255.f,0);
    btVector3 vY = tr*btVector3(0,fSize,0);
    vertex(tr.getOrigin());    vertex(vY);

    // z
    glColor3f(0,0,255.f);
    btVector3 vZ = tr*btVector3(0,0,fSize);
    vertex(tr.getOrigin());    vertex(vZ);

    glEnd();
}

std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) 
{
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

std::vector<std::string> split(const std::string &s, char delim) 
{
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

void HexapodServer::run()
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
        std::vector<std::string> splitCommands = split(requestStr, '#');
        for(int i=0; i<splitCommands.size(); ++i)
        {
            std::cout<<splitCommands[i]<<std::endl;
            std::vector<std::string> command = split(splitCommands[i], 'P');
            if(command.size()!=2) continue;
            int servoId = atoi(command[0].c_str());
            int pwm = atoi(command[1].c_str());
            servoId = servosMap[servoId];
            float anglePercent = (float)(pwm-500)/2000.0f;
            // flip angles for left part
            if(servoId>9 && servoId%3 != 0) {
                anglePercent =  1 - anglePercent;
            }
            hexapod->setServoPercentValue(0, servoId - 1, anglePercent);
        }
        serverSocket.send(request, ZMQ_SNDMORE);
    }
}

class HexapodRig
{
    btDynamicsWorld*    m_ownerWorld;
    btCollisionShape*    m_shapes[BODYPART_COUNT];
    btRigidBody*        m_bodies[BODYPART_COUNT];
    btTypedConstraint*    m_joints[JOINT_COUNT];

    btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
    {
        bool isDynamic = (mass != 0.f);

        btVector3 localInertia(0,0,0);
        if (isDynamic)
            shape->calculateLocalInertia(mass,localInertia);

        btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
        btRigidBody* body = new btRigidBody(rbInfo);

        m_ownerWorld->addRigidBody(body);

        return body;
    }


public:
    HexapodRig (btDynamicsWorld* ownerWorld, const btVector3& positionOffset, bool bFixed)
        : m_ownerWorld (ownerWorld)
    {
        btVector3 vUp(0, 1, 0);

        //
        // Setup geometry
        //
        float fBodySize  = 0.45f;
        float fLegLength = 0.45f;
        float fForeLegLength = 0.75f;
        float fCoxaLength = 0.15f;
        m_shapes[0] = new btBoxShape(btVector3(0.4,0.1,0.6));
        int i;
        for ( i=0; i<NUM_LEGS; i++)
        {
            m_shapes[1 + 3*i] = new btCapsuleShape(btScalar(0.10), btScalar(fLegLength));
            m_shapes[2 + 3*i] = new btCapsuleShape(btScalar(0.08), btScalar(fForeLegLength));
            m_shapes[3 + 3*i] = new btCapsuleShape(btScalar(0.08), btScalar(fCoxaLength));
        }

        //
        // Setup rigid bodies
        //
        float fHeight = 2;
        btTransform offset; offset.setIdentity();
        offset.setOrigin(positionOffset);        

        // root
        btVector3 vRoot = btVector3(btScalar(0.), btScalar(fHeight), btScalar(0.));
        btTransform transform;
        transform.setIdentity();
        transform.setOrigin(vRoot);
        if (bFixed)
        {
            m_bodies[0] = localCreateRigidBody(btScalar(0.), offset*transform, m_shapes[0]);
        } else
        {
            m_bodies[0] = localCreateRigidBody(btScalar(0.02), offset*transform, m_shapes[0]);
        }
        // legs
        for ( i=0; i<NUM_LEGS; i++)
        {
            float fAngle = 2 * M_PI * i / NUM_LEGS;
            float fSin = sin(fAngle);
            float fCos = cos(fAngle);

            btVector3 vBoneOrigin;
            if(i == 0 || i == 1 || i == 2)
            {
                vBoneOrigin = btVector3(-0.5 - fLegLength / 2, fHeight, (i-1)*0.6);
            } else 
            {
                vBoneOrigin = btVector3(0.5 + fLegLength / 2, fHeight, (4-i)*0.6);
            }
            transform.setIdentity();
            transform.setOrigin(vBoneOrigin);
            // thigh
            transform.setRotation(btQuaternion(btVector3(0,0,1), M_PI_2));
            m_bodies[1+3*i] = localCreateRigidBody(btScalar(0.01), offset*transform, m_shapes[1+3*i]);

            if(i == 0 || i == 1 || i == 2)
            {
                vBoneOrigin = btVector3(-0.5 - fLegLength, fHeight - fForeLegLength / 2, (i-1)*0.6);
            } else 
            {
                vBoneOrigin = btVector3(0.5 + fLegLength, fHeight - fForeLegLength / 2, (4-i)*0.6);
            }
            // shin
            transform.setIdentity();
            transform.setOrigin(vBoneOrigin);
            m_bodies[2+3*i] = localCreateRigidBody(btScalar(0.02), offset*transform, m_shapes[2+3*i]);

            // hip joint
            if(i == 0 || i == 1 || i == 2)
            {
                vBoneOrigin = btVector3(-0.5, fHeight, (i-1)*0.6);
            } else 
            {
                vBoneOrigin = btVector3(0.5, fHeight, (4-i)*0.6);
            }
            transform.setIdentity();
            transform.setOrigin(vBoneOrigin);
            m_bodies[3+3*i] = localCreateRigidBody(btScalar(0.02), offset*transform, m_shapes[3+3*i]);
        }

        // Setup some damping on the m_bodies
        for (i = 0; i < BODYPART_COUNT; ++i)
        {
            m_bodies[i]->setDamping(0.05, 0.85);
            m_bodies[i]->setDeactivationTime(0.8);
            //m_bodies[i]->setSleepingThresholds(1.6, 2.5);
            m_bodies[i]->setSleepingThresholds(0, 0);
        }


        //
        // Setup the constraints
        //
        btHingeConstraint* hingeC;
        //btConeTwistConstraint* coneC;

        btTransform localA, localB, localC;

        for ( i=0; i<NUM_LEGS; i++)
        {
            btVector3 vBoneOrigin;
            btQuaternion vQ;
            if(i == 0 || i == 1 || i == 2)
            {
                vBoneOrigin = btVector3(-0.5, 0, (i-1)*0.6);
            } else 
            {
                vBoneOrigin = btVector3(0.5, 0, (4-i)*0.6);
            }
            vQ = btQuaternion(btVector3(1,0,0), M_PI_2);
            // hip joints 1
            localA.setIdentity(); localB.setIdentity();
            localA = btTransform(vQ, vBoneOrigin);
            localB = m_bodies[3+3*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            hingeC = new btHingeConstraint(*m_bodies[0], *m_bodies[3+3*i], localA, localB);
            m_joints[2+3*i] = hingeC;
            hingeC->enableMotor(true);
            hingeC->setLimit(-M_PI_6,M_PI_6);
            hingeC->setMaxMotorImpulse(1000);
            m_ownerWorld->addConstraint(m_joints[2+3*i], true);
            // hip joints 2
            if(i == 0 || i == 1 || i == 2)
            {
                vQ = btQuaternion(btVector3(0,1,0), 0);
            } else 
            {
                vQ = btQuaternion(btVector3(0,1,0), M_PI);
            }
            localA.setIdentity(); localB.setIdentity();
            localA = btTransform(vQ, vBoneOrigin);
            localB = m_bodies[1+3*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            localC = m_bodies[3+3*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            hingeC = new btHingeConstraint(*m_bodies[3+3*i], *m_bodies[1+3*i], localC, localB);
            //hingeC->setLimit(btScalar(-0.1), btScalar(0.1));
            m_joints[3*i] = hingeC;
            hingeC->enableMotor(true);
            hingeC->setLimit(M_PI_6,3*M_PI_6);
            hingeC->setMaxMotorImpulse(1000);
            m_ownerWorld->addConstraint(m_joints[3*i], true);

            // knee joints
            if(i == 0 || i == 1 || i == 2)
            {
                vBoneOrigin = btVector3(-0.5 - fLegLength, 0, (i-1)*0.6);
            } else 
            {
                vBoneOrigin = btVector3(0.5 + fLegLength, 0, (4-i)*0.6);
            }
            localA.setIdentity(); localB.setIdentity(); localC.setIdentity();
            localA = btTransform(vQ, vBoneOrigin);
            localB = m_bodies[1+3*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            localC = m_bodies[2+3*i]->getWorldTransform().inverse() * m_bodies[0]->getWorldTransform() * localA;
            hingeC = new btHingeConstraint(*m_bodies[1+3*i], *m_bodies[2+3*i], localB, localC);
            //hingeC->setLimit(btScalar(-0.01), btScalar(0.01));
            m_joints[1+3*i] = hingeC;
            hingeC->enableMotor(true);
            hingeC->setLimit(M_PI_6 - M_PI_4, -M_PI_6 - M_PI_4);
            hingeC->setMaxMotorImpulse(1000);
            m_ownerWorld->addConstraint(m_joints[1+3*i], true);
        }
    }

    virtual    ~HexapodRig ()
    {
        int i;

        // Remove all constraints
        for ( i = 0; i < JOINT_COUNT; ++i)
        {
            m_ownerWorld->removeConstraint(m_joints[i]);
            delete m_joints[i]; m_joints[i] = 0;
        }

        // Remove all bodies and shapes
        for ( i = 0; i < BODYPART_COUNT; ++i)
        {
            m_ownerWorld->removeRigidBody(m_bodies[i]);
            
            delete m_bodies[i]->getMotionState();

            delete m_bodies[i]; m_bodies[i] = 0;
            delete m_shapes[i]; m_shapes[i] = 0;
        }
    }

    btTypedConstraint** GetJoints() {return &m_joints[0];}

};



void motorPreTickCallback (btDynamicsWorld *world, btScalar timeStep)
{
    Hexapod* hexapod = (Hexapod*)world->getWorldUserInfo();

    hexapod->setMotorTargets(timeStep);
    
}



void Hexapod::initPhysics()
{
    setTexturing(true);
    setShadows(true);

    // Setup the basic world

    m_Time = 0;
    m_fCyclePeriod = 2000.f; // in milliseconds

//    m_fMuscleStrength = 0.05f;
    // new SIMD solver for joints clips accumulated impulse, so the new limits for the motor
    // should be (numberOfsolverIterations * oldLimits)
    // currently solver uses 10 iterations, so:
    m_fMuscleStrength = 0.5f;

    setCameraDistance(btScalar(5.));

    m_collisionConfiguration = new btDefaultCollisionConfiguration();

    m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

    btVector3 worldAabbMin(-10000,-10000,-10000);
    btVector3 worldAabbMax(10000,10000,10000);
    m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

    m_solver = new btSequentialImpulseConstraintSolver;

    m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

    m_dynamicsWorld->setInternalTickCallback(motorPreTickCallback,this,true);
    m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);

    // Setup a big ground box
    {
        btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
        m_collisionShapes.push_back(groundShape);
        btTransform groundTransform;
        groundTransform.setIdentity();
        groundTransform.setOrigin(btVector3(0,-10,0));
        localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
    }

    // Spawn one ragdoll
    btVector3 startOffset(0,0,0);
    spawnHexapodRig(startOffset, false);
    clientResetScene();        
}


void Hexapod::spawnHexapodRig(const btVector3& startOffset, bool bFixed)
{
    HexapodRig* rig = new HexapodRig(m_dynamicsWorld, startOffset, bFixed);
    m_rigs.push_back(rig);
}

void    PreStep()
{

}




void Hexapod::setMotorTargets(btScalar deltaTime)
{

    float ms = deltaTime*1000000.;
    float minFPS = 1000000.f/60.f;
    if (ms > minFPS)
        ms = minFPS;

    m_Time += ms;
    //
    // set per-frame sinusoidal position targets using angular motor (hacky?)
    //    
    for (int r=0; r<m_rigs.size(); r++)
    {

        btScalar fTargetPercent;
        fTargetPercent = (int(m_Time / 1000) % int(m_fCyclePeriod)) / m_fCyclePeriod;
        btScalar fTargetPercent2 = 0.5+fTargetPercent;
        if(fTargetPercent2>=1.0) fTargetPercent2 -= 1.0f;
        for(int i=0; i<JOINT_COUNT; ++i)
        {
            setServoPercent(r, i, servoPercentage[i], ms);
        }

    }

    
}

void Hexapod::setServoPercentValue(int rigId, int jointId, btScalar targetPercent)
{
    servoPercentage[jointId] = targetPercent;
}

void Hexapod::setServoPercent(int rigId, int jointId, btScalar targetPercent, float deltaMs)
{
    btHingeConstraint* hingeC = static_cast<btHingeConstraint*>(m_rigs[rigId]->GetJoints()[jointId]);
    btScalar fCurAngle      = hingeC->getHingeAngle();
    
    btScalar fTargetAngle   = 0.5 * (1 + sin(2 * M_PI * targetPercent));
    btScalar fTargetLimitAngle = hingeC->getLowerLimit() + targetPercent * (hingeC->getUpperLimit() - hingeC->getLowerLimit());
    btScalar fAngleError  = fTargetLimitAngle - fCurAngle;
    btScalar fDesiredAngularVel = 1000000.f * fAngleError/deltaMs;
    hingeC->enableAngularMotor(true, fDesiredAngularVel, 1000);
}
void Hexapod::clientMoveAndDisplay()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    //simple dynamics world doesn't handle fixed-time-stepping
    float deltaTime = 1.0f/60.0f;
    

    if (m_dynamicsWorld)
    {
        m_dynamicsWorld->stepSimulation(deltaTime);
        m_dynamicsWorld->debugDrawWorld();
    }

    renderme(); 

    for (int i=2; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        drawFrame(body->getWorldTransform());
    }

    glFlush();

    glutSwapBuffers();
}

void Hexapod::displayCallback()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

    if (m_dynamicsWorld)
        m_dynamicsWorld->debugDrawWorld();

    renderme();

    glFlush();
    glutSwapBuffers();
}

void Hexapod::keyboardCallback(unsigned char key, int x, int y)
{
    switch (key)
    {
    case '+': case '=':
        m_fCyclePeriod /= 1.1f;
        if (m_fCyclePeriod < 1.f)
            m_fCyclePeriod = 1.f;
        break;
    case '-': case '_':
        m_fCyclePeriod *= 1.1f;
        break;
    case '[':
        m_fMuscleStrength /= 1.1f;
        break;
    case ']':
        m_fMuscleStrength *= 1.1f;
        break;
    case 27:
        exitPhysics();
        exit(0);
        break;
    default:
        DemoApplication::keyboardCallback(key, x, y);
    }    
}



void Hexapod::exitPhysics()
{

    int i;

    for (i=0;i<m_rigs.size();i++)
    {
        HexapodRig* rig = m_rigs[i];
        delete rig;
    }

    //cleanup in the reverse order of creation/initialization

    //remove the rigidbodies from the dynamics world and delete them
    
    for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
    {
        btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState())
        {
            delete body->getMotionState();
        }
        m_dynamicsWorld->removeCollisionObject( obj );
        delete obj;
    }

    //delete collision shapes
    for (int j=0;j<m_collisionShapes.size();j++)
    {
        btCollisionShape* shape = m_collisionShapes[j];
        delete shape;
    }

    //delete dynamics world
    delete m_dynamicsWorld;

    //delete solver
    delete m_solver;

    //delete broadphase
    delete m_broadphase;

    //delete dispatcher
    delete m_dispatcher;

    delete m_collisionConfiguration;    
}