#include "Hexapod.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

int main(int argc,char* argv[])
{
        Hexapod demoApp;
        demoApp.initPhysics();
        demoApp.setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
        return glutmain(argc, argv,640,480,"Hexapod Simulator",&demoApp);
}
