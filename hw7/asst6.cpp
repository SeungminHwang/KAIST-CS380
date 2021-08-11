////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <list>
#include <algorithm>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <memory>
#include <stdexcept>

#include <GL/glew.h>
#ifdef __APPLE__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"

#include "rigtform.h"
#include "arcball.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"
#include "geometry.h"

#include "sgutils.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff


// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event


// --------- Materials
// This should replace all the contents in the Shaders section, e.g., g_numShaders, g_shaderFiles, and so on
static shared_ptr<Material> g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat;

shared_ptr<Material> g_overridingMaterial;


// --------- Geometry
typedef SgGeometryShapeNode MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_sphereRobot;
static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_light1Node, g_light2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode = g_skyNode; // used later when you do picking
// --------- Scene
static const Cvec3 g_light1(2.0, 3.0, 10.0), g_light2(-2, 0.5, -5.0);  // define two lights positions in world space
//static RigTForm g_skyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
//static RigTForm g_objectRbt[2] = { RigTForm(Cvec3(-0.8,0,0)), RigTForm(Cvec3(0.8,0,0)) };  // currently only 1 obj is defined
//static Cvec3f g_objectColors[2] = { Cvec3f(1, 0, 0), Cvec3f(0, 1, 0) };

static RigTForm g_sphereRbt = RigTForm(Cvec3(0.0, 0.0, 0.0));
static Cvec3f g_sphereColor = Cvec3f(0.1, 1, 0.1);

static int g_currentEyeCode = 0; // 0 for sky, 1 for cube 1, ... and so forth 
static int g_numEyes = 3; // number of possible perspectives

static RigTForm g_currentRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
static RigTForm g_worldRbt = RigTForm(Cvec3(0.0, 0.0, 0.0)); // temp
static int g_skymode = -1; // -1 for world-sky, 1 for sky-sky frame

static double g_arcballScreenRadius = 0.25 * min(g_windowHeight, g_windowWidth);
static double g_arcballScale = getScreenToEyeScale((inv(g_currentRbt) * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight); // z, FovY, H

static bool g_pickAvailable = false;

// hw5 animation
static vector<shared_ptr<SgRbtNode> > g_allRbts;
static list<vector<RigTForm> > g_keyframes;
static list<vector<RigTForm> >::iterator g_keyframeIt;

static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback
static int g_currKeyframeCode = -1; // fix?

static bool g_aniRunning = false;
static bool g_aniHALT = false;


///////////////// END OF G L O B A L S //////////////////////////////////////////////////



static void initGround() {
    int ibLen, vbLen;
    getPlaneVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makePlane(g_groundSize * 2, vtx.begin(), idx.begin());
    g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
    int ibLen, vbLen;
    getCubeVbIbLen(vbLen, ibLen);

    // Temporary storage for cube Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makeCube(1, vtx.begin(), idx.begin());
    g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
    int ibLen, vbLen;
    getSphereVbIbLen(20, 10, vbLen, ibLen);

    // Temporary storage for sphere Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(g_arcballScale * g_arcballScreenRadius, 20, 10, vtx.begin(), idx.begin());
    g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initSphereRobot() {
    int ibLen, vbLen;
    getSphereVbIbLen(20, 10, vbLen, ibLen);

    // Temporary storage for sphere Geometry
    vector<VertexPNTBX> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(1, 20, 10, vtx.begin(), idx.begin());
    g_sphereRobot.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}



// takes a projection matrix and send to the the shaders
inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
    uniforms.put("uProjMatrix", projMatrix);
}


// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
    if (g_windowWidth >= g_windowHeight)
        g_frustFovY = g_frustMinFov;
    else {
        const double RAD_PER_DEG = 0.5 * CS175_PI / 180;
        g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
    }
}

static Matrix4 makeProjectionMatrix() {
    return Matrix4::makeProjection(
        g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
        g_frustNear, g_frustFar);
}


static void drawStuff(bool picking) {

    // Declare an empty uniforms
    Uniforms uniforms;

    // build & send proj. matrix to vshader
    const Matrix4 projmat = makeProjectionMatrix();
    sendProjectionMatrix(uniforms, projmat);

    // use the skyRbt as the eyeRbt
    const RigTForm eyeRbt = g_currentRbt;//g_skyRbt;
    const RigTForm invEyeRbt = inv(eyeRbt);

    Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
    Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();

    const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(light1, 1)); // g_light1 position in eye coordinates
    const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(light2, 1)); // g_light2 position in eye coordinates
    
    uniforms.put("uLight", eyeLight1);
    uniforms.put("uLight2", eyeLight2);
    //safe_glUniform3f(uniforms.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
    //safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);


    // draw cubes
    if (!picking) {
        Drawer drawer(invEyeRbt, uniforms);
        g_world->accept(drawer);

        // draw arcball as part of asst3
        // draw sphere


        bool condWorldSky = (g_currentEyeCode == 0 && g_skymode == -1);
        //bool condSkySky = (g_currentEyeCode == 0 && g_skymode == 1 && g_currentObjCode != 0);
        //bool condObject = (g_currentEyeCode != 0 && g_currentEyeCode != g_currentObjCode && g_currentObjCode != 0);
        bool condPick = g_currentPickedRbtNode != NULL; // if some is picked

        bool condRobot1View = (g_currentEyeCode == 1) && (g_currentPickedRbtNode != g_robot1Node) && (g_currentPickedRbtNode != NULL) && condPick;
        bool condRobot2View = (g_currentEyeCode == 2) && (g_currentPickedRbtNode != g_robot2Node) && (g_currentPickedRbtNode != NULL) && condPick;


        if (condWorldSky || condRobot1View || condRobot2View)
        {
            //cout << "CP2" << endl;
            if(g_currentPickedRbtNode != NULL)
                g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
            //cout << "CP3" << endl;

            // arcball scale update
            if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))) // don't change size before the buttons are released
            {
                //cout << g_arcballScale << endl;
                g_arcballScale = getScreenToEyeScale((invEyeRbt * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight); // z, FovY, H
            }
            initSphere();
            //g_arcballScale = getScreenToEyeScale((invEyeRbt * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight); // z, FovY, H


            Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_sphereRbt);
            //Matrix4 NMVM = normalMatrix(MVM);
            sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));
            //safe_glUniform3f(curSS.h_uColor, g_sphereColor[0], g_sphereColor[1], g_sphereColor[2]);

            //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            g_arcballMat->draw(*g_sphere, uniforms);
            //glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }
    else {
        cout << "picking process" << endl;
        Picker picker(invEyeRbt, uniforms);

        // set overiding material to our picking material
        g_overridingMaterial = g_pickingMat;

        g_world->accept(picker);

        // unset the overriding material
        g_overridingMaterial.reset();

        glFlush();
        g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
        //g_currentRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
        if (g_currentPickedRbtNode != g_groundNode)
        {
            cout << "picking done" << endl;
            //g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
        }
        if (g_currentPickedRbtNode == g_groundNode)
            g_currentPickedRbtNode = shared_ptr<SgRbtNode>();   // set to NULL
    }

}

static void display() {
    // No more glUseProgram

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    drawStuff(false); // no more curSS

    glutSwapBuffers();

    checkGlErrors();
}

static void reshape(const int w, const int h) {
    g_windowWidth = w;
    g_windowHeight = h;
    glViewport(0, 0, w, h);
    cerr << "Size of window is now " << w << "x" << h << endl;
    g_arcballScreenRadius = 0.25 * min(g_windowWidth, g_windowHeight);

    updateFrustFovY();
    glutPostRedisplay();
}

static void motion(const int x, const int y) {
    const double dx = x - g_mouseClickX;
    const double dy = g_windowHeight - y - 1 - g_mouseClickY;

    //bool isEgoMotion = (g_currentObjCode == 0 && g_skymode == +1) || (g_currentObjCode != 0 && g_currentEyeCode == g_currentObjCode);
    //cout << isEgoMotion << endl;

    bool isEgoMotion = 
        ((g_currentEyeCode == 1) && ((g_currentPickedRbtNode == g_robot1Node) || (g_currentPickedRbtNode == NULL)))||
        ((g_currentEyeCode == 2) && ((g_currentPickedRbtNode == g_robot2Node) || (g_currentPickedRbtNode == NULL)))
        || (g_currentEyeCode == 0 && g_skymode == +1);

    RigTForm m;
    if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
        //m = Matrix4::makeXRotation(-dy) * Matrix4::makeYRotation(dx);
        if (isEgoMotion)
        {
            m = RigTForm(Quat(cos(-dy * 3.14159265 / 180 / 2), sin(-dy * 3.14159265 / 180 / 2), 0, 0))
                * RigTForm(Quat(cos(dx * 3.14159265 / 180 / 2), 0, sin(dx * 3.14159265 / 180 / 2), 0));
        }
        // arcball rotation
        else if (g_currentPickedRbtNode == NULL) // sky mode?
        {
            //cout << g_currentEyeCode << "this is!" << endl;
            double R = g_arcballScreenRadius;
            Cvec2 O = getScreenSpaceCoord(
                (inv(g_currentRbt) * g_sphereRbt).getTranslation(), makeProjectionMatrix(),
                g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);

            Cvec2 s1 = Cvec2(g_mouseClickX, g_mouseClickY);
            Cvec2 s2 = Cvec2(x, g_windowHeight - y - 1);


            // comment: If compilation is failed due to this, please check once agin with different compiler
            // I heared that this is one of standard feature in Modern C++
            // Tested environment: Windows; configuration provided by VS2019
            // I used lambda expression for reducing duplicated codes.
            auto getV = [](Cvec2 s, Cvec2 O, double R) {
                Cvec3 v;
                Cvec2 v_2d = s - O;
                double zSq = R * R - norm2(v_2d);// positive for inside, otherwise out
                if (zSq > 0)
                {
                    v = Cvec3(v_2d[0], v_2d[1], sqrt(zSq)).normalize();
                }
                else // outside the arcball
                {
                    v_2d = v_2d.normalize() * R;
                    v = Cvec3(v_2d[0], v_2d[1], 0).normalize();
                }
                return v;
            };
            Cvec3 v1 = getV(s1, O, R);
            Cvec3 v2 = getV(s2, O, R);

            m = RigTForm(Quat(0, v2[0], v2[1], v2[2]) * Quat(0, -v1[0], -v1[1], -v1[2]));
        }
        else
        {
            // only at the front of eye(z_e < 0) will be moved(with picked object origin).
            bool cond_ObjIsBack = false;
            Cvec4 objOrigin = (0, 0, 0, 1);
            //cout << "CP4" << endl;
            Cvec4 objOriginEyeCoord = inv(g_currentRbt) * getPathAccumRbt(g_world, g_currentPickedRbtNode)*objOrigin;
            //cout << "CP5" << endl;
            //cout << objOriginEyeCoord[0] << "," << objOriginEyeCoord[1] << "," << objOriginEyeCoord[2] << endl;
            cond_ObjIsBack = (objOriginEyeCoord[2] > 0);
            if (cond_ObjIsBack) // If trying to manipulate 
            {
                m = RigTForm(Quat(1, 0, 0, 0));
            }
            else {

                double R = g_arcballScreenRadius;
                Cvec2 O = getScreenSpaceCoord(
                    (inv(g_currentRbt) * g_sphereRbt).getTranslation(), makeProjectionMatrix(),
                    g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);

                Cvec2 s1 = Cvec2(g_mouseClickX, g_mouseClickY);
                Cvec2 s2 = Cvec2(x, g_windowHeight - y - 1);


                // comment: If compilation is failed due to this, please check once agin with different compiler
                // I heared that this is one of standard feature in Modern C++
                // Tested environment: Windows; configuration provided by VS2019
                // I used lambda expression for reducing duplicated codes.
                auto getV = [](Cvec2 s, Cvec2 O, double R) {
                    Cvec3 v;
                    Cvec2 v_2d = s - O;
                    double zSq = R * R - norm2(v_2d);// positive for inside, otherwise out
                    if (zSq > 0)
                    {
                        v = Cvec3(v_2d[0], v_2d[1], sqrt(zSq)).normalize();
                    }
                    else // outside the arcball
                    {
                        v_2d = v_2d.normalize() * R;
                        v = Cvec3(v_2d[0], v_2d[1], 0).normalize();
                    }
                    return v;
                };
                Cvec3 v1 = getV(s1, O, R);
                Cvec3 v2 = getV(s2, O, R);

                m = RigTForm(Quat(0, v2[0], v2[1], v2[2]) * Quat(0, -v1[0], -v1[1], -v1[2]));
            }
        }
    }

    else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
        //m = Matrix4::makeTranslation(Cvec3(dx, dy, 0)* 0.01);
        if (isEgoMotion) { m = RigTForm(Cvec3(dx, dy, 0) * 0.01); }
        else { m = RigTForm(Cvec3(dx, dy, 0) * g_arcballScale); }

    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
        //m = Matrix4::makeTranslation(Cvec3(0, 0, -dy) * 0.01);
        if (isEgoMotion) { m = RigTForm(Cvec3(0, 0, -dy) * 0.01); }
        else { m = RigTForm(Cvec3(0, 0, -dy) * g_arcballScale); }
    }


    if (g_mouseClickDown) {
        if (g_currentEyeCode == 0 && g_skymode == -1  && (g_currentPickedRbtNode == NULL))
        {
            RigTForm a, inva;
            a = transFact(g_worldRbt) * linFact(g_currentRbt);
            inva = inv(a);
            //g_skyRbt = a * inv(m) * inva * g_skyRbt;
            g_skyNode->setRbt(a * inv(m) * inva * g_skyNode->getRbt());
            g_currentRbt = getPathAccumRbt(g_world, g_skyNode);// ->getRbt();
            g_sphereRbt = RigTForm();
        }

        else if (isEgoMotion)
        {
            if(g_currentEyeCode == 0)
            {
                RigTForm a, inva;
                a = transFact(g_currentRbt) * linFact(g_currentRbt);
                inva = inv(a);

                RigTForm m1 = transFact(m);
                RigTForm m2 = inv(linFact(m));

                //g_skyRbt = a * m1 * m2 * inva * g_skyRbt;
                g_skyNode->setRbt(a * m1 * m2 * inva * g_skyNode->getRbt());
                g_currentRbt = getPathAccumRbt(g_world, g_skyNode); //g_skyNode->getRbt();
                //g_currentRbt = g_skyRbt;

            }
            else if (g_currentEyeCode == 1) // robot1 ego motion
            {
                //cout << "robot1-ego" << endl;
                RigTForm a, inva;
                a = g_robot1Node->getRbt();
                inva = inv(a);
                m = transFact(m) * inv(linFact(m));
                g_robot1Node->setRbt(a * m * inva * g_robot1Node->getRbt());
                g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
            }
            else if (g_currentEyeCode == 2) // robot2 ego motion
            {
                //cout << "blue, here" << endl;
                RigTForm a, inva, as,c_L, L;
                a = g_robot2Node->getRbt();
                inva = inv(a);
                m = transFact(m) * inv(linFact(m));
                g_robot2Node->setRbt(a * m * inva * g_robot2Node->getRbt());
                g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
            }
        }
        
        else if (g_currentPickedRbtNode != g_groundNode && g_currentPickedRbtNode != NULL && g_currentPickedRbtNode != g_skyNode)
        //else // not in special motions
        {
            RigTForm a, as, c_L, L;
            
            a = transFact(getPathAccumRbt(g_world, g_currentPickedRbtNode)) * linFact(g_currentRbt);
            c_L = getPathAccumRbt(g_world, g_currentPickedRbtNode);
            L = inv(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) * c_L;
            as = L * inv(c_L) * a;
            g_currentPickedRbtNode->setRbt(as* m* inv(as)* g_currentPickedRbtNode->getRbt());
            //g_currentRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);


            //a = transFact(g_currentPickedRbtNode->getRbt()) * linFact(g_currentRbt);
            //a = transFact(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) * linFact(g_currentRbt);
            //a = inv(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) * a;
            //inva = inv(a);
            //g_currentPickedRbtNode->setRbt(a* m* inva* g_currentPickedRbtNode->getRbt());
            g_sphereRbt = g_currentPickedRbtNode->getRbt();

        }

        glutPostRedisplay(); // we always redraw if we changed the scene
    }

    g_mouseClickX = x;
    g_mouseClickY = g_windowHeight - y - 1;
}

static void pick() {
    // We need to set the clear color to black, for pick rendering.
    // so let's save the clear color
    GLdouble clearColor[4];
    glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

    glClearColor(0, 0, 0, 0);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // No more glUseProgram
    drawStuff(true); // no more curSS

    // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
    // to see result of the pick rendering pass
    // glutSwapBuffers();

    //Now set back the clear color
    glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

    checkGlErrors();
}



static void mouse(const int button, const int state, const int x, const int y) {
    g_mouseClickX = x;
    g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

    g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
    g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
    g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

    g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
    g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
    g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

    g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;

    //page 5
    // if the click was not over any robot, then the eye's transform node is activated
    // If over the robot, the chosen robot part is picked and its parent node is activated

    //modified
    if (g_mouseLClickButton && g_pickAvailable) // pick mode actiavated + Lmouse click
    {
        pick();
        g_pickAvailable = !g_pickAvailable; // only for first click
    }
    if (g_currentPickedRbtNode == NULL)
    {
        g_sphereRbt = RigTForm(Cvec3(0, 0, 0));
        //g_currentPickedRbtNode = g_groundNode;
    }
    else
    {
        g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);
    }


    glutPostRedisplay();//added
}


// hw5 animation
/*
void quatprint(Quat R1)
{
    cout << R1[0] << " " << R1[1] << " " << R1[2] << " " << R1[3] << endl;
}*/

Quat quat_pow(Quat Q, float alpha)
{
    if (Q[0] < 0) // for cn
        Q = Quat(-Q[0], -Q[1], -Q[2], -Q[3]);
    float temp_cos = Q[0];
    //cout << "temp_cos: " << temp_cos << endl;
    float temp_sin = sqrt(1 - temp_cos * temp_cos);

    
    Quat powered;
    if (temp_sin == 0)
    {   //cout << "theta: " << theta << endl;
        powered = Quat(cos(0 * alpha), 0, 0, 0);
    }
    else
    {
        double theta = atan2(temp_sin, temp_cos);
        
        powered = Quat(cos(theta * alpha), sin(theta * alpha) * (Q[1] / temp_sin),
            sin(theta * alpha) * (Q[2] / temp_sin), sin(theta * alpha) * (Q[3] / temp_sin));
    }

    //quatprint(powered);
    //cout << temp_sin << endl;
    return powered;
}
Quat slerp(Quat R0, Quat R1, float alpha)
{
    //cout << "CP1" << endl;
    
    Quat temp = R1 * inv(R0);
    //cout << "CP2" << endl;
    if (temp[0] < 0) // for cn
        temp = Quat(-temp[0], -temp[1], -temp[2], -temp[3]);

    Quat powered = quat_pow(temp, alpha);

    Quat R = powered * R0;

    return R;
}



bool interpolateAndDisplay(float t) {

    int floor_t = (int)t;
    float alpha = t - floor_t;

    //cout << "[t]: " << floor_t << ' ' << g_keyframes.size()<< endl;

    list<vector<RigTForm> >::iterator it0, it1, it2, it3, temp_begin, temp_end;

    temp_begin = g_keyframes.begin();
    temp_end = g_keyframes.end();

    //cout << floor_t << endl;
    it0 = it1 = it2 = it3 = temp_begin;// g_keyframes.begin();
    advance(it0, floor_t);
    advance(it1, floor_t + 1);
    advance(it2, floor_t + 2);
    advance(it3, floor_t + 3);
    if (it3 == temp_end)
    {
        //cout << "here!" << endl;
        return true;
    }


    vector<RigTForm> F0 = *it0;
    vector<RigTForm> F1 = *it1;
    vector<RigTForm> F2 = *it2;
    vector<RigTForm> F3 = *it3;

    vector<RigTForm>::iterator it_a0 = F0.begin();
    vector<RigTForm>::iterator it_a1 = F1.begin();
    vector<RigTForm>::iterator it_a2 = F2.begin();
    vector<RigTForm>::iterator it_a3 = F3.begin();

    //cout << floor_t + 1 << endl;
    // do the interpolation task for all Rbts
    for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++it_a0, ++it_a1, ++it_a2, ++it_a3)
    {
        RigTForm interpolatedRbt = RigTForm(Cvec3(0,0,0));

        RigTForm A0 = (*it_a0); // -1
        RigTForm A1 = (*it_a1); // 0
        RigTForm A2 = (*it_a2); // 1
        RigTForm A3 = (*it_a3); // 2

        Cvec3 T0 = A0.getTranslation();
        Quat R0 = A0.getRotation();
        Cvec3 T1 = A1.getTranslation();
        Quat R1 = A1.getRotation();
        Cvec3 T2 = A2.getTranslation();
        Quat R2 = A2.getRotation();
        Cvec3 T3 = A3.getTranslation();
        Quat R3 = A3.getRotation();

        // CRS
        Cvec3 d = (T2 - T0)/6 + T1;
        Cvec3 e = -(T3 - T1) / 6 + T2;
        Cvec3 T = T1 * (1 - alpha) * (1 - alpha) * (1 - alpha)
            + d * alpha * (1 - alpha) * (1 - alpha) * 3 + e * alpha * alpha * (1 - alpha) * 3 + T2 * alpha * alpha * alpha;
        //T = T1 * (1 - alpha) + T2 * alpha;

        //slerp
        Quat d_quat = quat_pow(R2 * inv(R0), 1/6)*R1;
        Quat e_quat = quat_pow(R3 * inv(R1), -1/6)*R2;
        //cout << norm2(d_quat) << " " << norm2(e_quat) << endl;
        //quatprint(R1);
        //quatprint(d_quat);
        //quatprint(e_quat);
        //quatprint(R2);
        //cout << "-----------" << endl;
        //problem!
        Quat p01 = slerp(R1, d_quat, alpha);
        Quat p12 = slerp(d_quat, e_quat, alpha);
        Quat p23 = slerp(e_quat, R2, alpha);
        //quatprint(p01);
        //quatprint(p12);
        //quatprint(p23);
        //cout << "-----------" << endl;


        
        Quat p012 = slerp(p01, p12, alpha);
        Quat p123 = slerp(p12, p23, alpha);
        //quatprint(p012);
        //quatprint(p123);
        //cout << "-----------" << endl;

        Quat R = slerp(p012, p123, alpha);
        //quatprint(R);
        //R = slerp(R1, R2, alpha);
        
        
        interpolatedRbt.setRotation(R);
        interpolatedRbt.setTranslation(T);// = RigTForm(T, R);

        (*it)->setRbt(interpolatedRbt); // load interpolated Rbt
    }

    //cout << "time: " << t << endl;


    return false;
}

//fix
static void animateTimerCallback(int ms) {
    float t = (float)ms / (float)g_msBetweenKeyFrames;

    bool endReached = interpolateAndDisplay(t);
    
    // the eyeframe will not changed
    if (g_currentEyeCode == 0){
        g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
    }
    else if (g_currentEyeCode == 1){
        g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
    }
    else if (g_currentEyeCode == 2){
        g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
    }
    
    glutPostRedisplay();
    if (!endReached && !g_aniHALT)
        glutTimerFunc(1000 / g_animateFramesPerSecond, animateTimerCallback, ms + 1000 / g_animateFramesPerSecond);
    else {
        //cout << "END!" << endl;
        g_aniRunning = false;
        g_currKeyframeCode = g_keyframes.size() -2;
        g_keyframeIt = g_keyframes.end();
        g_keyframeIt--;
        g_keyframeIt--;
        vector<RigTForm>::iterator keyframIt = (*g_keyframeIt).begin();
        for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++keyframIt)
        {
            (*it)->setRbt(*keyframIt); // load saved Rbt
        }

        if (g_currentEyeCode == 0) {
            g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
        }
        else if (g_currentEyeCode == 1) {
            g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
        }
        else if (g_currentEyeCode == 2) {
            g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
        }
        glutPostRedisplay();

        cout << "Now at frame[" << g_currKeyframeCode<<']' << endl;
    }
}


static void keyboard(const unsigned char key, const int x, const int y) {
    switch (key) {
    case 27:
        exit(0);                                  // ESC
    case 'h':
        cout << " ============== H E L P ==============\n\n"
            << "h\t\thelp menu\n"
            << "s\t\tsave screenshot\n"
            << "f\t\tToggle flat shading on/off.\n"
            << "o\t\tCycle object to edit\n"
            << "v\t\tCycle view\n"
            << "drag left mouse to rotate\n" << endl;
        break;
    case 's':
        glFlush();
        writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
        break;
    case 'p':
        g_pickAvailable = !g_pickAvailable; // pick availability status will be filpped.
        if (g_pickAvailable)
            cout << "pick mode on" << endl;
        else
            cout << "pick mode off" << endl;
        break;

    case 'v':
        g_currentEyeCode = (g_currentEyeCode + 1) % g_numEyes;

        if (g_currentEyeCode == 0) // skyRbt
        {
            //g_currentPickedRbtNode = NULL;
            g_currentRbt = getPathAccumRbt(g_world, g_skyNode);// g_skyRbt;
            cout << "Active eye is Sky" << endl;
        }
        else if (g_currentEyeCode == 1) // Robot1
        {
            //g_currentPickedRbtNode = g_robot1Node;
            g_currentRbt = getPathAccumRbt(g_world, g_robot1Node); //g_objectRbt[0];
            cout << "Active eye is Robot1 (Red)" << endl;
        }
        else if (g_currentEyeCode == 2)// Robot2
        {
            //g_currentPickedRbtNode = g_robot2Node;
            g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
            cout << "Active eye is Robot2 (Blue)" << endl;
        }
        break;
        //fix
    case 'm':
        g_skymode *= -1;
        if (g_skymode == -1)
        {
            cout << "Editing sky eye w.r.t. world-sky frame" << endl;

        }
        if (g_skymode == 1)
        {
            cout << "Editing sky eye w.r.t. sky-sky frame" << endl;
        }
        break;
    case ' ':{
        // return to the saved frame
        if (!g_aniRunning)
        {
            if (g_keyframes.empty())
            {
                cout << "No key frame defined" << endl;
            }
            else
            {
                vector<RigTForm>::iterator saved_curr = (*g_keyframeIt).begin();
                for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++saved_curr)
                {
                    (*it)->setRbt(*saved_curr); // load saved Rbt
                }

                // the eyeframe will not changed
                if (g_currentEyeCode == 0)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
                }
                else if (g_currentEyeCode == 1)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
                }
                else if (g_currentEyeCode == 2)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
                }
                cout << "Loading current key frame [" << g_currKeyframeCode << ']' << " to scene graph" << endl;
            }
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        
        break; }
    case 'u': {
        if (!g_aniRunning)
        {
            if (g_keyframes.empty())
            {
                vector<RigTForm> currFrame;
                //cout << g_allRbts.size() << endl;
                for (vector<shared_ptr<SgRbtNode> >::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it)
                {
                    currFrame.push_back((*it)->getRbt());
                }


                g_keyframes.push_back(currFrame);
                g_currKeyframeCode = 0;
                g_keyframeIt = g_keyframes.begin();

                cout << "Create new frame [" << g_currKeyframeCode << ']' << endl;
            }
            else
            {
                //fix
                vector<RigTForm> currFrame;
                for (vector<shared_ptr<SgRbtNode> >::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it)
                {
                    currFrame.push_back((*it)->getRbt());
                }
                (*g_keyframeIt) = currFrame; // replace keyframe with current state

                //Copying scene graph to current frame[2]
                cout << "Copying scene graph to current frame [" << g_currKeyframeCode << ']' << endl;

            }
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        
        break;}
    case '>':{
        if (!g_aniRunning)
        {
            if (g_keyframeIt == --g_keyframes.end()) // at the end
            {
                // no action
                //cout << "hello!" << endl;
            }
            else
            {
                g_keyframeIt++;
                g_currKeyframeCode++;
                vector<RigTForm>::iterator keyframIt = (*g_keyframeIt).begin();
                for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++keyframIt)
                {
                    (*it)->setRbt(*keyframIt); // load saved Rbt
                }
                cout << "Stepped forward to frame[" << g_currKeyframeCode << ']' << endl;

                // the eyeframe will not changed
                if (g_currentEyeCode == 0)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
                }
                else if (g_currentEyeCode == 1)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
                }
                else if (g_currentEyeCode == 2)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
                }
            }
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        break; }
    case '<': {
        if (!g_aniRunning)
        {
            if (g_keyframeIt == g_keyframes.begin()) // at the end
            {
                // no action
            }
            else
            {
                g_keyframeIt--;
                g_currKeyframeCode--;
                vector<RigTForm>::iterator keyframIt = (*g_keyframeIt).begin();
                for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++keyframIt)
                {
                    (*it)->setRbt(*keyframIt); // load saved Rbt
                }

                cout << "Stepped backward to frame[" << g_currKeyframeCode << ']' << endl;

                if (g_currentEyeCode == 0)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
                }
                else if (g_currentEyeCode == 1)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
                }
                else if (g_currentEyeCode == 2)
                {
                    g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
                }
            }
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        break; }
    case 'd':{
        //cout << "d key pressed" << endl;
        if (!g_aniRunning)
        {
            if (g_keyframes.empty())
            {
                cout << "Frame list is now EMPTY" << endl;
            }
            else
            {
                //fix
                if (g_keyframeIt == g_keyframes.begin()) // if the selected frame is the first frame
                {
                    g_keyframes.erase(g_keyframeIt);
                    cout << "Deleting current frame [" << g_currKeyframeCode << ']' << endl;
                    g_keyframeIt = g_keyframes.begin();
                    cout << "Now at frame [" << g_currKeyframeCode << ']' << endl;
                    if (!g_keyframes.empty())
                    {
                        vector<RigTForm>::iterator keyframIt = (*g_keyframeIt).begin();
                        for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++keyframIt)
                        {
                            (*it)->setRbt(*keyframIt); // load saved Rbt
                        }
                        if (g_currentEyeCode == 0)
                        {
                            g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
                        }
                        else if (g_currentEyeCode == 1)
                        {
                            g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
                        }
                        else if (g_currentEyeCode == 2)
                        {
                            g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
                        }
                    }
                    else { cout << "No frames defined" << endl; }
                }
                else
                {
                    list<vector<RigTForm> >::iterator temp = g_keyframeIt;
                    g_keyframeIt--;
                    cout << "Deleting current frame [" << g_currKeyframeCode << ']' << endl;
                    g_keyframes.erase(temp);
                    //cout << "okay" << endl;
                    
                    g_currKeyframeCode--;
                    //update screen
                    vector<RigTForm>::iterator keyframIt = (*g_keyframeIt).begin();
                    for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++keyframIt)
                    {
                        (*it)->setRbt(*keyframIt); // load saved Rbt
                    }
                    if (g_currentEyeCode == 0)
                    {
                        g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
                    }
                    else if (g_currentEyeCode == 1)
                    {
                        g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
                    }
                    else if (g_currentEyeCode == 2)
                    {
                        g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
                    }

                    cout << "Now at frame [" << g_currKeyframeCode << ']' << endl;
                }
            }
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        break; }
    case 'n':{ // finish?
        if (!g_aniRunning)
        {
            vector<RigTForm> currFrame;
            //cout << g_allRbts.size() << endl;
            for (vector<shared_ptr<SgRbtNode> >::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it)
            {
                currFrame.push_back((*it)->getRbt());
            }

            if (g_keyframes.empty())
            {
                g_keyframes.push_back(currFrame);
                g_currKeyframeCode = 0;
                g_keyframeIt = g_keyframes.begin();
            }
            else
            {
                g_keyframeIt++;
                g_keyframes.insert(g_keyframeIt, currFrame);
                g_keyframeIt--;
                g_currKeyframeCode += 1;
            }
            cout << "Create new frame [" << g_currKeyframeCode << ']' << endl;
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        break; }
    case 'i':{
        if (!g_aniRunning)
        {
            cout << "i key pressed: read mode" << endl;

            g_keyframes.clear();
            list<vector<RigTForm> >::iterator initializer;
            g_keyframeIt = initializer;

            const char* filename;
            int numFrames, numRbtsPerFrame;
            ifstream f("animation.txt", ios::binary);
            f >> numFrames >> numRbtsPerFrame;

            for (int i = 0; i < numFrames; i++)
            {
                vector<RigTForm> currFrame;
                for (int j = 0; j < numRbtsPerFrame; j++)
                {
                    RigTForm currRbt;
                    double t1, t2, t3, q1, q2, q3, q4;
                    f >> t1 >> t2 >> t3 >> q1 >> q2 >> q3 >> q4;

                    Cvec3 T = Cvec3(t1, t2, t3);
                    Quat R = Quat(q1, q2, q3, q4);
                    currRbt.setTranslation(T);
                    currRbt.setRotation(R);
                    currFrame.push_back(currRbt);
                }
                g_keyframes.push_back(currFrame);

                if (i == 0)
                {
                    g_keyframeIt = g_keyframes.begin();
                }
            }
            g_currKeyframeCode = 0;
            cout << numFrames << " frames are lodaed" << endl;
            //cout << g_keyframes.size() << " size" << endl;
            
            //update screen
            vector<RigTForm>::iterator keyframIt = (*g_keyframeIt).begin();
            //int temp_cnt = 0;
            //cout << "CP1" << endl;
            for (vector<shared_ptr<SgRbtNode>>::iterator it = g_allRbts.begin(); it != g_allRbts.end(); ++it, ++keyframIt)
            {
                //cout << temp_cnt++ << endl;
                (*it)->setRbt(*keyframIt); // load saved Rbt
            }
            //cout << "CP2" << endl;

            // the eyeframe will not changed
            if (g_currentEyeCode == 0)
            {
                g_currentRbt = getPathAccumRbt(g_world, g_skyNode);
            }
            else if (g_currentEyeCode == 1)
            {
                g_currentRbt = getPathAccumRbt(g_world, g_robot1Node);
            }
            else if (g_currentEyeCode == 2)
            {
                g_currentRbt = getPathAccumRbt(g_world, g_robot2Node);
            }
        }
        else
        {
            cout << "Cannot operate when playing animation" << endl;
        }
        break; }
    case 'w':{
        cout << "Exporting Animation... " << endl;
        ofstream f("animation.txt", ios::binary);
        f << g_keyframes.size() << ' ' << g_allRbts.size() << endl;
        for (list<vector<RigTForm> >::iterator it = g_keyframes.begin(); it != g_keyframes.end(); ++it)
        {
            vector<RigTForm> currFrame = *it;
            for (vector<RigTForm>::iterator it2 = currFrame.begin(); it2 != currFrame.end(); ++it2)
            {
                const RigTForm currRbt = *it2;
                Cvec3 T = currRbt.getTranslation();
                Quat R = currRbt.getRotation();
                f << T[0] << ' ' << T[1] << ' ' << T[2] << ' ' << R[0] << ' ' << R[1] << ' ' << R[2] << ' ' << R[3] << endl;
            }
        }
        break; }
    case 'y':
        //fix
       //cout << g_keyframes.size() << endl;
        if (!g_aniRunning)
        {
            if (g_keyframes.size() < 4)
            {
                cout << "Cannot play animation with less than 4 keyframes." << endl;
            }
            else
            {
                g_aniRunning = true;
                g_aniHALT = false;
                cout << "Playing animation..." << endl;

                animateTimerCallback(0);

                /*
                list<vector<RigTForm> >::iterator itFrame = g_keyframes.begin();
                for (list<vector<RigTForm> >::iterator it = itFrame; it != --g_keyframes.end(); ++it)
                {
                    list<vector<RigTForm> >::iterator tempIt = it;
                    vector<RigTForm> A0 = (*it);
                    vector<RigTForm> A1 = (*(++tempIt));
                }*/
            }
        }
        else
        {
            g_aniHALT = true;
            g_aniRunning = false;
            cout << "Finished playing animation" << endl;
        }
        break;
    case '+':
        g_msBetweenKeyFrames = max(g_msBetweenKeyFrames - 100, 100);
        cout << g_msBetweenKeyFrames << " ms between keyframes" << endl;
        break;
    case '-':
        g_msBetweenKeyFrames += 100;
        cout << g_msBetweenKeyFrames << " ms between keyframes" << endl;
        break;
    }
    glutPostRedisplay();
}

static void initGlutState(int argc, char* argv[]) {
    glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
    glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
    glutCreateWindow("Assignment 5");                       // title the window

    glutDisplayFunc(display);                               // display rendering callback
    glutReshapeFunc(reshape);                               // window reshape callback
    glutMotionFunc(motion);                                 // mouse movement callback
    glutMouseFunc(mouse);                                   // mouse click callback
    glutKeyboardFunc(keyboard);
}

static void initGLState() {
    glClearColor(128. / 255., 200. / 255., 255. / 255., 0.);
    glClearDepth(0.);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_GREATER);
    glReadBuffer(GL_BACK);
    if (!g_Gl2Compatible)
        glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initMaterials() {
    // Create some prototype materials
    Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
    Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");

    // copy diffuse prototype and set red color
    g_redDiffuseMat.reset(new Material(diffuse));
    g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

    // copy diffuse prototype and set blue color
    g_blueDiffuseMat.reset(new Material(diffuse));
    g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

    // normal mapping material
    g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
    g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true)));
    g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false)));

    // copy solid prototype, and set to wireframed rendering
    g_arcballMat.reset(new Material(solid));
    g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
    g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // copy solid prototype, and set to color white
    g_lightMat.reset(new Material(solid));
    g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

    // pick shader
    g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));
};

static void initGeometry() {
    initGround();
    initCubes();
    initSphere();

    initSphereRobot();// for robot shape
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {
    const double ARM_LEN = 0.7,
        ARM_THICK = 0.25,
        TORSO_LEN = 1.5,
        TORSO_THICK = 0.25,
        TORSO_WIDTH = 1,
        HEAD_SIZE = 0.5,
        LEG_LEN = 1.0,
        LEG_THICK = 0.25;
    const int NUM_JOINTS = 10,
        NUM_SHAPES = 10;

    struct JointDesc {
        int parent;
        float x, y, z;
    };

    JointDesc jointDesc[NUM_JOINTS] = {
      {-1}, // torso
      {0,  TORSO_WIDTH / 2, TORSO_LEN / 2, 0}, // upper right arm
      {1,  ARM_LEN, 0, 0}, // lower right arm
      {0,  -TORSO_WIDTH / 2, TORSO_LEN / 2, 0}, // upper left arm
      {3,  -ARM_LEN, 0, 0}, // lower left arm
      {0,   0, TORSO_LEN/2, 0}, // head
      {0, TORSO_WIDTH *0.4, -TORSO_LEN / 2, 0 }, // upper right leg
      {6, 0, -LEG_LEN, 0 }, // lower right leg
      {0, -TORSO_WIDTH * 0.4, -TORSO_LEN / 2, 0 }, // upper left leg
      {8, 0, -LEG_LEN, 0 }, // lower left leg
    };

    struct ShapeDesc {
        int parentJointId;
        float x, y, z, sx, sy, sz;
        shared_ptr<Geometry> geometry;
    };

    ShapeDesc shapeDesc[NUM_SHAPES] = {
      {0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
      {1, ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere}, // upper right arm
      {2, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
      {3, -ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere}, // upper left arm
      {4, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
      {5, 0, 0.3, 0, TORSO_THICK, TORSO_THICK, TORSO_THICK, g_sphere}, // head
      {6, 0, -LEG_LEN/2, 0, LEG_THICK / 2, LEG_LEN/2, LEG_THICK / 2, g_sphere}, // upper right leg
      {7, 0, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
      {8, 0, -LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_sphere}, // upper left leg
      {9, 0, -LEG_LEN/2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower left leg
    };

    shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

    for (int i = 0; i < NUM_JOINTS; ++i) {
        if (jointDesc[i].parent == -1)
            jointNodes[i] = base;
        else {
            jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
            jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
        }
    }
    // The new MyShapeNode takes in a material as opposed to color
    for (int i = 0; i < NUM_SHAPES; ++i) {
        shared_ptr<SgGeometryShapeNode> shape(
            new MyShapeNode(shapeDesc[i].geometry,
                material, // USE MATERIAL as opposed to color
                Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                Cvec3(0, 0, 0),
                Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
        jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
    }
}

static void initScene() {
    g_world.reset(new SgRootNode());

    g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));

    g_groundNode.reset(new SgRbtNode());
    g_groundNode->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

    g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
    g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

    constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
    constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

    /*
    SgGeometryShapeNode(std::shared_ptr<Geometry> _geometry,
                      std::shared_ptr<Material> _material,
                      const Cvec3& translation = Cvec3(0, 0, 0),
                      const Cvec3& eulerAngles = Cvec3(0, 0, 0),
                      const Cvec3& scales = Cvec3(1, 1, 1))
    */
    g_light1Node.reset(new SgRbtNode(RigTForm(g_light1)));
    g_light1Node->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0), Cvec3(0.5, 0.5, 0.5))));

    g_light2Node.reset(new SgRbtNode(RigTForm(g_light2)));
    g_light2Node->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0, 0, 0), Cvec3(0, 0, 0), Cvec3(0.5, 0.5, 0.5))));


    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    g_world->addChild(g_robot1Node);
    g_world->addChild(g_robot2Node);

    g_world->addChild(g_light1Node);
    g_world->addChild(g_light2Node);
}

int main(int argc, char* argv[]) {
    try {
        initGlutState(argc, argv);

        glewInit(); // load the OpenGL extensions

        cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
        if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
            throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
        else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
            throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

        initGLState();
        initMaterials();
        initGeometry();

        initScene();

        //g_currentPickedRbtNode = g_skyNode;

        dumpSgRbtNodes(g_world, g_allRbts);

        glutMainLoop();
        
        return 0;
    }
    catch (const runtime_error& e) {
        cout << "Exception caught: " << e.what() << endl;
        return -1;
    }
}
