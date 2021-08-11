////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

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
static int g_activeShader = 0;


static const int PICKING_SHADER = 2; // index of the picking shader is g_shaerFiles
static const int g_numShaders = 3;
static const char* const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"}
};
static const char* const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
    Cvec3f p, n;

    VertexPN() {}
    VertexPN(float x, float y, float z,
        float nx, float ny, float nz)
        : p(x, y, z), n(nx, ny, nz)
    {}

    // Define copy constructor and assignment operator from GenericVertex so we can
    // use make* functions from geometrymaker.h
    VertexPN(const GenericVertex& v) {
        *this = v;
    }

    VertexPN& operator = (const GenericVertex& v) {
        p = v.pos;
        n = v.normal;
        return *this;
    }
};

struct Geometry {
    GlBufferObject vbo, ibo;
    int vboLen, iboLen;

    Geometry(VertexPN* vtx, unsigned short* idx, int vboLen, int iboLen) {
        this->vboLen = vboLen;
        this->iboLen = iboLen;

        // Now create the VBO and IBO
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
    }

    void draw(const ShaderState& curSS) {
        // Enable the attributes used by our shader
        safe_glEnableVertexAttribArray(curSS.h_aPosition);
        safe_glEnableVertexAttribArray(curSS.h_aNormal);

        // bind vbo
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
        safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

        // bind ibo
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

        // draw!
        glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

        // Disable the attributes used by our shader
        safe_glDisableVertexAttribArray(curSS.h_aPosition);
        safe_glDisableVertexAttribArray(curSS.h_aNormal);
    }
};

typedef SgGeometryShapeNode<Geometry> MyShapeNode;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere, g_sphereRobot;

// --------- Scene

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode = g_skyNode; // used later when you do picking

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
//static RigTForm g_skyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
//static RigTForm g_objectRbt[2] = { RigTForm(Cvec3(-0.8,0,0)), RigTForm(Cvec3(0.8,0,0)) };  // currently only 1 obj is defined
//static Cvec3f g_objectColors[2] = { Cvec3f(1, 0, 0), Cvec3f(0, 1, 0) };

static RigTForm g_sphereRbt = RigTForm(Cvec3(0.0, 0.0, 0.0));
static Cvec3f g_sphereColor = Cvec3f(0.1, 1, 0.1);

//custom globals
//static int g_currentObjCode = 0; //index of g_object
//static int g_numObjs = 3;

static int g_currentEyeCode = 0; // 0 for sky, 1 for cube 1, ... and so forth 
static int g_numEyes = 3; // number of possible perspectives

static RigTForm g_currentRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
static RigTForm g_worldRbt = RigTForm(Cvec3(0.0, 0.0, 0.0)); // temp
static int g_skymode = -1; // -1 for world-sky, 1 for sky-sky frame

static double g_arcballScreenRadius = 0.25 * min(g_windowHeight, g_windowWidth);
static double g_arcballScale = getScreenToEyeScale((inv(g_currentRbt) * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight); // z, FovY, H

static bool g_pickAvailable = false;
                                                                                                                                     
///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
    // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
    VertexPN vtx[4] = {
      VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
      VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
      VertexPN(g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
      VertexPN(g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    };
    unsigned short idx[] = { 0, 1, 2, 0, 2, 3 };
    g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
    int ibLen, vbLen;
    getCubeVbIbLen(vbLen, ibLen);

    // Temporary storage for cube geometry
    vector<VertexPN> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    makeCube(1, vtx.begin(), idx.begin());
    g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
    int slices = 30, stacks = 30;
    int ibLen, vbLen;
    getSphereVbIbLen(slices, stacks, vbLen, ibLen);

    // Temporary storage for cube geometry
    vector<VertexPN> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    // radius = g_arcballScale*g_arcballScreenRadius
    makeSphere(g_arcballScale * g_arcballScreenRadius, slices, stacks, vtx.begin(), idx.begin());
    g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

void initSphereRobot() {
    int slices = 30, stacks = 30;
    int ibLen, vbLen;
    getSphereVbIbLen(slices, stacks, vbLen, ibLen);

    // Temporary storage for cube geometry
    vector<VertexPN> vtx(vbLen);
    vector<unsigned short> idx(ibLen);

    // radius = g_arcballScale*g_arcballScreenRadius
    makeSphere(1, 30, 30, vtx.begin(), idx.begin());
    g_sphereRobot.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
    GLfloat glmatrix[16];
    projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
    safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
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

static void drawStuff(const ShaderState& curSS, bool picking) {

    // short hand for current shader state
    //const ShaderState& curSS = *g_shaderStates[g_activeShader];

    // build & send proj. matrix to vshader
    const Matrix4 projmat = makeProjectionMatrix();
    sendProjectionMatrix(curSS, projmat);

    // use the skyRbt as the eyeRbt
    const RigTForm eyeRbt = g_currentRbt;//g_skyRbt;
    const RigTForm invEyeRbt = inv(eyeRbt);

    const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
    const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
    safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
    safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);



    // draw ground
    // ===========
    //
    //const RigTForm groundRbt = RigTForm();  // identity
    //Matrix4 MVM = rigTFormToMatrix(invEyeRbt * groundRbt);
    //Matrix4 NMVM = normalMatrix(MVM);
    //sendModelViewNormalMatrix(curSS, MVM, NMVM);
    //safe_glUniform3f(curSS.h_uColor, 0.1, 0.95, 0.1); // set color
    //g_ground->draw(curSS);

    // draw cubes
    // ==========
    // first cube
    if (!picking) {
        Drawer drawer(invEyeRbt, curSS);
        g_world->accept(drawer);

        // draw arcball as part of asst3
        // draw sphere

        bool condWorldSky = (g_currentEyeCode == 0 && g_skymode == -1);
        //bool condSkySky = (g_currentEyeCode == 0 && g_skymode == 1 && g_currentObjCode != 0);
        //bool condObject = (g_currentEyeCode != 0 && g_currentEyeCode != g_currentObjCode && g_currentObjCode != 0);
        bool condPick = g_currentPickedRbtNode != NULL; // if some is picked

        bool condRobot1View = (g_currentEyeCode == 1) && (g_currentPickedRbtNode != g_robot1Node) &&(g_currentPickedRbtNode != NULL) && condPick;
        bool condRobot2View = (g_currentEyeCode == 2) && (g_currentPickedRbtNode != g_robot2Node) && (g_currentPickedRbtNode != NULL) && condPick;

        if (condWorldSky || condRobot1View || condRobot2View)
        {
            g_sphereRbt = getPathAccumRbt(g_world, g_currentPickedRbtNode);

            // arcball scale update
            if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton))) // don't change size before the buttons are released
            {
                //cout << g_arcballScale << endl;
                g_arcballScale = getScreenToEyeScale((invEyeRbt * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight); // z, FovY, H
            }
            initSphere();
            //g_arcballScale = getScreenToEyeScale((invEyeRbt * g_sphereRbt).getTranslation()[2], g_frustFovY, g_windowHeight); // z, FovY, H


            Matrix4 MVM = rigTFormToMatrix(invEyeRbt * g_sphereRbt);
            Matrix4 NMVM = normalMatrix(MVM);
            sendModelViewNormalMatrix(curSS, MVM, NMVM);
            safe_glUniform3f(curSS.h_uColor, g_sphereColor[0], g_sphereColor[1], g_sphereColor[2]);

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            g_sphere->draw(curSS);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        }
    }
    else {
        cout << "picking process" << endl;
        Picker picker(invEyeRbt, curSS);
        g_world->accept(picker);
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
    glUseProgram(g_shaderStates[g_activeShader]->program);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

    drawStuff(*g_shaderStates[g_activeShader], false);

    glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

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
        else // arcball rotation
        {
            // only at the front of eye(z_e < 0) will be moved(with picked object origin).
            bool cond_ObjIsBack = false;
            Cvec4 objOrigin = (0, 0, 0, 1);
            Cvec4 objOriginEyeCoord = inv(g_currentRbt) * getPathAccumRbt(g_world, g_currentPickedRbtNode)*objOrigin;
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

    // using PICKING_SHADER as the shader
    glUseProgram(g_shaderStates[PICKING_SHADER]->program);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawStuff(*g_shaderStates[PICKING_SHADER], true);

    // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
    // to see result of the pick rendering pass
    //glutSwapBuffers();

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
    case 'f':
        g_activeShader ^= 1;
        break;

    case 'v':
        g_currentEyeCode = (g_currentEyeCode + 1) % g_numEyes;
        //printf("eye changed: %d\n", g_currentEyeCode);


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
    case 'p':
        g_pickAvailable = !g_pickAvailable; // pick availability status will be filpped.
        if (g_pickAvailable)
            cout << "pick mode on" << endl;
        else
            cout << "pick mode off" << endl;
        break;
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
    }
    glutPostRedisplay();
}

static void initGlutState(int argc, char* argv[]) {
    glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);  //  RGBA pixel channels and double buffering
    glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
    glutCreateWindow("Assignment 4");                       // title the window

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

static void initShaders() {
    g_shaderStates.resize(g_numShaders);
    for (int i = 0; i < g_numShaders; ++i) {
        if (g_Gl2Compatible)
            g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
        else
            g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
    }
}

static void initGeometry() {
    initGround();
    initCubes();
    initSphere();

    initSphereRobot();// for robot shape
}

static void constructRobot(shared_ptr<SgTransformNode> base, const Cvec3& color) {

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
      {1, ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphereRobot}, // upper right arm
      {2, ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
      {3, -ARM_LEN / 2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphereRobot}, // upper left arm
      {4, -ARM_LEN / 2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
      {5, 0, 0.3, 0, TORSO_THICK, TORSO_THICK, TORSO_THICK, g_sphereRobot}, // head
      {6, 0, -LEG_LEN/2, 0, LEG_THICK / 2, LEG_LEN/2, LEG_THICK / 2, g_sphereRobot}, // upper right leg
      {7, 0, -LEG_LEN / 2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
      {8, 0, -LEG_LEN / 2, 0, LEG_THICK / 2, LEG_LEN / 2, LEG_THICK / 2, g_sphereRobot}, // upper left leg
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
    for (int i = 0; i < NUM_SHAPES; ++i) {
        shared_ptr<MyShapeNode> shape(
            new MyShapeNode(shapeDesc[i].geometry,
                color,
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
        new MyShapeNode(g_ground, Cvec3(0.1, 0.95, 0.1))));

    g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
    g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

    constructRobot(g_robot1Node, Cvec3(1, 0, 0)); // a Red robot
    constructRobot(g_robot2Node, Cvec3(0, 0, 1)); // a Blue robot


    g_world->addChild(g_skyNode);
    g_world->addChild(g_groundNode);
    g_world->addChild(g_robot1Node);
    g_world->addChild(g_robot2Node);
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
        initShaders();
        initGeometry();

        initScene();

        glutMainLoop();
        return 0;
    }
    catch (const runtime_error& e) {
        cout << "Exception caught: " << e.what() << endl;
        return -1;
    }
}
