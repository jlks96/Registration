// CS3241Lab1.cpp : Defines the entry point for the console application.
#include <iostream>
#include "mesh.h"

#ifdef _WIN32
#include <Windows.h>
#include "GL\glut.h"
#define M_PI 3.141592654
#elif __APPLE__
#include <OpenGL/gl.h>
#include <GLUT/GLUT.h>
#endif

myObjType myObj;

// global variable
bool m_Smooth = FALSE;
bool m_Highlight = FALSE;
bool draw_error = FALSE;
GLfloat angle = 0;   /* in degrees */
GLfloat angle2 = 0;   /* in degrees */
GLfloat zoom = 1.0;
int mouseButton = 0;
int moving, startx, starty;

#define NO_OBJECT 4;
int current_object = 0;

using namespace std;

void setupLighting() {
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);

    // Lights, material properties
    GLfloat	ambientProperties[] = { 0.7f, 0.7f, 0.7f, 1.0f };
    GLfloat	diffuseProperties[] = { 0.8f, 0.8f, 0.8f, 1.0f };
    GLfloat	specularProperties[] = { 1.0f, 1.0f, 1.0f, 1.0f };
    GLfloat lightPosition[] = { -100.0f,100.0f,100.0f,1.0f };

    glClearDepth(1.0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientProperties);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseProperties);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularProperties);
    glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, 0.0);

    // Default : lighting
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

}

void display(void) {

    float mat_specular[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    float mat_ambient[] = { 0.3f, 0.3f, 0.3f, 1.0f };
    float mat_ambient_color[] = { 0.8f, 0.8f, 0.2f, 1.0f };
    float mat_diffuse[] = { 0.1f, 0.5f, 0.8f, 1.0f };
    float shininess = 20;
    glMaterialfv(GL_FRONT, GL_AMBIENT, mat_ambient);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_diffuse);

    glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glPushMatrix();
    gluLookAt(0, 0, 10, 0, 0, 0, 0, 1, 0);
    glRotatef(angle2, 1.0, 0.0, 0.0);
    glRotatef(angle, 0.0, 1.0, 0.0);
    glScalef(zoom, zoom, zoom);
    if (draw_error) {
        myObj.drawErrorMap();
    } else {
        myObj.draw(m_Smooth);
        myObj.draw3(m_Smooth);
    }
    glPopMatrix();
    glutSwapBuffers();
}

void keyboard(unsigned char key, int x, int y) {
    char filename[256];
    switch (key) {
    case 'p':
    case 'P':
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        break;
    case 'w':
    case 'W':
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        break;
    case 'v':
    case 'V':
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        break;
    case 's':
    case 'S':
        m_Smooth = !m_Smooth;
        break;
    case 'o':
    case 'O':
        cout << "Enter the filename you want to write:";
        cin >> filename;
        myObj.writeFile(filename);
        break;
    case '1':
    case '2':
    case '3':
    case '4':
        current_object = key - '1';
        break;
    case 'Q':
    case 'q':
        exit(0);
        break;
    case 'c':
    case 'C':
        myObj.edgeContraction();
        myObj.draw(m_Smooth);
        break;
    case 'u':
    case 'U':
        myObj.restoreInitial();
        myObj.draw(m_Smooth);
        break;
    case 'i':
    case 'I':
        cout << "Entering ICP......." << endl;
        cout << "Enter the filename of the second file you want to open (OBJ format only):";
        cin >> filename;
        myObj.readFile3(filename);
        cout << "Second file loaded." << endl;
        cout << "Press \"a\" or \"A\" to start ICP with kd-tree." << endl;
        cout << "Press \"z\" or \"Z\" to start ICP (basic)." << endl;
        break;
    case 'd':
    case 'D':
        myObj.toggleHasDrawnErrorLines();
        break;
    case 'a':
    case 'A':
        myObj.ICP();
        break;
    case 'z':
    case 'Z':
        myObj.ICPSlow();
        break;
    case 't':
    case 'T':
        myObj.transformVertices();
        break;
    case 'e':
    case 'E':
        draw_error = !draw_error;
        break;
    default:
        break;
    }

    glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
    if (state == GLUT_DOWN) {
        mouseButton = button;
        moving = 1;
        startx = x;
        starty = y;
    }
    if (state == GLUT_UP) {
        mouseButton = button;
        moving = 0;
    }
}

void motion(int x, int y) {
    if (moving) {
        if (mouseButton == GLUT_LEFT_BUTTON) {
            angle = angle + (x - startx);
            angle2 = angle2 + (y - starty);
        } else zoom += ((y - starty)*0.001);
        startx = x;
        starty = y;
        glutPostRedisplay();
    }

}

int main(int argc, char **argv) {
    char filename[255];
    char filetype[3];
    cout << "CS3242" << endl << endl;

    cout << "Enter the file type you want to open (obj OR stl):";
    cin >> filetype;
    cout << "Enter the filename you want to open:";
    cin >> filename;
    if (strcmp(filetype, "obj") == 0) {
        myObj.readFile(filename);
    } else if (strcmp(filetype, "stl") == 0) {
        myObj.readStlFile(filename);
    }
    //cout << "1-4: Draw different objects"<<endl;
    cout << "S: Toggle Smooth Shading" << endl;
    cout << "W: Draw Wireframe" << endl;
    cout << "P: Draw Polygon" << endl;
    cout << "V: Draw Vertices" << endl;
    cout << "C: Contract Edge" << endl;
    cout << "U: Undo Contract Edge" << endl;
    cout << "I: Enter ICP Mode" << endl;
    cout << "T: Transform Object" << endl;
    cout << "D: Toggle Drawing Initial Error Lines" << endl;
    cout << "E: Toggle Drawing Final Error Map" << endl;
    cout << "O: Save File" << endl;
    cout << "Q: Quit" << endl << endl;

    cout << "Left mouse click and drag: rotate the object" << endl;
    cout << "Right mouse click and drag: zooming" << endl;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(600, 600);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("CS3241 Assignment 3");
    glClearColor(1.0, 1.0, 1.0, 1.0);
    glutDisplayFunc(display);
    glutIdleFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutKeyboardFunc(keyboard);
    setupLighting();
    glDisable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);

    glMatrixMode(GL_PROJECTION);
    gluPerspective(
        /* field of view in degree */ 40.0,
        /* aspect ratio */ 1.0,
        /* Z near */ 1.0, 
        /* Z far */ 80.0);
    glMatrixMode(GL_MODELVIEW);
    glutMainLoop();

    return 0;
}
