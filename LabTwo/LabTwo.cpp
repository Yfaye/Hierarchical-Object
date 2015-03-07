/* CS6555 Computer Animation, Fall 2014
* Lab 2£ºHierarchical Object Motion Control System
* Edited by Fei Yan (Email:hcifaye@gwu.edu)
* reference: Lab 0's code ; Lab 1's code ;
* for Torso Facing Tangent  http://www.cs.cmu.edu/~jkh/462_s07/assts/assignment2/asst2camera.html
*/

// window
#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

// glut
#include <GL/glut.h>


//================================
// variables initialization
//================================
// screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

// Number of Points for Spline
static int points = 0; //points index started from 0
static int number = 7; //The total number of points 

// time variables for generate Q(t)
static GLfloat t = 0;

// vectors for computing tangent orientation
static GLfloat tangent[3] = { 0 };
static GLfloat binormal[3] = { 0 };
static GLfloat normal[3] = { 0 };
static GLfloat loopIndex = 0;

// The final M matrix for torso, Left Leg and Right Leg
static GLfloat M[16] = { 0 }; //torso
static GLfloat tempM[3] = {0};//temporate matrix to store the interpolation track (torso position)
static GLfloat M1[16] = { 0 };//Left Leg
static GLfloat M2[16] = { 0 };//Right Leg


// The Catmul-Rom Spline M Marix
static GLfloat CRSplineM[16] = { -0.5f, 1.0f, -0.5f, 0.0f,	  // Column 1
								1.5f, -2.5f, 0.0f, 1.0f,      // Column 2
								-1.5f, 2.0f, 0.5f, 0.0f,      // Column 3
								0.5f, -0.5f, 0.0f, 0.0f };    // Column 4

// The B Spline M Marix
static GLfloat BSplineM[16] = { -1.0f / 6.0f, 3.0f / 6.0f, -3.0f / 6.0f, 1.0f / 6.0f,  // Column 1
								3.0f / 6.0f, -6.0f / 6.0f, 0.0f / 6.0f, 4.0f / 6.0f,  // Column 2
								-3.0f / 6.0f, 3.0f / 6.0f, 3.0f / 6.0f, 1.0f / 6.0f,  // Column 3
								1.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f, 0.0f / 6.0f };// Column 4

// Position of 7 Pionts£º 3 numbers represent the position x,y,z in world Cartisian System
static GLfloat point_position[7][3] = { { 8, 0, -20 },  //point 1
										{ -8, 0, -20 }, //point 2
										{ -5, 0, -10 }, //point 3
										{ 5, 0, -10 },	//point 4
										{ 3, 0, -5 },	//point 5
										{-3,0,-5},		//point 6
										{ 1, 0, -3 } }; //point 7

//==============================================================================
// Blending Function : Q(t) = T*M*G, finding out the vector position of time t
//==============================================================================
GLfloat blend(GLfloat T[4], GLfloat MS[16], GLfloat G[4])
{
	// B[4] is the result of T*M
	GLfloat B[4] = { 0 };
	B[0] = T[0] * MS[0] + T[1] * MS[1] + T[2] * MS[2] + T[3] * MS[3];	 //column 1
	B[1] = T[0] * MS[4] + T[1] * MS[5] + T[2] * MS[6] + T[3] * MS[7];	 //column 2
	B[2] = T[0] * MS[8] + T[1] * MS[9] + T[2] * MS[10] + T[3] * MS[11];  //column 3
	B[3] = T[0] * MS[12] + T[1] * MS[13] + T[2] * MS[14] + T[3] * MS[15];//column 4

	// Generate the result of T*M*G
	GLfloat Qt = B[0] * G[0] + B[1] * G[1] + B[2] * G[2] + B[3] * G[3];

	return Qt;
}

//=========================================================================================
// Matrix Multiply : Multiply two 4*4 matrix for Leg Animation Function 
//=========================================================================================
//
void Matrix4Mult4(GLfloat TempM1[16], GLfloat TempM2[16], GLfloat MResult[16])

{
	MResult[0] = TempM1[0] * TempM2[0] + TempM1[4] * TempM2[1] + TempM1[8] * TempM2[2] + TempM1[12] * TempM2[3];
	MResult[1] = TempM1[1] * TempM2[0] + TempM1[5] * TempM2[1] + TempM1[9] * TempM2[2] + TempM1[13] * TempM2[3];
	MResult[2] = TempM1[2] * TempM2[0] + TempM1[6] * TempM2[1] + TempM1[10] * TempM2[2] + TempM1[14] * TempM2[3];
	MResult[3] = TempM1[3] * TempM2[0] + TempM1[7] * TempM2[1] + TempM1[11] * TempM2[2] + TempM1[15] * TempM2[3];
	MResult[4] = TempM1[0] * TempM2[4] + TempM1[4] * TempM2[5] + TempM1[8] * TempM2[6] + TempM1[12] * TempM2[7];
	MResult[5] = TempM1[1] * TempM2[4] + TempM1[5] * TempM2[5] + TempM1[9] * TempM2[6] + TempM1[13] * TempM2[7];
	MResult[6] = TempM1[2] * TempM2[4] + TempM1[6] * TempM2[5] + TempM1[10] * TempM2[6] + TempM1[14] * TempM2[7];
	MResult[7] = TempM1[3] * TempM2[4] + TempM1[7] * TempM2[5] + TempM1[11] * TempM2[6] + TempM1[15] * TempM2[7];
	MResult[8] = TempM1[0] * TempM2[8] + TempM1[4] * TempM2[9] + TempM1[8] * TempM2[10] + TempM1[12] * TempM2[11];
	MResult[9] = TempM1[1] * TempM2[8] + TempM1[5] * TempM2[9] + TempM1[9] * TempM2[10] + TempM1[13] * TempM2[11];
	MResult[10] = TempM1[2] * TempM2[8] + TempM1[6] * TempM2[9] + TempM1[10] * TempM2[10] + TempM1[14] * TempM2[11];
	MResult[11] = TempM1[3] * TempM2[8] + TempM1[7] * TempM2[9] + TempM1[11] * TempM2[10] + TempM1[15] * TempM2[11];
	MResult[12] = TempM1[0] * TempM2[12] + TempM1[4] * TempM2[13] + TempM1[8] * TempM2[14] + TempM1[12] * TempM2[15];
	MResult[13] = TempM1[1] * TempM2[12] + TempM1[5] * TempM2[13] + TempM1[9] * TempM2[14] + TempM1[13] * TempM2[15];
	MResult[14] = TempM1[2] * TempM2[12] + TempM1[6] * TempM2[13] + TempM1[10] * TempM2[14] + TempM1[14] * TempM2[15];
	MResult[15] = TempM1[3] * TempM2[12] + TempM1[7] * TempM2[13] + TempM1[11] * TempM2[14] + TempM1[15] * TempM2[15];
}

//=====================================================================
// Vector Normalization : normalize the vector for tangent calculation
//=====================================================================
void Normalization(GLfloat N_tempV[3])
{
	GLfloat squa_quaterion = N_tempV[0] * N_tempV[0] + N_tempV[1] * N_tempV[1] + N_tempV[2] * N_tempV[2];
	if (squa_quaterion != 0) // avoid being divided by 0
	{
		GLfloat base_quaternion = sqrt(squa_quaterion);
		N_tempV[0] = N_tempV[0] / base_quaternion;
		N_tempV[1] = N_tempV[1] / base_quaternion;
		N_tempV[2] = N_tempV[2] / base_quaternion;
	}
}

//=====================================================================
// Vector Multiply : two vectors' cross product
//=====================================================================
void VectorMult(GLfloat TempV1[3], GLfloat TempV2[3], GLfloat VResult[3])
{
	VResult[0] = TempV1[1] * TempV2[2] - TempV1[2] * TempV2[1];
	VResult[1] = TempV1[2] * TempV2[0] - TempV1[0] * TempV2[2];
	VResult[2] = TempV1[0] * TempV2[1] - TempV1[1] * TempV2[0];
}

//====================================================================================================================
// Torso Orientation Interpolating: generate interpolation with given postions and spline styles ; Torso facing tagent
//====================================================================================================================
void Torso_interpolate(GLfloat p_position[7][3], GLfloat SplineM[16])
{
	// Set up T matrix T = {t*t*t,t*t,t,1}
	GLfloat TMatrix[4] = { t*t*t, t*t, t, 1 };

	// Set up Tangent T matrix T = {3*t*t,2*t,1,0} 
	GLfloat TangentTMatrix[4] = { 3*t*t, 2*t, 1, 0 };

	// Loop to generate the position interpolation track based on 4 points every time
	for (int i = 0; i < 3; i++)
	{
		// the value of points would be changed by timer function in the following
		GLfloat GMatrix[4] = { p_position[points][i],
			p_position[(points + 1) % number][i],
			p_position[(points + 2) % number][i],
			p_position[(points + 3) % number][i] };

		tempM[i] = blend(TMatrix, SplineM, GMatrix);
		tangent[i] = blend(TangentTMatrix, SplineM, GMatrix);
	}


	// Generate Tangent Vector
	Normalization(tangent);

	if (points == 0 && loopIndex == 0) // loop starts from the beginning
	{
		GLfloat TempVector[3] = { 1, 0, 0 }; //Pick an arbitrary vector V
		Normalization(TempVector);
		VectorMult(tangent, TempVector, normal);
		Normalization(normal);
		VectorMult(normal, tangent, binormal);
		Normalization(binormal);
		loopIndex++;
	}
	else // loop does not start from the beginning
	{
		VectorMult(tangent, binormal, normal);
		Normalization(normal);
		VectorMult(normal, tangent, binormal);
		Normalization(binormal);
	}

	// Generate the Interpolation Matrix M
	M[0] = tangent[0]; // column 1 row 1
	M[1] = normal[0]; // ..........row 2
	M[2] = binormal[0];// .........row 3
	M[3] = 0;			// ........row 4
	M[4] = tangent[1]; // column 2 row 1
	M[5] = normal[1]; // ..........row 2
	M[6] = binormal[1];// .........row 3
	M[7] = 0;			// ........row 4
	M[8] = tangent[2]; // column 3 row 1
	M[9] = normal[2]; // ..........row 2
	M[10] = binormal[2];// ........row 3
	M[11] = 0;			// ........row 4
	M[12] = tempM[0];  // column 4 row 1
	M[13] = tempM[1]; // ..........row 2
	M[14] = tempM[2]; // ..........row 3
	M[15] = 1;			// ........row 4
}



//=========================================================================
// Torso animation : walking along the given spline and facing the tangent
//=========================================================================
void TorsoAnimation()
{
	Torso_interpolate(point_position, CRSplineM);
	glLoadMatrixf(M);
	glutSolidCube(1.0);
}

//==============================================================================================================
// Leg animation : using Hierarchical Object Motion Control System with 3 Matrices: Translate Rotation Translate 
//==============================================================================================================
void LLegAnimation()
{

	// First Translate Matrix
	GLfloat LT1[16] = { 1, 0, 0, 0,		//column 1
						0, 1, 0, 0,		//column 2
						0, 0, 1, 0,		//column 3
						0, -1, 0, 1 };	//column 4

	//Rotation Matrix by Z axix
	GLfloat LAngle = (sin(4 * 3.14*t - 3.14 / 2)*3.14) / 4; // To animate rotation, change ¦È(t)
	GLfloat LT2[16] = { cos(LAngle), sin(LAngle), 0, 0,	 //column 1
						-sin(LAngle), cos(LAngle), 0, 0, //column 2
						0, 0, 1, 0,						//column 3
						0, 0, 0, 1 };					//column 4 

	// Second Translate Matrix
	GLfloat LT3[16] = { 1, 0, 0, 0,		//column 1
						0, 1, 0, 0,		//column 2
						0, 0, 1, 0,		//column 3
						0, 0, 0.3, 1 };	//column 4

	// Transformation that describes B in A coordinate system: Tab = T3*T2*T1
	Matrix4Mult4(M, LT2, M1);
	Matrix4Mult4(M1, LT1, M1);
	Matrix4Mult4(M1, LT3, M1);

	// Show the Left Leg
	glLoadMatrixf(M1);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);

}


void RLegAnimation()
{
	// First Translate Matrix
	GLfloat RT1[16] = { 1, 0, 0, 0,		//column 1
						0, 1, 0, 0,		//column 2
						0, 0, 1, 0,		//column 3
						0, -1, 0, 1 };	//column 4

	//Rotation Matrix by Z axis
	GLfloat LAngle = (sin(4*3.14*t-3.14/2)*3.14)/4; // To animate rotation, change ¦È(t)
	GLfloat RT2[16] = { cos(-LAngle), sin(-LAngle), 0, 0,	//column 1
						-sin(-LAngle), cos(-LAngle), 0, 0, //column 2
						0, 0, 1, 0,							//column 3
						0, 0, 0, 1 };						//column 4 

	// Second Translate Matrix
	GLfloat RT3[16] = { 1, 0, 0, 0,		//column 1
						0, 1, 0, 0,		//column 2
						0, 0, 1, 0,		//column 3
						0, 0, -0.3, 1 };//column 4

	// Transformation that describes B in A coordinate system: Tab = T3*T2*T1
	Matrix4Mult4(M, RT2, M2);
	Matrix4Mult4(M2, RT1, M2);
	Matrix4Mult4(M2, RT3, M2);

	// Show the right leg
	glLoadMatrixf(M2);
	glScalef(0.3f, 2.0f, 0.3f);
	glutSolidCube(1.0);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer(int value) {
	// render
	glutPostRedisplay();

	// Set time increase by 0.005, changing the value of points from 0 to 2
	t = t + 0.005;
	if (t >= 1)
	{
		t = 0;
		if (points < number - 1)
		{
			points++;
		}
		else
		{
			points = 0;
		}
	}
	// reset timer
	glutTimerFunc(16, timer, 0);
}


// user initialization
void init(void)
{

}

void display(void) {
	// clear buffer
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);


	// light source attributes
	GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[] = {1.0f, 1.0f, 0.0f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f, 0.0f, 0.1f, 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	// animation
	TorsoAnimation();
	LLegAnimation();
	RLegAnimation();
	// light source
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();


}

// update viewport and projection matrix when the window is resized
void reshape(int w, int h) {
	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(70.0, (GLfloat)w / (GLfloat)h, 1.0, 30.0);

	g_screenWidth = w;
	g_screenHeight = h;
}

// keyboard input
void keyboard(unsigned char key, int x, int y) {
	switch (key) {
	case VK_ESCAPE:
		exit(0);
		break;

	default:
		break;
	}
}

int main(int argc, char** argv) {
	// create GL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Lab 2 - Computer Animation - Fei Yan");

	// user initialization
	init();

	// set callback functions
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();

	return 0;
}

