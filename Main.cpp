/*
Author: Maham Tanveer
Class: ECE6122
Last Date Modified: 11/29/2019
Description:
Final Project APT
*/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <GL/glut.h>
#include <chrono>
#include <thread>
#include "ECE_Bitmap.h"
#include <algorithm>    // std::swap

using namespace std;

#define includeTime 1 //variable to turn on and off the initial wait time
#define massUAV 1 //kg
#define maxForce 20 //N in each direction
#define lenUAV 1 //m

#define numUAV 15 //total UAVs
#define sizeShare (numUAV+1)*6 //set size of share arrays for MPI 
#define sizeShare2 (numUAV)*6
#define yLim 110/2 // 120 yard to meter
#define xLim 48/2 //53 yard to meter
#define zLim 2 //to give the objects a bit of a height
#define timeStamp 0.1 //100 msec timestamp 

//--------------------------------------------
int timeAroundGlobe = 0; //to turn off after 60 seconds
bool endSimulation = 0; //to signal to end simulation
float angleBoard = 0; //rotating the board
float eyeX = 0, eyeY = 0, eyeZ = 160; //for the lookat function
float w, h, tip = 0, turn = 0; //for multi angle rotation
double xLocArray[15]; //locations of the UAVS
double yLocArray[15];
double zLocArray[15];
GLuint texture[2]; //for applying texture map 
BMP inBitmap; //for texture maping
bool initial = 1;

//for the MPI shre variables
double shareArr[sizeShare];
double ownArr[6];
double shareColor[numUAV + 1];

//for changing color 
double ownColor = 255;
double colorVal = 255;

//light variables
bool L0 = true;


/*
* A function that renders the UAVs to screen
* @Param none
* @return none
*/
void drawUAVS()
{
	glBindTexture(GL_TEXTURE_2D, texture[1]); //bind texture
	for (int i = 0; i < numUAV; i++)
	{
		colorVal = shareColor[i + 1];//this value is recieved from shared array
		glPushMatrix();
            glColor3f(colorVal / 255, 0.0, 0.0); //for the changing color //letter T 
            glScalef(1, 1, 1); //to make sure previous things dont have effect
            glTranslatef(xLocArray[i], yLocArray[i], zLocArray[i]);
            glScalef(2, 2, 2); //scaling to make it easier to see
            glutSolidCone(1.0, 1.0, 10, 10); //letter : M 
		glPopMatrix();
	}
}

/*
* A function that renders the Football field to screen
* @Param none
* @return none
*/

void drawField()
{
	//Function : Draws the field and applies its texture

	glPushMatrix();
        glBindTexture(GL_TEXTURE_2D, texture[0]); //takes the texture 
        glColor3f(0.0, 1.0, 0.0);
        glScalef(xLim * 2 + 9 * 2, yLim * 2 + 9 * 2, 1); //adjust to relavnt size
        glBegin(GL_QUADS);
        // glTexCoord2f(1, 1);
        glTexCoord2f(1, 0);
        glVertex3f(0.5f, 0.5f, 1.0f);
        // glTexCoord2f(0, 1);
        glTexCoord2f(1, 1);
        glVertex3f(-0.5f, 0.5f, 1.0f);
        // glTexCoord2f(0, 0);
        glTexCoord2f(0, 1);
        glVertex3f(-0.5f, -0.5f, 1.0f);
        // glTexCoord2f(1, 0);
        glTexCoord2f(0, 0);
        glVertex3f(0.5f, -0.5f, 1.0f);
        glEnd();
        // glutSolidCube(1.0);
	glPopMatrix();
	glEnable(GL_TEXTURE_2D);//enable texture
}


/*
* A function that updates the values of all UAVs and saves locally using MPI shared array
* @Param none
* @return none
*/

void updateUAVS()
{
	for (int i = 0; i < 15; i++)
	{
		xLocArray[i] = shareArr[6 + 6 * i + 0]; //give a bump of 6 to ignore the rank = 0 data
		yLocArray[i] = shareArr[6 + 6 * i + 1];
		zLocArray[i] = shareArr[6 + 6 * i + 2];
	}
}

/*
* A function that makes the 10m sphere
* @Param none
* @return none
*/

void drawCentre()
{
	glPushMatrix();
        glColor3f(0.5, 0.2, 0.6);
        glScalef(10, 10, 10);
        glTranslatef(0, 0, 5.0);
        glutWireSphere(1.0, 10, 10);
	glPopMatrix();
}

/*
* A function that runs in bg of openGl
* @Param none
* @return none
*/

void display(void)
{

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();             /* clear the matrix */
			/* viewing transformation  */
	gluLookAt(eyeX, eyeY, eyeZ, //eye location
		0.0, 0.0, 0.0,
		1.0, 0.0, 0.0);
	glMatrixMode(GL_MODELVIEW);

	//rotation functions
	glRotatef(tip, 1, 0, 0); 
	glRotatef(turn, 0, 1, 0);
	glRotatef(angleBoard, 0, 0, 1); //rotate 
	glRotatef(35, 0, 1, 0);

    //call relevant functions 
	drawUAVS();
	updateUAVS();
	drawField();
	drawCentre();

    //light function
	if (L0)
		glEnable(GL_LIGHT0);
	else
		glDisable(GL_LIGHT0);


	glutSwapBuffers(); //for double buffering

}

/*
* A function that manages chance in size of screen
* @Param w width of the screen
* @Param h height of the screen
* @return none
*/

void changeSize(int w, int h)
{
	float ratio = ((float)w) / ((float)h); // window aspect ratio
	glMatrixMode(GL_PROJECTION); // projection matrix is active
	glLoadIdentity(); // reset the projection
	gluPerspective(60.0, ratio, 0.1, 1000.0); // perspective transformation
	glMatrixMode(GL_MODELVIEW); // return to modelview mode
	glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}

/*
* Timer function: used for stalling threads
* @Param id 
* @return none
*/

void timer(int id)
{
	glutPostRedisplay();
	glutTimerFunc(100, timer, 0);
}

/*
* Timer function
* @Param id 
* @return none
*/

void timerFunction(int id)
{
	glutPostRedisplay();
	glutTimerFunc(100, timerFunction, 0);
}

/*
* For processing key characetrs entered using keyboard
* @Param key Key character given as inpiut
* @return none
*/

void processNormalKeys(unsigned char key, int x, int y)
{

	if (key == 'R' || key == 'r') //roatate about the centre of board
	{
		angleBoard += 10;

		angleBoard = float((int)angleBoard % 360);
	}

	if (key == 'd' || key == 'D') //look down
	{
		eyeZ = eyeZ - 20;
	}

	if (key == 'u' || key == 'U') //look up
	{
		eyeZ = eyeZ + 20;
	}

	if (key == '0') //enable and disable light 0 
	{
		L0 = !L0;
	}

}

/*
* Rotation function
* @Param key the key entered by user, inclued direction keys  
* @return none
*/

void turnRotation(int key, int x, int y)
{
	switch (key)
	{
		case GLUT_KEY_RIGHT: turn += 5; break;
		case GLUT_KEY_LEFT: turn -= 5; break;
		case GLUT_KEY_UP: tip -= 5; break;
		case GLUT_KEY_DOWN: tip += 5; break;
	}
}

/*
* Idle function : Responsible for running in bg and sharing information for all threads
* @Param id 
* @return none
*/

void idle()
{
	//hold the thread for some time
	std::this_thread::sleep_for(std::chrono::microseconds(100));
	//share the MPI data
	MPI_Allgather(ownArr, 6, MPI_DOUBLE, shareArr, 6, MPI_DOUBLE, MPI_COMM_WORLD);
	MPI_Allgather(&ownColor, 1, MPI_DOUBLE, shareColor, 1, MPI_DOUBLE, MPI_COMM_WORLD);
}

/*
* Timer function
* @Param id 
* @return none
*/

int main(int argc, char** argv)
{
	using namespace std::this_thread; // sleep_for, sleep_until
	using namespace std::chrono; // nanoseconds, system_clock, seconds

	int  numtasks, rank, rc;
	rc = MPI_Init(&argc, &argv);
	double* locMID = new double[3];

    //variables 
	double* diffDist = new double[3];
	double* diffNorm = new double[3];
	double* hookFor = new double[3];
	double totDiff;
	double totForce = 0;
	double P, I, D;
	double* forceDir = new double[3];
	double totDist;
	double k;
	bool initWait = true;
	double xLoc, yLoc, zLoc;
	double u = 0;
    bool flipColor = false;

	//variables for setting velovity, accellaration , location and force of each individuL UAV
	double* accUAV = new double[3];
	double* velUAV = new double[3];
	double* locUAV = new double[3];
	double* forUAV = new double[3];

    //for colorchanging
    double stepsColor = 0;
    
    //set location of the virtual point
	locMID[0] = 0;
	locMID[1] = 0;
	locMID[2] = 50;



	//initialize as 0
	for (int i = 0; i < 3; i++)
	{
		accUAV[i] = 0;
		velUAV[i] = 0;
		locUAV[i] = 0;
		forUAV[i] = 0;
	}


    //setting up MPI 

	if (rc != MPI_SUCCESS)
	{
		printf("Error starting MPI program. Terminating.\n");
		MPI_Abort(MPI_COMM_WORLD, rc);
	}

	//setting up MPI
	MPI_Comm_size(MPI_COMM_WORLD, &numtasks);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	//setting locations of all UAVs
	//based on rank 

	double yLocs[3];
	yLocs[0] = yLim - 5; //edges
	yLocs[1] = yLocs[0] / 2; //centre
	yLocs[2] = 0; //at 0 

	switch (rank)
	{
	case 1:
		locUAV[0] = xLim; locUAV[1] = -yLocs[0]; locUAV[2] = zLim;
		break;
	case 2:
		locUAV[0] = xLim; locUAV[1] = -yLocs[1]; locUAV[2] = zLim;
		break;
	case 3:
		locUAV[0] = xLim; locUAV[1] = yLocs[2]; locUAV[2] = zLim;
		break;
	case 4:
		locUAV[0] = xLim; locUAV[1] = yLocs[1]; locUAV[2] = zLim;
		break;
	case 5:
		locUAV[0] = xLim; locUAV[1] = yLocs[0]; locUAV[2] = zLim;
		break;
	case 6:
		locUAV[0] = 0; locUAV[1] = -yLocs[0]; locUAV[2] = zLim;
		break;
	case 7:
		locUAV[0] = 0; locUAV[1] = -yLocs[1]; locUAV[2] = zLim;
		break;
	case 8:
		locUAV[0] = 0; locUAV[1] = yLocs[2]; locUAV[2] = zLim;
		break;
	case 9:
		locUAV[0] = 0; locUAV[1] = yLocs[1]; locUAV[2] = zLim;
		break;
	case 10:
		locUAV[0] = 0; locUAV[1] = yLocs[0]; locUAV[2] = zLim;
		break;
	case 11:
		locUAV[0] = -xLim; locUAV[1] = -yLocs[0]; locUAV[2] = zLim;
		break;
	case 12:
		locUAV[0] = -xLim; locUAV[1] = -yLocs[1]; locUAV[2] = zLim;
		break;
	case 13:
		locUAV[0] = -xLim; locUAV[1] = yLocs[2]; locUAV[2] = zLim;
		break;
	case 14:
		locUAV[0] = -xLim; locUAV[1] = yLocs[0]; locUAV[2] = zLim;
		break;
	case 15:
		locUAV[0] = -xLim; locUAV[1] = yLocs[1]; locUAV[2] = zLim;
		break;
	}

	//share the locations and velocies
	for (int i = 0; i < 3; i++)
	{
		ownArr[i] = locUAV[i];
		ownArr[i + 3] = velUAV[i];
	}

	if (rank == 0)
	{
		//setting up OpenGL

		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);

		//setting the window
		glutInitWindowPosition(300, 200);
		glutInitWindowSize(400, 400);
		glutCreateWindow(argv[0]);

		//setting light and texture
		glEnable(GL_DEPTH_TEST);
		glClearColor(52 / 255, 80 / 255, 92 / 255, 1.0);
		glColor3f(52 / 255, 80 / 255, 92 / 255); //set bg color
		glShadeModel(GL_SMOOTH);
		glEnable(GL_NORMALIZE);
		glEnable(GL_LIGHTING);
		GLfloat light_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
		GLfloat light_diffuse[] = { 0.5, 0.5, 0.5, 1.0 };

		//setting ambient light
		//making diffuse and specualr to 0
		glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
		//setting material properties

		//glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		glEnable(GL_COLOR_MATERIAL);

		glEnable(GL_DEPTH_TEST);
		glutReshapeFunc(changeSize);
		glutDisplayFunc(display);
		glutIdleFunc(idle);
		glutSpecialFunc(turnRotation);
		glutKeyboardFunc(processNormalKeys); // process standard key clicks
		glutTimerFunc(100, timerFunction, 0);

		gluPerspective(45.0, 1.5, 0.1, 250.0);
		glMatrixMode(GL_MODELVIEW);
		///loading textyre files
		inBitmap.read("ff.bmp");
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
		glGenTextures(2, texture);

		glBindTexture(GL_TEXTURE_2D, texture[0]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR); //scale linearly when image bigger than texture
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); //scale linearly when image smalled than texture
		glTexImage2D(GL_TEXTURE_2D, 0, 3, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0,
			GL_BGR_EXT, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
		glBindTexture(GL_TEXTURE_2D, texture[1]);

		glEnable(GL_TEXTURE_2D);

		glutMainLoop();
	}
	else //for all UAVs 
	{
		double minVel, maxVel;
		double randF, randA, randB;
		double dirInside[3];

		for (int i = 0; i < 3; i++)
		{
			ownArr[i] = locUAV[i];
			ownArr[i + 3] = velUAV[i];
		}
		int sleep_int = 0;
		//wait initially 5 seconds
		while (sleep_int < 1000)
		{
			MPI_Allgather(ownArr, 6, MPI_DOUBLE, shareArr, 6, MPI_DOUBLE, MPI_COMM_WORLD);
			MPI_Allgather(&ownColor, 1, MPI_DOUBLE, shareColor, 1, MPI_DOUBLE, MPI_COMM_WORLD);
			std::this_thread::sleep_for(std::chrono::microseconds(5000));
			sleep_int++;
		}
		ownColor = 255;
		bool stopped = true;

		//runs until 60 seconds pass
		
		while (!endSimulation)
		{
			//calculate the distance from 0,0,50 to centre of the object 
			totDiff = 0;
			for (int i = 0; i < 3; i++)
			{
				diffDist[i] = locUAV[i] - locMID[i];
				totDiff = totDiff + diffDist[i] * diffDist[i];
			}
			totDiff = sqrt(totDiff);


			//set unit vector to 0,0,50
			for (int i = 0; i < 3; i++)
			{
				diffNorm[i] = diffDist[i] / totDiff;
			}

			forUAV[2] = -10; //aply the gravity force

			double totVel = 0, totVelPrev = 0, totAcc = 0, currAcc = 0;

            // getupdated velocty value from shared array if it was updated by collision
			
			//if distance is outside 10
			if (totDiff > 10 )
			{
				minVel = 0;
				maxVel = 2 / timeStamp;

				P = 2; //PID controller
				I = 1e-2;
				D = 2e-2;
				totDist = (10 - totDiff);

				forUAV[2] = forUAV[2] + 10; //counter gravity force first of all
				for (int i = 0; i < 3; i++)
				{
					// if(i==2)
					//     forUAV[i] = forUAV[i] + 10; 
					forUAV[i] = P * totDist * diffNorm[i] - I * velUAV[i];  //controller
					if (abs(forUAV[i]) > 20) //clip to 20 N
						forUAV[i] = 20 * ((forUAV[i] > 0) - (forUAV[i] < 0));
				}

				//get total force
				totForce = 0;
				for (int i = 0; i < 3; i++)
				{
					totForce = totForce + forUAV[i] * forUAV[i];
				}
				totForce = sqrt(totForce);
				for (int i = 0; i < 3; i++)
					accUAV[i] = forUAV[i] / massUAV;

				// update accellaration based on force
				// clip to 2.0 m/s velocity f needed 
				totVel = 0; totVelPrev = 0; totAcc = 0; currAcc = 0;
				for (int i = 0; i < 3; i++)
				{
					totVelPrev = totVelPrev + velUAV[i] * velUAV[i];
					totVel = totVel + (velUAV[i] + accUAV[i] * timeStamp) * (velUAV[i] + accUAV[i] * timeStamp);
				}
				totVel = sqrt(totVel);
				totVelPrev = sqrt(totVelPrev);

				//make sure velocity stays below 2m/s
				if (totVel > maxVel)
				{
					totAcc = (maxVel - totVelPrev) / timeStamp;

                    //adjust acceleartion based on max velocity and direction of force needed
					for (int i = 0; i < 3; i++)
					{
						accUAV[i] = -1 * totAcc * diffNorm[i];
						forUAV[i] = accUAV[i] * massUAV;
                        //clip for to 20N
						if (abs(forUAV[i]) > 20)
							forUAV[i] = 20 * ((forUAV[i] > 0) - (forUAV[i] < 0));
					}

                    //this is relevant if the force was clipped
					for (int i = 0; i < 3; i++)
						accUAV[i] = forUAV[i] / massUAV;
				}


				//update location and velocity based on accellaration 
				for (int i = 0; i < 3; i++)
				{
					velUAV[i] = velUAV[i] + accUAV[i] * timeStamp;
					locUAV[i] = locUAV[i] + velUAV[i] * timeStamp + 0.5 * accUAV[i] * timeStamp * timeStamp;
				}

				//timeAroundGlobe = 0;
                //in case the distance is smal consider it within the bounds 
				if (totDiff < 15)
				{
					timeAroundGlobe++;
					if (timeAroundGlobe >= (60 / timeStamp))
					{
						endSimulation = 1;
					}
				}

			}
			else //on the circle now 
			{
			
                //continue;
                minVel = 2  / timeStamp;
                maxVel = 10  / timeStamp;

                //generate a random force
                randF = rand() % ((10 - 0) + 1) + 0; 
                forUAV[2] = forUAV[2] + 10; //counter gravity force first of all
                randA = rand() % ((5 - 0) + 1) + 0;
			//	randF= 10;
             //   randA = 0; 
                switch ((int)randA) //get a random movement on a plane perpendicular to the direction vector
                {

                case 0: //set a = 0 and b as random
                    dirInside[0] = 0;
                    dirInside[1] = rand();
                    dirInside[2] = (dirInside[1] * locUAV[1]) / (0 - locUAV[2]);
                    break;
                case 1: //set a = 0 and c as random
                    dirInside[0] = 0;
                    dirInside[2] = rand();
                    dirInside[1] = (dirInside[2] * (0 - locUAV[2])) / locUAV[1];

                    break;
                case 2: //set b = 0 and a as random
                    dirInside[1] = 0;
                    dirInside[0] = rand();
                    dirInside[2] = (dirInside[0] * locUAV[0]) / (0 - locUAV[2]);

                    break;
                case 3: //set b = 0 and c as random
                    dirInside[1] = 0;
                    dirInside[2] = rand();
                    dirInside[0] = dirInside[2] * ((0 - locUAV[2]) / locUAV[0]);


                    break;
                case 4: //set c = 0 and a as random
                    dirInside[2] = 0;
                    dirInside[0] = rand();
                    dirInside[1] = -dirInside[0] * (locUAV[0] / locUAV[1]);

                    break;
                case 5: //set c = 0 and b as random
                    dirInside[2] = 0;
                    dirInside[1] = rand();
                    dirInside[0] = -dirInside[1] * (locUAV[1] / locUAV[0]);

                    break;
                }
            

                //find unit vector for generated perpendicular direction vector
                double mag = 0;
                for (int i = 0; i < 3; i++)
                {
                    mag = mag + dirInside[i] * dirInside[i];
                }
                mag = sqrt(mag);

                //find force in each direction 
                for (int i = 0; i < 3; i++)
                    forUAV[i] = randF * dirInside[i] / mag;


                totForce = 0;
                for (int i = 0; i < 3; i++)
                {
                    totForce = totForce + forUAV[i] * forUAV[i];
                }
                totForce = sqrt(totForce);

                // update accellaration based on force
                // clip to 2.0 m/s velocity f needed4

                //get accelaration based on force
                for (int i = 0; i < 3; i++)
                {
                    accUAV[i] = forUAV[i] / massUAV;

                }

                //get magnitue of accellation
                currAcc = 0;
                for (int i = 0; i < 3; i++)
                    currAcc = currAcc + accUAV[i] * accUAV[i];
                currAcc = sqrt(currAcc);

                totVel = 0; totVelPrev = 0; totAcc = 0; currAcc = 0;
				for (int i = 0; i < 3; i++)
				{
					totVelPrev = totVelPrev + velUAV[i] * velUAV[i];
					totVel = totVel + (velUAV[i] + accUAV[i] * timeStamp) * (velUAV[i] + accUAV[i] * timeStamp);
				}
				totVel = sqrt(totVel);
				totVelPrev = sqrt(totVelPrev);


                //clip to max 10 m/s
                if (totVel > maxVel)
                {
                    totAcc = (maxVel - totVelPrev) / timeStamp;
                    for (int i = 0; i < 3; i++)
                    {
                        accUAV[i] = totAcc * (accUAV[i] / currAcc);
                        forUAV[i] = accUAV[i] * massUAV;
                        if (abs(forUAV[i]) > 20)
                            forUAV[i] = 20 * ((forUAV[i] > 0) - (forUAV[i] < 0));
                    }


                    for (int i = 0; i < 3; i++)
                        accUAV[i] = forUAV[i] / massUAV;
                }

                //  clip to min 2 m/s
                if (totVel < minVel & currAcc != 0)
                {
                    totAcc = (minVel - totVelPrev) / timeStamp;

                    for (int i = 0; i < 3; i++)
                    {
                        accUAV[i] = totAcc * (accUAV[i] / currAcc);
                        forUAV[i] = accUAV[i] * massUAV;
                        if (abs(forUAV[i]) > 20)
                            forUAV[i] = 20 * ((forUAV[i] > 0) - (forUAV[i] < 0));
                    }

                    for (int i = 0; i < 3; i++)
                        accUAV[i] = forUAV[i] / massUAV;
                }

                //if there is no accelration clip to min velocty 
                if (currAcc == 0 )
                {
                    totAcc = (minVel - totVelPrev) / timeStamp;

                    for (int i = 0; i < 3; i++)
                    {
                        accUAV[i] = totAcc * (dirInside[i] / mag);
                        forUAV[i] = accUAV[i] * massUAV;
                        if (abs(forUAV[i]) > 20)
                            forUAV[i] = 20 * ((forUAV[i] > 0) - (forUAV[i] < 0));
                    }

                    for (int i = 0; i < 3; i++)
                        accUAV[i] = forUAV[i] / massUAV;
                }

                //update location and velocity based on accellaration 
                for (int i = 0; i < 3; i++)
                {
                    locUAV[i] = locUAV[i] + velUAV[i] * timeStamp + 0.5 * accUAV[i] * timeStamp * timeStamp;
                    velUAV[i] = velUAV[i] + accUAV[i] * timeStamp;
                }

                //end simulation after 1 min around the circle
                timeAroundGlobe++;
                if (timeAroundGlobe >= (60 / timeStamp))
                {
                    endSimulation = 1;
                }
            
			}



			//exchange velocites of two collision UAVS
			for (int i = 1; i < 16; i++)
			{
				if (i != rank)
				{
					totDiff = 0;
					for (int j = 0; j < 3; j++)
					{
						diffDist[j] = locUAV[j] - shareArr[6 * i + j];
						totDiff = totDiff + diffDist[j] * diffDist[j];
					}
					totDiff = sqrt(totDiff);
					if (totDiff <= 0.01)//distance less a 1cm
					{
						std::swap(velUAV[0], shareArr[6 * i + 3]);
						std::swap(velUAV[1], shareArr[6 * i + 4]);
						std::swap(velUAV[2], shareArr[6 * i + 5]);
					}

				}
			}

            
                
			// store updated loc and velocity values 
			for (int i = 0; i < 3; i++)
			{
				ownArr[i] = locUAV[i];
				ownArr[i + 3] = velUAV[i];
			}

            

			//changing color 

        
            if(flipColor)
                ownColor = ownColor - 1;
            else
                ownColor = ownColor + 1;

            if(ownColor >= 255)
                flipColor = true;
            if(ownColor <= 128)
                flipColor = false;

			//in case it was swapped this value needs to be updated 
            for (int i = 0; i < 3; i++)
				velUAV[i] = shareArr[6 * rank + i + 3];


			//share MPI values
			MPI_Allgather(ownArr, 6, MPI_DOUBLE, shareArr, 6, MPI_DOUBLE, MPI_COMM_WORLD);
			MPI_Allgather(&ownColor, 1, MPI_DOUBLE, shareColor, 1, MPI_DOUBLE, MPI_COMM_WORLD);

            
			//sleep for timeSTamp 
			std::this_thread::sleep_for(std::chrono::microseconds((int)(timeStamp * 1e6)));



		}
		exit(0);
	}

	MPI_Finalize();

}
