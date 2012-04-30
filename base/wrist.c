#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

//#include "rs232.h"

//Compatible for my laptop and atom board...
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif



static	FILE *file;
static float arm [6] = {0,0,0,0,0,0};

//arm[0] = Shoulder Translate 	arm[1] = Shoulder Rotate 
//arm[2] = Elbow Translate 	arm[3] = Elbow Rotate 
//arm[4] = Wrist Translate	arm[5] = Wrist Rotate 

void init(void) 
{
	glClearColor (0.0, 0.0, 0.0, 0.0);
	glShadeModel (GL_FLAT);
}

void display(void)
{

	glClear (GL_COLOR_BUFFER_BIT);
	glPushMatrix();


	glColor3f(1.0f, 0.0f, 0.0f);
	glRotatef ((GLfloat) arm[1], 0.0, 0.5, 0.0);
	glRotatef ((GLfloat) arm[0], 0.0, 0.0, 0.5);
	glTranslatef (1.0, 0.0, 0.0);
		
	glPushMatrix();
		glScalef (2.0, 0.4, 1.0);
		glutWireCube (0.5);
	glPopMatrix();
	
	glColor3f(0.0f, 0.0f, 1.0f);
	glTranslatef (0.5, 0.0, 0.0);
	glRotatef ((GLfloat) arm[2], 0.0, 0.0, 1.0);
	glTranslatef (0.5, 0.0, 0.0);

	glPushMatrix();
		glScalef (2.0, 0.4, 1.0);
		glutWireCube (0.5);
	glPopMatrix();

	glTranslatef (0.5, 0.0, 0.0);
	glColor3f(0.0f, 1.0f, 0.0f);
	glRotatef ((GLfloat) arm[5], 0.0, 0.5, 0.0);
	glRotatef ((GLfloat) arm[4], 0.0, 0.0, 0.5);
	glTranslatef (0.25, 0.0, 0.0);

	glPushMatrix();
		glScalef (2.0, 0.4, 1.0);
		glutWireCube (0.25);
	glPopMatrix();
	


	glPopMatrix();


glPushMatrix ();

glTranslatef (-2.4, -1.5, -5);
glRotatef (0 , 1,0,0);
glRotatef (0, 0,1,0);
glScalef (0.25, 0.25, 0.25);

glPushMatrix ();

	glLineWidth (2.0);

	glColor3f (1,0,0); // X axis is red.
	glRasterPos3f (11,0,0);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, "x axis");

	glColor3f (0,1,0); // Y axis is green.
	glRasterPos3f (0,10,0);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, "y axis");

	glColor3f (0,0,1); // Y axis is green.
	glRasterPos3f (0,0,15);
	glutBitmapString(GLUT_BITMAP_HELVETICA_18, "z axis");

glPopMatrix();

	glBegin (GL_LINES);
	glColor3f (1,0,0); // X axis is red.

	glVertex3f (0,0,0);
	glVertex3f (10,0,0 );

	glColor3f (0,1,0); // Y axis is green.
	glVertex3f (0,0,0);
	glVertex3f (0,10,0 );
	glColor3f (0,0,1); // z axis is blue.
	glVertex3f (0,0,0);
	glVertex3f (0,0,10 );
	glEnd();

glPopMatrix ();


	glutSwapBuffers();
}



void reshape (int w, int h)
{
	glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	glFrustum (-1.0, 1.0, -1.0, 1.0, 1.5, 300.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef (0.0, 0.0, -5.0);
}


/*
int readFromXbee() 
{

	//write to huge buffer...

	FILE *file;
	file = fopen("temp.txt","w+");

	int i, n,
	    cport_nr=0,        // /dev/ttyS0 (COM1 on windows) 
	    bdrate=9600;       // 9600 baud 

	unsigned char buf[4096];


	if(OpenComport(cport_nr, bdrate))
	{
		fprintf(file, "%s", "Can not open comport\n");
		fclose(file);
		return 1;
	}

	while(1)
	{
		n = PollComport(cport_nr, buf, 4095);

		if(n > 0)
		{
			buf[n] = 0;   // always put a "null" at the end of a string! 

			for(i=0; i < n; i++)
			{
				if(buf[i] < 32)  // replace unreadable control-codes by dots 
				{
					buf[i] = 'x';
				}
			}
			fprintf(file,"%d:%s\n", n, (char *)buf); 
			}

			
			//updateArm() or dump line to file...

		

			//THIS SLOWS DOWN THE RENDERING...
			#ifdef _WIN32
				Sleep(100);  // it's ugly to use a sleeptimer, in a real program, change the while-loop into a (interrupt) timerroutine 
			#else
				usleep(100000);  // sleep for 100 milliSeconds 
			#endif
		}

	fclose(file);
	return 0;

}*/

float quaternionToPitch(float* q)
{
        float r;
        r = asinf( -2.0 * (q[1]*q[3] - q[0]*q[2]) );    
		r=r*180/M_PI;
		return r;
}


float quaternionToYaw(const float* q)
{
        float r;
        r = atan2( 2.0 * ( q[0]*q[3] + q[1]*q[2] ) ,
                  (1.0 - 2.0 * (q[2]*q[2] + q[3]*q[3])) ); 
		r=r*180/M_PI; 
        return r;
}  

void kalmanFilterResults(int trash) {
	
	char line[128];
	float token[5];


	//tp = augustus();
	//if (tp.id = 1) {
	//	arm[] = quaternionToPitch(tp.q);
	//  arm[] = quaternionToYaw(tp.q);
	//} else if (tp.id = 2) {
	//	arm[] = quaternionToPitch(tp.q);
	//} else if (tp.id = 3) {
	//	arm[] = quaternionToPitch(tp.q);
	//  arm[] = quaternionToYaw(tp.q);
	//} else {
		//perror ("Error reading device id");
	//}
	fscanf(file, "%f,%f,%f,%f", &token[0], &token[1], &token[2], &token[3]);

	if (!feof(file)) {

		//printf("%f ", arm[4]);
		//printf("%f \n", arm[5]);

		//if deviceid=1 {
			arm[0]=quaternionToPitch(token);
			arm[1]=quaternionToYaw(token);	
			arm[2]=quaternionToPitch(token);
			//arm[5]=quaternionToYaw(token);	
		//}

		glutPostRedisplay();
		glutTimerFunc(5,kalmanFilterResults,0);				

	} else {

		fclose ( file );

	}
}
    

int main(int argc, char** argv)
{

	//Read from the XBee to temp1.txt
	//if (readFromXbee()) {
		//return 1;
	//	int b;
	//}

	file = fopen("quaternion.txt","r");	

	glutInit(&argc, argv);
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize (500, 500); 
	glutInitWindowPosition (100, 100);
	glutCreateWindow (argv[0]);
	init ();
	glutDisplayFunc(display); 
	glutReshapeFunc(reshape);
	glutTimerFunc(0,kalmanFilterResults,0);
	glutMainLoop();


	return 0;
}


