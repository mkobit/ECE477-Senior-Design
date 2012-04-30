#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "rs232.h"

//Compatible for my laptop and atom board...
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

const char start_sequence[] = "abcde";

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

float quaternionToPitch(QUATERNION q)
{
        float r;
        r = asinf( -2.0 * (q.q1*q.q3 - q.q0*q.q2) );    
		r=r*180/M_PI;
		return r;
}

float quaternionToYaw(QUATERNION q)
{
        float r;
        r = atan2( 2.0 * ( q.q0*q.q3 + q.q1*q.q2 ) ,
                  (1.0 - 2.0 * (q.q2*q.q2 + q.q3*q.q3)) ); 
		r=r*180/M_PI; 
        return r;
}  

float quaternionToRoll(QUATERNION q)
{
        float r;
        r = atan2( 2.0 * ( q.q2*q.q3 + q.q0*q.q1 ) ,
                  (1.0 - 2.0 * (q.q1*q.q1 + q.q2*q.q2)) ); 
		r=r*180/M_PI; 
        return r;
}  

void kalmanFilterResults(int trash) {
	
		char line[128];
		float token[5];
		int  send_back_status;
		int  data_correct;
		char num_byte_char;
		int  num_byte_int;
		int j, k, l;
		int i, n,
			cport_nr=16,        // /dev/ttyS0 (COM1 on windows) 
			bdrate=57600;       // 9600 baud 

		unsigned char buf[4096];
		char in_size;

		TRANSMIT_PACKAGE tp;

		data_correct = XBEE_FAIL;
		


		//send_back_status = SendByte(cport_nr, 'a');

		n = PollComport(cport_nr, buf, 5);

		//Check for start sequence
		if(n == 5)
		{
			buf[n] = 0;   /* always put a "null" at the end of a string! */

			if (strcmp(buf, start_sequence) == 0)
			{
				printf("Start sequence obtained\n");
				data_correct = XBEE_SUCCESS;
			}
			else
			{
				printf("Wating for start sequence, start sequence doesn't match\n");
				data_correct =  XBEE_FAIL;
			}

			printf("received start sequence %i bytes: %s\n", n, (char *)buf);
		}
		else
		{
			data_correct= XBEE_FAIL;
			printf("Hello\n");
		}
	
		if(data_correct == XBEE_SUCCESS)
		{	 
			n = PollComport(cport_nr, &in_size, 1);
			//printf("Reading %d bytes\n", (int)in_size);
			n = PollComport(cport_nr, buf, (int)in_size);
			if (n == (int)in_size){

				memcpy((void*)&tp, buf,n);
			
				printf("IMU ID: %d\n", tp.id);
				printf("IMU Qs: q1 = %f,q2 = %f, q3 = %f, q4 = %f\n", tp.q.q0, tp.q.q1, tp.q.q2, tp.q.q3);  
				printf("received %d bytes: %s\n", n, buf);

				if (tp.id == 0) {						//shoulder
					//arm[0]=quaternionToPitch(tp.q);
					//arm[1]=quaternionToYaw(tp.q);
				} else if (tp.id == 1) {					//elbow
					//arm[2] = quaternionToPitch(tp.q);
					//printf("HERE");
				} else if (tp.id == 2) {					//wrist
					//arm[5] = 45//quaternionToPitch(tp.q); //updown
					//arm[4] = 45//quaternionToRoll(tp.q);
				} else {
					perror ("Error reading device id");
				}

				data_correct = XBEE_SUCCESS;

				glutPostRedisplay();
			}
			else{

				printf("read %d bytes, not enough bytes read\n", n);
				data_correct = XBEE_FAIL;
			}
		}
		glutTimerFunc(1,kalmanFilterResults,0);			
}
    

int main(int argc, char** argv)
{

	int i, n,
	    cport_nr=16,        // /dev/ttyS0 (COM1 on windows) 
	    bdrate=57600;       // 57600 baud 

	if(OpenComport(cport_nr, bdrate))
	{
		printf("Can not open comport\n");
	}

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


