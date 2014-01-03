
/*
	gl_lcd.c

	Copyright Yann GOUY <yann_gouy@yahoo.fr>

	partially taken by examples/board_hd77480
	Copyright 2008, 2009 Michel Pollet <buserror@gmail.com>

 	This file is part of simavr.

	simavr is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	simavr is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with simavr.  If not, see <http://www.gnu.org/licenses/>.
 */


//
// quite basic simulation of HD77480 LCD display
//

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "sim_avr.h"
#include "avr_ioport.h"

#include <pthread.h>
#include <GL/glut.h>
#include "hd44780_glut.h"
#include "ac_input.h"

int window;
ac_input_t ac_input;
hd44780_t hd44780;

int color = 0;
uint32_t colors[][4] = {
		{ 0x00aa00ff, 0x00cc00ff, 0x000000ff, 0x00000055 },	// fluo green
		{ 0xaa0000ff, 0xcc0000ff, 0x000000ff, 0x00000055 },	// red
};


static void * glut_thread( void * ignore)
{
	glutMainLoop();
	return NULL;
}

void displayCB(void)		/* function called whenever redisplay needed */
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW); // Select modelview matrix
	glPushMatrix();
	glLoadIdentity(); // Start with an identity matrix
	glScalef(3, 3, 1);

	hd44780_gl_draw(
		&hd44780,
			colors[color][0], /* background */
			colors[color][1], /* character background */
			colors[color][2], /* text */
			colors[color][3] /* shadow */ );
	glPopMatrix();
    glutSwapBuffers();
}

// gl timer. if the lcd is dirty, refresh display
void timerCB(int i)
{
	//static int oldstate = -1;
	// restart timer
	glutTimerFunc(1000/64, timerCB, 0);
	glutPostRedisplay();
}

int initGL(int w, int h)
{
	// Set up projection matrix
	glMatrixMode(GL_PROJECTION); // Select projection matrix
	glLoadIdentity(); // Start with an identity matrix
	glOrtho(0, w, 0, h, 0, 10);
	glScalef(1,-1,1);
	glTranslatef(0, -1 * h, 0);

	glutDisplayFunc(displayCB);		/* set window's display callback */
	//glutKeyboardFunc(keyCB);		/* set window's key callback */
	glutTimerFunc(1000 / 24, timerCB, 0);

	glEnable(GL_TEXTURE_2D);
	glShadeModel(GL_SMOOTH);

	glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	hd44780_gl_init();

	return 1;
}

void simu_component_init(struct avr_t * avr)
{
#if 0
	ac_input_init(avr, &ac_input);
	avr_connect_irq(ac_input.irq + IRQ_AC_OUT, avr_io_getirq(avr,
	        AVR_IOCTL_IOPORT_GETIRQ('D'), 2));
#endif

	hd44780_init(avr, &hd44780, 20, 4);

	/* Connect Data Lines to Port B, 0-3 */
	/* These are bidirectional too */
	for (int i = 0; i < 4; i++) {
		avr_irq_t * iavr = avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), i);
		avr_irq_t * ilcd = hd44780.irq + IRQ_HD44780_D4 + i;
		// AVR -> LCD
		avr_connect_irq(iavr, ilcd);
		// LCD -> AVR
		avr_connect_irq(ilcd, iavr);
	}
	avr_connect_irq(
			avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 4),
			hd44780.irq + IRQ_HD44780_RS);
	avr_connect_irq(
			avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 5),
			hd44780.irq + IRQ_HD44780_E);
	avr_connect_irq(
			avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 6),
			hd44780.irq + IRQ_HD44780_RW);

	/*
	 * OpenGL init, can be ignored
	 */
    static int argc = 1;
    static char* argv[] = { "truc" };
	glutInit(&argc, argv);		/* initialize GLUT system */

	int w = 5 + hd44780.w * 6;
	int h = 5 + hd44780.h * 8;
	int pixsize = 3;

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize(w * pixsize, h * pixsize);		/* width=400pixels height=500pixels */
	window = glutCreateWindow("Press 'q' to quit");	/* create window */

	initGL(w * pixsize, h * pixsize);

	pthread_t run;
	pthread_create(&run, NULL, glut_thread, NULL);
}


void simu_component_fini(struct avr_t * avr)
{
}

