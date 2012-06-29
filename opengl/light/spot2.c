
/* Copyright (c) Mark J. Kilgard, 1994. */

/**
 * (c) Copyright 1993, 1994, Silicon Graphics, Inc.
 * ALL RIGHTS RESERVED 
 * Permission to use, copy, modify, and distribute this software for 
 * any purpose and without fee is hereby granted, provided that the above
 * copyright notice appear in all copies and that both the copyright notice
 * and this permission notice appear in supporting documentation, and that 
 * the name of Silicon Graphics, Inc. not be used in advertising
 * or publicity pertaining to distribution of the software without specific,
 * written prior permission. 
 *
 * THE MATERIAL EMBODIED ON THIS SOFTWARE IS PROVIDED TO YOU "AS-IS"
 * AND WITHOUT WARRANTY OF ANY KIND, EXPRESS, IMPLIED OR OTHERWISE,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY OR
 * FITNESS FOR A PARTICULAR PURPOSE.  IN NO EVENT SHALL SILICON
 * GRAPHICS, INC.  BE LIABLE TO YOU OR ANYONE ELSE FOR ANY DIRECT,
 * SPECIAL, INCIDENTAL, INDIRECT OR CONSEQUENTIAL DAMAGES OF ANY
 * KIND, OR ANY DAMAGES WHATSOEVER, INCLUDING WITHOUT LIMITATION,
 * LOSS OF PROFIT, LOSS OF USE, SAVINGS OR REVENUE, OR THE CLAIMS OF
 * THIRD PARTIES, WHETHER OR NOT SILICON GRAPHICS, INC.  HAS BEEN
 * ADVISED OF THE POSSIBILITY OF SUCH LOSS, HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, ARISING OUT OF OR IN CONNECTION WITH THE
 * POSSESSION, USE OR PERFORMANCE OF THIS SOFTWARE.
 * 
 * US Government Users Restricted Rights 
 * Use, duplication, or disclosure by the Government is subject to
 * restrictions set forth in FAR 52.227.19(c)(2) or subparagraph
 * (c)(1)(ii) of the Rights in Technical Data and Computer Software
 * clause at DFARS 252.227-7013 and/or in similar or successor
 * clauses in the FAR or the DOD or NASA FAR Supplement.
 * Unpublished-- rights reserved under the copyright laws of the
 * United States.  Contractor/manufacturer is Silicon Graphics,
 * Inc., 2011 N.  Shoreline Blvd., Mountain View, CA 94039-7311.
 *
 * OpenGL(TM) is a trademark of Silicon Graphics, Inc.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <GL/glut.h>

/* Some <math.h> files do not define M_PI... */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TWO_PI (2*M_PI)

typedef struct lightRec {
    float amb[4];
    float diff[4];
    float spec[4];
    float pos[4];
    float spotDir[3];
    float spotExp;
    float spotCutoff;
    float atten[3];

    float trans[3];
} Light;

/* *INDENT-OFF* */
static float modelAmb[4] = {0.2, 0.2, 0.2, 1.0};

static float matAmb[4] = {0.2, 0.2, 0.2, 1.0};
static float matDiff[4] = {0.8, 0.8, 0.8, 1.0};
static float matSpec[4] = {0.4, 0.4, 0.4, 1.0};
static float matEmission[4] = {0.0, 0.0, 0.0, 1.0};
/* *INDENT-ON* */

#define NUM_LIGHTS 3
static Light spots[] =
{
    {
        {0.2, 0.0, 0.0, 1.0},  /* ambient */
        {0.8, 0.0, 0.0, 1.0},  /* diffuse */
        {0.4, 0.0, 0.0, 1.0},  /* specular */
        {0.0, 0.0, 0.0, 1.0},  /* position */
        {0.0, -1.0, 0.0},   /* direction */
        20.0,
        30.0,               /* exponent, cutoff */
        {1.0, 0.0, 0.0},    /* attenuation */
        {-0.5, 1.25, 0.0},   /* translation */
    },
    {
        {0.0, 0.2, 0.0, 1.0},  /* ambient */
        {0.0, 0.8, 0.0, 1.0},  /* diffuse */
        {0.0, 0.4, 0.0, 1.0},  /* specular */
        {0.0, 0.0, 0.0, 1.0},  /* position */
        {0.0, -1.0, 0.0},   /* direction */
        20.0,
        30.0,               /* exponent, cutoff */
        {1.0, 0.0, 0.0},    /* attenuation */
        {0.0, 1.25, 0.0},   /* translation */
    },
    {
        {0.0, 0.0, 0.2, 1.0},  /* ambient */
        {0.0, 0.0, 0.8, 1.0},  /* diffuse */
        {0.0, 0.0, 0.4, 1.0},  /* specular */
        {0.0, 0.0, 0.0, 1.0},  /* position */
        {0.0, -1.0, 0.0},   /* direction */
        20.0,
        30.0,               /* exponent, cutoff */
        {1.0, 0.0, 0.0},    /* attenuation */
        {0.5, 1.25, 0.0},   /* translation */
    }
};

static void
usage(char *name)
{
    printf("\n");
    printf("usage: %s [options]\n", name);
    printf("\n");
    printf("  Options:\n");
    printf("    -geometry Specify size and position WxH+X+Y\n");
    printf("    -lm       Toggle lighting(SPECULAR and AMBIENT are/not same\n");
    printf("\n");
#ifndef EXIT_FAILURE /* should be defined by ANSI C <stdlib.h> */
#define EXIT_FAILURE 1
#endif
    exit(EXIT_FAILURE);
}

static void
initLights(void)
{
    int k;

    for (k = 0; k < NUM_LIGHTS; ++k) {
        int lt = GL_LIGHT0 + k;
        Light *light = &spots[k];

        glEnable(lt);
        glLightfv(lt, GL_DIFFUSE, light->diff);
        //glLightfv(lt, GL_AMBIENT, light->amb);
        //glLightfv(lt, GL_SPECULAR, light->amb);

        glLightf(lt, GL_SPOT_EXPONENT, light->spotExp);
        glLightf(lt, GL_SPOT_CUTOFF, light->spotCutoff);
        glLightf(lt, GL_CONSTANT_ATTENUATION, light->atten[0]);
        //glLightf(lt, GL_LINEAR_ATTENUATION, light->atten[1]);
        //glLightf(lt, GL_QUADRATIC_ATTENUATION, light->atten[2]);
    }
}

static void
setLights(void)
{
    int k;

    for (k = 0; k < NUM_LIGHTS; ++k) {
        int lt = GL_LIGHT0 + k;
        Light *light = &spots[k];

        glPushMatrix();
        glTranslatef(light->trans[0], light->trans[1], light->trans[2]);
        glLightfv(lt, GL_POSITION, light->pos);
        glLightfv(lt, GL_SPOT_DIRECTION, light->spotDir);
        glPopMatrix();
    }
}

static void
drawLights(void)
{
    int k;

    glDisable(GL_LIGHTING);
    for (k = 0; k < NUM_LIGHTS; ++k) {
        Light *light = &spots[k];

        glColor4fv(light->diff);

        glPushMatrix();
        glTranslatef(light->trans[0], light->trans[1], light->trans[2]);
        glBegin(GL_LINES);
        glVertex3f(light->pos[0], light->pos[1], light->pos[2]);
        glVertex3f(light->spotDir[0], light->spotDir[1], light->spotDir[2]);
        glEnd();
        glPopMatrix();
    }
    glEnable(GL_LIGHTING);
}

static void
drawPlane(int w, int h)
{
    int i, j;
    float dw = 2.0 / w;
    float dh = 2.0 / h;

    glNormal3f(0.0, 0.0, 1.0);
    for (j = 0; j < h; ++j) {
        glBegin(GL_TRIANGLE_STRIP);
        for (i = 0; i <= w; ++i) {
            glVertex2f(dw * i, dh * (j + 1));
            glVertex2f(dw * i, dh * j);
        }
        glEnd();
    }
}

void
display(void)
{
    glClear(GL_COLOR_BUFFER_BIT);

    glPushMatrix();

    glEnable(GL_LIGHTING);
    initLights();
    setLights();

    glPushMatrix();
    glRotatef(-90.0, 1, 0, 0);
    glTranslatef(-1, -1, 0.0);
    drawPlane(200, 200); // xy plane
    glPopMatrix();

    //drawLights();
    glPopMatrix();

    glutSwapBuffers();
}

void
animate(void)
{
    usleep(100*1000);
    glutPostRedisplay();
}

int
main(int argc, char **argv)
{
    int i;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    glutCreateWindow("GLUT spotlight swing");
    glutDisplayFunc(display);
    glutIdleFunc(animate);

    glMatrixMode(GL_PROJECTION);
    glFrustum(-1, 1, -1, 1, 2, 6);

    glMatrixMode(GL_MODELVIEW);
    glTranslatef(0.0, 0.0, -3.0);
    glRotatef(45.0, 1, 0, 0);

    //glEnable(GL_LIGHTING);

    //initLights();

    glutMainLoop();
    return 0;             /* ANSI C requires main to return int. */
}
