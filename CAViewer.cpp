#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include "CAViewer.h"
#include <BVH.h>
#include <BVHChannel.h>
#include <BVHJoint.h>
#include <Mat4.h>
#include <Quaternion.h>

using namespace chara;
using namespace std;


void CAViewer::help()
{
	printf("Animation:\n");
	printf("   n: Next character pose\n");
	printf("   b: Back(Previous) character pose");
	printf("   shift+arrows: Move the target point");
	Viewer::help();
}


CAViewer::~CAViewer()
{
}



void CAViewer::init()
{
	Viewer::init();

	//std::string fn_front = "G:/alex/code/CharAnim_m2pro/data/OneArm.bvh";
	//std::string fn_front = "/home/pers/alexandre.meyer/code/CharAnim_m2pro/data/OneArm.bvh";
	std::string fn_front = "../data/divers/G25Ballet2.bvh";

	if (fn_front!="")
	{
		std::string current_file( fn_front );
		printf("%s\n", current_file.c_str());
		m_bvh = new BVH(current_file.c_str(), true );

		cout<<"BVH"<<endl;
		cout<<*m_bvh<<endl;
		cout<<"------------"<<endl;
	}
	else cout<<"No BVH\n";

    m_target.set( 10, 10, 0);

//    // TEST quaternion (ne sert à rien)
//    math::Quaternion q1( math::Vec3f(0,0,1),  0.08f);
//    math::Quaternion q2( math::Vec3f(0,0,1),  0.04f);
//    math::Quaternion qr;
//    qr = math::Quaternion::slerp( q1, q2, 0.5f);
//    math::Vec3f v;
//    float a;
//    qr.getAxisAngle(v,a);
//    cout<<"v="<<v<<"  angle="<<a<<endl;
//
//
//    math::QuaternionD qq1( math::Vec3d(0,0,1),  0.08);
//    math::QuaternionD qq2( math::Vec3d(0,0,1),  0.04);
//    math::QuaternionD qqr;
//    qqr = math::QuaternionD::slerp( qq1, qq2, 0.5);
//    math::Vec3d vv;
//    double aa;
//    qqr.getAxisAngle(vv,aa);
//    cout<<"vv="<<vv<<"  angleD="<<aa<<endl;

}



void CAViewer::draw()
{
  glPushMatrix();
  //if (m_skel) m_skel->render();

  glColor3f(1, 1, 0);

  printf("Drawing frame #%d\r\n", m_bvhFrame);

  bvhDrawGL(*m_bvh, m_bvhFrame);

  glPopMatrix();
}

void CAViewer::bvhDrawGL(const chara::BVH& bvh, int frameNumber)
{
  bvhDrawGLRec(*bvh.getRoot(), m_bvhFrame);
}
void CAViewer::bvhDrawGLRec(const chara::BVHJoint& joint, int frameNumber)
{
  int i;

  math::Vec3<float> jointOffset;
  math::Vec3<float> jointTranslation;
  math::Vec3<float> jointRotations;

  // Sauvegarde la matrice de transformation courante
  glPushMatrix();

  // Obtention des coordonnées de l'articulation courante dans l'espace de l'articulation parente
  bvhGetJointTransforms(joint, jointOffset, jointTranslation, jointRotations, frameNumber);

  // Affichage de l'os reliant l'articulation parente et l'articulation courante
  if (joint.getParent())
  {
    glBegin(GL_LINES);

    glVertex3f(0, 0, 0);
    glVertex3f(jointOffset.x + jointTranslation.x,
	       jointOffset.y + jointTranslation.y,
	       jointOffset.z + jointTranslation.z);
    glEnd();
  }

  // Passage du repère de l'articulation parente au repère de l'articulation courante
  glTranslatef(jointOffset.x + jointTranslation.x,
	       jointOffset.y + jointTranslation.y,
	       jointOffset.z + jointTranslation.z);

  glRotatef(jointRotations.z, 0, 0, 1);
  glRotatef(jointRotations.x, 1, 0, 0);
  glRotatef(jointRotations.y, 0, 1, 0);

  // Dessin de l'articulation courante
  draw_cube();

  // Récursion sur les articulations descendantes
  for (i = 0; i < joint.getNumChild(); ++i)
    bvhDrawGLRec(*joint.getChild(i), frameNumber);

  // Restaure la matrice de transformation précédente
  glPopMatrix();
}
void CAViewer::bvhTransitionDrawGL(const chara::BVH& bvhSRC, int frameNumberSRC,
				   const chara::BVH& bvhDST, int frameNumberDST,
				   const float interpolationValue)
{
  
}
void CAViewer::bvhTransitionDrawGLRec(const chara::BVHJoint& jointSRC,int frameNumberSRC,
				      const chara::BVHJoint& jointDST, int frameNumberDST,
				      const float interpolationValue)
{
  
}

void CAViewer::bvhGetJointTransforms(const chara::BVHJoint& joint,
				     math::Vec3<float>& offset,
				     math::Vec3<float>& translation,
				     math::Vec3<float>& rotations,
				     int frameNumber)
{
  int i;
  float data;
  int channelCount;
  chara::BVHChannel* channel;
  chara::BVHChannel::TYPE type;
  chara::AXIS axis;

  offset.x = 0;
  offset.y = 0;
  offset.z = 0;

  translation.x = 0;
  translation.y = 0;
  translation.z = 0;

  rotations.x = 0;
  rotations.y = 0;
  rotations.z = 0;

  joint.getOffset(offset.x, offset.y, offset.z);

  channelCount = joint.getNumChannel();

  for (i = 0; i < channelCount; ++i)
  {
    channel = joint.getChannel(i);
    data = channel->getData(frameNumber);

    type = channel->getType();
    axis = channel->getAxis();

    if (type == chara::BVHChannel::TYPE_TRANSLATION)
    {
      if (axis == AXIS_X)
	translation.x = data;
      else if (axis == AXIS_Y)
	translation.y = data;
      else if (axis == AXIS_Z)
	translation.z = data;
    }
    else if (type == BVHChannel::TYPE_ROTATION)
    {
      if (axis == AXIS_X)
	rotations.x = data;
      else if (axis == AXIS_Y)
	rotations.y = data;
      else if (axis == AXIS_Z)
	rotations.z = data;
    }
  }
}

void CAViewer::keyPressed(unsigned char key, int x, int y)
{
	bool handled = false;
	if ((key=='n'))
	{
		++m_bvhFrame;
		//m_skel->setPostureFromBVH( *m_bvh, m_bvhFrame);
		handled = true;
	}
	else
	if ((key=='b'))
	{
		--m_bvhFrame;
		//m_skel->setPostureFromBVH( *m_bvh, m_bvhFrame);
		handled = true;
	}
	else
	if (key=='w')
	{
		bWireframe = !bWireframe;
		if (bWireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		handled = true;
	}

	if (!handled)
	{
		Viewer::keyPressed(key,x,y);
	}
	updateGL();
}


void CAViewer::specialKeyPressed(int key, int x, int y)
{
	bool handled = false;

	if (glutGetModifiers()==GLUT_ACTIVE_SHIFT)
	{
		switch (key)
		{
		case GLUT_KEY_UP:
			m_target.y += 1;
			handled=true;
			break;
		case GLUT_KEY_DOWN:
			m_target.y -= 1;
			handled=true;
			break;
		case GLUT_KEY_LEFT:
			m_target.x -= 1;
			handled=true;
			break;
		case GLUT_KEY_RIGHT:
			m_target.x += 1;
			handled=true;
			break;
		case GLUT_KEY_PAGE_UP:
			m_target.z += 1;
			handled=true;
			break;
		case GLUT_KEY_PAGE_DOWN:
			m_target.z -= 1;
			handled=true;
			break;
		}
	}


	if (!handled)
	{
        Viewer::specialKeyPressed(key,x,y);
	}
	updateGL();
}



void CAViewer::animate()
{
	if (m_bvh)
	{
		++m_bvhFrame;
		if (m_bvhFrame>=m_bvh->getNumFrame()) m_bvhFrame=0;
		//m_skel->setPostureFromBVH( *m_bvh, m_bvhFrame);
	}
}



