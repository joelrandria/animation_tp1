#include "CAViewer.h"
#include "CASkeleton.h"

#include <BVH.h>
#include <BVHChannel.h>
#include <BVHJoint.h>
#include <Mat4.h>
#include <Quaternion.h>

#include <glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

using namespace chara;
using namespace std;

CAViewer::CAViewer()
  :Viewer(),
   isPhysics(false),
   m_bvh(NULL),
   m_bvhFrame(0),
   m_skel(0)
{
}
CAViewer::~CAViewer()
{
}

void CAViewer::help()
{
  printf("Animation:\n");
  printf("   n: Next character pose\n");
  printf("   b: Back(Previous) character pose");
  printf("   shift+arrows: Move the target point");
  Viewer::help();
}

void CAViewer::init()
{
  Viewer::init();

  std::string fn_front = "../data/divers/G25Ballet2.bvh";

  if (fn_front != "")
  {
    std::string current_file(fn_front);
    printf("%s\n", current_file.c_str());

    m_bvh = new BVH(current_file.c_str(), true);

    cout << "BVH" << endl;
    cout << *m_bvh << endl;
    cout << "------------" << endl;

    m_skel = new CASkeleton(*m_bvh);

    CASkeleton skel2(*m_bvh);
    skel2.setPose(*m_bvh, 5);

    std::cout << "Distance(skel1, skel2) = " << m_skel->distance(skel2) << std::endl;
  }
  else
  {
    cout << "No BVH\n";
  }

  m_target.set(10, 10, 0);
}

void CAViewer::animate()
{
  if (m_bvh)
  {
    ++m_bvhFrame;
    if (m_bvhFrame >= m_bvh->getNumFrame())
      m_bvhFrame=0;

    m_skel->setPose(*m_bvh, m_bvhFrame);
  }
}

void CAViewer::draw()
{
  // glPushMatrix();
  // glColor3f(1, 0, 0);
  // bvhDrawGL(*m_bvh, m_bvhFrame);
  // glPopMatrix();

  // glPushMatrix();
  // glColor3f(0, 1, 0);
  // bvhDrawGL(*m_bvh, m_bvhFrame + 50);
  // glPopMatrix();

  // glPushMatrix();
  // glColor3f(1, 1, 0);
  // bvhTransitionDrawGL(*m_bvh, m_bvhFrame, *m_bvh, m_bvhFrame + 50, 0.5f);
  // glPopMatrix();

  if (m_skel)
    m_skel->drawGL();
}
void CAViewer::bvhDrawGL(const chara::BVH& bvh, int frameNumber)
{
  bvhDrawGLRec(*bvh.getRoot(), frameNumber);
}
void CAViewer::bvhDrawGLRec(const chara::BVHJoint& joint, int frameNumber)
{
  int i;

  math::Vec3<float> jointOffset;
  math::Vec3<float> jointTranslation;
  math::Vec3<float> jointRotations;

  // Sauvegarde la matrice de transformation courante
  glPushMatrix();

  // Obtention des transformations de l'articulation courante dans l'espace de l'articulation parente
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

void CAViewer::bvhTransitionDrawGL(const chara::BVH& bvhSRC, int frameNumberSRC,
				   const chara::BVH& bvhDST, int frameNumberDST,
				   const float interpolationValue)
{
  bvhTransitionDrawGLRec(*bvhSRC.getRoot(), frameNumberSRC,
			 *bvhDST.getRoot(), frameNumberDST,
			 interpolationValue);
}
void CAViewer::bvhTransitionDrawGLRec(const chara::BVHJoint& jointSRC,int frameNumberSRC,
				      const chara::BVHJoint& jointDST, int frameNumberDST,
				      const float interpolationValue)
{
  int i;

  math::Vec3<float> interpolatedJointOffset;
  math::Vec3<float> interpolatedJointTranslation;
  math::Vec3<float> interpolatedRotationAxis;
  float interpolatedRotationAngle;

  // Sauvegarde la matrice de transformation courante
  glPushMatrix();

  {
    math::Vec3<float> jointOffsetSRC;
    math::Vec3<float> jointTranslationSRC;
    math::TQuaternion<float> jointRotationSRC;

    math::Vec3<float> jointOffsetDST;
    math::Vec3<float> jointTranslationDST;
    math::TQuaternion<float> jointRotationDST;

    // Obtention des transformations de l'articulation source et destination dans l'espace de leurs articulations parentes
    bvhGetJointTransforms(jointSRC, jointOffsetSRC, jointTranslationSRC, jointRotationSRC, frameNumberSRC);
    bvhGetJointTransforms(jointDST, jointOffsetDST, jointTranslationDST, jointRotationDST, frameNumberDST);

    bvhVec3LinearInterpolation(jointOffsetSRC, jointOffsetDST, interpolatedJointOffset, interpolationValue);
    bvhVec3LinearInterpolation(jointTranslationSRC, jointTranslationDST, interpolatedJointTranslation, interpolationValue);

    bvhRotationInterpolation(jointRotationSRC, jointRotationDST, interpolationValue, interpolatedRotationAxis, interpolatedRotationAngle);
  }

  // Affichage de l'os reliant l'articulation parente et l'articulation courante
  if (jointSRC.getParent())
  {
    glBegin(GL_LINES);

    glVertex3f(0, 0, 0);
    glVertex3f(interpolatedJointOffset.x + interpolatedJointTranslation.x,
	       interpolatedJointOffset.y + interpolatedJointTranslation.y,
	       interpolatedJointOffset.z + interpolatedJointTranslation.z);
    glEnd();
  }

  // Passage du repère de l'articulation parente au repère de l'articulation courante
  glTranslatef(interpolatedJointOffset.x + interpolatedJointTranslation.x,
	       interpolatedJointOffset.y + interpolatedJointTranslation.y,
	       interpolatedJointOffset.z + interpolatedJointTranslation.z);

  glRotatef(interpolatedRotationAngle, interpolatedRotationAxis.x, interpolatedRotationAxis.y, interpolatedRotationAxis.z);

  // Dessin de l'articulation courante
  draw_cube();

  // Récursion sur les articulations descendantes
  for (i = 0; i < jointSRC.getNumChild(); ++i)
    bvhTransitionDrawGLRec(*jointSRC.getChild(i), frameNumberSRC,
			   *jointDST.getChild(i), frameNumberDST,
			   interpolationValue);

  // Restaure la matrice de transformation précédente
  glPopMatrix();
}

void CAViewer::bvhGetJointTransforms(const chara::BVHJoint& joint,
				     math::Vec3<float>& offset,
				     math::Vec3<float>& translation,
				     math::TQuaternion<float>& rotation,
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

  rotation = math::TQuaternion<float>();

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
	rotation *= math::TQuaternion<float>(math::Vec3<float>(1, 0, 0), data * M_PI / 180);
      else if (axis == AXIS_Y)
	rotation *= math::TQuaternion<float>(math::Vec3<float>(0, 1, 0), data * M_PI / 180);
      else if (axis == AXIS_Z)
	rotation *= math::TQuaternion<float>(math::Vec3<float>(0, 0, 1), data * M_PI / 180);
    }
  }
}

void CAViewer::bvhVec3LinearInterpolation(const math::Vec3<float>& v1,
					  const math::Vec3<float>& v2,
					  math::Vec3<float>& r,
					  float value)
{
  r = (1.f - value) * v1 + (value * v2);
}
void CAViewer::bvhRotationInterpolation(const math::TQuaternion<float>& q1,
					const math::TQuaternion<float>& q2,
					float value,
					math::Vec3<float>& rotationAxisResult,
					float& rotationDegreeResult)
{
  float radians;

  math::TQuaternion<float> qi;

  // Slerp
  qi = math::TQuaternion<float>::slerp(q1, q2, value);

  // Obtention de l'axe de rotation du quaternion
  qi.getAxisAngle(rotationAxisResult, radians);
  rotationDegreeResult = 180.f * radians / M_PI;
}

void CAViewer::keyPressed(unsigned char key, int x, int y)
{
  bool handled = false;
  if (key == 'n')
  {
    ++m_bvhFrame;
    //m_skel->setPostureFromBVH( *m_bvh, m_bvhFrame);
    handled = true;
  }
  else if (key == 'b')
  {
    --m_bvhFrame;
    //m_skel->setPostureFromBVH( *m_bvh, m_bvhFrame);
    handled = true;
  }
  else if (key == 'w')
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

  if (glutGetModifiers() == GLUT_ACTIVE_SHIFT)
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
