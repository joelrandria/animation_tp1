#include "CASkeleton.h"

#include <BVHJoint.h>
#include <BVHChannel.h>
#include <BVHAxis.h>

#include <glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

#include <iostream>

CASkeleton::CASkeleton()
{
}
CASkeleton::CASkeleton(const chara::BVH& bvh)
{
  setPose(bvh, 0);
}
CASkeleton::CASkeleton(const CASkeleton& instance)
{
  *this = instance;
}
CASkeleton::~CASkeleton()
{
}

CASkeleton& CASkeleton::operator=(const CASkeleton& instance)
{
  m_joints = instance.m_joints;
}

void CASkeleton::setPose(const chara::BVH& bvh, const int frameNumber)
{
  m_joints.clear();

  addJoint(*bvh.getRoot(), frameNumber, -1);
}
void CASkeleton::addJoint(const chara::BVHJoint& bvhJoint, const int frameNumber, int fatherId)
{
  CAJoint joint;

  int i;
  int jointId;

  const int childCount = bvhJoint.getNumChild();

  joint.m_fatherId = fatherId;
  //joint.m_local2world = getLocal2AncestorJointMatrix(bvhJoint, frameNumber);
  joint.m_worldPosition = getLocal2AncestorJointMatrix(bvhJoint, frameNumber) * math::Vec3f();
  joint.m_rootPosition = getLocal2AncestorJointMatrix(bvhJoint, frameNumber, false) * math::Vec3f();

  jointId = (int)m_joints.size();

  m_joints.push_back(joint);

  for (i = 0; i < childCount; ++i)
    addJoint(*bvhJoint.getChild(i), frameNumber, jointId);
}

math::Mat4f CASkeleton::getLocal2AncestorJointMatrix(const chara::BVHJoint& bvhJoint,
						     const int frameNumber,
						     bool bIncludeRootTransform) const
{
  math::Mat4f m;
  const chara::BVHJoint* j;

  j = &bvhJoint;

  for (;;)
  {
    if (!bIncludeRootTransform && !j->getParent())
      break;
    if (!j)
      break;

    m = getLocal2ParentJointMatrix(*j, frameNumber) * m;

    j = j->getParent();
  }

  return m;
}
math::Mat4f CASkeleton::getLocal2ParentJointMatrix(const chara::BVHJoint& bvhJoint, const int frameNumber) const
{
  math::Mat4f m;
  math::Vec3f offset;

  int i;
  float data;
  int channelCount;
  chara::BVHChannel* channel;
  chara::BVHChannel::TYPE type;
  chara::AXIS axis;

  channelCount = bvhJoint.getNumChannel();

  bvhJoint.getOffset(offset.x, offset.y, offset.z);
  m.addTranslation(offset);

  for (i = 0; i < channelCount; ++i)
  {
    channel = bvhJoint.getChannel(i);
    data = channel->getData(frameNumber);

    type = channel->getType();
    axis = channel->getAxis();

    if (type == chara::BVHChannel::TYPE_TRANSLATION)
    {
      if (axis == chara::AXIS_X)
	m.addTranslation(math::Vec3f(data, 0, 0));
      else if (axis == chara::AXIS_Y)
	m.addTranslation(math::Vec3f(0, data, 0));
      else if (axis == chara::AXIS_Z)
	m.addTranslation(math::Vec3f(0, 0, data));
    }
    else if (type == chara::BVHChannel::TYPE_ROTATION)
    {
      if (axis == chara::AXIS_X)
	m.addRotationX(data * M_PI / 180);
      else if (axis == chara::AXIS_Y)
	m.addRotationY(data * M_PI / 180);
      else if (axis == chara::AXIS_Z)
	m.addRotationZ(data * M_PI / 180);
    }
  }

  return m;
}

bool CASkeleton::hasSameLogicalStructure(const CASkeleton& skel) const
{
  uint i;

  if (m_joints.size() != skel.m_joints.size())
    return false;

  const uint jointCount = m_joints.size();

  for (i = 0; i < jointCount; ++i)
    if (m_joints[i].m_fatherId != skel.m_joints[i].m_fatherId)
      return false;

  return true;
}
float CASkeleton::distance(const CASkeleton& skel) const
{
  float d;

  uint i;
  const uint jointCount = m_joints.size();

  d = 0;
  for (i = 0; i < jointCount; ++i)
    d += (m_joints[i].m_rootPosition - skel.m_joints[i].m_rootPosition).norm();

  return d;
}

void CASkeleton::drawGL() const
{
  int i;

  const int jointCount = (int)m_joints.size();

  glMatrixMode(GL_MODELVIEW);

  for (i = 0; i < jointCount; ++i)
  {
    glPushMatrix();

    glTranslatef(m_joints[i].m_worldPosition.x, m_joints[i].m_worldPosition.y, m_joints[i].m_worldPosition.z);
    glColor3f(1, 1, 0);

    drawCube();

    glPopMatrix();

    if (m_joints[i].m_fatherId >= 0)
    {
      glBegin(GL_LINES);

      glVertex3f(m_joints[i].m_worldPosition.x,
		 m_joints[i].m_worldPosition.y,
		 m_joints[i].m_worldPosition.z);
      glVertex3f(m_joints[m_joints[i].m_fatherId].m_worldPosition.x,
		 m_joints[m_joints[i].m_fatherId].m_worldPosition.y,
		 m_joints[m_joints[i].m_fatherId].m_worldPosition.z);

      glEnd();
    }
  }
}
void CASkeleton::drawCube() const
{
  static float pt[8][3] = { {0,0,0}, {1,0,0}, {1,0,1}, {0,0,1}, {0,1,0}, {1,1,0}, {1,1,1}, {0,1,1} };
  static int f[6][4] = { {0,1,2,3}, {5,4,7,6}, {1,5,6,2}, {0,4,7,3}, {3,2,6,7}, {0,4,5,1} };
  static float n[6][3] = { {0,-1,0}, {0,1,0}, {1,0,0}, {-1,0,0}, {0,0,1}, {0,0,-1} };
  static float uv[4][2] = { {0,0}, {1,0}, {1,1}, {0,1} };
  int i,j;

  glPushMatrix();
  glTranslatef(-0.5,-0.5,-0.5);
  glBegin(GL_QUADS);
  for (i=0;i<6;i++)
  {
    glNormal3f( n[ i ][0], n[ i ][1], n[ i ][2] );
    for (j=0;j<4;j++)
    {
      glTexCoord2f( uv[j][0], uv[j][1] );
      glVertex3f( pt[ f[i][j] ][0], pt[ f[i][j] ][1], pt[ f[i][j] ][2] );
    }
  }
  glEnd();
  glPopMatrix();
}

void CASkeleton::printJoints() const
{
  uint i;

  const uint jointCount = m_joints.size();

  for (i = 0; i < jointCount; ++i)
    printf("[%d]: Parent=%d\r\n", i, m_joints[i].m_fatherId);
}
void CASkeleton::printMatrix(const math::Mat4f& m) const
{
  int r;
  int c;

  for (r = 0; r < 4; ++r)
  {
    for (c = 0; c < 4; ++c)
      std::cout << m[r][c] << " ";

    std::cout << std::endl;
  }
}
