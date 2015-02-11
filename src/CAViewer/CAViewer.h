#ifndef _CAVIEWER_H
#define _CAVIEWER_H

#include "CAMotionGraph.h"

#include <BVH.h>
#include <Vec3.h>
#include <Viewer.h>
#include <Quaternion.h>

#include <vector>
#include <string>

class CASkeleton;

class CAViewer : public Viewer
{
protected:

  bool isPhysics;

  //! mocap data
  chara::BVH* m_bvh;
  //! mocap frame number (when mocap is used)
  int m_bvhFrame;

  //! skeleton (build from the mocap-BVH data m_bvh)
  CASkeleton* m_skel;

  //! motion graph
  CAMotionGraph m_graph;

public:

  CAViewer();
  virtual ~CAViewer();

  virtual void help();
  virtual void init();
  virtual void loadMotionGraph(const std::vector<std::string>& bvhFilenames, const float transitionThreshold = 1);
  virtual void animate();
  virtual void draw();

  virtual void keyPressed(unsigned char key, int x, int y);
  virtual void specialKeyPressed(int key, int x, int y);

private:

  void bvhDrawGL(const chara::BVH& bvh, int frameNumber);
  void bvhDrawGLRec(const chara::BVHJoint& joint, int frameNumber);

  void bvhTransitionDrawGL(const chara::BVH& bvhSRC,int frameNumberSRC,
			   const chara::BVH& bvhDST, int frameNumberDST,
			   const float interpolationValue);
  void bvhTransitionDrawGLRec(const chara::BVHJoint& jointSRC,int frameNumberSRC,
			      const chara::BVHJoint& jointDST, int frameNumberDST,
			      const float interpolationValue);

  void bvhGetJointTransforms(const chara::BVHJoint& joint,
			     math::Vec3<float>& offset,
			     math::Vec3<float>& translation,
			     math::Vec3<float>& rotations,
			     int frameNumber);
  void bvhGetJointTransforms(const chara::BVHJoint& joint,
			     math::Vec3<float>& offset,
			     math::Vec3<float>& translation,
			     math::TQuaternion<float>& rotation,
			     int frameNumber);

  void bvhVec3LinearInterpolation(const math::Vec3<float>& v1,
				  const math::Vec3<float>& v2,
				  math::Vec3<float>& r,
				  float value);
  void bvhRotationInterpolation(const math::TQuaternion<float>& r1,
				const math::TQuaternion<float>& r2,
				float value,
				math::Vec3<float>& rotationAxisResult,
				float& rotationDegreeResult);
};

#endif
