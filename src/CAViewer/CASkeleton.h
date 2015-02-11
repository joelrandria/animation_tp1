#ifndef __CASKELETON_H__
#define __CASKELETON_H__

#include <BVH.h>
#include <Mat4.h>

#include <vector>

class CAJoint
{
public:

  int m_fatherId;		// Le numéro du père dans le tableau de CAJoint de CASkeleton

  //math::Mat4f m_local2world;	// La matrice passant du repère de l'articulation vers le monde
  //math::Mat4f m_world2local;	// Si besoin : La matrice passant du repère du monde vers l'articulation

  math::Vec3f m_rootPosition;	// La position de l'articulation par rapport au noeud root
  math::Vec3f m_worldPosition;	// La position de l'articulation dans le repère global
};

class CASkeleton
{
private:

  //! L'ensemble des articulations.
  //! Remarque : la notion de hiérarchie (arbre) n'est plus nécessaire ici,
  //! pour tracer les os on utilise l'information "fatherID" de la class CAJoint
  std::vector<CAJoint> m_joints;

public:

  CASkeleton();
  CASkeleton(const chara::BVH& bvh);
  CASkeleton(const CASkeleton& instance);
  ~CASkeleton();

  CASkeleton& operator=(const CASkeleton& instance);

  //! Positionne ce squelette dans la position n du BVH
  void setPose(const chara::BVH& bvh, const int frameNumber);

  void addJoint(const chara::BVHJoint& bvhJoint, const int frameNumber, int fatherId);

  math::Mat4f getLocal2AncestorJointMatrix(const chara::BVHJoint& bvhJoint,
					   const int frameNumber,
					   bool bIncludeRootTransform = true) const;
  math::Mat4f getLocal2ParentJointMatrix(const chara::BVHJoint& bvhJoint,
					 const int frameNumber) const;

  bool hasSameLogicalStructure(const CASkeleton& skel) const;
  //! Calcule la distance entre deux poses
  //! precond: les deux squelettes doivent avoir le même nombre d'articulations (même structure d'arbre)
  float distance(const CASkeleton& skel) const;

  //! Affiche en OpenGL le squelette, les liens entre les articulations
  //! sont donnés par le champ m_fatherId de CAJoint
  void drawGL() const;
  void drawCube() const;

  void printJoints() const;
  void printMatrix(const math::Mat4f& m) const;
};

#endif
