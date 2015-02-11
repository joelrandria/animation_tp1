#ifndef __CAMOTIONGRAPH_H__
#define __CAMOTIONGRAPH_H__

#include "CASkeleton.h"

#include <BVH.h>

#include <map>
#include <vector>
#include <string>

class CAMotionGraph
{
private:

  typedef int BVH_ID;		//! Une animation BVH est repérée par un identifiant=un entier
  typedef int GrapheNodeID;	//! Un noeud du graphe d'animation est repéré par un entier=un identifiant

  //! Un noeud du graphe contient l'identifiant de l'animation, le numéro de la frame et les identifiants des noeuds successeurs
  //! Remarque : du code plus "joli" aurait créer une classe CAGrapheNode
  struct GrapheNode
  {
    BVH_ID id_bvh;
    int frame;
    std::vector<GrapheNodeID> ids_next;	//! Liste des nœuds successeurs
  };

  std::vector<chara::BVH*> m_BVH;	//! L'ensemble des BVH du graphe d'animation
  std::vector<GrapheNode> m_GrapheNode;	//! Tous les noeuds du graphe d'animation

  std::map<BVH_ID,std::vector<CASkeleton> > m_bvhSkeletons;	//! L'ensemble des squelettes décrits par chaque BVH

public:

  CAMotionGraph();
  ~CAMotionGraph();

  void clear();

  void load(const std::vector<std::string>& bvhFilenames, const float transitionThreshold = 1);

private:

  void loadBVHs(const std::vector<std::string>& bvhFilenames);

  void createSkeletons();
  void ensureSketetonsValidity();

  void createGraphNodes();

};

#endif
