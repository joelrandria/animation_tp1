#ifndef __CAMOTIONGRAPH_H__
#define __CAMOTIONGRAPH_H__

#include "CASkeleton.h"

#include <BVH.h>

#include <map>
#include <vector>
#include <string>

class CAMotionGraph
{
public:

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

private:

  typedef std::map<BVH_ID, std::vector<CASkeleton> > CASkeletonMap;

  std::vector<std::string> m_bvhFilenames;	//! Les noms de fichiers des BVH chargés
  CASkeletonMap m_bvhSkeletons;			//! L'ensemble des squelettes décrits par chaque BVH

  std::vector<chara::BVH*> m_BVH;	//! L'ensemble des BVH du graphe d'animation
  std::vector<GrapheNode> m_GrapheNode;	//! Tous les noeuds du graphe d'animation

public:

  CAMotionGraph();
  ~CAMotionGraph();

  void clear();

  void load(const std::vector<std::string>& bvhFilenames, const float transitionThreshold);

  void write(const std::string& filename) const;

private:

  void loadBVHs();

  void createSkeletons();
  void ensureSketetonsValidity();

  void createGraphNodes();

  BVH_ID getGraphNodeId(const BVH_ID bvhId, const int frame);
  GrapheNode& getGraphNode(const BVH_ID bvhId, const int frame);

  void createGraphArcs(const float transitionThreshold);
  void createGraphArcs(const BVH_ID refBvhId, const float transitionThreshold);
  void createGraphArcs(const BVH_ID srcBvhId,
		       const std::vector<CASkeleton>& srcSkeletons,
		       const BVH_ID dstBvhId,
		       const std::vector<CASkeleton>& dstSkeletons,
		       const float transitionThreshold);
};

#endif
