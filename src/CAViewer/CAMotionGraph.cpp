#include "CAMotionGraph.h"
#include "CASkeleton.h"

#include <stdio.h>
#include <stdlib.h>

CAMotionGraph::CAMotionGraph()
{
}
CAMotionGraph::~CAMotionGraph()
{
}

void CAMotionGraph::clear()
{
  unsigned int i;
  const unsigned int bvhCount = m_BVH.size();

  for (i = 0; i < bvhCount; ++i)
    delete m_BVH[i];

  m_BVH.clear();
  m_bvhSkeletons.clear();
  m_GrapheNode.clear();
}

void CAMotionGraph::load(const std::vector<std::string>& bvhFilenames, const float transitionThreshold)
{
  clear();

  loadBVHs(bvhFilenames);

  createSkeletons();
  ensureSketetonsValidity();

  createGraphNodes();
}
void CAMotionGraph::loadBVHs(const std::vector<std::string>& bvhFilenames)
{
  unsigned int i;
  const unsigned int bvhCount = bvhFilenames.size();

  for (i = 0; i < bvhCount; ++i)
    m_BVH.push_back(new chara::BVH(bvhFilenames[i], true));
}

void CAMotionGraph::createSkeletons()
{
  unsigned int i;
  const unsigned int bvhCount = m_BVH.size();

  int f;
  int bvhFrameCount;
  CASkeleton frameSkeleton;
  std::vector<CASkeleton> bvhSkeletons;

  for (i = 0; i < bvhCount; ++i)
  {
    bvhSkeletons.clear();
    bvhFrameCount = m_BVH[i]->getNumFrame();

    for (f = 0; f < bvhFrameCount; ++f)
    {
      frameSkeleton.setPose(*m_BVH[i], f);
      bvhSkeletons.push_back(frameSkeleton);
    }

    m_bvhSkeletons[i] = bvhSkeletons;
  }
}
void CAMotionGraph::ensureSketetonsValidity()
{
  unsigned int i;
  const unsigned int bvhCount = m_BVH.size();

  for (i = 1; i < bvhCount; ++i)
  {
    if (!m_bvhSkeletons[0][0].hasSameLogicalStructure(m_bvhSkeletons[i][0]))
    {
      fprintf(stderr, "CAMotionGraph::load(): Impossible de construire le graphe d'animation: ");
      fprintf(stderr, "Les BVH spécifiées ne possèdent pas la même structure logique\r\n");

      exit(-1);
    }
  }
}

void CAMotionGraph::createGraphNodes()
{
}
