#include "CAMotionGraph.h"
#include "CASkeleton.h"

#include <fstream>
#include <stdexcept>
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
  m_bvhFilenames = bvhFilenames;

  clear();

  loadBVHs();

  createSkeletons();
  ensureSketetonsValidity();

  createGraphNodes();
  createGraphArcs(transitionThreshold);

  write("motion_graph.txt");
}
void CAMotionGraph::loadBVHs()
{
  unsigned int i;
  const unsigned int bvhCount = m_bvhFilenames.size();

  for (i = 0; i < bvhCount; ++i)
    m_BVH.push_back(new chara::BVH(m_bvhFilenames[i], true));
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
  unsigned int i;
  const unsigned int bvhCount = m_BVH.size();

  int f;
  int bvhFrameCount;

  GrapheNode n;

  for (i = 0; i < bvhCount; ++i)
  {
    n.id_bvh = i;

    bvhFrameCount = m_BVH[i]->getNumFrame();

    for (f = 0; f < bvhFrameCount; ++f)
    {
      n.frame = f;
      n.ids_next.clear();

      m_GrapheNode.push_back(n);
    }
  }
}

CAMotionGraph::BVH_ID CAMotionGraph::getGraphNodeId(const BVH_ID bvhId, const int frame)
{
  unsigned int i;
  const unsigned int nodeCount = m_GrapheNode.size();

  for (i = 0; i < nodeCount; ++i)
    if (m_GrapheNode[i].id_bvh == bvhId && m_GrapheNode[i].frame == frame)
      return (BVH_ID)i;

  throw std::invalid_argument("CAMotionGraph::getGraphNode(): @bvhId et/ou @frame invalide(s)");
}
CAMotionGraph::GrapheNode& CAMotionGraph::getGraphNode(const BVH_ID bvhId, const int frame)
{
  return m_GrapheNode[getGraphNodeId(bvhId, frame)];
}

void CAMotionGraph::createGraphArcs(const float transitionThreshold)
{
  unsigned int i;
  const unsigned int bvhCount = m_BVH.size();

  for (i = 0; i < bvhCount; ++i)
    createGraphArcs((BVH_ID)i, transitionThreshold);
}
void CAMotionGraph::createGraphArcs(const BVH_ID refBvhId, const float transitionThreshold)
{
  CASkeletonMap::iterator it;
  CASkeletonMap::iterator itEnd;

  const std::vector<CASkeleton>& refBvhSkeletons = m_bvhSkeletons[refBvhId];

  itEnd = m_bvhSkeletons.end();

  for (it = m_bvhSkeletons.begin(); it != itEnd; ++it)
  {
    if (it->first == refBvhId)
      continue;

    createGraphArcs(refBvhId, refBvhSkeletons, it->first, it->second, transitionThreshold);
  }
}
void CAMotionGraph::createGraphArcs(const BVH_ID srcBvhId,
				    const std::vector<CASkeleton>& srcSkeletons,
				    const BVH_ID dstBvhId,
				    const std::vector<CASkeleton>& dstSkeletons,
				    const float transitionThreshold)
{
  float minDistance;

  unsigned int srcFrame;
  const unsigned int srcFrameCount = srcSkeletons.size();

  unsigned int dstFrame;
  const unsigned int dstFrameCount = dstSkeletons.size();

  minDistance = 1000;

  for (srcFrame = 0; srcFrame < srcFrameCount; ++srcFrame)
  {
    for (dstFrame = 0; dstFrame < dstFrameCount; ++dstFrame)
    {
      if (srcSkeletons[srcFrame].distance(dstSkeletons[dstFrame]) < transitionThreshold)
	getGraphNode(srcBvhId, srcFrame).ids_next.push_back(getGraphNodeId(dstBvhId, dstFrame));

      if (srcSkeletons[srcFrame].distance(dstSkeletons[dstFrame]) < minDistance)
	minDistance = srcSkeletons[srcFrame].distance(dstSkeletons[dstFrame]);
    }
  }

  printf("CAMotionGraph::createGraphArcs(srcBvhId = %d, dstBvhId = %d, tolerance = %f): Distance minimale = %f\r\n",
	 srcBvhId, dstBvhId,
	 transitionThreshold,
	 minDistance);
}

void CAMotionGraph::write(const std::string& filename) const
{
  std::ofstream file;

  unsigned int i;
  const unsigned int nodeCount = m_GrapheNode.size();

  unsigned int arc;
  unsigned int arcCount;

  file.open(filename.c_str());

  for (i = 0; i < nodeCount; ++i)
  {
    file << i << " => ";

    arcCount = m_GrapheNode[i].ids_next.size();
    for (arc = 0; arc < arcCount; ++arc)
      file << m_GrapheNode[i].ids_next[arc] << " ";

    file << std::endl;
  }

  file.close();
}
