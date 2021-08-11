#ifndef PICKER_H
#define PICKER_H

#include <vector>
#include <map>
#include <memory>
#include <stdexcept>

#include "cvec.h"
#include "scenegraph.h"
#include "asstcommon.h"
#include "ppm.h"
#include "drawer.h"

class Picker : public SgNodeVisitor {
  std::vector<std::shared_ptr<SgNode> > nodeStack_;

  typedef std::map<int, std::shared_ptr<SgRbtNode> > IdToRbtNodeMap;
  IdToRbtNodeMap idToRbtNode_;

  int idCounter_;
  bool srgbFrameBuffer_;

  Drawer drawer_;

  void addToMap(int id, std::shared_ptr<SgRbtNode> node);
  std::shared_ptr<SgRbtNode> find(int id);

  Cvec3 idToColor(int id);
  int colorToId(const PackedPixel& p);

public:
  Picker(const RigTForm& initialRbt, Uniforms& uniforms);

  virtual bool visit(SgTransformNode& node);
  virtual bool postVisit(SgTransformNode& node);
  virtual bool visit(SgShapeNode& node);
  virtual bool postVisit(SgShapeNode& node);

  std::shared_ptr<SgRbtNode> getRbtNodeAtXY(int x, int y);
};


#endif