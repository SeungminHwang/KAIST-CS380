#include <algorithm>

#include "scenegraph.h"

using namespace std;

bool SgTransformNode::accept(SgNodeVisitor& visitor) {
  if (!visitor.visit(*this))
    return false;
  for (int i = 0, n = children_.size(); i < n; ++i) {
    if (!children_[i]->accept(visitor))
      return false;
  }
  return visitor.postVisit(*this);
}

void SgTransformNode::addChild(shared_ptr<SgNode> child) {
  children_.push_back(child);
}

void SgTransformNode::removeChild(shared_ptr<SgNode> child) {
  children_.erase(find(children_.begin(), children_.end(), child));
}

bool SgShapeNode::accept(SgNodeVisitor& visitor) {
  if (!visitor.visit(*this))
    return false;
  return visitor.postVisit(*this);
}

class RbtAccumVisitor : public SgNodeVisitor {
protected:
  vector<RigTForm> rbtStack_;
  SgTransformNode& target_;
  bool found_;
public:
  RbtAccumVisitor(SgTransformNode& target)
    : target_(target)
    , found_(false) {}

  const RigTForm getAccumulatedRbt(int offsetFromStackTop = 0) {
    // TODO
      RigTForm rbt = RigTForm(Cvec3(0, 0, 0));
      int size = rbtStack_.size();
      for (int i = 0; i < size - offsetFromStackTop;i++)
      {
          rbt = rbt * rbtStack_[i];
      }
      return rbt;
  }

  virtual bool visit(SgTransformNode& node) {
    // TODO
      rbtStack_.push_back(node.getRbt());
      if (target_ == node)
          return false;
      return true;
      //return 0;
  }

  virtual bool postVisit(SgTransformNode& node) {
    // TODO
      rbtStack_.pop_back();
      return true;
  }
};

RigTForm getPathAccumRbt(
  shared_ptr<SgTransformNode> source,
  shared_ptr<SgTransformNode> destination,
  int offsetFromDestination) {

  RbtAccumVisitor accum(*destination);
  source->accept(accum);
  return accum.getAccumulatedRbt(offsetFromDestination);
}
