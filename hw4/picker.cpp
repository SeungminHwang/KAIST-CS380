#include <GL/glew.h>

#include "picker.h"

using namespace std;

Picker::Picker(const RigTForm& initialRbt, const ShaderState& curSS)
  : drawer_(initialRbt, curSS)
  , idCounter_(0)
  , srgbFrameBuffer_(!g_Gl2Compatible) {}

bool Picker::visit(SgTransformNode& node) {
  // TODO
    shared_ptr<SgNode> p = node.shared_from_this();
    shared_ptr<SgRbtNode> q = dynamic_pointer_cast<SgRbtNode>(p);
    if (q != NULL)
    {
        nodeStack_.push_back(q);
    }
  return drawer_.visit(node);
}

bool Picker::postVisit(SgTransformNode& node) {
  // TODO
    if (nodeStack_.size() > 0)
    {
        nodeStack_.pop_back();
    }
  return drawer_.postVisit(node);
}

bool Picker::visit(SgShapeNode& node) {
  // TODO

    // 1. increase idCounter
    idCounter_++;
    // 2. find the SgRbtNode closest from the top of the stack
    // ???
    shared_ptr<SgRbtNode> q = dynamic_pointer_cast<SgRbtNode>(nodeStack_.back());

    // 3. add the association between idCounter and SgRbtNode to the map
    addToMap(idCounter_, q);

    // 4. RGB color
    Cvec3 color = idToColor(idCounter_);
    const ShaderState& curSS = drawer_.getCurSS();
    safe_glUniform3f(curSS.h_uIdColor, color[0], color[1], color[2]);
    //fix
    /*
    shared_ptr<SgNode> p = node.shared_from_this();
    shared_ptr<SgShapeNode> q = dynamic_pointer_cast<SgShapeNode>(p);
    if (q != NULL)   nodeStack_.push_back(q);*/
  return drawer_.visit(node);
}

bool Picker::postVisit(SgShapeNode& node) {
  // TODO
  return drawer_.postVisit(node);
}

shared_ptr<SgRbtNode> Picker::getRbtNodeAtXY(int x, int y) {
  // TODO
    vector<char> px(1 * 1 * 3);
    glReadPixels(x, y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, &px[0]);

    struct PackedPixel px_;
    px_.r = px[0]; px_.g = px[1]; px_.b = px[2];

    const int id = colorToId(px_);
    //cout << "id: " << id << endl;
    //cout << idCounter_ << endl;

    return find(id);//shared_ptr<SgRbtNode>(); // return null for now
}

//------------------
// Helper functions
//------------------
//
void Picker::addToMap(int id, shared_ptr<SgRbtNode> node) {
  idToRbtNode_[id] = node;
}

shared_ptr<SgRbtNode> Picker::find(int id) {
  IdToRbtNodeMap::iterator it = idToRbtNode_.find(id);
  if (it != idToRbtNode_.end())
    return it->second;
  else
    return shared_ptr<SgRbtNode>(); // set to null
}

// encode 2^4 = 16 IDs in each of R, G, B channel, for a total of 16^3 number of objects
static const int NBITS = 4, N = 1 << NBITS, MASK = N-1;

Cvec3 Picker::idToColor(int id) {
  assert(id > 0 && id < N * N * N);
  Cvec3 framebufferColor = Cvec3(id & MASK, (id >> NBITS) & MASK, (id >> (NBITS+NBITS)) & MASK);
  framebufferColor = framebufferColor / N + Cvec3(0.5/N);

  if (!srgbFrameBuffer_)
    return framebufferColor;
  else {
    // if GL3 is used, the framebuffer will be in SRGB format, and the color we supply needs to be in linear space
    Cvec3 linearColor;
    for (int i = 0; i < 3; ++i) {
      linearColor[i] = framebufferColor[i] <= 0.04045 ? framebufferColor[i]/12.92 : pow((framebufferColor[i] + 0.055)/1.055, 2.4);
    }
    return linearColor;
  }
}

int Picker::colorToId(const PackedPixel& p) {
  const int UNUSED_BITS = 8 - NBITS;
  int id = p.r >> UNUSED_BITS;
  id |= ((p.g >> UNUSED_BITS) << NBITS);
  id |= ((p.b >> UNUSED_BITS) << (NBITS+NBITS));
  return id;
}
