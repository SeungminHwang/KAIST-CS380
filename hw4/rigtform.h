#ifndef RIGTFORM_H
#define RIGTFORM_H

#include <iostream>
#include <cassert>

#include "matrix4.h"
#include "quat.h"

class RigTForm {
  Cvec3 t_; // translation component
  Quat r_;  // rotation component represented as a quaternion

public:
  RigTForm() : t_(0) {
    assert(norm2(Quat(1,0,0,0) - r_) < CS175_EPS2);
  }

  RigTForm(const Cvec3& t, const Quat& r) {
    //TODO
      t_ = Cvec3(t[0], t[1], t[2]);
      r_ = Quat(r[0], r[1], r[2], r[3]);
  }

  explicit RigTForm(const Cvec3& t) {
    // TODO
      t_ = Cvec3(t[0], t[1], t[2]);
      r_ = Quat(1, 0, 0, 0);// identity rotation
  }

  explicit RigTForm(const Quat& r) {
    // TODO
      t_ = Cvec3(0, 0, 0); // no translation
      r_ = Quat(r[0], r[1], r[2], r[3]);
  }

  Cvec3 getTranslation() const {
    return t_;
  }

  Quat getRotation() const {
    return r_;
  }

  RigTForm& setTranslation(const Cvec3& t) {
    t_ = t;
    return *this;
  }

  RigTForm& setRotation(const Quat& r) {
    r_ = r;
    return *this;
  }

  Cvec4 operator * (const Cvec4& a) const {
    // TODO
      return r_ * a + Cvec4(t_[0], t_[1], t_[2], 0); // A.r*c + Cvec4(A.t, 0)
  }

  RigTForm operator * (const RigTForm& a) const {
    // TODO
      Cvec3 nt; // matrix representation : t1 + r1t2;
      Quat nr; // matrix representation : r1r2;

      Quat t2_rot = r_ * Quat(0, a.t_[0], a.t_[1], a.t_[2]) * inv(r_);

      nt = t_ + Cvec3(t2_rot[1], t2_rot[2], t2_rot[3]);
      nr = r_ * a.r_;

      return RigTForm(nt, nr);
  }
};

inline RigTForm inv(const RigTForm& tform) {
  // TODO
    Quat r_in = tform.getRotation();
    Cvec3 t_in = tform.getTranslation();
    
    Quat t_rot = inv(r_in) * Quat(0, -t_in[0], -t_in[1], -t_in[2]) * r_in; 

    Cvec3 nt = Cvec3(t_rot[1], t_rot[2], t_rot[3]);//mat. rep.: inv(r)inv(t)
    Quat nr = inv(r_in);//mat. rep.: inv(r)

    return RigTForm(nt, nr);
}

inline RigTForm transFact(const RigTForm& tform) {
  return RigTForm(tform.getTranslation());
}

inline RigTForm linFact(const RigTForm& tform) {
  return RigTForm(tform.getRotation());
}

inline Matrix4 rigTFormToMatrix(const RigTForm& tform) {
  // TODO
    Matrix4 t, r;
    r = quatToMatrix(tform.getRotation());
    //Cvec3 temp = tform.getTranslation();
    t = Matrix4::makeTranslation(tform.getTranslation());
    return t*r;
}

#endif
