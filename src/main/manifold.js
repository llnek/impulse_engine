// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Copyright © 2020, Kenneth Leung. All rights reserved.

;(function(global){
  "use strict";
  //export--------------------------------------------------------------------
  if(typeof module === "object" &&
     module && typeof module.exports === "object"){
    global=module.exports;
  }
  else if(typeof exports === "object" && exports){
    global=exports;
  }
  /**
   * @public
   * @function
   */
  global["io.czlab.impulse_engine.manifold"]=function(IE,Core,_M){
    /**
     * @public
     * @class
     */
    class Manifold{
      constructor(a, b){
        this.A=a;
        this.B=b;
        this.penetration=0; // Depth of penetration from collision
        this.normal=_M.V2(); // From A to B
        this.contacts=[_M.V2(),_M.V2()]; // Points of contact during collision
        this.contact_count=0; // Number of contacts that occured during collision
        this.e=0;               // Mixed restitution
        this.df=0;              // Mixed dynamic friction
        this.sf=0;              // Mixed static friction
      }
      solve(){
        IE.dispatch(this.A.shape.getType(),this.B.shape.getType()).call(IE, this, this.A,this.B);
        return this;
      }
      initialize(){
        // Calculate average restitution
        this.e = Math.min(this.A.restitution, this.B.restitution);
        // Calculate static and dynamic friction
        this.sf = Math.sqrt(this.A.staticFriction * this.B.staticFriction);
        this.df = Math.sqrt(this.A.dynamicFriction * this.B.dynamicFriction);
        for(let i = 0; i < this.contact_count; ++i){
          // Calculate radii from COM to contact
          let ra = _M.vecSub(this.contacts[i], this.A.position);
          let rb = _M.vecSub(this.contacts[i], this.B.position);
          let rv = _M.vecAdd(this.B.velocity,_M.vec2Cross(this.B.angularVelocity, rb));
          rv= _M.vecSub(_M.vecSub(rv,this.A.velocity),
                        _M.vec2Cross(this.A.angularVelocity, ra));
          // Determine if we should perform a resting collision or not
          // The idea is if the only thing moving this object is gravity,
          // then the collision should be performed without any restitution
          if(_M.vecLen2(rv) < _M.vecLen2(_M.vecMul(IE.gravity,IE.dt)) + _M.EPSILON)
            this.e = 0.0;
        }
        return this;
      }
      applyImpulse(){
        // Early out and positional correct if both objects have infinite mass
        if(_M.fuzzyZero(this.A.im + this.B.im)){
          this.infiniteMassCorrection();
          return this;
        }
        for(let i = 0; i < this.contact_count; ++i){
          // Calculate radii from COM to contact
          let ra = _M.vecSub(this.contacts[i], this.A.position);
          let rb = _M.vecSub(this.contacts[i], this.B.position);
          // Relative velocity
          let rv = _M.vecAdd(this.B.velocity,_M.vec2Cross(this.B.angularVelocity, rb));
          rv= _M.vecSub(_M.vecSub(rv, this.A.velocity),
                        _M.vec2Cross(this.A.angularVelocity, ra));
          // Relative velocity along the normal
          let contactVel = _M.vecDot(rv, this.normal);
          // Do not resolve if velocities are separating
          if(contactVel > 0)
            return this;
          let raCrossN = _M.vec2Cross(ra, this.normal);
          let rbCrossN = _M.vec2Cross(rb, this.normal);
          let invMassSum = this.A.im + this.B.im + (raCrossN*raCrossN) *
                           this.A.iI + (rbCrossN*rbCrossN) * this.B.iI;
          // Calculate impulse scalar
          let j = -(1.0 + this.e) * contactVel;
          j /= invMassSum;
          j /= this.contact_count;
          // Apply impulse
          let impulse = _M.vecMul(this.normal,j);
          this.A.applyImpulse(_M.vecFlip(impulse), ra);
          this.B.applyImpulse(impulse, rb);
          // Friction impulse
          rv = _M.vecAdd(this.B.velocity,_M.vec2Cross(this.B.angularVelocity, rb));
          rv = _M.vecSub(_M.vecSub(rv, this.A.velocity),
                         _M.vec2Cross(this.A.angularVelocity, ra));
          let t = _M.vecSub(rv,_M.vecMul(this.normal,_M.vecDot(rv, this.normal)));
          t = _M.vecUnit(t);
          // j tangent magnitude
          let jt = -_M.vecDot(rv, t);
          jt /= invMassSum;
          jt /= this.contact_count;
          // Don't apply tiny friction impulses
          if(_M.fuzzyZero(jt))
            return this;
          // Coulumb's law
          let tangentImpulse;
          if(Math.abs(jt) < j * this.sf)
            tangentImpulse = _M.vecMul(t, jt);
          else
            tangentImpulse = _M.vecMul(t, -j * this.df);
          // Apply friction impulse
          this.A.applyImpulse(_M.vecFlip(tangentImpulse), ra);
          this.B.applyImpulse(tangentImpulse, rb);
        }
        return this;
      }
      positionalCorrection(){
        const k_slop = 0.05; // Penetration allowance
        const percent = 0.4; // Penetration percentage to correct
        let correction =
          _M.vecMul(this.normal,
          (Math.max(this.penetration-k_slop, 0.0)/(this.A.im+this.B.im)) * percent);
        this.A.position = _M.vecSub(this.A.position,_M.vecMul(correction, this.A.im));
        this.B.position = _M.vecAdd(this.B.position,_M.vecMul(correction, this.B.im));
        return this;
      }
      infiniteMassCorrection(){
        _M.vecCopy(this.A.velocity,0, 0);
        _M.vecCopy(this.B.velocity,0, 0);
        return this;
      }
    }

    IE.Manifold=Manifold;
    return IE;
  };

})(this);

