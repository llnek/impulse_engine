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
// Copyright © 2020-2022, Kenneth Leung. All rights reserved.

;(function(gscope,UNDEF){

  "use strict";

  /** Create Module */
  function _module(IE,Core,_M,_V){

    const {u:_}= Core;

    /**
     * @public
     * @class
     */
    class Manifold{
      constructor(a, b){
        this.A=a;
        this.B=b;
        this.penetration=0; // Depth of penetration from collision
        this.normal=_V.vec(); // From A to B
        this.contacts=[_V.vec(),_V.vec()]; // Points of contact during collision
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
          let ra = _V.sub(this.contacts[i], this.A.position);
          let rb = _V.sub(this.contacts[i], this.B.position);
          let rv = _V.add(this.B.velocity,_V.cross(this.B.angularVelocity, rb));
          rv= _V.sub(_V.sub(rv,this.A.velocity),
                     _V.cross(this.A.angularVelocity, ra));
          // Determine if we should perform a resting collision or not
          // The idea is if the only thing moving this object is gravity,
          // then the collision should be performed without any restitution
          if(_V.len2(rv) < _V.len2(_V.mul(IE.gravity,IE.dt)) + _M.EPSILON)
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
          let ra = _V.sub(this.contacts[i], this.A.position);
          let rb = _V.sub(this.contacts[i], this.B.position);
          // Relative velocity
          let rv = _V.add(this.B.velocity,_V.cross(this.B.angularVelocity, rb));
          rv= _V.sub(_V.sub(rv, this.A.velocity),
                     _V.cross(this.A.angularVelocity, ra));
          // Relative velocity along the normal
          let contactVel = _V.dot(rv, this.normal);
          // Do not resolve if velocities are separating
          if(contactVel > 0)
            return this;
          let raCrossN = _V.cross(ra, this.normal);
          let rbCrossN = _V.cross(rb, this.normal);
          let invMassSum = this.A.im + this.B.im + (raCrossN*raCrossN) *
                           this.A.iI + (rbCrossN*rbCrossN) * this.B.iI;
          // Calculate impulse scalar
          let j = -(1.0 + this.e) * contactVel;
          j /= invMassSum;
          j /= this.contact_count;
          // Apply impulse
          let impulse = _V.mul(this.normal,j);
          this.A.applyImpulse(_V.flip(impulse), ra);
          this.B.applyImpulse(impulse, rb);
          // Friction impulse
          rv = _V.add(this.B.velocity,_V.cross(this.B.angularVelocity, rb));
          rv = _V.sub(_V.sub(rv, this.A.velocity),
                      _V.cross(this.A.angularVelocity, ra));
          let t = _V.sub(rv,_V.mul(this.normal,_V.dot(rv, this.normal)));
          t = _V.unit(t);
          // j tangent magnitude
          let jt = -_V.dot(rv, t);
          jt /= invMassSum;
          jt /= this.contact_count;
          // Don't apply tiny friction impulses
          if(_.feq0(jt))
            return this;
          // Coulumb's law
          let tangentImpulse;
          if(Math.abs(jt) < j * this.sf)
            tangentImpulse = _V.mul(t, jt);
          else
            tangentImpulse = _V.mul(t, -j * this.df);
          // Apply friction impulse
          this.A.applyImpulse(_V.flip(tangentImpulse), ra);
          this.B.applyImpulse(tangentImpulse, rb);
        }
        return this;
      }
      positionalCorrection(){
        const k_slop = 0.05; // Penetration allowance
        const percent = 0.4; // Penetration percentage to correct
        let correction =
          _V.mul(this.normal,
          (Math.max(this.penetration-k_slop, 0.0)/(this.A.im+this.B.im)) * percent);
        _V.sub$(this.A.position,_V.mul(correction, this.A.im));
        _V.add$(this.B.position,_V.mul(correction, this.B.im));
        return this;
      }
      infiniteMassCorrection(){
        _V.set(this.A.velocity,0, 0);
        _V.set(this.B.velocity,0, 0);
        return this;
      }
    }

    return _.inject(IE, { Manifold: Manifold })
  }

  //;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  //exports
  if(typeof module=="object" && module.exports){
    throw "Panic: browser only"
  }else{
    gscope["io/czlab/impulse_engine/manifold"]=function(IE,Core,_M,_V){
      return _module(IE,Core,_M,_V)
    }
  }

})(this);

