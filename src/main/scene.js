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

  /**Create Module */
  function _module(IE,Core,_M,_V,_G,_2d){

    const {u:_}=Core;

    // Acceleration
    //    F = mA
    // => A = F * 1/m
    // Explicit Euler
    // x += v * dt
    // v += (1/m * F) * dt
    // Semi-Implicit (Symplectic) Euler
    // v += (1/m * F) * dt
    // x += v * dt

    /** @ignore */
    function _integrateForces(b, dt){
      if(!_.feq0(b.im)){
        let dt2= dt/2.0;
        _V.add$(b.velocity,
                _V.mul(_V.add(_V.mul(b.force,b.im),IE.gravity),dt2));
        b.angularVelocity += b.torque * b.iI * dt2;
      }
    }

    /** @ignore */
    function _integrateVelocity(b, dt){
      if(!_.feq0(b.im)){
        _V.add$(b.position,_V.mul(b.velocity,dt));
        b.orient += b.angularVelocity * dt;
        b.setOrient(b.orient);
        _integrateForces(b, dt);
      }
    }

    /**
     * @public
     * @class
     */
    class World{
      constructor(dt, iterations){
        this.dt= dt;
        this.bodies=[];
        this.contacts=[];
        this.tries= iterations;
      }
      step(){
        // Generate new collision info
        this.contacts.length=0;
        for(let A,i=0; i < this.bodies.length; ++i){
          A = this.bodies[i];
          for(let m,B,j= i+1; j < this.bodies.length; ++j){
            B = this.bodies[j];
            if(!(_.feq0(A.im) && _.feq0(B.im))){
              m= new IE.Manifold(A, B).solve();
              if(m.contact_count>0) this.contacts.push(m);
            }
          }
        }
        // Integrate forces
        // Initialize collision
        // Solve collisions
        // Integrate velocities
        // Correct positions
        // Clear all forces
        this.bodies.forEach(b => _integrateForces(b, this.dt));
        this.contacts.forEach(c => c.initialize());
        for(let i=0;i<this.tries;++i)
          this.contacts.forEach(c => c.applyImpulse());
        this.bodies.forEach(b => _integrateVelocity(b, this.dt));
        this.contacts.forEach(c => c.positionalCorrection());
        this.bodies.forEach(b => {
          _V.set(b.force, 0,0);
          b.torque = 0;
        });
      }
      render(ctx){
        this.bodies.forEach(b => b.shape.draw(ctx));
      }
      add(shape, x, y){
        let b = new IE.Body(shape, x, y);
        this.bodies.push(b);
        return b;
      }
    }

    return _.inject(IE, { World: World })
  };

  //;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  //exports
  if(typeof module=="object" && module.exports){
    throw "Panic: browser only"
  }else{
    gscope["io/czlab/impulse_engine/scene"]=function(IE,Core,_M,_V,_G,_2d){
      return _module(IE,Core,_M,_V,_G,_2d)
    }
  }

})(this);

