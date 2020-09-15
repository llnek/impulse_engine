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
  global["io.czlab.impulse_engine.scene"]=function(IE,Core,_M,_G,_2d){
    const _=Core.u;

    // Acceleration
    //    F = mA
    // => A = F * 1/m
    // Explicit Euler
    // x += v * dt
    // v += (1/m * F) * dt
    // Semi-Implicit (Symplectic) Euler
    // v += (1/m * F) * dt
    // x += v * dt
    /**
     * @private
     * @function
     */
    function _integrateForces(b, dt){
      if(_M.fuzzyZero(b.im))
        return;
      let dt2= dt/2.0;
      _M.vecAddSelf(b.velocity,
                    _M.vecMul(_M.vecAdd(_M.vecMul(b.force,b.im),IE.gravity),dt2));
      b.angularVelocity += b.torque * b.iI * dt2;
    }
    /**
     * @private
     * @function
     */
    function _integrateVelocity(b, dt){
      if(_M.fuzzyZero(b.im))
        return;
      _M.vecAddSelf(b.position,_M.vecMul(b.velocity,dt));
      b.orient += b.angularVelocity * dt;
      b.setOrient(b.orient);
      _integrateForces(b, dt);
    }
    /**
     * @public
     * @class
     */
    class Scene{
      constructor(dt, iterations){
        this.dt= dt;
        this.bodies=[];
        this.contacts=[];
        this.tries= iterations;
      }
      step(){
        // Generate new collision info
        this.contacts.length=0;
        for(let A,i = 0; i < this.bodies.length; ++i){
          A = this.bodies[i];
          for(let m,B,j = i+1; j < this.bodies.length; ++j){
            B = this.bodies[j];
            if(_M.fuzzyZero(A.im) && _M.fuzzyZero(B.im))
              continue;
            m= new IE.Manifold(A, B).solve();
            if(m.contact_count>0)
              this.contacts.push(m);
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
          _M.vecCopy(b.force, 0,0);
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

    return _.inject(IE, { Scene: Scene })
  };

})(this);

