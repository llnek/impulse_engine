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
  global["io.czlab.impulse_engine.scene"]=function(IE,Core,_M,_G){
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
      b.velocity =
        _M.vecAdd(b.velocity,
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
      b.position = _M.vecAdd(b.position,_M.vecMul(b.velocity,dt));
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
        this.m_dt= dt;
        this.bodies=[];
        this.contacts=[];
        this.m_iterations= iterations;
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
            m= new IE.Manifold(A, B);
            m.solve();
            if(m.contact_count>0)
              this.contacts.push(m);
          }
        }
        // Integrate forces
        for(let i=0; i < this.bodies.length; ++i)
          _integrateForces(this.bodies[i], this.m_dt);
        // Initialize collision
        for(let i=0; i < this.contacts.length; ++i)
          this.contacts[i].initialize();
        // Solve collisions
        for(let j = 0; j < this.m_iterations; ++j)
          for(let i = 0; i < this.contacts.length; ++i)
            this.contacts[i].applyImpulse();
        // Integrate velocities
        for(let i = 0; i < this.bodies.length; ++i)
          _integrateVelocity(this.bodies[i], this.m_dt);
        // Correct positions
        for(let i = 0; i < this.contacts.length; ++i)
          this.contacts[i].positionalCorrection();
        // Clear all forces
        for(let b, i = 0; i < this.bodies.length; ++i){
          b = this.bodies[i];
          _M.vecCopy(b.force, 0,0);
          b.torque = 0;
        }
        for(let b,i=0;i<this.bodies.length;++i){
          b=this.bodies[i];
          if(!_G.containsPoint(IE.gWorld,b.position[0],b.position[1]))
            if(!_G.rectContainsRect(IE.gWorld,b.shape.getAABB())){
              this.bodies.splice(i,1);
              --i;
            }
        }
      }
      render(ctx){
        for(let b,i = 0; i < this.bodies.length; ++i){
          b = this.bodies[i];
          b.shape.draw(ctx);
        }
        //glPointSize( 4.0f );
        //glBegin( GL_POINTS );
        //glColor3f( 1.0f, 0.0f, 0.0f );
        for(let m,i = 0; i < this.contacts.length; ++i){
          m = this.contacts[i];
          for(let c,j = 0; j < m.contact_count; ++j){
            c = m.contacts[j];
            //glVertex2f( c.x, c.y );
          }
        }
        //glEnd( );
        //glPointSize( 1.0f );
        //glBegin( GL_LINES );
        //glColor3f( 0.0f, 1.0f, 0.0f );
        for(let m,n,i = 0; i < this.contacts.length; ++i){
          m = this.contacts[i];
          n = m.normal;
          for(let c,j = 0; j < m.contact_count; ++j){
            c = m.contacts[j];
            //glVertex2f( c.x, c.y );
            n *= 0.75;
            c += n;
            //glVertex2f( c.x, c.y );
          }
        }
        //glEnd();
      }
      add(shape, x, y){
        _.assert(shape);
        let b = new IE.Body(shape, x, y);
        this.bodies.push(b);
        return b;
      }
    }

    IE.Scene=Scene;
    return IE;
  };

})(this);

