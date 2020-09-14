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
  global["io.czlab.impulse_engine.body"]=function(IE,Core,_M){
    const _= Core.u;
    /**
     * @public
     * @class
     */
    class Body{
      constructor(shape_, x, y){
        this.shape= shape_.clone();
        this.shape.body = this;
        this.position= _M.V2(x,y);
        this.velocity= _M.V2();
        this.angularVelocity = 0;
        this.torque = 0;
        this.orient = _.randFloat(-Math.PI, Math.PI);
        this.force= _M.V2();
        this.staticFriction = 0.5;
        this.dynamicFriction = 0.3;
        this.restitution = 0.2;
        this.shape.initialize();
        if(this.shape.isCircular()){
          this.rgb = "red"
        }else{
          this.rgb = "blue";
        }
      }
      applyForce(f){
        this.force= _M.vecAdd(this.force,f);
        return this;
      }
      applyImpulse(impulse, contactVector){
        this.velocity= _M.vecAdd(this.velocity, _M.vecMul(impulse,this.im));
        this.angularVelocity += this.iI * _M.vec2Cross(contactVector, impulse);
        return this;
      }
      setStatic(){
        this.I = 0.0;
        this.iI = 0.0;
        this.m = 0.0;
        this.im = 0.0;
        return this;
      }
      setOrient(radians){
        this.orient = radians;
        this.shape.setOrient(radians);
        return this;
      }
    }

    IE.Body=Body;
    return IE;
  };

})(this);

