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
  const _gravityScale = 5.0;
  /**
   * @public
   * @function
   */
  global["io.czlab.impulse_engine.core"]=function(){
    const Core=global["io.czlab.mcfud.core"]();
    const _M=global["io.czlab.mcfud.math"]();
    const _G=global["io.czlab.mcfud.gfx"]();
    const _2d=global["io.czlab.mcfud.geo2d"]();
    const _=Core.u;
    const is=Core.is;
    /**
     * @public
     * @class
     */
    class Mat2{
      constructor(a,b,c,d){
        this.m00= a; this.m01=b;
        this.m10=c; this.m11=d;
      }
      clone(){
        return new Mat2(this.m00, this.m01, this.m10, this.m11)
      }
      set(radians){
        let c = Math.cos(radians);
        let s = Math.sin(radians);
        this.m00 = c; this.m01 = -s;
        this.m10 = s; this.m11 =  c;
        return this;
      }
      abs(){
        return new Mat2(Math.abs(this.m00),
                        Math.abs(this.m01),
                        Math.abs(this.m10),
                        Math.abs(this.m11))
      }
      axisX(){
        return _M.V2(this.m00, this.m10)
      }
      axisY(){
        return _M.V2(this.m01, this.m11)
      }
      transpose(){
        return new Mat2(this.m00, this.m10, this.m01, this.m11)
      }
    }
    /**
     * @public
     * @function
     */
    Mat2.mul=function(a,rhs){
      if(is.vec(rhs))
        return _M.V2(a.m00 * rhs[0] + a.m01 * rhs[1],
                     a.m10 * rhs[0] + a.m11 * rhs[1])
      else
        return new Mat2(a.m00 * rhs.m00 + a.m01 * rhs.m10,
                        a.m00 * rhs.m01 + a.m01 * rhs.m11,
                        a.m10 * rhs.m00 + a.m11 * rhs.m10,
                        a.m10 * rhs.m01 + a.m11 * rhs.m11)
    };
    const _XP={
      ePoly: 100, eCircle: 200,
      M2: Mat2,
      dt: 1 / 60.0,
      gravity: _M.V2(0, 10 * _gravityScale)
    };
    global["io.czlab.impulse_engine.shape"](_XP,Core,_M);
    global["io.czlab.impulse_engine.body"](_XP,Core,_M);
    global["io.czlab.impulse_engine.manifold"](_XP,Core,_M);
    global["io.czlab.impulse_engine.collision"](_XP,Core,_M);
    global["io.czlab.impulse_engine.scene"](_XP,Core,_M,_G,_2d);
    return _XP;
  };


})(this);

