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

  /**Create module */
  function _module(Core,_V){

    const _2d=gscope["io/czlab/mcfud/geo2d"]();
    const _M=gscope["io/czlab/mcfud/math"]();
    const _G=gscope["io/czlab/mcfud/gfx"]();

    const _gravityScale = 5.0;
    const {u:_,is}=Core;

    /**
     * @public
     * @class
     */
    class Mat2{
      constructor(a,b,c,d){
        this.m00= a; this.m01=b;
        this.m10=c; this.m11=d;
        if(a===undefined){
          this.set(0);
        }
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
        return _V.vec(this.m00, this.m10)
      }
      axisY(){
        return _V.vec(this.m01, this.m11)
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
        return _V.vec(a.m00 * rhs[0] + a.m01 * rhs[1],
                      a.m10 * rhs[0] + a.m11 * rhs[1])
      else
        return new Mat2(a.m00 * rhs.m00 + a.m01 * rhs.m10,
                        a.m00 * rhs.m01 + a.m01 * rhs.m11,
                        a.m10 * rhs.m00 + a.m11 * rhs.m10,
                        a.m10 * rhs.m01 + a.m11 * rhs.m11)
    };

    const _$={
      ePoly:100,
      eCircle: 200,
      Mat2,
      dt: 1.0/60,
      gravity: _V.vec(0, 10*_gravityScale)
    };

    gscope["io/czlab/impulse_engine/shape"](_$,Core,_M,_V,_G,_2d);
    gscope["io/czlab/impulse_engine/body"](_$,Core,_M,_V);
    gscope["io/czlab/impulse_engine/manifold"](_$,Core,_M,_V);
    gscope["io/czlab/impulse_engine/collision"](_$,Core,_M,_V);
    gscope["io/czlab/impulse_engine/scene"](_$,Core,_M,_V,_G,_2d);

    return _$;
  }

  //;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  //exports
  if(typeof module=="object" && module.exports){
    throw "Panic: browser only"
  }else{
    gscope["io/czlab/impulse_engine/core"]=function(){
      return _module(gscope["io/czlab/mcfud/core"](),
                     gscope["io/czlab/mcfud/vec2"]())
    }
  }

})(this);

