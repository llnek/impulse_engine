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
;
(function(global){
  //export--------------------------------------------------------------------
  if(typeof module === "object" &&
     module && typeof module.exports === "object"){
    global=module.exports;
  }
  else if(typeof exports === "object" && exports){
    global=exports;
  }
  let _singleton=null;
  global["io.czlab.mcfud.geo2d"]=function(){
    if(_singleton) { return _singleton }
    const _M=global["io.czlab.mcfud.math"]();
    const Core=global["io.czlab.mcfud.core"]();
    const _=Core.u;
    const _G={};
    /**
     * @public
     * @class
     */
    class Box4{
      constructor(left,bottom,right,top){
        this.left=left;
        this.right=right;
        this.top=top;
        this.bottom=bottom;
      }
    }
    /**
     * @public
     * @class
     */
    class Rect{
      constructor(x,y,width,height){
        if(arguments.length===2){
          this.width=x;
          this.height=y;
          this.pos= _M.V2();
        }else{
          this.pos=_M.V2(x,y);
          this.width=width;
          this.height=height;
        }
      }
    }
    /**
     * @public
     * @class
     */
    class Area{
      constructor(w,h){
        this.width=w;
        this.height=h;
      }
      half(){
        return new Area(this.width/2,this.height/2)
      }
    }
    /**
     * Calculate the area of this polygon.
     * @private
     * @function
     */
    _G.polyArea=function(points){
      let area=0;
      for(let p,q,i2,len=points.length,i=0;i<len;++i){
        i2= (i+1)%len;
        p=poly.points[i];
        q=poly.points[i2];
        area += (p[0]*q[1] - q[0]*p[1]);
      }
      return Math.abs(area)/2
    }
    /**
     * Find the center point of this polygon.
     * @public
     * @function
     */
    _G.calcPolyCenter=function(points){
      let A= 6*this.polyArea(points);
      let cx=0;
      let cy=0;
      for(let p,q,i2,i=0,len=points.length;i<len;++i){
        i2= (i+1)%len;
        p=points[i];
        q=points[i2];
        cx += (p[0]+q[0]) * (p[0]*q[1]-q[0]*p[1]);
        cy += (p[1]+q[1]) * (p[0]*q[1]-q[0]*p[1]);
      }
      return _M.V2(cx/A, cy/A)
    };
    /**
     * @public
     * @class
     */
    class Polygon{
      constructor(){
        this.points=[];
      }
    }
    /**
     * @public
     * @class
     */
    class Line{
      constructor(x1,y1,x2,y2){
        this.p= _M.V2(x1,y1);
        this.q= _M.V2(x2,y2);
      }
    }
    /**
     * @public
     * @class
     */
    class Circle{
      constructor(radius){
        this.radius=radius;
      }
    }
    /**
     * Shift a set of points.
     * @public
     * @function
     */
    _G.shiftPoints=function(ps,delta){
      return ps.map(v => _M.vecAdd(v,delta))
    };
    /**
     * Rotate a set of points.
     * @public
     * @function
     */
    _G.rotPoints=function(ps,rot,pivot){
      return ps.map(v => _M.vec2Rot(v,rot,pivot))
    };
    /**
     * Find the vertices of a rectangle.
     * @public
     * @function
     * @returns points in counter-cwise, bottom-right first.
     */
    _G.calcRectPoints=function(w,h){
      let w2=w/2;
      let h2=h/2;
      return [_M.V2(hw,-hh),
              _M.V2(hw,hh),
              _M.V2(-hw,hh),
              _M.V2(-hw,-hh)];
    };
    /**
     * @public
     * @function
     */
    _G.line=function(x1,y1,x2,y2){
      return new Line(x1,y1,x2,y2)
    };
    /**
     * @public
     * @function
     */
    _G.rectEqRect=function(r1,r2){
      return r1.width===r2.width &&
             r1.height===r2.height &&
             r1.pos[0]===r2.pos[0] &&
             r1.pos[1]===r2.pos[1]
    };
    /**
     * @public
     * @function
     */
    _G.rectContainsRect=function(R,r){
      return !(R.pos[0] >= r.pos[0] ||
               R.pos[1] >= r.pos[1] ||
               (R.pos[0]+R.width) <= (r.pos[0]+r.width) ||
               (R.pos[1]+R.height) <= (r.pos[1]+r.height))
    };
    /**
     * Right side on the x-axis.
     * @public
     * @function
     */
    _G.rectGetMaxX=function(r){
      return r.pos[0] + r.width
    }
    /**
     * Middle on the x-axis.
     * @public
     * @function
     */
    _G.rectGetMidX=function(r){
      return r.pos[0] + r.width/2
    };
    /**
     * @public
     * @function
     */
    _G.rectGetMinX=function(r){
      return r.pos[0]
    };
    /**
     * @public
     * @function
     */
    _G.rectGetMaxY=function(r){
      return r.pos[1] + r.height
    };
    /**
     * Mid point on the y-axis.
     * @public
     * @function
     */
    _G.rectGetMidY=function(r){
      return r.pos[1] + r.height/2
    };
    /**
     * @public
     * @function
     */
    _G.rectGetMinY=function(r){
      return r.pos[1]
    }
    /**
     * If point lies inside rect.
     * @public
     * @function
     */
    _G.containsPoint=function(r,x,y){
      return x >= this.rectGetMinX(r) &&
             x <= this.rectGetMaxX(r) &&
             y >= this.rectGetMinY(r) &&
             y <= this.rectGetMaxY(r)
    };
    /**
     * @public
     * @function
     */
    _G.rectIntersectsRect=function(r1,r2){
      return !((r1.pos[0]+r1.width) < r2.pos[0] ||
               (r2.pos[0]+r2.width) < r1.pos[0] ||
               (r1.pos[1]+r1.height) < r2.pos[1] ||
               (r2.pos[1]+r2.height) < r1.pos[1])
    };
    /**
     * Find the union of two rects.
     * @public
     * @function
     */
    _G.rectUnionsRect=function(r1,r2){
      let x= Math.min(r1.pos[0],r2.pos[0]);
      let y= Math.min(r1.pos[1],r2.pos[1]);
      return new Rect(x,y,
                      Math.max(r1.pos[0]+r1.width, r2.pos[0]+r2.width)-x,
                      Math.max(r1.pos[1]+r1.height, r2.pos[1]+r2.height)-y)
    };
    /**
     * @public
     * @function
     */
    _G.rectIntersectsRect=function(r1,r2){
      let x= Math.max(r1.pos[0],r2.pos[0]);
      let y= Math.max(r1.pos[1],r2.pos[1]);
      return new Rect(x,y,
                      Math.min(r1.pos[0]+r1.width, r2.pos[0]+r2.width)-x,
                      Math.min(r1.pos[1]+r1.height, r2.pos[1]+r2.height)-y)
    };

    return _singleton= _.inject(_G, {Circle: Circle,
                                     Line: Line,
                                     Polygon: Polygon, Rect: Rect, Area: Area});
  };

})(this);
