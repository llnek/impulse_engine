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
  global["io.czlab.impulse_engine.shape"]=
  function(IE,Core,_M){
    const _2d= global["io.czlab.mcfud.geo2d"]();
    const _G= global["io.czlab.mcfud.gfx"]();
    const MaxPolyVertexCount= 64;
    const _M2=IE.M2;
    const _=Core.u;
    class Shape{
      constructor(){
        this.body=null;
        //this.radius=0;
        this.u = new _M2();
        // Orientation matrix from model to world
      }
      isCircular(){
        return false
      }
    }
    /**
     * @public
     * @class
     */
    class Circle extends Shape{
      constructor(r){
        super()
        this.radius = r;
      }
      isCircular(){
        return true
      }
      getAABB(){
        let r= this.body.position[0]+this.radius;
        let l= this.body.position[0]-this.radius;
        let t= this.body.position[1]+this.radius;
        let b= this.body.position[1]-this.radius;
        return new _2d.Rect(l,b,r-l,t-b);
      }
      clone(){
        return new Circle(this.radius);
      }
      initialize(){
        this.computeMass(1.0);
      }
      computeMass(density){
        let r2=this.radius*this.radius;
        this.body.m = Math.PI * r2 * density;
        this.body.I = this.body.m * r2;
        this.body.im = this.body.m ? 1.0/this.body.m : 0.0;
        this.body.iI = this.body.I ? 1.0/this.body.I : 0.0;
      }
      setOrient(radians){
        this.u.set(radians);
        return this;
      }
      draw(ctx){
        ctx.save();
        ctx.strokeStyle=this.body.rgb;
        _G.drawCircle(ctx,this.body.position[0], this.body.position[1],this.radius);
        let r=_M2.mul(this.u,_M.V2(1,0));
        r=_M.vecMul(r,this.radius);
        r = _M.vecAdd(r, this.body.position);
        ctx.strokeStyle="green";
        _G.drawLine(ctx,this.body.position[0],this.body.position[1],r[0],r[1]);
        ctx.restore();
      }
      getType(){ return IE.eCircle }
    }
    /**
     * @public
     * @class
     */
    class PolygonShape extends Shape{
      constructor(){
        super();
        this.m_vertices= new Array(MaxPolyVertexCount);
        this.m_normals= new Array(MaxPolyVertexCount);
        this.m_vertexCount=0;
      }
      getAABB(){
        let cps=this._calcPoints();
        let minX= Infinity;
        let minY=Infinity;
        let maxX= -Infinity;
        let maxY= -Infinity;
        for(let p,i=0;i<cps.length;++i){
          p=cps[i];
          if(p[0] < minX) minX=p[0];
          if(p[0] > maxX) maxX=p[0];
          if(p[1] < minY) minY=p[1];
          if(p[1] > maxY) maxY=p[1];
        }
        return new _2d.Rect(minX, minY,maxX-minX, maxY-minY);
      }
      initialize(){
        this.computeMass(1.0);
        return this;
      }
      clone(){
        let poly = new PolygonShape();
        poly.u= this.u.clone();
        for(let i = 0; i < this.m_vertexCount; ++i){
          poly.m_vertices[i]=_M.vecClone(this.m_vertices[i]);
          poly.m_normals[i]=_M.vecClone(this.m_normals[i]);
        }
        poly.m_vertexCount = this.m_vertexCount;
        return poly;
      }
      computeMass(density){
        // Calculate centroid and moment of interia
        let c= _M.V2(); // centroid
        let area = 0.0;
        let I = 0.0;
        const k_inv3 = 1.0/3.0;
        for(let i1 = 0; i1 < this.m_vertexCount; ++i1){
          // Triangle vertices, third vertex implied as (0, 0)
          let p1= this.m_vertices[i1];
          let i2 = (i1+1)%this.m_vertexCount;
          let p2= this.m_vertices[i2];
          let D = _M.vec2Cross(p1, p2);
          let triangleArea = 0.5 * D;
          area += triangleArea;
          // Use area to weight the centroid average, not just vertex position
          c = _M.vecAdd(c, _M.vecMul(_M.vecAdd(p1, p2),triangleArea *k_inv3))
          let intx2 = p1[0] * p1[0] + p2[0] * p1[0] + p2[0] * p2[0];
          let inty2 = p1[1] * p1[1] + p2[1] * p1[1] + p2[1] * p2[1];
          I += (0.25 * k_inv3 * D) * (intx2 + inty2);
        }

        c = _M.vecMul(c,1.0 / area);

        // Translate vertices to centroid (make the centroid (0, 0)
        // for the polygon in model space)
        // Not really necessary, but I like doing this anyway
        for(let i = 0; i < this.m_vertexCount; ++i)
          this.m_vertices[i] = _M.vecSub(this.m_vertices[i],c);

        this.body.m = density * area;
        this.body.im = this.body.m ? 1.0/this.body.m : 0.0;
        this.body.I = I * density;
        this.body.iI = this.body.I ? 1.0/this.body.I : 0.0;
        return this;
      }
      setOrient(radians){
        this.u.set(radians);
        return this;
      }
      _calcPoints(){
        let cps=[];
        for(let i=0;i<this.m_vertexCount;++i){
          //Vec2 v = body.position + u * m_vertices[i]
          cps.push(_M.vecAdd(this.body.position,_M2.mul(this.u,this.m_vertices[i])));
        }
        return cps;
      }
      draw(ctx){
        ctx.save();
        ctx.strokeStyle=this.body.rgb;
        _G.drawPoints(ctx,this._calcPoints());
        ctx.restore();
      }
      getType(){ return IE.ePoly }
      // Half width and half height
      setBox(hw,hh){
        this.m_vertexCount = 4;
        this.m_vertices[0]= _M.V2( -hw, -hh );
        this.m_vertices[1]= _M.V2(  hw, -hh );
        this.m_vertices[2]= _M.V2(  hw,  hh );
        this.m_vertices[3]= _M.V2( -hw,  hh );
        this.m_normals[0]= _M.V2( 0.0, -1.0);
        this.m_normals[1]= _M.V2(  1.0,   0.0);
        this.m_normals[2]= _M.V2(  0.0,   1.0);
        this.m_normals[3]= _M.V2( -1.0,   0.0);
        return this;
      }
      set(vertices){
        let count=vertices.length;
        // No hulls with less than 3 vertices (ensure actual polygon)
        _.assert(count > 2 && count <= MaxPolyVertexCount);
        //count = Math.min(count, MaxPolyVertexCount);
        // Find the right most point on the hull
        let rightMost = 0;
        let highestXCoord = vertices[0][0];
        for(let x,i = 1; i < count; ++i){
          x = vertices[i][0];
          if(x > highestXCoord){
            highestXCoord = x;
            rightMost = i;
          }
          // If matching x then take farthest negative y
          else if(_M.fuzzyEq(x, highestXCoord)){
            if(vertices[i][1] < vertices[rightMost][1]) rightMost = i;
          }
        }
        let hull=new Array(MaxPolyVertexCount);
        let outCount = 0;
        let indexHull = rightMost;
        for(;;){
          hull[outCount] = indexHull;
          // Search for next index that wraps around the hull
          // by computing cross products to find the most counter-clockwise
          // vertex in the set, given the previos hull index
          let nextHullIndex = 0;
          for(let i = 1; i < count; ++i){
            // Skip if same coordinate as we need three unique
            // points in the set to perform a cross product
            if(nextHullIndex === indexHull){
              nextHullIndex = i;
              continue;
            }
            // Cross every set of three unique vertices
            // Record each counter clockwise third vertex and add
            // to the output hull
            // See : http://www.oocities.org/pcgpe/math2d.html
            let e1 = _M.vecSub(vertices[nextHullIndex], vertices[hull[outCount]]);
            let e2 = _M.vecSub(vertices[i], vertices[hull[outCount]]);
            let c = _M.vec2Cross( e1, e2 );
            if(c < 0.0)
              nextHullIndex = i;
            // Cross product is zero then e vectors are on same line
            // therefor want to record vertex farthest along that line
            if(_M.fuzzyZero(c) && _M.vecLen2(e2) > _M.vecLen2(e1))
              nextHullIndex = i;
          }
          ++outCount;
          indexHull = nextHullIndex;
          // Conclude algorithm upon wrap-around
          if(nextHullIndex === rightMost){
            this.m_vertexCount = outCount;
            break;
          }
        }
        // Copy vertices into shape's vertices
        for(let i = 0; i < this.m_vertexCount; ++i)
          this.m_vertices[i]= vertices[hull[i]].slice();
        // Compute face normals
        for(let i1 = 0; i1 < this.m_vertexCount; ++i1){
          let i2 = (i1+1)%this.m_vertexCount;
          let face = _M.vecSub(this.m_vertices[i2], this.m_vertices[i1]);
          // Ensure no zero-length edges, because that's bad
          _.assert(_M.vecLen2(face) > _M.EPSILON * _M.EPSILON);
          // Calculate normal with 2D cross product between vector and scalar
          this.m_normals[i1]= _M.V2(face[1], -face[0]);
          this.m_normals[i1]= _M.vecUnit(this.m_normals[i1]);
        }
        return this;
      }
      // The extreme point along a direction within a polygon
      getSupport(dir){
        let bestProjection = -Infinity;
        let bestVertex;
        for(let i = 0; i < this.m_vertexCount; ++i){
          let v = this.m_vertices[i];
          let projection = _M.vecDot(v, dir);
          if(projection > bestProjection){
            bestVertex = v;
            bestProjection = projection;
          }
        }
        return bestVertex;
      }
    }

    IE.Polygon=PolygonShape;
    IE.Circle=Circle;
    return IE;
  };

})(this);

