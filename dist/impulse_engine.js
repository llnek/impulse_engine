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
    const _V= global["io.czlab.mcfud.vec2"]();
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
        return _V.V2(this.m00, this.m10)
      }
      axisY(){
        return _V.V2(this.m01, this.m11)
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
        return _V.V2(a.m00 * rhs[0] + a.m01 * rhs[1],
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
      gravity: _V.V2(0, 10 * _gravityScale)
    };
    global["io.czlab.impulse_engine.shape"](_XP,Core,_M);
    global["io.czlab.impulse_engine.body"](_XP,Core,_M);
    global["io.czlab.impulse_engine.manifold"](_XP,Core,_M);
    global["io.czlab.impulse_engine.collision"](_XP,Core,_M);
    global["io.czlab.impulse_engine.scene"](_XP,Core,_M,_G,_2d);
    return _XP;
  };


})(this);


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
    const _V=_M.Vec2;
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
        let r=_M2.mul(this.u,_V.V2(1,0));
        r=_V.vecMul(r,this.radius);
        r = _V.vecAdd(r, this.body.position);
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
        this.points=[];
        this.normals =[];
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
        for(let i = 0; i < this.points.length; ++i){
          poly.points[i]=_V.vecClone(this.points[i]);
          poly.normals[i]=_V.vecClone(this.normals[i]);
        }
        return poly;
      }
      computeMass(density){
        // Calculate centroid and moment of interia
        let c= _V.V2(); // centroid
        let area = 0.0;
        let I = 0.0;
        const k_inv3 = 1.0/3.0;
        const len=this.points.length;
        for(let i1 = 0; i1 < len; ++i1){
          // Triangle vertices, third vertex implied as (0, 0)
          let p1= this.points[i1];
          let i2 = (i1+1)%len;
          let p2= this.points[i2];
          let D = _V.vec2Cross(p1, p2);
          let triangleArea = 0.5 * D;
          area += triangleArea;
          // Use area to weight the centroid average, not just vertex position
          c = _V.vecAdd(c, _V.vecMul(_V.vecAdd(p1, p2),triangleArea *k_inv3))
          let intx2 = p1[0] * p1[0] + p2[0] * p1[0] + p2[0] * p2[0];
          let inty2 = p1[1] * p1[1] + p2[1] * p1[1] + p2[1] * p2[1];
          I += (0.25 * k_inv3 * D) * (intx2 + inty2);
        }

        c = _V.vecMul(c,1.0 / area);

        // Translate vertices to centroid (make the centroid (0, 0)
        // for the polygon in model space)
        // Not really necessary, but I like doing this anyway
        for(let i=0; i < len; ++i)
          _V.vecSubSelf(this.points[i],c);

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
        for(let i=0;i<this.points.length;++i)
          cps.push(_V.vecAdd(this.body.position,_M2.mul(this.u,this.points[i])));
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
        this.normals.length=0;
        this.points.length=0;
        this.points[0]= _V.V2( -hw, -hh );
        this.points[1]= _V.V2(  hw, -hh );
        this.points[2]= _V.V2(  hw,  hh );
        this.points[3]= _V.V2( -hw,  hh );
        this.normals[0]= _V.V2( 0.0, -1.0);
        this.normals[1]= _V.V2(  1.0,   0.0);
        this.normals[2]= _V.V2(  0.0,   1.0);
        this.normals[3]= _V.V2( -1.0,   0.0);
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
            let e1 = _V.vecSub(vertices[nextHullIndex], vertices[hull[outCount]]);
            let e2 = _V.vecSub(vertices[i], vertices[hull[outCount]]);
            let c = _V.vec2Cross( e1, e2 );
            if(c < 0.0)
              nextHullIndex = i;
            // Cross product is zero then e vectors are on same line
            // therefor want to record vertex farthest along that line
            if(_M.fuzzyZero(c) && _V.vecLen2(e2) > _V.vecLen2(e1))
              nextHullIndex = i;
          }
          ++outCount;
          indexHull = nextHullIndex;
          // Conclude algorithm upon wrap-around
          if(nextHullIndex === rightMost){
            break;
          }
        }
        this.normals.length=0;
        this.points.length=0;
        // Copy vertices into shape's vertices
        for(let i = 0; i < outCount; ++i)
          this.points[i]= vertices[hull[i]].slice();
        // Compute face normals
        for(let i1 = 0; i1 < outCount; ++i1){
          let i2 = (i1+1)%outCount;
          let face = _V.vecSub(this.points[i2], this.points[i1]);
          // Ensure no zero-length edges, because that's bad
          _.assert(_V.vecLen2(face) > _M.EPSILON * _M.EPSILON);
          // Calculate normal with 2D cross product between vector and scalar
          this.normals[i1]= _V.vecUnit(_V.V2(face[1], -face[0]));
        }
        return this;
      }
      // The extreme point along a direction within a polygon
      getSupport(dir){
        let bestProjection = -Infinity;
        let bestVertex;
        for(let p,v,i = 0; i < this.points.length; ++i){
          v = this.points[i];
          p= _V.vecDot(v, dir);
          if(p > bestProjection){
            bestVertex = v;
            bestProjection = p;
          }
        }
        return bestVertex;
      }
    }

    return _.inject(IE, { Circle: Circle, Polygon: PolygonShape })
  };

})(this);


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
    const _V=_M.Vec2;
    const _= Core.u;
    /**
     * @public
     * @class
     */
    class Body{
      constructor(shape_, x, y){
        this.shape= shape_;
        this.shape.body = this;
        this.position= _V.V2(x,y);
        this.velocity= _V.V2();
        this.angularVelocity = 0;
        this.torque = 0;
        this.rgb = "blue";
        this.force= _V.V2();
        this.staticFriction = 0.5;
        this.dynamicFriction = 0.3;
        this.restitution = 0.2;
        this.orient = _.randFloat(-Math.PI, Math.PI);
        if(this.shape.isCircular()){ this.rgb = "magenta" }
        //do this last
        this.shape.initialize();
      }
      applyForce(f){
        this.force= _V.vecAdd(this.force,f);
        return this;
      }
      applyImpulse(impulse, contactVector){
        _V.vecAddSelf(this.velocity, _V.vecMul(impulse,this.im));
        this.angularVelocity += this.iI * _V.vec2Cross(contactVector, impulse);
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

    return _.inject(IE, { Body:Body })
  };

})(this);


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
    const _V=_M.Vec2;
    const _= Core.u;
    /**
     * @public
     * @class
     */
    class Manifold{
      constructor(a, b){
        this.A=a;
        this.B=b;
        this.penetration=0; // Depth of penetration from collision
        this.normal=_V.V2(); // From A to B
        this.contacts=[_V.V2(),_V.V2()]; // Points of contact during collision
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
          let ra = _V.vecSub(this.contacts[i], this.A.position);
          let rb = _V.vecSub(this.contacts[i], this.B.position);
          let rv = _V.vecAdd(this.B.velocity,_V.vec2Cross(this.B.angularVelocity, rb));
          rv= _V.vecSub(_V.vecSub(rv,this.A.velocity),
                        _V.vec2Cross(this.A.angularVelocity, ra));
          // Determine if we should perform a resting collision or not
          // The idea is if the only thing moving this object is gravity,
          // then the collision should be performed without any restitution
          if(_V.vecLen2(rv) < _V.vecLen2(_V.vecMul(IE.gravity,IE.dt)) + _M.EPSILON)
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
          let ra = _V.vecSub(this.contacts[i], this.A.position);
          let rb = _V.vecSub(this.contacts[i], this.B.position);
          // Relative velocity
          let rv = _V.vecAdd(this.B.velocity,_V.vec2Cross(this.B.angularVelocity, rb));
          rv= _V.vecSub(_V.vecSub(rv, this.A.velocity),
                        _V.vec2Cross(this.A.angularVelocity, ra));
          // Relative velocity along the normal
          let contactVel = _V.vecDot(rv, this.normal);
          // Do not resolve if velocities are separating
          if(contactVel > 0)
            return this;
          let raCrossN = _V.vec2Cross(ra, this.normal);
          let rbCrossN = _V.vec2Cross(rb, this.normal);
          let invMassSum = this.A.im + this.B.im + (raCrossN*raCrossN) *
                           this.A.iI + (rbCrossN*rbCrossN) * this.B.iI;
          // Calculate impulse scalar
          let j = -(1.0 + this.e) * contactVel;
          j /= invMassSum;
          j /= this.contact_count;
          // Apply impulse
          let impulse = _V.vecMul(this.normal,j);
          this.A.applyImpulse(_V.vecFlip(impulse), ra);
          this.B.applyImpulse(impulse, rb);
          // Friction impulse
          rv = _V.vecAdd(this.B.velocity,_V.vec2Cross(this.B.angularVelocity, rb));
          rv = _V.vecSub(_V.vecSub(rv, this.A.velocity),
                         _V.vec2Cross(this.A.angularVelocity, ra));
          let t = _V.vecSub(rv,_V.vecMul(this.normal,_V.vecDot(rv, this.normal)));
          t = _V.vecUnit(t);
          // j tangent magnitude
          let jt = -_V.vecDot(rv, t);
          jt /= invMassSum;
          jt /= this.contact_count;
          // Don't apply tiny friction impulses
          if(_M.fuzzyZero(jt))
            return this;
          // Coulumb's law
          let tangentImpulse;
          if(Math.abs(jt) < j * this.sf)
            tangentImpulse = _V.vecMul(t, jt);
          else
            tangentImpulse = _V.vecMul(t, -j * this.df);
          // Apply friction impulse
          this.A.applyImpulse(_V.vecFlip(tangentImpulse), ra);
          this.B.applyImpulse(tangentImpulse, rb);
        }
        return this;
      }
      positionalCorrection(){
        const k_slop = 0.05; // Penetration allowance
        const percent = 0.4; // Penetration percentage to correct
        let correction =
          _V.vecMul(this.normal,
          (Math.max(this.penetration-k_slop, 0.0)/(this.A.im+this.B.im)) * percent);
        _V.vecSubSelf(this.A.position,_V.vecMul(correction, this.A.im));
        _V.vecAddSelf(this.B.position,_V.vecMul(correction, this.B.im));
        return this;
      }
      infiniteMassCorrection(){
        _V.vecCopy(this.A.velocity,0, 0);
        _V.vecCopy(this.B.velocity,0, 0);
        return this;
      }
    }

    return _.inject(IE, { Manifold: Manifold })
  };

})(this);


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
  global["io.czlab.impulse_engine.collision"]=function(IE,Core,_M){
    const _V=_M.Vec2;
    const _M2=IE.M2;
    const _=Core.u;
    const _C={};
    /**
     * @public
     * @function
     */
    _C.dispatch=function(t1,t2){
      if(t1===IE.eCircle){
        return t2===IE.eCircle ? this.circleCircle : this.circlePolygon;
      } else if(t1===IE.ePoly){
        return t2===IE.eCircle ? this.polygonCircle : this.polygonPolygon;
      }
    };
    /**
     * @public
     * @function
     */
    _C.circleCircle=function(m, a, b){
      let A = a.shape;
      let B = b.shape;
      // Calculate translational vector, which is normal
      let normal = _V.vecSub(b.position, a.position);
      let dist_sqr = _V.vecLen2(normal);
      let radius = A.radius + B.radius;
      // Not in contact
      if(dist_sqr >= radius*radius){
        m.contact_count = 0;
        return;
      }
      let distance = Math.sqrt(dist_sqr);
      m.contact_count = 1;
      if(_M.fuzzyZero(distance)){
        m.penetration = A.radius;
        m.normal = _V.V2(1, 0);
        _V.vecSet(m.contacts[0], a.position);
      }else{
        m.penetration = radius - distance;
        m.normal = _V.vecDiv(normal,distance);
        _V.vecSet(m.contacts[0],
                  _V.vecAdd(_V.vecMul(m.normal,A.radius),a.position));
      }
    };
    /**
     * @public
     * @function
     */
    _C.circlePolygon=function(m, a, b){
      let A = a.shape;
      let B = b.shape;
      m.contact_count = 0;
      // Transform circle center to Polygon model space
      let center = _M2.mul(B.u.transpose(), _V.vecSub(a.position,b.position));
      // Find edge with minimum penetration
      // Exact concept as using support points in Polygon vs Polygon
      let separation = -Infinity;
      let faceNormal = 0;
      for(let i=0; i < B.points.length; ++i){
        let s = _V.vecDot(B.normals[i], _V.vecSub(center,B.points[i]));
        if(s > A.radius)
          return;
        if(s > separation){
          separation = s;
          faceNormal = i;
        }
      }
      // Grab face's vertices
      let v1 = B.points[faceNormal];
      let i2 = (faceNormal+1) % B.points.length;
      let v2 = B.points[i2];
      // Check to see if center is within polygon
      if(separation < IE.EPSILON){
        m.contact_count = 1;
        m.normal = _V.vecFlip(_M2.mul(B.u,B.normals[faceNormal]));
        _V.vecSet(m.contacts[0],
                  _V.vecAdd(_V.vecMul(m.normal,A.radius),a.position));
        m.penetration = A.radius;
        return;
      }
      // Determine which voronoi region of the edge center of circle lies within
      let dot1 = _V.vecDot(_V.vecSub(center,v1), _V.vecSub(v2,v1));
      let dot2 = _V.vecDot(_V.vecSub(center,v2), _V.vecSub(v1,v2));
      m.penetration = A.radius - separation;
      // Closest to v1
      if(dot1 <= 0.0){
        if(_V.vecDist2(center, v1) > A.radius*A.radius)
          return;
        m.contact_count = 1;
        let n = _V.vecSub(v1,center);
        m.normal = _V.vecUnit(_M2.mul(B.u, n));
        _V.vecSet(m.contacts[0], _V.vecAdd(_M2.mul(B.u,v1),b.position));
      }
      // Closest to v2
      else if(dot2 <= 0.0){
        if(_V.vecDist2(center, v2) > A.radius*A.radius)
          return;
        m.contact_count = 1;
        let n = _V.vecSub(v2,center);
        m.normal = _V.vecUnit(_M2.mul(B.u, n));
        _V.vecSet(m.contacts[0], _V.vecAdd(_M2.mul(B.u,v2),b.position));
      }else{
        // Closest to face
        let n = B.normals[faceNormal];
        if(_V.vecDot(_V.vecSub(center,v1), n) > A.radius)
          return;
        m.normal = _V.vecFlip(_M2.mul(B.u, n));
        _V.vecSet(m.contacts[0],
                  _V.vecAdd(_V.vecMul(m.normal,A.radius),a.position));
        m.contact_count = 1;
      }
    };
    /**
     * @public
     * @function
     */
    IE.polygonCircle=function(m, a, b){
      this.circlePolygon(m, b, a);
      _V.vecFlipSelf(m.normal);
    };
    /**
     * @private
     * @function
     */
    function _findAxisLeastPenetration(A, B){
      let bestDistance = -Infinity;
      let bestIndex;
      for(let i=0; i < A.points.length; ++i){
        // Retrieve a face normal from A
        let n = A.normals[i];
        let v = A.points[i];
        let nw = _M2.mul(A.u, n);
        // Transform face normal into B's model space
        let buT = B.u.transpose();
        n = _M2.mul(buT,nw);
        // Retrieve support point from B along -n
        let s = B.getSupport(_V.vecFlip(n));
        // Retrieve vertex on face from A, transform into
        // B's model space
        v = _V.vecAdd(_M2.mul(A.u,v),A.body.position);
        _V.vecSubSelf(v,B.body.position);
        v = _M2.mul(buT,v);
        // Compute penetration distance (in B's model space)
        let d = _V.vecDot(n, _V.vecSub(s,v ));
        // Store greatest distance
        if(d > bestDistance){
          bestDistance = d;
          bestIndex = i;
        }
      }
      return [bestDistance, bestIndex]
    }
    /**
     * @private
     * @function
     */
    function _findIncidentFace(RefPoly, IncPoly, refIndex){
      let refNormal = RefPoly.normals[refIndex];
      // Calculate normal in incident's frame of reference
      refNormal = _M2.mul(RefPoly.u, refNormal); // To world space
      refNormal = _M2.mul(IncPoly.u.transpose(),refNormal); // To incident's model space
      // Find most anti-normal face on incident polygon
      let incidentFace = 0;
      let minDot = Infinity;
      for(let dot,i = 0; i < IncPoly.points.length; ++i){
        dot = _V.vecDot(refNormal, IncPoly.normals[i]);
        if(dot < minDot){
          minDot = dot;
          incidentFace = i;
        }
      }
      // Assign face vertices for incidentFace
      let v0= _V.vecAdd(_M2.mul(IncPoly.u,IncPoly.points[incidentFace]), IncPoly.body.position);
      incidentFace = (incidentFace+1) % IncPoly.points.length;
      let v1 = _V.vecAdd(_M2.mul(IncPoly.u,IncPoly.points[incidentFace]), IncPoly.body.position);
      return [v0,v1]
    }
    /**
     * @private
     * @function
     */
    function _clip(n, c, face){
      let out=[face[0],face[1]];
      let sp = 0;
      // Retrieve distances from each endpoint to the line
      // d = ax + by - c
      let d1 = _V.vecDot(n, face[0]) - c;
      let d2 = _V.vecDot(n, face[1]) - c;
      // If negative (behind plane) clip
      if(d1 <= 0.0) out[sp++] = face[0];
      if(d2 <= 0.0) out[sp++] = face[1];
      // If the points are on different sides of the plane
      if(d1*d2 < 0.0){ // less than to ignore -0.0f
        // Push interesection point
        let alpha = d1/(d1-d2);
        out[sp] = _V.vecAdd(face[0],_V.vecMul(_V.vecSub(face[1],face[0]),alpha));
        ++sp;
      }
      // Assign our new converted values
      face[0] = out[0];
      face[1] = out[1];
      _.assert(sp != 3);
      return sp;
    }
    /**
     * @public
     * @function
     */
    IE.polygonPolygon=function(m, a, b){
      let A = a.shape;
      let B = b.shape;
      m.contact_count = 0;
      // Check for a separating axis with A's face planes
      let [penetrationA,faceA] = _findAxisLeastPenetration(A, B);
      if(penetrationA >= 0.0)
        return;
      // Check for a separating axis with B's face planes
      let [penetrationB,faceB] = _findAxisLeastPenetration(B, A);
      if(penetrationB >= 0.0)
        return;

      let RefPoly; // Reference
      let IncPoly; // Incident
      let refIndex;
      let flip; // Always point from a to b
      // Determine which shape contains reference face
      if(_M.biasGreater(penetrationA, penetrationB)){
        RefPoly = A;
        IncPoly = B;
        refIndex = faceA;
        flip = false;
      }else{
        RefPoly = B;
        IncPoly = A;
        refIndex = faceB;
        flip = true;
      }
      // World space incident face
      let incidentFace= _findIncidentFace(RefPoly, IncPoly, refIndex);
      //        y
      //        ^  ->n       ^
      //      +---c ------posPlane--
      //  x < | i |\
      //      +---+ c-----negPlane--
      //             \       v
      //              r
      //
      //  r : reference face
      //  i : incident poly
      //  c : clipped point
      //  n : incident normal
      // Setup reference face vertices
      let v1 = RefPoly.points[refIndex];
      refIndex = (refIndex+1) % RefPoly.points.length;
      let v2 = RefPoly.points[refIndex];
      // Transform vertices to world space
      v1 = _V.vecAdd(_M2.mul(RefPoly.u,v1),RefPoly.body.position);
      v2 = _V.vecAdd(_M2.mul(RefPoly.u,v2),RefPoly.body.position);
      // Calculate reference face side normal in world space
      let sidePlaneNormal = _V.vecUnit(_V.vecSub(v2,v1));
      // Orthogonalize
      let refFaceNormal= _V.V2(sidePlaneNormal[1], -sidePlaneNormal[0]);
      // ax + by = c
      // c is distance from origin
      let refC = _V.vecDot( refFaceNormal, v1 );
      let negSide = -_V.vecDot( sidePlaneNormal, v1 );
      let posSide =  _V.vecDot( sidePlaneNormal, v2 );
      // Clip incident face to reference face side planes
      if(_clip(_V.vecFlip(sidePlaneNormal), negSide, incidentFace) < 2)
        return; // Due to floating point error, possible to not have required points
      if(_clip(sidePlaneNormal, posSide, incidentFace ) < 2)
        return; // Due to floating point error, possible to not have required points
      // Flip
      m.normal = flip ? _V.vecFlip(refFaceNormal) : refFaceNormal;
      // Keep points behind reference face
      let cp = 0; // clipped points behind reference face
      let separation = _V.vecDot(refFaceNormal, incidentFace[0]) - refC;
      if(separation <= 0.0){
        m.contacts[cp] = incidentFace[0];
        m.penetration = -separation;
        ++cp;
      }else{
        m.penetration = 0;
      }
      separation = _V.vecDot(refFaceNormal, incidentFace[1]) - refC;
      if(separation <= 0.0){
        m.contacts[cp] = incidentFace[1];
        m.penetration += -separation;
        ++cp;
        // Average penetration
        m.penetration /= cp;
      }
      m.contact_count = cp;
    };

    return _.inject(IE, _C)
  };

})(this);


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
    const _V=_M.Vec2;
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
      _V.vecAddSelf(b.velocity,
                    _V.vecMul(_V.vecAdd(_V.vecMul(b.force,b.im),IE.gravity),dt2));
      b.angularVelocity += b.torque * b.iI * dt2;
    }
    /**
     * @private
     * @function
     */
    function _integrateVelocity(b, dt){
      if(_M.fuzzyZero(b.im))
        return;
      _V.vecAddSelf(b.position,_V.vecMul(b.velocity,dt));
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
          _V.vecCopy(b.force, 0,0);
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

