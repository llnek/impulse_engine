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

    const {Mat2}=IE;
    const {u:_}=Core;

    const _$={
      dispatch(t1,t2){
        if(t1===IE.eCircle){
          return t2===IE.eCircle ? this.circleCircle : this.circlePolygon
        }else if(t1===IE.ePoly){
          return t2===IE.eCircle ? this.polygonCircle : this.polygonPolygon
        }
      },
      circleCircle(m, a, b){
        let A = a.shape;
        let B = b.shape;
        // calculate translational vector, which is normal
        let normal = _V.sub(b.position, a.position);
        let dist_sqr = _V.len2(normal);
        let radius = A.radius + B.radius;
        // Not in contact
        if(dist_sqr >= radius*radius){
          m.contact_count = 0;
          return;
        }
        let distance = Math.sqrt(dist_sqr);
        m.contact_count = 1;
        if(_.feq0(distance)){
          m.penetration = A.radius;
          m.normal = _V.vec(1, 0);
          _V.copy(m.contacts[0], a.position);
        }else{
          m.penetration = radius - distance;
          m.normal = _V.div(normal,distance);
          _V.copy(m.contacts[0],
                  _V.add(_V.mul(m.normal,A.radius),a.position));
        }
      },
      circlePolygon(m, a, b){
        let A = a.shape;
        let B = b.shape;
        m.contact_count = 0;
        // Transform circle center to Polygon model space
        let center = Mat2.mul(B.u.transpose(), _V.sub(a.position,b.position));
        // Find edge with minimum penetration
        // Exact concept as using support points in Polygon vs Polygon
        let separation = -Infinity;
        let faceNormal = 0;
        for(let i=0; i < B.points.length; ++i){
          let s = _V.dot(B.normals[i], _V.sub(center,B.points[i]));
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
          m.normal = _V.flip(Mat2.mul(B.u,B.normals[faceNormal]));
          _V.copy(m.contacts[0],
                  _V.add(_V.mul(m.normal,A.radius),a.position));
          m.penetration = A.radius;
          return;
        }
        // Determine which voronoi region of the edge center of circle lies within
        let dot1 = _V.dot(_V.sub(center,v1), _V.sub(v2,v1));
        let dot2 = _V.dot(_V.sub(center,v2), _V.sub(v1,v2));
        m.penetration = A.radius - separation;
        // Closest to v1
        if(dot1 <= 0.0){
          if(_V.dist2(center, v1) > A.radius*A.radius)
            return;
          m.contact_count = 1;
          let n = _V.sub(v1,center);
          m.normal = _V.unit(Mat2.mul(B.u, n));
          _V.copy(m.contacts[0], _V.add(Mat2.mul(B.u,v1),b.position));
        }
        // Closest to v2
        else if(dot2 <= 0.0){
          if(_V.dist2(center, v2) > A.radius*A.radius)
            return;
          m.contact_count = 1;
          let n = _V.sub(v2,center);
          m.normal = _V.unit(Mat2.mul(B.u, n));
          _V.copy(m.contacts[0], _V.add(Mat2.mul(B.u,v2),b.position));
        }else{
          // Closest to face
          let n = B.normals[faceNormal];
          if(_V.dot(_V.sub(center,v1), n) > A.radius)
            return;
          m.normal = _V.flip(Mat2.mul(B.u, n));
          _V.copy(m.contacts[0],
                  _V.add(_V.mul(m.normal,A.radius),a.position));
          m.contact_count = 1;
        }
      }
    };

    /**
     * @public
     * @function
     */
    IE.polygonCircle=function(m, a, b){
      this.circlePolygon(m, b, a);
      _V.flip$(m.normal);
    };

    /** @ignore */
    function _findAxisLeastPenetration(A, B){
      let bestDistance = -Infinity;
      let bestIndex;
      for(let i=0; i < A.points.length; ++i){
        // Retrieve a face normal from A
        let n = A.normals[i];
        let v = A.points[i];
        let nw = Mat2.mul(A.u, n);
        // Transform face normal into B's model space
        let buT = B.u.transpose();
        n = Mat2.mul(buT,nw);
        // Retrieve support point from B along -n
        let s = B.getSupport(_V.flip(n));
        // Retrieve vertex on face from A, transform into
        // B's model space
        v = _V.add(Mat2.mul(A.u,v),A.body.position);
        _V.sub$(v,B.body.position);
        v = Mat2.mul(buT,v);
        // Compute penetration distance (in B's model space)
        let d = _V.dot(n, _V.sub(s,v ));
        // Store greatest distance
        if(d > bestDistance){
          bestDistance = d;
          bestIndex = i;
        }
      }
      return [bestDistance, bestIndex]
    }

    /** @ignore */
    function _findIncidentFace(RefPoly, IncPoly, refIndex){
      let refNormal = RefPoly.normals[refIndex];
      // Calculate normal in incident's frame of reference
      refNormal = Mat2.mul(RefPoly.u, refNormal); // To world space
      refNormal = Mat2.mul(IncPoly.u.transpose(),refNormal); // To incident's model space
      // Find most anti-normal face on incident polygon
      let incidentFace = 0;
      let minDot = Infinity;
      for(let dot,i = 0; i < IncPoly.points.length; ++i){
        dot = _V.dot(refNormal, IncPoly.normals[i]);
        if(dot < minDot){
          minDot = dot;
          incidentFace = i;
        }
      }
      // Assign face vertices for incidentFace
      let v0= _V.add(Mat2.mul(IncPoly.u,IncPoly.points[incidentFace]), IncPoly.body.position);
      incidentFace = (incidentFace+1) % IncPoly.points.length;
      let v1 = _V.add(Mat2.mul(IncPoly.u,IncPoly.points[incidentFace]), IncPoly.body.position);
      return [v0,v1]
    }

    /** @ignore */
    function _clip(n, c, face){
      let out=[face[0],face[1]];
      let sp = 0;
      // Retrieve distances from each endpoint to the line
      // d = ax + by - c
      let d1 = _V.dot(n, face[0]) - c;
      let d2 = _V.dot(n, face[1]) - c;
      // If negative (behind plane) clip
      if(d1 <= 0.0) out[sp++] = face[0];
      if(d2 <= 0.0) out[sp++] = face[1];
      // If the points are on different sides of the plane
      if(d1*d2 < 0.0){ // less than to ignore -0.0f
        // Push interesection point
        let alpha = d1/(d1-d2);
        out[sp] = _V.add(face[0],_V.mul(_V.sub(face[1],face[0]),alpha));
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
      if(penetrationA >= 0)
        return;
      // Check for a separating axis with B's face planes
      let [penetrationB,faceB] = _findAxisLeastPenetration(B, A);
      if(penetrationB >= 0)
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
      v1 = _V.add(Mat2.mul(RefPoly.u,v1),RefPoly.body.position);
      v2 = _V.add(Mat2.mul(RefPoly.u,v2),RefPoly.body.position);
      // Calculate reference face side normal in world space
      let sidePlaneNormal = _V.unit(_V.sub(v2,v1));
      // Orthogonalize
      let refFaceNormal= _V.vec(sidePlaneNormal[1], -sidePlaneNormal[0]);
      // ax + by = c
      // c is distance from origin
      let refC = _V.dot( refFaceNormal, v1 );
      let negSide = -_V.dot( sidePlaneNormal, v1 );
      let posSide =  _V.dot( sidePlaneNormal, v2 );
      // Clip incident face to reference face side planes
      if(_clip(_V.flip(sidePlaneNormal), negSide, incidentFace) < 2)
        return; // Due to floating point error, possible to not have required points
      if(_clip(sidePlaneNormal, posSide, incidentFace ) < 2)
        return; // Due to floating point error, possible to not have required points
      // Flip
      m.normal = flip ? _V.flip(refFaceNormal) : refFaceNormal;
      // Keep points behind reference face
      let cp = 0; // clipped points behind reference face
      let separation = _V.dot(refFaceNormal, incidentFace[0]) - refC;
      if(separation <= 0){
        m.contacts[cp] = incidentFace[0];
        m.penetration = -separation;
        ++cp;
      }else{
        m.penetration = 0;
      }
      separation = _V.dot(refFaceNormal, incidentFace[1]) - refC;
      if(separation <= 0){
        m.contacts[cp] = incidentFace[1];
        m.penetration += -separation;
        ++cp;
        // Average penetration
        m.penetration /= cp;
      }
      m.contact_count = cp;
    };

    return _.inject(IE, _$)
  }

  //;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  //exports
  if(typeof module=="object" && module.exports){
    throw "Panic: browser only"
  }else{
    gscope["io/czlab/impulse_engine/collision"]=function(IE,Core,_M,_V){
      return _module(IE,Core,_M,_V)
    }
  }

})(this);

