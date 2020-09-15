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
  //export--------------------------------------------------------------------
  if(typeof module === "object" &&
     module && typeof module.exports === "object"){
    global=module.exports;
  }
  else if(typeof exports === "object" && exports){
    global=exports;
  }
  let _singleton=null;
  global["io.czlab.mcfud.gfx"]=function(){
    if(_singleton){ return _singleton }
    const Core=global["io.czlab.mcfud.core"]();
    const _M=global["io.czlab.mcfud.math"]();
    const _=Core.u;
    const _G={};
    const TWO_PI=Math.PI*2;
    /**
     * Html5 Text Style object.
     * @public
     * @function
     */
    _G.textStyle=function(font,fill,align,base){
      //"14px 'Arial'" "#dddddd" "left" "top"
      return {font: font, fill: fill, align: align, base: base}
    };
    /**
     * Draw the shape onto the html5 canvas.
     * @public
     * @function
     */
    _G.drawShape=function(ctx,s,...args){
      if(s.draw)
        s.draw(ctx,...args)
    };
    /**
     * Apply styles to the canvas.
     * @public
     * @function
     */
    _G.cfgStyle=function(ctx,styleObj){
      if(styleObj){
        let line=styleObj.line;
        let stroke=styleObj.stroke;
        if(line){
          if(line.cap)
            ctx.lineCap=line.cap;
          if(line.width)
            ctx.lineWidth=line.width;
        }
        if(stroke){
          if(stroke.style)
            ctx.strokeStyle=stroke.style;
        }
      }
    };
    /**
     * Draw and connect this set of points onto the canvas.
     * @public
     * @function
     */
    _G.drawPoints=function(ctx,points,size){
      if(size === undefined) size=points.length;
      ctx.beginPath();
      for(let i=0;i<size;++i){
        let i2= (i+1)%size;
        let p=points[i];
        let q=points[i2];
        ctx.moveTo(p[0],p[1]);
        ctx.lineTo(q[0],q[1]);
      }
      ctx.stroke();
    };
    /**
     * @public
     * @function
     */
    _G.drawShapePoly=function(ctx,poly){
      return this.drawPoints(ctx,poly.points);
    };
    /**
     * Draw a circle onto the canvas.  If a starting point
     * is provided, draw a line to the center.
     * @public
     * @function
     */
    _G.drawCircle=function(ctx,x,y,radius){
      ctx.beginPath();
      ctx.arc(x,y,radius,0,TWO_PI,true);
      ctx.closePath();
      ctx.stroke();
    };
    /**
     * @public
     * @function
     */
    _G.drawShapeCircle=function(ctx,circle){
      this.drawCircle(ctx,circle.pos[0],circle.pos[1],circle.radius);
    };
    /**
     * @public
     * @function
     */
    _G.drawRect=function(ctx,x,y,width,height,rot){
      let left=x;
      let top= y - height;
      ctx.save();
      ctx.translate(left,top);
      ctx.rotate(rot);
      ctx.strokeRect(0,0,width,height);
      ctx.restore();
    };
    /**
     * @public
     * @function
     */
    _G.drawShapeRect=function(ctx,rect){
      return this.drawRet(ctx,rect.pos[0],rect.pos[1],
                          rect.width,rect.height,rect.rotation);
    };
    /**
     * @public
     * @function
     */
    _G.drawLine=function(ctx,x1,y1,x2,y2){
      ctx.beginPath();
      ctx.moveTo(x1,y1);
      ctx.lineTo(x2,y2);
      ctx.stroke();
    };
    /**
     * @public
     * @function
     */
    _G.drawShapeLine=function(ctx,line){
      return this.drawLine(ctx,line.p[0],line.p[1],line.q[0],line.q[1]);
    };

    return (_singleton=_G);
  };

})(this);

