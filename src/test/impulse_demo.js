
function userControl(e){

}

let XXXX={
  root: null,
  n:1
};
function mouseControl(evt){
  let btn= (0=== XXXX.n%2);
  let _=XXXX.Core.u;
  ++XXXX.n;
  if(btn){
    let e= _.randInt2(50, 100);
    let z= _.randInt2(3,12);
    let vs=[];
    let p= new XXXX.IE.Polygon();
    for(let i=0;i<z;++i)
      vs.push([_.randInt2(-e, e),_.randInt2(-e,e)]);
    p.set(vs);
    let b=XXXX.s.add(p, evt.clientX, evt.clientY);
    b.staticFriction = 0.4;
    b.dynamicFriction = 0.2;
    b.restitution = 0.2;
    b.setOrient(_.randFloat(-Math.PI,Math.PI));
  }else{
    let c=new XXXX.IE.Circle(4+_.randInt(20));
    XXXX.s.add(c, evt.clientX, evt.clientY);
  }
}

function _draw(s,ctx){
  ctx.clearRect(0,0,s.width,s.height);
  s.render(ctx);
}

function _run(s,canvas){
  let ctx=canvas.getContext("2d");
  ctx.strokeStyle="#ffffff";
  s.height=canvas.height;
  s.width=canvas.width;
  function loop(){
    requestAnimationFrame(loop);
    _draw(s,ctx);
    s.step();
    for(let b,i=0;i<s.bodies.length;++i){
      b=s.bodies[i];
      if(!XXXX._2d.containsPoint(XXXX.IE.gWorld,b.position[0],b.position[1]))
        if(!XXXX._2d.rectContainsRect(XXXX.IE.gWorld,b.shape.getAABB())){
          s.bodies.splice(i,1);
          --i;
        }
    }
    if(s.bodies.length===1 && !XXXX.root){
      let b= s.bodies[0];
      XXXX.rootOrient=b.orient;
      XXXX.root=b;
      XXXX.root.setOrient(0);
    }
    if(s.bodies.length>1 && XXXX.root){
      XXXX.root.setOrient(XXXX.rootOrient);
      XXXX.root=null;
    }
  }
  loop();
}

function PhysicsGame(){
  let IE=window["io.czlab.impulse_engine.core"]();
  let canvas=document.getElementById("canvas");
  let w=800,h=640;
  let s= new IE.Scene(1/60.0,10);
  XXXX.Core=window["io.czlab.mcfud.core"]();
  XXXX._2d=window["io.czlab.mcfud.geo2d"]();
  XXXX.IE=IE;
  XXXX.s=s;
  canvas.width=w;
  canvas.height=h;
  IE.gWorld= new XXXX._2d.Rect(0,0,w,h);
  let c=s.add(new IE.Circle(20),400,40);
  let p=s.add(new IE.Polygon().setBox(300,20),400,320);
  p.setOrient(-2.8).setStatic();
  _run(s,canvas);
}


