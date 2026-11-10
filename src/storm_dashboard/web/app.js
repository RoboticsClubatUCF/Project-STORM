"use strict";
const $ = id => document.getElementById(id);
let state = null, site = null, terrain = new Image(), online = false, addMode = true;
let scale = 1, center = [0, 0], width = 0, height = 0, drag = null, toastTimer;
const keys = new Set();
const canvas = $("map"), ctx = canvas.getContext("2d");

function toast(message) { $("toast").textContent = message; $("toast").hidden = false; clearTimeout(toastTimer); toastTimer = setTimeout(() => $("toast").hidden = true, 5500); }
async function command(name, args = {}, quiet = false) {
  try {
    const response = await fetch("/api/commands", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify({command:name,args}), keepalive:name.startsWith("teleop.")});
    const data = await response.json();
    if (!response.ok) throw new Error(data.error || "Command failed");
    if (data.result?.telemetry) {state = data.result; render();}
    return data.result;
  } catch (error) {if (!quiet) toast(error.message); return null;}
}
function live(key) {return state?.telemetry[key]?.fresh ? state.telemetry[key] : null;}
function running(key) {return Boolean(state?.processes[key]?.running);}
function point(x,y) {return [(x-center[0])*scale+width/2, height/2-(y-center[1])*scale];}
function world(x,y) {return [(x-width/2)/scale+center[0], center[1]-(y-height/2)/scale];}
function fit() {if (!site) return; const [w,s,e,n] = site.bounds; center=[(w+e)/2,(s+n)/2]; scale=Math.min(width/(e-w),height/(n-s))*.93; draw();}
function resize() {const rect=canvas.getBoundingClientRect(); width=rect.width;height=rect.height; canvas.width=Math.round(width*devicePixelRatio);canvas.height=Math.round(height*devicePixelRatio); if(site && !canvas.dataset.fit){fit();canvas.dataset.fit="true";}draw();}
function zoom(factor, x=width/2,y=height/2) {const before=world(x,y);scale=Math.max(.05,Math.min(55,scale*factor));const after=world(x,y);center[0]+=before[0]-after[0];center[1]+=before[1]-after[1];draw();}
function path(points,color,lineWidth=2,dash=[]) {if(!points?.length)return;ctx.beginPath();for(let i=0;i<points.length;i++){const [x,y]=point(...points[i]);if(i)ctx.lineTo(x,y);else ctx.moveTo(x,y);}ctx.strokeStyle=color;ctx.lineWidth=lineWidth;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);}
function draw() {
  ctx.setTransform(devicePixelRatio,0,0,devicePixelRatio,0,0);ctx.clearRect(0,0,width,height);ctx.fillStyle="#eeece2";ctx.fillRect(0,0,width,height);
  if(!site)return;
  const [w,s,e,n]=site.bounds;const [x,y]=point(w,n);ctx.imageSmoothingEnabled=true;if(terrain.complete && terrain.naturalWidth)ctx.drawImage(terrain,x,y,(e-w)*scale,(n-s)*scale);
  ctx.fillStyle="#f6f3df22";ctx.fillRect(x,y,(e-w)*scale,(n-s)*scale);
  const raw=100/scale, exponent=10**Math.floor(Math.log10(raw));const grid=[1,2,5,10].map(v=>v*exponent).find(v=>v>=raw)||10*exponent;
  const [left,top]=world(0,0),[right,bottom]=world(width,height);ctx.strokeStyle="#ffffff22";ctx.lineWidth=1;ctx.beginPath();
  for(let gx=Math.ceil(left/grid)*grid;gx<right;gx+=grid){const [px]=point(gx,0);ctx.moveTo(px,0);ctx.lineTo(px,height);}
  for(let gy=Math.ceil(bottom/grid)*grid;gy<top;gy+=grid){const [,py]=point(0,gy);ctx.moveTo(0,py);ctx.lineTo(width,py);}ctx.stroke();
  const [ox,oy]=point(0,0);ctx.strokeStyle="#4e624b66";ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(ox-12,oy);ctx.lineTo(ox+12,oy);ctx.moveTo(ox,oy-12);ctx.lineTo(ox,oy+12);ctx.stroke();ctx.setLineDash([]);ctx.font="9px Consolas,monospace";ctx.fillStyle="#445b3b";ctx.fillText("C2 / DATUM",ox+15,oy+4);
  const coverage=live("truth");if(coverage){const [cx,cy]=point(coverage.x,coverage.y);ctx.beginPath();ctx.arc(cx,cy,(site.sensor_limits?.vlp16.maximum_range_m||100)*scale,0,Math.PI*2);ctx.setLineDash([4,4]);ctx.strokeStyle="#376a4599";ctx.lineWidth=1;ctx.stroke();ctx.setLineDash([]);}path(state?.trail,"#496f704a",2);
  if(state?.plan){let previous=state.plan.start_xy;for(const stop of state.plan.waypoints){const legs=stop.legs_xy_yaw.map(v=>v.slice(0,2));path([previous,...legs],"#fff9e3",4);path([previous,...legs],"#c9683f",2);previous=stop.goal_xy;}}
  else if(state?.waypoints?.length)path([[0,0],...state.waypoints.map(v=>[v.x,v.y])],"#a1684788",1.5,[4,5]);
  (state?.waypoints||[]).forEach((wp,index)=>{const [px,py]=point(wp.x,wp.y);ctx.beginPath();ctx.arc(px,py,12,0,Math.PI*2);ctx.fillStyle="#fffdf4";ctx.fill();ctx.strokeStyle="#8c653d";ctx.lineWidth=1.7;ctx.stroke();ctx.fillStyle="#755331";ctx.font="bold 10px Consolas,monospace";ctx.textAlign="center";ctx.textBaseline="middle";ctx.fillText(String(index+1).padStart(2,"0"),px,py);ctx.textAlign="left";ctx.textBaseline="alphabetic";});
  const estimate=live("estimate");if(estimate){const [px,py]=point(estimate.x,estimate.y);ctx.strokeStyle="#8a70ad";ctx.lineWidth=2;ctx.strokeRect(px-7,py-7,14,14);}
  const truth=live("truth");if(truth){const [px,py]=point(truth.x,truth.y);ctx.beginPath();ctx.arc(px,py,23,0,Math.PI*2);ctx.fillStyle="#285b4822";ctx.fill();ctx.save();ctx.translate(px,py);ctx.rotate(-truth.yaw);ctx.beginPath();ctx.moveTo(13,0);ctx.lineTo(-9,-8);ctx.lineTo(-5,0);ctx.lineTo(-9,8);ctx.closePath();ctx.fillStyle="#285b41";ctx.fill();ctx.strokeStyle="#f9ffef";ctx.lineWidth=2;ctx.stroke();ctx.restore();ctx.fillStyle="#345536";ctx.font="bold 9px Consolas,monospace";ctx.fillText("STORM",px+18,py-12);}
  const span=80/scale;$("scale").textContent=span>=1000?`${(span/1000).toFixed(1)} km`:`${span<10?span.toFixed(1):Math.round(span)} m`;
}
function renderWaypoints() {
  const list=$("waypoints");list.replaceChildren();
  if(!state.waypoints.length){const li=document.createElement("li");li.className="empty";li.append("Your next route starts here.");const hint=document.createElement("span");hint.textContent="Add a waypoint on the site map.";li.append(hint);list.append(li);}
  state.waypoints.forEach((wp,index)=>{const li=document.createElement("li"),number=document.createElement("span"),copy=document.createElement("div"),title=document.createElement("strong"),coordinates=document.createElement("span"),remove=document.createElement("button");number.className="number";number.textContent=String(index+1).padStart(2,"0");copy.className="point-copy";title.textContent=wp.id;coordinates.textContent=`${wp.latitude.toFixed(6)}, ${wp.longitude.toFixed(6)}`;copy.append(title,coordinates);remove.className="remove";remove.textContent="×";remove.setAttribute("aria-label",`Remove ${wp.id}`);remove.disabled=state.busy||running("mission");remove.onclick=()=>command("plan.set",{waypoints:state.waypoints.filter((_,i)=>i!==index).map(v=>({id:v.id,latitude:v.latitude,longitude:v.longitude}))});li.append(number,copy,remove);list.append(li);});
}
let waypointsKey="";
function render() {
  if(!state)return;
  const truth=live("truth"),estimate=live("estimate"),busy=state.busy||running("mission"),sim=running("simulation"),teleop=state.teleop.enabled;
  $("connection").textContent=truth?"Simulator live":"Console online";$("connection-dot").classList.add("online");
  $("clock").textContent=`SIM TIME ${live("clock")?live("clock").seconds.toFixed(1)+" s":"—"}`;
  $("count").textContent=String(state.waypoints.length).padStart(2,"0");const key=JSON.stringify([state.waypoints,busy]);if(key!==waypointsKey){renderWaypoints();waypointsKey=key;}
  $("distance").textContent=state.plan?`${state.plan.route_length_m.toFixed(1)} m`:"— m";$("legs").textContent=state.plan?.leg_count??"—";
  $("true-position").textContent=truth?`${truth.x.toFixed(2)} / ${truth.y.toFixed(2)} m`:"— / — m";
  $("pose-error").textContent=truth&&estimate?`${Math.hypot(truth.x-estimate.x,truth.y-estimate.y).toFixed(3)} m`:"— m";
  $("gnss-status").textContent=live("gnss")?(live("gnss").valid?"Valid fix":"Invalid fix"):"Awaiting fix";$("nav-status").textContent=live("nav2")?"Ready":sim?"Starting / blocked":"Offline";
  $("nav-status").title=(state.navigation_blockers||[]).join("; ");
  $("sim-status").textContent=truth?"LIVE":sim?"STARTING":"OFFLINE";
  $("launch").disabled=sim||busy;$("close-sim").disabled=!sim;$("preview").disabled=!state.waypoints.length||busy;$("execute").disabled=!state.waypoints.length||!truth||!live("nav2")||!live("collision")||!live("objects")||(state.navigation_blockers||[]).length>0||busy||teleop;$("clear-plan").disabled=busy||!state.waypoints.length;$("add-mode").disabled=busy;
  $("import-plan").disabled=busy;$("cancel-mission").disabled=!running("mission");$("run-test").disabled=sim||busy;$("cancel-test").disabled=!state.busy;
  $("teleop-toggle").textContent=teleop?"Disable":"Enable";$("teleop-toggle").classList.toggle("active",teleop);$("teleop-toggle").disabled=!truth||!live("collision")||!live("objects")||busy;
  document.querySelectorAll("[data-drive]").forEach(button=>button.disabled=!teleop||busy);
  $("teleop-note").textContent=teleop?"Hold to drive · 0.30 m/s max · release or leave this window to stop.":"Manual control is disabled. Input expires automatically after 350 ms.";
  $("drive-speed").textContent=`${state.teleop.velocity[0].toFixed(2)} m/s`;$("drive-turn").textContent=`${state.teleop.velocity[1].toFixed(2)} rad/s`;
  const completed=state.report?.waypoints.filter(w=>w.status==="succeeded").length||0,current=state.report?.waypoints.find(w=>w.status==="running");
  $("mission-status").textContent=running("mission")?`Mission: ${completed}/${state.report?.targets?.length??state.waypoints.length} stops reached${current?" · approaching "+current.id:""}.`:sim&&(state.navigation_blockers||[]).length?`Navigation blocked: ${state.navigation_blockers.join("; ")}`:state.report?`Last mission: ${state.report.status}${state.report.reason?" · "+state.report.reason:""}`:"Plan offline or launch the simulator to drive.";
  const chips=$("sensor-chips");chips.replaceChildren();for(const [key,label]of[["truth","TRUTH"],["gnss","GNSS"],["scan","SCAN"],["cloud","VLP-16"],["rgb","RGB"],["depth","DEPTH"],["imu","IMU"],["rviz","RVIZ"],["objects","OBJECTS"],["collision","COLLISION"]]){const chip=document.createElement("span");chip.className="chip"+(live(key)?" live":"");chip.textContent=label;chips.append(chip);}
  const collision=state.telemetry.collision_state;if(teleop&&collision?.action===1)$("teleop-note").textContent="Collision stop · "+collision.polygon;const test=state.test;$("test-status").textContent=test?`${test.status.toUpperCase()} · ${test.stage}`:"No test running";$("test-status").className="test-status"+(test?.status==="passed"?" passed":test?.status==="failed"?" failed":"");
  $("test-description").textContent=state.tests[$("test-select").value].description;
  const events=$("events");events.replaceChildren();state.events.slice(-6).reverse().forEach(item=>{const row=document.createElement("div"),time=document.createElement("time"),message=document.createElement("span");row.className="event";time.textContent=new Date(item.time*1000).toLocaleTimeString([], {hour:"2-digit",minute:"2-digit",second:"2-digit"});message.textContent=item.message;row.append(time,message);events.append(row);});
  $("logs").textContent=state.logs.map(log=>`[${log.source}]\n${log.text}`).join("\n")||"No process logs yet.";$("report").textContent=state.report?JSON.stringify(state.report,null,2):"";draw();
}
async function poll() {try{const response=await fetch("/api/state");if(!response.ok)throw new Error("Disconnected");state=await response.json();online=true;render();}catch(_){online=false;$("connection").textContent="Disconnected";$("connection-dot").classList.remove("online");keys.clear();}setTimeout(poll,700);}
$("fit").onclick=fit;$("zoom-in").onclick=()=>zoom(1.6);$("zoom-out").onclick=()=>zoom(1/1.6);$("follow").onclick=()=>{const truth=live("truth");if(truth){center=[truth.x,truth.y];scale=Math.max(scale,8);draw();}else toast("Start the simulator to locate the rover.");};
$("add-mode").onclick=()=>{addMode=!addMode;$("add-mode").classList.toggle("active",addMode);$("add-mode").setAttribute("aria-pressed",String(addMode));$("map-caption").textContent=addMode?"Click to add a waypoint":"Drag to explore the site";};
canvas.addEventListener("pointerdown",event=>{if(event.button!==0)return;canvas.setPointerCapture(event.pointerId);drag={x:event.offsetX,y:event.offsetY,center:[...center],moved:false};});
canvas.addEventListener("pointermove",event=>{const[x,y]=world(event.offsetX,event.offsetY);$("cursor").textContent=`E ${x.toFixed(1)} / N ${y.toFixed(1)} m`;if(drag){const dx=event.offsetX-drag.x,dy=event.offsetY-drag.y;if(Math.hypot(dx,dy)>5)drag.moved=true;if(drag.moved){center=[drag.center[0]-dx/scale,drag.center[1]+dy/scale];draw();}}});
canvas.addEventListener("pointerup",async event=>{if(!drag)return;const moved=drag.moved;drag=null;if(!moved&&addMode&&online&&!state.busy&&!running("mission")){const[x,y]=world(event.offsetX,event.offsetY);let id=1;const used=new Set(state.waypoints.map(w=>w.id));while(used.has(`WP-${String(id).padStart(2,"0")}`))id++;await command("plan.add",{id:`WP-${String(id).padStart(2,"0")}`,x,y});}});
canvas.addEventListener("pointercancel",()=>drag=null);canvas.addEventListener("wheel",event=>{event.preventDefault();zoom(Math.exp(-event.deltaY*.0015),event.offsetX,event.offsetY);},{passive:false});
$("clear-plan").onclick=()=>command("plan.clear");$("preview").onclick=async()=>{const plan=await command("plan.preview");if(plan){state.plan=plan;render();toast(`${plan.route_length_m.toFixed(1)} m terrain route · ${plan.leg_count} local legs`);}};
$("execute").onclick=()=>command("mission.start",{allow_unobserved:$("allow-unknown").checked});$("cancel-mission").onclick=()=>command("mission.cancel");$("launch").onclick=()=>command("simulation.start");$("close-sim").onclick=()=>command("simulation.stop");$("stop").onclick=()=>{keys.clear();command("stop");};
$("teleop-toggle").onclick=()=>{keys.clear();command(state.teleop.enabled?"teleop.disable":"teleop.enable");};$("teleop-zero").onclick=()=>{keys.clear();command("teleop.drive",{linear:0,angular:0},true);};$("test-select").onchange=render;$("run-test").onclick=()=>command("test.run",{id:$("test-select").value});$("cancel-test").onclick=()=>command("test.cancel");
const keyMap={w:"forward",ArrowUp:"forward",s:"back",ArrowDown:"back",a:"left",ArrowLeft:"left",d:"right",ArrowRight:"right"};
function release(){keys.clear();if(state?.teleop.enabled)command("teleop.drive",{linear:0,angular:0},true);document.querySelectorAll("[data-drive]").forEach(button=>button.classList.remove("held"));}
document.addEventListener("keydown",event=>{if(!state?.teleop.enabled||["INPUT","SELECT","TEXTAREA"].includes(event.target.tagName))return;const key=keyMap[event.key]||keyMap[event.key.toLowerCase()];if(key){event.preventDefault();keys.add(key);}});
document.addEventListener("keyup",event=>{const key=keyMap[event.key]||keyMap[event.key.toLowerCase()];if(key){keys.delete(key);if(!keys.size)release();}});
document.querySelectorAll("[data-drive]").forEach(button=>{button.addEventListener("pointerdown",event=>{if(button.disabled)return;event.preventDefault();button.setPointerCapture(event.pointerId);keys.add(button.dataset.drive);button.classList.add("held");});button.addEventListener("pointerup",()=>{keys.delete(button.dataset.drive);button.classList.remove("held");if(!keys.size)release();});button.addEventListener("pointercancel",release);button.addEventListener("lostpointercapture",release);});
let sending=false;setInterval(async()=>{if(!keys.size||!state?.teleop.enabled||!online||sending)return;sending=true;await command("teleop.drive",{linear:.3*((keys.has("forward")?1:0)-(keys.has("back")?1:0)),angular:.6*((keys.has("left")?1:0)-(keys.has("right")?1:0))},true);sending=false;},100);
window.addEventListener("blur",release);document.addEventListener("visibilitychange",()=>{if(document.hidden)release();});window.addEventListener("pagehide",release);new ResizeObserver(resize).observe($("map-wrap"));
fetch("/api/site").then(response=>response.json()).then(value=>{site=value;$("datum-lat").textContent=site.datum.latitude.toFixed(6)+"°";$("datum-lon").textContent=site.datum.longitude.toFixed(6)+"°";$("elevation").textContent=`${Math.round(site.elevation_m[0])}–${Math.round(site.elevation_m[1])} m`;terrain.onload=()=>{fit();draw();};terrain.src=site.map;resize();}).catch(error=>toast(error.message));poll();

$("export-plan").onclick=async()=>{const plan=await command("plan.export");if(!plan)return;const link=document.createElement("a");link.href=URL.createObjectURL(new Blob([JSON.stringify(plan,null,2)],{type:"application/json"}));link.download="storm-waypoints.json";link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);};
$("import-plan").onclick=()=>$("import-file").click();
$("import-file").onchange=async()=>{const file=$("import-file").files[0];if(!file)return;try{if(file.size>65536)throw new Error("Mission file exceeds 64 KB");const data=JSON.parse(await file.text());if(data.version!==1||data.frame!=="WGS84"||!Array.isArray(data.waypoints))throw new Error("Expected a version-1 WGS84 mission file");await command("plan.set",{waypoints:data.waypoints});}catch(error){toast(error.message);}$("import-file").value="";};
