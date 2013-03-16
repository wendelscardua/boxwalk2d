// Global Vars
var timeStep = 1.0 / 60.0;

var doDraw = true;
var cw_paused = false;

var box2dfps = 60;
var screenfps = 60;

var debugbox = document.getElementById("debug");

var canvas = document.getElementById("mainbox");
var ctx = canvas.getContext("2d");

var graphcanvas = document.getElementById("graphcanvas");
var graphctx = graphcanvas.getContext("2d");
var graphheight = 200;
var graphwidth = 400;

var minimapcanvas = document.getElementById("minimap");
var minimapctx = minimapcanvas.getContext("2d");
var minimapscale = 3;
var minimapfogdistance = 0;
var minimarkerdistance = document.getElementById("minimapmarker").style;
var fogdistance = document.getElementById("minimapfog").style;

var generationSize = 20;
var cw_carGeneration = new Array();
var cw_carScores = new Array();
var cw_topScores = new Array();
var cw_graphTop = new Array();
var cw_graphElite = new Array();
var cw_graphAverage = new Array();

var gen_trackType = 0;
var gen_trackFraction = 1;

var gen_champions = 1;
var gen_parentality = 0.2;
var gen_mutation = 0.05;
var gen_counter = 0;
var nWheels = 8;
var nCargo = 2;
var nAttributes = nWheels /*num wheels*/ * 5 /* radius+density+vertex+speed+type */ + 8 /*vertices*/; // change this when genome changes
var leg_kind_tier = 0.33;
var wheel_kind_tier = 0.67;
var gravity = new b2Vec2(0.0, -9.81);
var doSleep = true;

var world = new b2World(gravity, doSleep);

var listener = new b2ContactListener;
    listener.BeginContact = function(contact) {
		var idA = contact.GetFixtureA().GetBody();
		var idB = contact.GetFixtureB().GetBody();
		if (idA.wheelType == 10 && idB.wheelType == -1) {
			idA.is_dead = true;
//            console.log("Colide");
//            console.log(contact.GetFixtureA().GetBody());
//            console.log(contact.GetFixtureB().GetBody());
		}
		
		if (idA.wheelType == -1 && idB.wheelType == 10) {
			idB.is_dead = true;
//            console.log("Colide");
//            console.log(contact.GetFixtureA().GetBody());
//            console.log(contact.GetFixtureB().GetBody());
		}
    }
    listener.EndContact = function(contact) {
        // console.log(contact.GetFixtureA().GetBody().GetUserData());
    }
    listener.PostSolve = function(contact, impulse) {
        
    }
    listener.PreSolve = function(contact, oldManifold) {

    }
world.SetContactListener(listener);

var zoom = 70;

var maxFloorTiles = 200 * gen_trackFraction;
var cw_floorTiles = new Array();
var last_drawn_tile = 0;
var bottom_height = 1000;

var groundPieceWidth = 1.5 / gen_trackFraction;
var groundPieceHeight = 0.15 / gen_trackFraction;

var chassisMaxAxis = 1.1;
var chassisMinAxis = 0.1;

var wheelMaxRadius = 0.5;
var wheelMinRadius = 0.2;
var wheelMaxDensity = 100;
var wheelMinDensity = 40;
var wheelMaxSpeed = 16;
var wheelMinSpeed = -8;

var maxVelocityFIFO = 50;
var velocityFIFO = new Array();
var velocityIndex = 0;
var deathSpeed = 0.1;
var max_car_health = box2dfps * 10;
var car_health = max_car_health;

var motorSpeed = 10;

var attributeIndex = 0;
var swapPoint1 = 0;
var swapPoint2 = 0;

var distanceMeter = document.getElementById("distancemeter");

function debug(str, clear) {
  if(clear) {
    debugbox.innerHTML = "";
  }
  debugbox.innerHTML += str+"<br />";
}

function showDistance(distance, height) {
  distanceMeter.innerHTML = "distance: "+distance+" meters<br />";
  distanceMeter.innerHTML += "height: "+height+" meters";
  minimarkerdistance.left = ((distance + 5) * minimapscale) + "px";
  if(distance > minimapfogdistance) {
    fogdistance.width = 800 - (distance + 15) * minimapscale + "px";
    minimapfogdistance = distance;
  }
}

/* ========================================================================= */
/* === Car ================================================================= */
var cw_Car = function() {
  this.__constructor.apply(this, arguments);
}

cw_Car.prototype.chassis = null;
cw_Car.prototype.wheels = null;

cw_Car.prototype.__constructor = function(car_def) {
  this.chassis = cw_createChassis(car_def.vertex_list);
  this.wheels = new Array();
  for(var i = 0; i < nWheels - nCargo; i++) {
	  this.wheels.push( cw_createWheel(car_def.wheel_radius[i], car_def.wheel_density[i], car_def.wheel_type[i]) );
  }
  for(var j = 0; j < nCargo; j++) {
  	this.wheels.push( cw_createWheel(wheelMinRadius +  .5 * wheelMaxRadius, wheelMinDensity + .5 * wheelMaxDensity, 10) )
  }
  var carmass = this.chassis.GetMass();
  for(var i = 0; i < nWheels; i++)  {
	carmass = carmass + this.wheels[i].GetMass();
  }

  var anchors = new Array();
  var bodies  = new Array();
  for(var i = 0; i < this.chassis.vertex_list.length; i++) {
	anchors.push(this.chassis.vertex_list[i]);
	bodies.push(this.chassis);
  }

  var joint_def = new b2RevoluteJointDef();
  for(var i = 0; i < nWheels; i++) {
	  var torque = carmass * -gravity.y / car_def.wheel_radius[i];
	  var vertex = car_def.wheel_vertex[i];
	  var randvertex = anchors[vertex];
	  joint_def.localAnchorA.Set(randvertex.x, randvertex.y);
	  joint_def.localAnchorB.Set(0, 0);
	  joint_def.maxMotorTorque = torque;
	  joint_def.motorSpeed = car_def.wheel_speed[i];
	  joint_def.enableMotor = true;
	  joint_def.bodyA = bodies[vertex];
	  joint_def.bodyB = this.wheels[i];
	
	  bodies[vertex] = this.wheels[i];

	  anchors[vertex] = this.wheels[i].wheelType == 0 ? new b2Vec2(car_def.wheel_radius[i] * 2, 0) : 
						this.wheels[i].wheelType == 2 ? new b2Vec2(car_def.wheel_radius[i], car_def.wheel_radius[i]) :new b2Vec2(car_def.wheel_radius[i], 0);
	
	  var joint = world.CreateJoint(joint_def);
  }
}

cw_Car.prototype.getPosition = function() {
  return this.chassis.GetPosition();
}

cw_Car.prototype.draw = function() {
  drawObject(this.chassis);
  for(var i = 0; i < nWheels; i++) {
  	drawObject(this.wheels[i]);
  }
}

function cw_createChassisPart(body, vertex1, vertex2) {
  var vertex_list = new Array();
  vertex_list.push(vertex1);
  vertex_list.push(vertex2);
  vertex_list.push(b2Vec2.Make(0,0));
  var fix_def = new b2FixtureDef();
  fix_def.shape = new b2PolygonShape();
  fix_def.density = 4 * (wheelMaxDensity + wheelMinDensity);
  fix_def.friction = 10;
  fix_def.restitution = 0.2;
  fix_def.filter.groupIndex = -1;
  fix_def.shape.SetAsArray(vertex_list,3);

  body.CreateFixture(fix_def);
}

function cw_createChassis(vertex_list) {
  var body_def = new b2BodyDef();
  body_def.type = b2Body.b2_dynamicBody;
  body_def.position.Set(0.0, 4.0);

  var body = world.CreateBody(body_def);

  cw_createChassisPart(body, vertex_list[0],vertex_list[1]);
  cw_createChassisPart(body, vertex_list[1],vertex_list[2]);
  cw_createChassisPart(body, vertex_list[2],vertex_list[3]);
  cw_createChassisPart(body, vertex_list[3],vertex_list[4]);
  cw_createChassisPart(body, vertex_list[4],vertex_list[5]);
  cw_createChassisPart(body, vertex_list[5],vertex_list[6]);
  cw_createChassisPart(body, vertex_list[6],vertex_list[7]);
  cw_createChassisPart(body, vertex_list[7],vertex_list[0]);

  body.vertex_list = vertex_list;

  return body;
}

function cw_createWheel(radius, density, type) {
  var body_def = new b2BodyDef();
  body_def.type = b2Body.b2_dynamicBody;
  body_def.position.Set(0, 0);

  var body = world.CreateBody(body_def);

  var fix_def = new b2FixtureDef();
  if (type == 0) { // leg kind
  //fix_def.shape = new b2CircleShape(radius);
    fix_def.shape = new b2PolygonShape();
    fix_def.shape.SetAsOrientedBox(radius, radius * .1, new b2Vec2(radius, 0), 0);
  } else if (type == 1) { // wheel kind
	fix_def.shape = new b2CircleShape(radius);
  } else if (type == 2) { // square wheel kind
    fix_def.shape = new b2PolygonShape();
    fix_def.shape.SetAsOrientedBox(radius, radius, new b2Vec2(0, 0), 0);
  } else if (type == 10) { // cargo kind
	fix_def.shape = new b2CircleShape(radius);
  }
  fix_def.density = density;
  fix_def.friction = 1;
  fix_def.restitution = 0.2;
  fix_def.filter.groupIndex = -1;

  body.CreateFixture(fix_def);
  body.wheelType = type;
  body.is_dead = false;
  return body;
}

function cw_randomWheelType() {
	var rnd = Math.random();
	if (rnd < leg_kind_tier) {
		return 0;
	} else if (rnd < wheel_kind_tier) {
		return 1;
	} else { // boxkind tier
		return 2;
	}
}

function cw_createRandomCar() {
  var v2;
  var car_def = new Object();
  car_def.wheel_radius = new Array();
  car_def.wheel_density = new Array();
  car_def.wheel_vertex = new Array();
  car_def.wheel_speed = new Array();
  car_def.wheel_type = new Array();
  for(var i = 0; i < nWheels; i++) {
	car_def.wheel_radius.push( Math.random()*wheelMaxRadius+wheelMinRadius );
	car_def.wheel_density.push( Math.random()*wheelMaxDensity+wheelMinDensity );
	car_def.wheel_vertex.push( Math.floor(Math.random()*8)%8 );
	car_def.wheel_speed.push( Math.random()*wheelMaxSpeed+wheelMinSpeed );
	car_def.wheel_type.push( i < nWheels - nCargo ? cw_randomWheelType() : 10 );
  }

  car_def.vertex_list = new Array();
  car_def.vertex_list.push(new b2Vec2(Math.random()*chassisMaxAxis + chassisMinAxis,0));
  car_def.vertex_list.push(new b2Vec2(Math.random()*chassisMaxAxis + chassisMinAxis,Math.random()*chassisMaxAxis + chassisMinAxis));
  car_def.vertex_list.push(new b2Vec2(0,Math.random()*chassisMaxAxis + chassisMinAxis));
  car_def.vertex_list.push(new b2Vec2(-Math.random()*chassisMaxAxis - chassisMinAxis,Math.random()*chassisMaxAxis + chassisMinAxis));
  car_def.vertex_list.push(new b2Vec2(-Math.random()*chassisMaxAxis - chassisMinAxis,0));
  car_def.vertex_list.push(new b2Vec2(-Math.random()*chassisMaxAxis - chassisMinAxis,-Math.random()*chassisMaxAxis - chassisMinAxis));
  car_def.vertex_list.push(new b2Vec2(0,-Math.random()*chassisMaxAxis - chassisMinAxis));
  car_def.vertex_list.push(new b2Vec2(Math.random()*chassisMaxAxis + chassisMinAxis,-Math.random()*chassisMaxAxis - chassisMinAxis));

  return car_def;
}

/* === END Car ============================================================= */
/* ========================================================================= */

/* ========================================================================= */
/* ==== Floor ============================================================== */

function cw_createFloor() {
  var last_tile = null;
  var tile_position = new b2Vec2(-5,0);
  minimapfogdistance = 0;
  fogdistance.width = "800px";
  minimapctx.clearRect(0,0,minimapcanvas.width,minimapcanvas.height);
  minimapctx.strokeStyle = "#000";
  minimapctx.beginPath();
  minimapctx.moveTo(0,35 * minimapscale);
  cw_floorTiles = new Array();
  bottom_height = 1000;
  for(var k = 0; k < maxFloorTiles; k++) {
	switch(gen_trackType) {
		case 0: // random
			last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: k >= maxFloorTiles - 3 ? 1.5 : (Math.random()*3 - 1.5) * 1.5*k/maxFloorTiles);
			break;
		case 1: // upstairs
			last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: k % 10 == 0 && k ? 1.5 : (Math.random()*3 - 1.5) * 1.5*k/maxFloorTiles);
			break;
		case 2: // upwards
    		last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: k >= maxFloorTiles - 3 ? 1.5 : k % 10 == 0 && k ? (Math.random()*3-1.5)*(k-3)/(maxFloorTiles-3) : 1.0 * (k-3)/(maxFloorTiles-3));
			break;
		case 3: // wave
			last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: 1.5*k/maxFloorTiles*Math.sin(30*k/maxFloorTiles));
			break;
        case 4: // downstairs
			last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: k % 10 == 0 && k ? -1.5 : (Math.random()*3 - 1.5) * 1.5*k/maxFloorTiles);
			break;
		case 5: // full random
			last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: k >= maxFloorTiles - 3 ? 1.5 : (Math.random()*3 - 1.5));
			break;
        case 6: // untitled
            last_tile = cw_createFloorTile(tile_position, k < 3 ? -1.5: k % 10 == 0 ? -1.5 : k % 5 == 0 ? 1.5 : (Math.random()*3 - 1.5) * 1.5*k/maxFloorTiles);
			break;
    }
	cw_floorTiles.push(last_tile);
    last_fixture = last_tile.GetFixtureList();
    last_world_coords = last_tile.GetWorldPoint(last_fixture.GetShape().m_vertices[3]);
    tile_position = last_world_coords;
    if (tile_position.y < bottom_height) {
        bottom_height = tile_position.y;
    }
    minimapctx.lineTo((tile_position.x + 5) * minimapscale, (-tile_position.y + 35) * minimapscale);
  }
  minimapctx.stroke();
}



function cw_createFloorTile(position, angle) {
  body_def = new b2BodyDef();

  body_def.position.Set(position.x, position.y);
  var body = world.CreateBody(body_def);
  fix_def = new b2FixtureDef();
  fix_def.shape = new b2PolygonShape();
  fix_def.friction = 0.5;

  var coords = new Array();
  coords.push(new b2Vec2(0,0));
  coords.push(new b2Vec2(0,-groundPieceHeight));
  coords.push(new b2Vec2(groundPieceWidth,-groundPieceHeight));
  coords.push(new b2Vec2(groundPieceWidth,0));

  var center = new b2Vec2(0,0);

  var newcoords = cw_rotateFloorTile(coords, center, angle);

  fix_def.shape.SetAsArray(newcoords);

  body.CreateFixture(fix_def);
  body.wheelType = -1;
  return body;
}

function cw_rotateFloorTile(coords, center, angle) {
  var newcoords = new Array();
  for(var k = 0; k < coords.length; k++) {
    nc = new Object();
    nc.x = Math.cos(angle)*(coords[k].x - center.x) - Math.sin(angle)*(coords[k].y - center.y) + center.x;
    nc.y = Math.sin(angle)*(coords[k].x - center.x) + Math.cos(angle)*(coords[k].y - center.y) + center.y;
    newcoords.push(nc);
  }
  return newcoords;
}

/* ==== END Floor ========================================================== */
/* ========================================================================= */

/* ========================================================================= */
/* ==== Generation ========================================================= */

function cw_generationZero() {
  for(var k = 0; k < generationSize; k++) {
    var car_def = cw_createRandomCar();
    cw_carGeneration.push(car_def);
  }
  gen_counter = 0;
  document.getElementById("generation").innerHTML = "generation 0";
}

function cw_createNextCar() {
  car_health = max_car_health;
  cw_clearVelocityFIFO();
  document.getElementById("cars").innerHTML += "Car #"+(current_car_index+1)+": ";
  var newcar = new cw_Car(cw_carGeneration[current_car_index]);
  newcar.maxPosition = 0;
  newcar.maxPositiony = 0;
  newcar.minPositiony = 0;
  newcar.frames = 0;
  return newcar;
}

function cw_killCar() {
  if(typeof myCar !== 'undefined') {
    world.DestroyBody(myCar.chassis);
    for(var i = 0; i < nWheels; i++) {
    	world.DestroyBody(myCar.wheels[i]);
	}
  }
}

function cw_clearVelocityFIFO() {
  for(var k = 0; k < maxVelocityFIFO; k++) {
    velocityFIFO[k] = 9999;
  }
}

function cw_nextGeneration() {
  var newGeneration = new Array();
  var newborn;
  cw_getChampions();
  cw_topScores.push({i:gen_counter,v:cw_carScores[0].v,x:cw_carScores[0].x,y:cw_carScores[0].y,y2:cw_carScores[0].y2});
  plot_graphs();
  for(var k = 0; k < gen_champions; k++) {
    newGeneration.push(cw_carGeneration[cw_carScores[k].i]);
  }
  for(k = gen_champions; k < generationSize; k++) {
    var parent1 = cw_getParents();
    var parent2 = parent1;
    while(parent2 == parent1) {
      parent2 = cw_getParents();
    }
    newborn = cw_makeChild(cw_carGeneration[parent1],cw_carGeneration[parent2]);
    newborn = cw_mutate(newborn);
    newGeneration.push(newborn);
  }
  cw_carScores = new Array();
  cw_carGeneration = newGeneration;
  gen_counter++;
  document.getElementById("generation").innerHTML = "generation "+gen_counter;
  document.getElementById("cars").innerHTML = "";
}

function cw_getChampions() {
  var ret = new Array();
  cw_carScores.sort(function(a,b) {if(a.v > b.v) {return -1} else {return 1}});
  for(var k = 0; k < generationSize; k++) {
    ret.push(cw_carScores[k].i);
  }
  return ret;
}

function cw_getParents() {
  var parentIndex = -1;
  for(var k = 0; k < generationSize; k++) {
    if(Math.random() <= gen_parentality) {
      parentIndex = k;
      break;
    }
  }
  if(parentIndex == -1) {
    parentIndex = Math.round(Math.random()*(generationSize-1));
  }
  return parentIndex;
}

function cw_makeChild(car_def1, car_def2) {
  var newCarDef = new Object();
  attributeIndex = 0;
  swapPoint1 = Math.round(Math.random()*(nAttributes-1));
  swapPoint2 = swapPoint1;
  while(swapPoint2 == swapPoint1) {
    swapPoint2 = Math.round(Math.random()*(nAttributes-1));
  }
  var parents = [car_def1, car_def2];
  var curparent = 0;

  newCarDef.wheel_radius = new Array();
  newCarDef.wheel_vertex = new Array();
  newCarDef.wheel_density = new Array();
  newCarDef.wheel_speed = new Array();
  newCarDef.wheel_type = new Array();

  for(var i = 0; i < nWheels; i++) {
  	curparent = cw_chooseParent(curparent);
  	newCarDef.wheel_radius.push( parents[curparent].wheel_radius[i] );
    curparent = cw_chooseParent(curparent);
    newCarDef.wheel_vertex.push( parents[curparent].wheel_vertex[i] );
    curparent = cw_chooseParent(curparent);
    newCarDef.wheel_density.push( parents[curparent].wheel_density[i] );
    curparent = cw_chooseParent(curparent);
    newCarDef.wheel_speed.push( parents[curparent].wheel_speed[i] );
    curparent = cw_chooseParent(curparent);
    newCarDef.wheel_type.push( parents[curparent].wheel_type[i] );
  }

  newCarDef.vertex_list = new Array();
  for(var i = 0; i < 8; i++) {
    curparent = cw_chooseParent(curparent);
    newCarDef.vertex_list[i] = parents[curparent].vertex_list[i];
  }

  return newCarDef;
}

function cw_mutate(car_def) {
  var i;
  for(i = 0; i < nWheels; i++) {
	  if(Math.random() < gen_mutation)
	    car_def.wheel_radius[i] = Math.random()*wheelMaxRadius+wheelMinRadius;
	  if(Math.random() < gen_mutation)
	    car_def.wheel_density[i] = Math.random()*wheelMaxDensity+wheelMinDensity;	
	  if(Math.random() < gen_mutation)
	    car_def.wheel_vertex[i] = Math.floor(Math.random()*8)%8;
	  if(Math.random() < gen_mutation)
	    car_def.wheel_speed[i] = Math.abs(Math.random()*wheelMaxSpeed+wheelMinSpeed) * ( car_def.wheel_speed[i] > 0 ? 1 : -1);
	  if(Math.random() < gen_mutation)
	    car_def.wheel_speed[i] = -car_def.wheel_speed[i];
	  if(Math.random() < gen_mutation && i < nWheels - nCargo)
	    car_def.wheel_type[i] = cw_randomWheelType();
  }

  if(Math.random() < gen_mutation)
      car_def.vertex_list[0] = new b2Vec2(Math.random()*chassisMaxAxis + chassisMinAxis,0);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[1] = new b2Vec2(Math.random()*chassisMaxAxis + chassisMinAxis,Math.random()*chassisMaxAxis + chassisMinAxis);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[2] = new b2Vec2(0,Math.random()*chassisMaxAxis + chassisMinAxis);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[3] = new b2Vec2(-Math.random()*chassisMaxAxis - chassisMinAxis,Math.random()*chassisMaxAxis + chassisMinAxis);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[4] = new b2Vec2(-Math.random()*chassisMaxAxis - chassisMinAxis,0);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[5] = new b2Vec2(-Math.random()*chassisMaxAxis - chassisMinAxis,-Math.random()*chassisMaxAxis - chassisMinAxis);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[6] = new b2Vec2(0,-Math.random()*chassisMaxAxis - chassisMinAxis);
  if(Math.random() < gen_mutation)
      car_def.vertex_list[7] = new b2Vec2(Math.random()*chassisMaxAxis + chassisMinAxis,-Math.random()*chassisMaxAxis - chassisMinAxis);
  return car_def;
}

function cw_chooseParent(curparent) {
  var ret;
  if((swapPoint1 == attributeIndex) || (swapPoint2 == attributeIndex)) {
    if(curparent == 1) {
      ret = 0;
    } else {
      ret = 1;
    }
  } else {
    ret = curparent;
  }
  attributeIndex++;
  return ret;
}

function cw_setMutation(mutation) {
  gen_mutation = parseFloat(mutation);
}

function cw_setEliteSize(clones) {
  gen_champions = parseInt(clones, 10);
}

function cw_setTrackType(arg_trackType) {
	gen_trackType = parseInt(arg_trackType, 10);
	cw_resetWorld();
}

function cw_setTrackFraction(arg_trackFraction) {
	gen_trackFraction = parseInt(arg_trackFraction, 10);
	
	maxFloorTiles = 200 * gen_trackFraction;
	groundPieceWidth = 1.5 / gen_trackFraction;
	groundPieceHeight = 0.15 / gen_trackFraction;

	cw_resetWorld();
}

function cw_changeWheelKindTiers() {
	leg_kind_tier = (document.getElementById("legKindTier").value);
	wheel_kind_tier = (document.getElementById("wheelKindTier").value);
	
	cw_resetWorld();
}


function cw_setNumCargo(arg_numCargo) {
	nCargo = parseInt(arg_numCargo, 10);
	//nAttributes = nWheels /*num wheels*/ * 5 /* radius+density+vertex+speed+type */ + 8 /*vertices*/; // change this when genome changes
	
	cw_resetWorld();
}

/* ==== END Genration ====================================================== */
/* ========================================================================= */

/* ========================================================================= */
/* ==== Drawing ============================================================ */

function cw_drawScreen() {
  carPosition = myCar.getPosition();
  ctx.clearRect(0,0,canvas.width,canvas.height);
  ctx.save();
  ctx.translate(200-(carPosition.x*zoom), 200+(carPosition.y*zoom));
  ctx.scale(zoom, -zoom);
  cw_drawFloor();
  cw_drawCar(myCar);
  ctx.restore();
}

function cw_drawFloor() {
  ctx.strokeStyle = "#000";
  ctx.fillStyle = "#777";
  ctx.lineWidth = 1/zoom;
  ctx.beginPath();

  outer_loop:
  for(var k = Math.max(0,last_drawn_tile-cw_floorTiles.length/5); k < cw_floorTiles.length; k++) {
    var b = cw_floorTiles[k];
    for (f = b.GetFixtureList(); f; f = f.m_next) {
      var s = f.GetShape();
      var shapePosition = b.GetWorldPoint(s.m_vertices[0]).x;
      if((shapePosition > (carPosition.x - 5)) && (shapePosition < (carPosition.x + 10))) {
        cw_drawVirtualPoly(b, s.m_vertices, s.m_vertexCount);
      }
      if(shapePosition > carPosition.x + 10) {
        last_drawn_tile = k;
        break outer_loop;
      }
    }
  }
  ctx.fill();
  ctx.stroke();
}

function cw_drawCar(myCar) {

  /*
  b = myCar.wheel1;
  for (f = b.GetFixtureList(); f; f = f.m_next) {
    var s = f.GetShape();
    var color = Math.round(255 - (255 * (f.m_density - wheelMinDensity)) / wheelMaxDensity).toString();
    var rgbcolor = "rgb("+color+","+color+","+color+")";
    cw_drawCircle(b, s.m_p, s.m_radius, b.m_sweep.a, rgbcolor);
  }
  b = myCar.wheel2;
  for (f = b.GetFixtureList(); f; f = f.m_next) {
    var s = f.GetShape();
    var color = Math.round(255 - (255 * (f.m_density - wheelMinDensity)) / wheelMaxDensity).toString();
    var rgbcolor = "rgb("+color+","+color+","+color+")";
    cw_drawCircle(b, s.m_p, s.m_radius, b.m_sweep.a, rgbcolor);
  }
  */
  var i;
  ctx.strokeStyle = "#555";
  ctx.lineWidth = 1/zoom;

  for(i = 0; i < nWheels; i++) {
    if (myCar.wheels[i].wheelType == 0 || myCar.wheels[i].wheelType == 2) {
		  ctx.beginPath();
		  var b = myCar.wheels[i];
		  for (f = b.GetFixtureList(); f; f = f.m_next) {
		    var s = f.GetShape();
		    var color = Math.round(255 - (255 * (f.m_density - wheelMinDensity)) / wheelMaxDensity).toString();
		    var rgbcolor = "rgb("+color+","+color+","+color+")";
		    ctx.fillStyle=rgbcolor;
		    cw_drawVirtualPoly(b, s.m_vertices, s.m_vertexCount);
		  }
		  ctx.fill();
		  ctx.stroke();
    } else {
		var b = myCar.wheels[i];
		for (f = b.GetFixtureList(); f; f = f.m_next) {
		  var s = f.GetShape();
		  var color = Math.round(255 - (255 * (f.m_density - wheelMinDensity)) / wheelMaxDensity).toString();
		  var rgbcolor = (myCar.wheels[i].wheelType == 1) ? "rgb("+color+","+color+","+color+")" : "rgb(0, 0, "+color+")";
		  cw_drawCircle(b, s.m_p, s.m_radius, b.m_sweep.a, rgbcolor);
		}
    }
  }

  
  ctx.strokeStyle = "#c44";
  ctx.fillStyle = "#fdd";
  ctx.beginPath();
  var b = myCar.chassis;
  for (f = b.GetFixtureList(); f; f = f.m_next) {
    var s = f.GetShape();
    cw_drawVirtualPoly(b, s.m_vertices, s.m_vertexCount);
  }
  ctx.fill();
  ctx.stroke();
}

function toggleDisplay() {
  if(cw_paused) {
    return;
  }
  canvas.width = canvas.width;
  if(doDraw) {
    doDraw = false;
    clearInterval(cw_drawInterval);
    clearInterval(cw_runningInterval);
    cw_runningInterval = setInterval(simulationStep, 1); // simulate 1000x per second when not drawing
  } else {
    doDraw = true;
    cw_drawInterval = setInterval(cw_drawScreen, Math.round(1000/screenfps));
    clearInterval(cw_runningInterval);
    cw_runningInterval = setInterval(simulationStep, Math.round(1000/box2dfps));
  }
}

function cw_drawVirtualPoly(body, vtx, n_vtx) {
  // set strokestyle and fillstyle before call
  // call beginPath before call

  var p0 = body.GetWorldPoint(vtx[0]);
  ctx.moveTo(p0.x, p0.y);
  for (var i = 1; i < n_vtx; i++) {
    p = body.GetWorldPoint(vtx[i]);
    ctx.lineTo(p.x, p.y);
  }
  ctx.lineTo(p0.x, p0.y);
}

function cw_drawPoly(body, vtx, n_vtx) {
  // set strokestyle and fillstyle before call
  ctx.beginPath();

  var p0 = body.GetWorldPoint(vtx[0]);
  ctx.moveTo(p0.x, p0.y);
  for (var i = 1; i < n_vtx; i++) {
    p = body.GetWorldPoint(vtx[i]);
    ctx.lineTo(p.x, p.y);
  }
  ctx.lineTo(p0.x, p0.y);

  ctx.fill();
  ctx.stroke();
}

function cw_drawCircle(body, center, radius, angle, color) {
  var p = body.GetWorldPoint(center);
  ctx.fillStyle = color;

  ctx.beginPath();
  ctx.arc(p.x, p.y, radius, 0, 2*Math.PI, true);

  ctx.moveTo(p.x, p.y);
  ctx.lineTo(p.x + radius*Math.cos(angle), p.y + radius*Math.sin(angle));

  ctx.fill();
  ctx.stroke();
}

/* ==== END Drawing ======================================================== */
/* ========================================================================= */


/* ========================================================================= */
/* ==== Graphs ============================================================= */

function cw_storeGraphScores() {
  cw_graphAverage.push(cw_average(cw_carScores));
  cw_graphElite.push(cw_eliteaverage(cw_carScores));
  cw_graphTop.push(cw_carScores[0].v);
}

function cw_plotTop() {
  var graphsize = cw_graphTop.length;
  graphctx.strokeStyle = "#f00";
  graphctx.beginPath();
  graphctx.moveTo(0,0);
  for(var k = 0; k < graphsize; k++) {
    graphctx.lineTo(400*(k+1)/graphsize,cw_graphTop[k]);
  }
  graphctx.stroke();
}

function cw_plotElite() {
  var graphsize = cw_graphElite.length;
  graphctx.strokeStyle = "#0f0";
  graphctx.beginPath();
  graphctx.moveTo(0,0);
  for(var k = 0; k < graphsize; k++) {
    graphctx.lineTo(400*(k+1)/graphsize,cw_graphElite[k]);
  }
  graphctx.stroke();
}

function cw_plotAverage() {
  var graphsize = cw_graphAverage.length;
  graphctx.strokeStyle = "#00f";
  graphctx.beginPath();
  graphctx.moveTo(0,0);
  for(var k = 0; k < graphsize; k++) {
    graphctx.lineTo(400*(k+1)/graphsize,cw_graphAverage[k]);
  }
  graphctx.stroke();
}

function plot_graphs() {
  cw_storeGraphScores();
  cw_clearGraphics();
  cw_plotAverage();
  cw_plotElite();
  cw_plotTop();
  cw_listTopScores();
}


function cw_eliteaverage(scores) {
  var sum = 0;
  for(var k = 0; k < Math.floor(generationSize/2); k++) {
    sum += scores[k].v;
  }
  return sum/Math.floor(generationSize/2);
}

function cw_average(scores) {
  var sum = 0;
  for(var k = 0; k < generationSize; k++) {
    sum += scores[k].v;
  }
  return sum/generationSize;
}

function cw_clearGraphics() {
  graphcanvas.width = graphcanvas.width;
  graphctx.translate(0,graphheight);
  graphctx.scale(1,-1);
  graphctx.lineWidth = 1;
  graphctx.strokeStyle="#888";
  graphctx.beginPath();
  graphctx.moveTo(0,graphheight/2);
  graphctx.lineTo(graphwidth, graphheight/2);
  graphctx.moveTo(0,graphheight/4);
  graphctx.lineTo(graphwidth, graphheight/4);
  graphctx.moveTo(0,graphheight*3/4);
  graphctx.lineTo(graphwidth, graphheight*3/4);
  graphctx.stroke();
}

function cw_listTopScores() {
  var ts = document.getElementById("topscores");
  ts.innerHTML = "Top Scores:<br />";
  cw_topScores.sort(function(a,b) {if(a.v > b.v) {return -1} else {return 1}});
  for(var k = 0; k < Math.min(10,cw_topScores.length); k++) {
    document.getElementById("topscores").innerHTML += "#"+(k+1)+": "+Math.round(cw_topScores[k].v*100)/100+" d:"+Math.round(cw_topScores[k].x*100)/100+" h:"+Math.round(cw_topScores[k].y2*100)/100+"/"+Math.round(cw_topScores[k].y*100)/100+"m (gen "+cw_topScores[k].i+")<br />";
  }
}

/* ==== END Graphs ========================================================= */
/* ========================================================================= */

function simulationStep() {
  world.Step(1/box2dfps, 20, 20);
  myCar.frames++;
  showDistance(Math.round(myCar.getPosition().x*100)/100, Math.round(myCar.getPosition().y*100)/100);
  cw_storeVelocity(Math.abs(myCar.chassis.GetLinearVelocity().x) + Math.abs(myCar.chassis.GetLinearVelocity().y));
  if(cw_checkDeath()) {
    cw_kill();
  }
}

function cw_kill() {
  var avgspeed = (myCar.maxPosition / myCar.frames) * box2dfps;
  var position = myCar.maxPosition;
  var score = position + avgspeed;
  document.getElementById("cars").innerHTML += Math.round(position*100)/100 + "m + " +" "+Math.round(avgspeed*100)/100+" m/s = "+ Math.round(score*100)/100 +"pts<br />";
  cw_carScores.push({ i:current_car_index, v:score, s: avgspeed, x:position, y:myCar.maxPositiony, y2:myCar.minPositiony });
  current_car_index++;
  cw_killCar();
  if(current_car_index >= generationSize) {
    cw_nextGeneration();
    current_car_index = 0;
  }
  myCar = cw_createNextCar();
  last_drawn_tile = 0;
}

function cw_storeVelocity(velocity) {
  velocityIndex++;
  if(velocityIndex >= maxVelocityFIFO) {
    velocityIndex = 0;
  }
  velocityFIFO[velocityIndex] = velocity;
}

function cw_checkDeath() {
  // check health
  if(myCar.getPosition().y > myCar.maxPositiony) {
    myCar.maxPositiony = myCar.getPosition().y;
  }
  if(myCar.getPosition().y < myCar.minPositiony) {
    myCar.minPositiony = myCar.getPosition().y;
  }
  if(myCar.getPosition().x > myCar.maxPosition + 1) {
    car_health = max_car_health;
    myCar.maxPosition = myCar.getPosition().x;
  } else if (myCar.getPosition().y < bottom_height - 20) {
    car_health = 0;
    return true;
  }

  car_health--;

  if(car_health <= 0) {
    return true;
  }

  for(var j = 0; j < nCargo; j++) {
  	if (myCar.wheels[nWheels - j - 1].is_dead) {
		return true;
  	}
  }

  document.getElementById("health").innerHTML = "Health: " + car_health;

  // check speed
  var result = 0;
  for(var k = 0; k < maxVelocityFIFO; k++) {
    result += velocityFIFO[k];
  }
  if(result < deathSpeed*maxVelocityFIFO) {
    return true;
  } else {
    return false;
  }
}

function cw_resetPopulation() {
  document.getElementById("generation").innerHTML = "";
  document.getElementById("cars").innerHTML = "";
  document.getElementById("topscores").innerHTML = "";
  cw_clearGraphics();
  cw_carGeneration = new Array();
  cw_carScores = new Array();
  cw_topScores = new Array();
  cw_graphTop = new Array();
  cw_graphElite = new Array();
  cw_graphAverage = new Array();
  velocityFIFO = new Array();
  velocityIndex = 0;
  lastmax = 0;
  lastaverage = 0;
  lasteliteaverage = 0;
  swapPoint1 = 0;
  swapPoint2 = 0;
  cw_killCar();
  cw_clearVelocityFIFO()
  cw_generationZero();
  current_car_index = 0;
  myCar = cw_createNextCar();
}

function cw_resetWorld() {
  for (b = world.m_bodyList; b; b = b.m_next) {
    world.DestroyBody(b);
  }
  Math.seedrandom(document.getElementById("newseed").value);
  cw_createFloor();
  Math.seedrandom();
  cw_resetPopulation();
}

function cw_confirmResetWorld() {
  if(confirm('Really reset world?')) {
    cw_resetWorld();
  } else {
    return false;
  }
}

function cw_pauseSimulation() {
  cw_paused = true;
  clearInterval(cw_runningInterval);
  clearInterval(cw_drawInterval);
  old_last_drawn_tile = last_drawn_tile;
  last_drawn_tile = 0;
}

function cw_resumeSimulation() {
  cw_paused = false;
  last_drawn_tile = old_last_drawn_tile;
  cw_runningInterval = setInterval(simulationStep, Math.round(1000/box2dfps));
  cw_drawInterval = setInterval(cw_drawScreen, Math.round(1000/screenfps));
}

var oldrandom = Math.seedrandom();
cw_createFloor();

cw_resetPopulation();

cw_runningInterval = setInterval(simulationStep, Math.round(1000/box2dfps));
cw_drawInterval = setInterval(cw_drawScreen, Math.round(1000/screenfps));
