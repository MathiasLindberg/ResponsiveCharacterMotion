const svg = document.getElementById("svg");
const width = svg.getAttributeNS(null, "width");
const height = svg.getAttributeNS(null, "height");
const sliderMax = document.getElementById("slider_max");
const textMax = document.getElementById("text_max");
const buttonW = document.getElementById("button_w");
const buttonS = document.getElementById("button_s");
const buttonA = document.getElementById("button_a");
const buttonD = document.getElementById("button_d");
const buttonShift = document.getElementById("button_shift");
const sliderMoveDamp = document.getElementById("slider_movement_damping");
const sliderAngDamp = document.getElementById("slider_angular_damping");
const textMoveDamp = document.getElementById("text_movement_damping");
const textAngDamp = document.getElementById("text_angular_damping");
const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
const dirPath = document.createElementNS("http://www.w3.org/2000/svg", "path");
const head = document.createElementNS("http://www.w3.org/2000/svg", "circle");
const target = document.createElementNS("http://www.w3.org/2000/svg", "circle");
const predictions = [];
svg.style.cursor = "none";
path.style.strokeWidth = 5;
path.style.strokeLinecap = "round";
path.style.stroke = "black";
path.style.fill = "transparent";
dirPath.style.strokeWidth = 1;
dirPath.style.stroke = "blue";
dirPath.style.fill = "transparent";
head.setAttributeNS(null, "r", 3);
head.style.fill = "blue";
head.style.stroke = "black";
head.style.strokeWidth = 1;
target.setAttributeNS(null, "r", 5);
target.style.fill = "red";
target.style.stroke = "black";
head.style.strokeWidth = 2;

svg.appendChild(path);
svg.appendChild(dirPath);
svg.appendChild(head);
svg.appendChild(target);

for (let i = 0; i < 3; i++) {
    pos = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    dir = document.createElementNS("http://www.w3.org/2000/svg", "line");
    pos.style.fill = "red";
    pos.setAttributeNS(null, "r", 3);
    dir.style.stroke = "red";
    dir.style.strokeWidth = 1;
    predictions.push(pos);
    predictions.push(dir);
    svg.appendChild(pos);
    svg.appendChild(dir);
}

d1 = "";
d2 = "";

function calculateCriticallyDampedSpring(value, velocity, goal, dampingCoefficient, deltatime) {
    const acceleration = dampingCoefficient * dampingCoefficient * (value - goal);
    const deltaDamping = 1.0 + dampingCoefficient * deltatime;
    const newVelocity = (velocity - acceleration * deltatime) / (deltaDamping * deltaDamping);
    return [value + deltatime * newVelocity, newVelocity];
}

// inputs
let moveForward = false;
let movebackwards = false;
let moveLeft = false;
let moveRight = false;
let running = false;

let mouseX = 0;
let mouseY = 0;
let lastUpdate = 0;

const linelength = 10;

let movementDamping = 0;
let angularDamping = 0;
let maxAcc = 0;

let desiredAngle = 0;
let angle = 0;
let angularVelocity = 0;
let desiredAcceleration = [0, 0];
let direction = [1, 0];
let position = [width * 0.5, height * 0.5];
let velocity = [0, 0];
let acceleration = [0, 0];
let jerk = [0, 0];

let positionHistory = [];
let directionHistory = [];
// 1. set direction, update it towards mouse cursor by damping angular velocity
// 2. define movement direction by WASD keys
// 3. step in this direction using velocity, in extend jerk
// TODO: add inputs for angle change
// TODO: fix angle to either continuously increase/decrease or use shortestAngleBetween

function stepOnce(dt) {
    const accResX = calculateCriticallyDampedSpring(acceleration[0], jerk[0], desiredAcceleration[0], movementDamping, dt);
    const accResY = calculateCriticallyDampedSpring(acceleration[1], jerk[1], desiredAcceleration[1], movementDamping, dt);
    const angleRes = calculateCriticallyDampedSpring(angle, angularVelocity, desiredAngle, angularDamping, dt);
    acceleration = [accResX[0], accResY[0]];
    jerk = [accResX[1], accResY[1]];
    angle = angleRes[0];
    angularVelocity = angleRes[1];
    if (positionHistory.length >= linelength) {
        positionsTemp = [];
        directionsTemp = [];
        for (let i = 1; i < linelength - 1; i++) {
            positionsTemp.push(positionHistory[i]);
            directionsTemp.push(directionHistory[i]);
        }
        positionsTemp.push(position);
        directionsTemp.push(direction);
        positionHistory = positionsTemp;
        directionHistory = directionsTemp;
    }
    else {
        positionHistory.push(position);
        directionHistory.push(direction);
    }
    let dt_ = 1.0 / 60;
    let stepPos = [position[0], position[1]];
    let stepVel = [velocity[0], velocity[1]];
    let stepX = [accResX[0], accResX[1]];
    let stepY = [accResY[0], accResY[1]];
    let stepAng = [angleRes[0], angleRes[1]];
    for (let i = 0; i < 3; i++) {
        for (let j = 0; j < 20; j++) {
            stepX = calculateCriticallyDampedSpring(stepX[0], stepX[1], desiredAcceleration[0], movementDamping, dt_);
            stepY = calculateCriticallyDampedSpring(stepY[0], stepY[1], desiredAcceleration[1], movementDamping, dt_);
            stepAng = calculateCriticallyDampedSpring(stepAng[0], stepAng[1], desiredAngle, angularDamping, dt_);
            stepVel[0] += stepX[0] * dt_;
            stepVel[1] += stepY[0] * dt_;
            stepPos[0] += stepVel[0] * dt_;
            stepPos[1] += stepVel[1] * dt_;
        }
        const i1 = i * 2;
        const i2 = i1 + 1;
        predictions[i1].setAttributeNS(null, "cx", stepPos[0]);
        predictions[i1].setAttributeNS(null, "cy", stepPos[1]);
        predictions[i2].setAttributeNS(null, "x1", stepPos[0]);
        predictions[i2].setAttributeNS(null, "y1", stepPos[1]);
        predictions[i2].setAttributeNS(null, "x2", stepPos[0] + Math.cos(stepAng[0]) * 15);
        predictions[i2].setAttributeNS(null, "y2", stepPos[1] + Math.sin(stepAng[0]) * 15);
    }
}

function clamp(val, min, max) {
    return Math.min(Math.max(val, min), max);
}
function shortestSignedAngleBetween(vec1, vec2) {
    let vecDiff = rotateBy(vec2, -Math.atan2(vec1[1], vec1[0]));
    let angle = Math.acos(vecDiff[0]);
    return Math.atan2(vecDiff[1], vecDiff[0]) < 0 ? angle : -angle;
}
function rotateBy(vec, rad) {
    return [Math.cos(rad) * vec[0] - Math.sin(rad) * vec[1], Math.sin(rad) * vec[0] + Math.cos(rad) * vec[1]];
}

function update() {
    // Update inputs
    maxAcc = sliderMax.value;
    textMax.innerText = maxAcc;
    movementDamping = sliderMoveDamp.value;
    textMoveDamp.innerText = movementDamping;
    angularDamping = sliderAngDamp.value;
    textAngDamp.innerText = angularDamping;
    // Update desired facing direction
    let dir = [mouseX - position[0], mouseY - position[1]];
    if (dir[0] != 0 && dir[1] != 0) {
        dirMag = Math.sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
        dir = [dir[0] / dirMag, dir[1] / dirMag];
        // Update desired movement direction
        let angleBetween = shortestSignedAngleBetween([Math.cos(desiredAngle), Math.sin(desiredAngle)], dir);
        if (Number.isNaN(angleBetween)) angleBetween = 0;
        desiredAngle -= angleBetween;
    }
    // Update movement
    direction = [Math.cos(angle), Math.sin(angle)];
    rightDir = rotateBy(direction, Math.PI * 0.5);
    desiredAcceleration = [0, 0];
    multiplier = running ? 2.0  : 1.0;
    if (moveForward) {
        desiredAcceleration[0] += direction[0] * maxAcc * multiplier;
        desiredAcceleration[1] += direction[1] * maxAcc * multiplier;
    }
    else if (movebackwards) {
        desiredAcceleration[0] -= direction[0] * maxAcc * multiplier;
        desiredAcceleration[1] -= direction[1] * maxAcc * multiplier;
    }
    if (moveRight) {
        desiredAcceleration[0] += rightDir[0] * maxAcc * multiplier;
        desiredAcceleration[1] += rightDir[1] * maxAcc * multiplier;
    }
    else if (moveLeft) {
        desiredAcceleration[0] -= rightDir[0] * maxAcc * multiplier;
        desiredAcceleration[1] -= rightDir[1] * maxAcc * multiplier;
    }

    const time = Date.now();
    const dt = (time - lastUpdate) * 0.001;
    // update facing direction angle
    angle += angularVelocity * dt;
    // Update movement
    velocity[0] += acceleration[0] * dt;
    velocity[1] += acceleration[1] * dt;
    position[0] += velocity[0] * dt;
    position[1] += velocity[1] * dt;

    if (!(moveForward || movebackwards || moveRight || moveLeft)) {
        velXRes = calculateCriticallyDampedSpring(velocity[0], acceleration[0], 0, movementDamping, dt);
        velYRes = calculateCriticallyDampedSpring(velocity[1], acceleration[1], 0, movementDamping, dt);
        velocity[0] = velXRes[0];
        velocity[1] = velYRes[0];
    }
    
    if (position[0] < 0 || position[0] > width) {
        velocity[0] = 0;
        acceleration[0] = 0;
    }
    else if (position[1] < 0 || position[1] > height) {
        velocity[1] = 0;
        acceleration[1] = 0;
    }
    position = [clamp(position[0], 0, width), clamp(position[1], 0, height)];

    stepOnce(dt);

    for (let i = 0; i < positionHistory.length; i++) {
        p = positionHistory[i];
        d = directionHistory[i];
        length = i == positionHistory.length - 1 ? 20 : 10;
        if (i == 0) {
            d1 = "M " + p[0] + " " + p[1];
            d2 = d1 + " L " + (p[0] + d[0] * length) + " " + (p[1] + d[1] * length);
        }
        else {
            d1 += " L " + p[0] + " " + p[1];
            d2 += " M " + p[0] + " " + p[1] + " L " + (p[0] + d[0] * length) + " " + (p[1] + d[1] * length);
        }

    }
    head.setAttributeNS(null, "cx", position[0]);
    head.setAttributeNS(null, "cy", position[1]);
    target.setAttributeNS(null, "cx", mouseX);
    target.setAttributeNS(null, "cy", mouseY);
    path.setAttributeNS(null, "d", d1);
    dirPath.setAttributeNS(null, "d", d2);



    lastUpdate = time;
    requestAnimationFrame(update);
}

function start() {
    lastUpdate = Date.now();
    requestAnimationFrame(update);
}

function onKeyDown(e) {
    switch (e.keyCode) {
        case 87: moveForward = true; buttonW.style.opacity = 0.8; break;
        case 83: movebackwards = true; buttonS.style.opacity = 0.8; break;
        case 65: moveLeft = true; buttonA.style.opacity = 0.8; break;
        case 68: moveRight = true; buttonD.style.opacity = 0.8; break;
        case 16: running = true; buttonShift.style.opacity = 0.9; break;
    }
}
function onKeyUp(e) {
    switch (e.keyCode) {
        case 87: moveForward = false; buttonW.style.opacity = 0.5; break;
        case 83: movebackwards = false; buttonS.style.opacity = 0.5; break;
        case 65: moveLeft = false; buttonA.style.opacity = 0.5; break;
        case 68: moveRight = false; buttonD.style.opacity = 0.5; break;
        case 16: running = false; buttonShift.style.opacity = 0.6; break;
    }
}

svg.addEventListener("mousemove", e => {mouseX = e.clientX; mouseY = e.clientY;});
window.addEventListener("keydown", onKeyDown);
window.addEventListener("keyup", onKeyUp);

window.onload = start;