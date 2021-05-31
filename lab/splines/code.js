const svg = document.getElementById("svg");
const points = [];
const pointGraphics = [];
const directionGraphics = [];
const pointRad = 8;
const splineSegmentSteps = 10;
const directionLength = 50;
const pathG = document.createElementNS("http://www.w3.org/2000/svg", "path");
pathG.setAttributeNS(null, "stroke", "black");
pathG.setAttributeNS(null, "stroke-width", 5);
pathG.setAttributeNS(null, "fill", "transparent");
let d = "";
svg.appendChild(pathG);
const tooltipG = document.createElementNS("http://www.w3.org/2000/svg", "text");
svg.appendChild(tooltipG);
const character = document.createElementNS("http://www.w3.org/2000/svg", "circle");
character.setAttributeNS(null, "stroke", "black");
character.setAttributeNS(null, "stroke-width", 3);
character.setAttributeNS(null, "fill", "orange");
character.setAttributeNS(null, "r", pointRad * 2);
const charLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
charLine.setAttributeNS(null, "stroke", "orange");
charLine.setAttributeNS(null, "stroke-width", 3);
svg.appendChild(charLine);
svg.appendChild(character);

let charUpdateEvent = -1;
let timeTableFrom = [];
let timeTableTo = [];

function getSpline(p0, p1, p2, p3) {
    return t => {
        const tt = t * t;
        const ttt = tt * t;
        const a0 = 1 - 3 * t + 3 * tt - ttt;
        const a1 = 4 - 6 * tt + 3 * ttt;
        const a2 = 1 + 3 * t + 3 * tt - 3 * ttt;
        return [(a0 * p0[0] + a1 * p1[0] + a2 * p2[0] + ttt * p3[0]) / 6,
                (a0 * p0[1] + a1 * p1[1] + a2 * p2[1] + ttt * p3[1]) / 6];
    }
}
function getSplineDerived(p0, p1, p2, p3) {
    return t => {
        const tt = t * t;
        const a0 = -3 * (t - 1) * (t - 1);
        const a1 = 3 * t * (3 * t - 4);
        const a2 = -9 * tt + 6 * t + 3;
        const a3 = 3 * tt;
        return [(a0 * p0[0] + a1 * p1[0] + a2 * p2[0] + a3 * p3[0]) / 6,
                (a0 * p0[1] + a1 * p1[1] + a2 * p2[1] + a3 * p3[1]) / 6];
    }
}

function clearDirections() {
    for (let i = directionGraphics.length - 1; i >= 0; i--) {
        directionGraphics[i].remove();
        directionGraphics.pop();
    }
}
function addDirection(x, y, xDir, yDir) {
    line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttributeNS(null, "x1", x);
    line.setAttributeNS(null, "y1", y);
    line.setAttributeNS(null, "x2", x + xDir);
    line.setAttributeNS(null, "y2", y + yDir);
    line.setAttributeNS(null, "stroke", "red");
    line.setAttributeNS(null, "stroke-width", 3);
    directionGraphics.push(line);
    svg.appendChild(line);
}

function updatePath() {
    clearDirections();
    pathG.remove();
    let totalLength = 0;
    timeTableFrom = [0];
    timeTableTo = [0];
    let prevPos = null;
    let first = true;
    for (let i = 0; i < points.length - 3; i++) {
        const spline = getSpline(points[i], points[i + 1], points[i + 2], points[i + 3]);
        const splineDer = getSplineDerived(points[i], points[i + 1], points[i + 2], points[i + 3]);
        for (let j = 0; j < splineSegmentSteps; j++) {
            const t = (j + 1) / splineSegmentSteps;
            pos = spline(t);
            if (first) {
                d = "M " + pos[0] + " " + pos[1];
                first = false;
            }
            else {
                d += " L " + pos[0] + " " + pos[1];
                totalLength += Math.sqrt((pos[0] - prevPos[0])**2 + (pos[1] - prevPos[1])**2);
                timeTableFrom.push(totalLength);
                timeTableTo.push(i + t);
            }
            prevPos = pos;
        }
        p0 = spline(0);
        d0 = splineDer(0);
        d0Mag = Math.sqrt(d0[0]**2 + d0[1]**2);
        addDirection(p0[0], p0[1], d0[0] / d0Mag * directionLength, d0[1] / d0Mag * directionLength);
    }
    for (let i = 0; i < timeTableFrom.length; i++) timeTableFrom[i] /= totalLength;
    pathG.setAttributeNS(null, "d", d);
    svg.appendChild(pathG);
    if (points.length > 3 && charUpdateEvent == -1) charUpdateEvent = setInterval(updateCharacter, 10);
    startTime = Date.now();
    /*console.log("-----------------------")
    for (let i = 0; i < timeTableFrom.length; i++) {
        console.log("| " + i + " || " + timeTableTo[i] + " | " + timeTableFrom[i] + " |");
    }
    console.log("-----------------------")*/
}

function createPoint(x, y) {
    points.push([x,y, pointGraphics.length]);
    pointG = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    pointG.setAttributeNS(null, "cx", x);
    pointG.setAttributeNS(null, "cy", y);
    pointG.setAttributeNS(null, "r", pointRad);
    pointG.setAttributeNS(null, "fill", "green");
    pointG.setAttributeNS(null, "stroke", "black");
    pointG.setAttributeNS(null, "stroke-width", 1);
    svg.appendChild(pointG);
    pointGraphics.push(pointG);
}

function lengthToParametric(l) {
    let prevLength = 0;
    for (let i = 0; i < timeTableFrom.length; i++) {
        let length = timeTableFrom[i];
        if (length > l) {
            const prevParametric = i == 0 ? 0 : timeTableTo[i - 1];
            const parametric = timeTableTo[i];
            const pct = (l - prevLength) / (length - prevLength);
            return prevParametric + pct * (parametric - prevParametric);
        }
        prevLength = length;
    }
    return 1;
}

function clear() {
    for (let i = pointGraphics.length - 1; i >= 0; i--) {
        pointGraphics[i].remove();
        pointGraphics.pop();
        points.pop();
    }
    d = "";
    pathG.setAttributeNS(null, "d", d);
    clearDirections();
    if (charUpdateEvent != -1) clearInterval(charUpdateEvent);
    charUpdateEvent = -1;
}

function drawPoint(x, y) {
    createPoint(x, y);
    lastPointPlacement = Date.now();
}

function findClosestPoint(x, y) {
    let closestID = -1;
    let closestDist = Number.MAX_VALUE;
    for (let i = 0; i < points.length; i++) {
        const [px, py, _] = points[i];
        const dist = Math.sqrt((x - px) ** 2 + (y - py) ** 2);
        if (dist < closestDist) {
            closestDist = dist;
            closestID = i;
        }
    }
    return [closestID, closestDist];
}

let mouseDown = false;
let draggedPointID = -1;
const draggingOffset = [0, 0];
let lastPointPlacement = 0;
const pointPlacementInterval = 100;
let dDown = false;
function onMouseDown(e) {
    mouseDown = true;
    const point = findClosestPoint(e.clientX - 5, e.clientY - 5);
    if (point[1] <= pointRad) {
        draggedPointID = point[0];
        draggingOffset[0] = e.clientX - points[draggedPointID][0];
        draggingOffset[1] = e.clientY - points[draggedPointID][1];
    }
}
function onMouseUp(e) {
    mouseDown = false;
    draggedPointID = -1;
    draggingOffset[0] = 0;
    draggingOffset[1] = 0;
    tooltipG.innerHTML = "";
    if (points.length > 3) updatePath();
}


function onMouseMove(e) {
    if (draggedPointID != -1) {
        const newX = e.clientX - draggingOffset[0];
        const newY = e.clientY - draggingOffset[1];
        points[draggedPointID][0] = newX;
        points[draggedPointID][1] = newY;
        draggedPointG = pointGraphics[points[draggedPointID][2]];
        draggedPointG.setAttributeNS(null, "cx", newX);
        draggedPointG.setAttributeNS(null, "cy", newY);
        tooltipG.innerHTML = draggedPointID;
        tooltipG.setAttributeNS(null, "x", newX + 5);
        tooltipG.setAttributeNS(null, "y", newY - 5);
    }
    else if (dDown && Date.now() - lastPointPlacement > pointPlacementInterval) drawPoint(e.clientX, e.clientY);
}
function onKeyUp(e) {
    switch (e.keyCode) {
        case 68: // d - draw
            dDown = false;
            if (points.length > 3) updatePath();
            break;
        case 82: // r - reset
            clear();
            break;
    }
}
function onKeyDown(e) {
    switch (e.keyCode) {
        case 68: // d - draw
            dDown = true;
            break;
    }
}

let startTime = Date.now();
let lapTime = 4;
function updateCharacter() {
    const elapsedTime = (Date.now() - startTime) * 0.001;
    const u = lengthToParametric((elapsedTime % lapTime) / lapTime);
    const frac = u - Math.floor(u);
    const ind = Math.floor(u);
    const ind1 = Math.min(ind + 1, points.length - 1);
    const ind2 = Math.min(ind + 2, points.length - 1);
    const ind3 = Math.min(ind + 3, points.length - 1);
    const spline = getSpline(points[ind], points[ind1], points[ind2], points[ind3]);
    const splineDer = getSplineDerived(points[ind], points[ind1], points[ind2], points[ind3]);
    const p = spline(frac);
    const d = splineDer(frac);
    const dMag = Math.sqrt(d[0]**2 + d[1]**2);
    character.setAttributeNS(null, "cx", p[0]);
    character.setAttributeNS(null, "cy", p[1]);
    charLine.setAttributeNS(null, "x1", p[0]);
    charLine.setAttributeNS(null, "y1", p[1]);
    charLine.setAttributeNS(null, "x2", p[0] + d[0] / dMag * directionLength * 2);
    charLine.setAttributeNS(null, "y2", p[1] + d[1] / dMag * directionLength * 2);
}

svg.addEventListener("mousedown", onMouseDown);
svg.addEventListener("mouseup", onMouseUp);
svg.addEventListener("mousemove", onMouseMove);
window.addEventListener("keyup", onKeyUp);
window.addEventListener("keydown", onKeyDown);