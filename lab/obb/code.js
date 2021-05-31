const svg = document.getElementById("svg");
const minSlider = document.getElementById("slider_min");
const maxSlider = document.getElementById("slider_max");
const minText = document.getElementById("text_min");
const maxText = document.getElementById("text_max");
const capsule1 = {position:[150, 150], height:100, radius:25, rotation:0, color:"orange", id:0};
const capsule2 = {position:[300, 300], height:100, radius:25, rotation:0, color:"blue", id:0};
const cursor = document.createElementNS("http://www.w3.org/2000/svg", "circle");
cursor.setAttributeNS(null, "r", 5);
cursor.setAttributeNS(null, "stroke", "black");

const rad2Deg = 57.2958;

function rotateBy(vec, rad) {
    return [Math.cos(rad) * vec[0] - Math.sin(rad) * vec[1], Math.sin(rad) * vec[0] + Math.cos(rad) * vec[1]];
}
function lerp(val1, val2, t) {
    return val1 + (val2 - val1) * t;
}
function lerpVectors(vec1, vec2, t) {
    return [lerp(vec1[0], vec2[0], t), lerp(vec1[1], vec2[1], t), lerp(vec1[2], vec2[2], t)];
}
function clamp(val, min, max) {
    return Math.max(Math.min(val, max), min);
}

function createOutline(box, lines = null) {
    let upperLeft = [box.position[0] - box.dimensions[0] * 0.5, box.position[1] - box.dimensions[1] * 0.5];
    let lowerRight = [box.position[0] + box.dimensions[0] * 0.5, box.position[1] + box.dimensions[1] * 0.5];
    let p1 = upperLeft;
    let p2 = [lowerRight[0], upperLeft[1]];
    let p3 = [upperLeft[0], lowerRight[1]];
    let p4 = lowerRight;
    if (lines == null) {
        const l1 = document.createElementNS("http://www.w3.org/2000/svg", "line");
        const l2 = document.createElementNS("http://www.w3.org/2000/svg", "line");
        const l3 = document.createElementNS("http://www.w3.org/2000/svg", "line");
        const l4 = document.createElementNS("http://www.w3.org/2000/svg", "line");
        svg.appendChild(l1);
        svg.appendChild(l2);
        svg.appendChild(l3);
        svg.appendChild(l4);
        lines = [l1, l2, l3, l4];
    }
    if (box.rotation != 0) { // OBB
        p1 = rotateBy([box.position[0] - p1[0], box.position[1] - p1[1]], box.rotation);
        p1 = [box.position[0] + p1[0], box.position[1] + p1[1]];
        p2 = rotateBy([box.position[0] - p2[0], box.position[1] - p2[1]], box.rotation);
        p2 = [box.position[0] + p2[0], box.position[1] + p2[1]];
        p3 = rotateBy([box.position[0] - p3[0], box.position[1] - p3[1]], box.rotation);
        p3 = [box.position[0] + p3[0], box.position[1] + p3[1]];
        p4 = rotateBy([box.position[0] - p4[0], box.position[1] - p4[1]], box.rotation);
        p4 = [box.position[0] + p4[0], box.position[1] + p4[1]];
    }
    lines[0].setAttributeNS(null, "x1", p1[0]);
    lines[0].setAttributeNS(null, "y1", p1[1]);
    lines[0].setAttributeNS(null, "x2", p2[0]);
    lines[0].setAttributeNS(null, "y2", p2[1]);
    lines[1].setAttributeNS(null, "x1", p3[0]);
    lines[1].setAttributeNS(null, "y1", p3[1]);
    lines[1].setAttributeNS(null, "x2", p4[0]);
    lines[1].setAttributeNS(null, "y2", p4[1]);
    lines[2].setAttributeNS(null, "x1", p1[0]);
    lines[2].setAttributeNS(null, "y1", p1[1]);
    lines[2].setAttributeNS(null, "x2", p3[0]);
    lines[2].setAttributeNS(null, "y2", p3[1]);
    lines[3].setAttributeNS(null, "x1", p2[0]);
    lines[3].setAttributeNS(null, "y1", p2[1]);
    lines[3].setAttributeNS(null, "x2", p4[0]);
    lines[3].setAttributeNS(null, "y2", p4[1]);
    lines[0].setAttributeNS(null, "stroke-width", 2);
    lines[1].setAttributeNS(null, "stroke-width", 2);
    lines[2].setAttributeNS(null, "stroke-width", 2);
    lines[3].setAttributeNS(null, "stroke-width", 2);
    lines[0].setAttributeNS(null, "stroke", "black");
    lines[1].setAttributeNS(null, "stroke", "black");
    lines[2].setAttributeNS(null, "stroke", "black");
    lines[3].setAttributeNS(null, "stroke", "black");
    lines[0].setAttributeNS(null, "stroke-linecap", "round");
    lines[1].setAttributeNS(null, "stroke-linecap", "round");
    lines[2].setAttributeNS(null, "stroke-linecap", "round");
    lines[3].setAttributeNS(null, "stroke-linecap", "round");
    return lines;
}
function createCapsule(capsule, path = null) {
    let d = "M " + (-capsule.height * 0.5) + " " + (-capsule.radius);
    d += " L " + (capsule.height * 0.5) + " " + (-capsule.radius);
    let cx = capsule.height * 0.5;
    let cy = 0;
    const circlePoints = 15;
    for (let i = 0; i < circlePoints; i++) {
        const r = Math.PI * i / circlePoints;
        const x = cx + Math.cos(r - Math.PI * 0.5) * capsule.radius;
        const y = cy + Math.sin(r - Math.PI * 0.5) * capsule.radius;
        d += " L " + x + " " + y;
    }
    d += " L " + (capsule.height * 0.5) + " " + (capsule.radius);
    d += " L " + (-capsule.height * 0.5) + " " + (capsule.radius);
    cx = -capsule.height * 0.5;
    cy = 0;
    for (let i = circlePoints; i >= 0; i--) {
        const r = Math.PI * i / circlePoints;
        const x = cx + Math.cos(-r - Math.PI * 0.5) * capsule.radius;
        const y = cy + Math.sin(-r - Math.PI * 0.5) * capsule.radius;
        d += " L " + x + " " + y;
    }
    d += " L " + (-capsule.height * 0.5) + " " + (-capsule.radius);
    if (path == null) {
        path = document.createElementNS("http://www.w3.org/2000/svg", "path");
        svg.appendChild(path);
    }
    path.setAttributeNS(null, "d", d);
    path.setAttributeNS(null, "stroke-width", 2);
    path.setAttributeNS(null, "stroke", "black");
    path.setAttributeNS(null, "fill", capsule.color);
    path.setAttributeNS(null, "transform", "translate(" + capsule.position[0] + "," + capsule.position[1] + ")");
    return path;
}
function capsuleAABB(capsule) {
    const center1 = [capsule.position[0] - Math.cos(capsule.rotation) * capsule.height * 0.5, capsule.position[1] - Math.sin(capsule.rotation) * capsule.height * 0.5];
    const center2 = [capsule.position[0] + Math.cos(capsule.rotation) * capsule.height * 0.5, capsule.position[1] + Math.sin(capsule.rotation) * capsule.height * 0.5];
    const upperLeft = [0, 0];
    const lowerRight = [0, 0];
    if (center1[0] < center2[0]) {
        upperLeft[0] = center1[0] - capsule.radius;
        lowerRight[0] = center2[0] + capsule.radius;
    }
    else {
        upperLeft[0] = center2[0] - capsule.radius;
        lowerRight[0] = center1[0] + capsule.radius;
    }
    if (center1[1] < center2[1]) {
        upperLeft[1] = center1[1] - capsule.radius;
        lowerRight[1] = center2[1] + capsule.radius;
    }
    else {
        upperLeft[1] = center2[1] - capsule.radius;
        lowerRight[1] = center1[1] + capsule.radius;
    }
    return {position:[upperLeft[0] + (lowerRight[0] - upperLeft[0]) * 0.5, upperLeft[1] + (lowerRight[1] - upperLeft[1]) * 0.5], 
            dimensions:[lowerRight[0] - upperLeft[0], lowerRight[1] - upperLeft[1]], 
            rotation:0, id:capsule.id};
}
function capsuleOBB(capsule) {
    const center1 = [capsule.position[0] - capsule.height * 0.5, capsule.position[1]];
    const center2 = [capsule.position[0] + capsule.height * 0.5, capsule.position[1]];
    const upperLeft = [center1[0] - capsule.radius, center1[1] - capsule.radius];
    const lowerRight = [center2[0] + capsule.radius, center2[1] + capsule.radius];
    return {position:[upperLeft[0] + (lowerRight[0] - upperLeft[0]) * 0.5, upperLeft[1] + (lowerRight[1] - upperLeft[1]) * 0.5], 
            dimensions:[lowerRight[0] - upperLeft[0], lowerRight[1] - upperLeft[1]], 
            rotation:capsule.rotation, id:capsule.id};
}
function createLinesBetween(lines1, lines2, output = null) {
    const min = minSlider.value;
    const max = maxSlider.value;
    minText.innerText = min;
    maxText.innerText = max;
    if (output == null) {
        output = [];
        for (let i = 0; i < lines1.length; i++) {
            const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
            line.setAttributeNS(null, "stroke-width", 5);
            line.setAttributeNS(null, "stroke-linecap", "round");
            svg.appendChild(line);
            output.push(line);
        }
    }
    for (let i = 0; i < lines1.length; i++) {
        const l1 = lines1[i];
        const l2 = lines2[i];
        const x1 = lerp(parseFloat(l1.getAttributeNS(null, "x1")), parseFloat(l1.getAttributeNS(null, "x2")), 0.5);
        const y1 = lerp(parseFloat(l1.getAttributeNS(null, "y1")), parseFloat(l1.getAttributeNS(null, "y2")), 0.5);
        const x2 = lerp(parseFloat(l2.getAttributeNS(null, "x1")), parseFloat(l2.getAttributeNS(null, "x2")), 0.5);
        const y2 = lerp(parseFloat(l2.getAttributeNS(null, "y1")), parseFloat(l2.getAttributeNS(null, "y2")), 0.5);
        output[i].setAttributeNS(null, "x1", x1);
        output[i].setAttributeNS(null, "y1", y1);
        output[i].setAttributeNS(null, "x2", x2);
        output[i].setAttributeNS(null, "y2", y2);
        const dist = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        const col = lerpVectors([0, 255, 0], [255, 0, 0], clamp((dist - min) / max, 0, 1));
        output[i].setAttributeNS(null, "stroke", "rgb(" + col[0] + "," + col[1] + "," + col[2] + ")");
    }

    return output;
}


const shapes = [];
const colliders = [];
const graphics = [];
const outlines = [];
const distanceLines = [];

const content1 = createCapsule(capsule1);
const aabb1 = capsuleAABB(capsule1);
const outline1 = createOutline(aabb1);
capsule1.id = shapes.length;
aabb1.id = capsule1.id;
shapes.push(capsule1);
colliders.push(aabb1);
graphics.push(content1)
outlines.push(outline1);
const content2 = createCapsule(capsule2);
const aabb2 = capsuleAABB(capsule2);
const outline2 = createOutline(aabb2);
const distances = createLinesBetween(outline1, outline2);
capsule2.id = shapes.length;
aabb2.id = capsule2.id;
shapes.push(capsule2);
colliders.push(aabb2);
graphics.push(content2)
outlines.push(outline2);
distanceLines.push(distances);

let translate = false;
let targetID = -1;
let rotate = false;
let mouseX = 0;
let mouseY = 0;
let offsetX = 0;
let offsetY = 0;
let useOBB = false;

function onMouseDown(e) {
    if (e.button == 0) translate = true;
    else if (e.button == 2) rotate = true;
    targetID = getClosestCollidingObject(mouseX, mouseY);
}
function onMouseUp(e) {
    if (e.button == 0) translate = false;
    else if (e.button == 2) rotate = false;
    targetID = -1;
}
function onMouseMove(e) {
    mouseX = e.clientX;
    mouseY = e.clientY;
    cursor.setAttributeNS(null, "cx", mouseX);
    cursor.setAttributeNS(null, "cy", mouseY);
}

function getClosestCollidingObject(x, y) {
    const candidates = [];
    for (let i = 0; i < colliders.length; i++) {
        const collider = colliders[i];
        const upperLeft = [collider.position[0] - collider.dimensions[0] * 0.5, collider.position[1] - collider.dimensions[1] * 0.5];
        const lowerRight = [collider.position[0] + collider.dimensions[0] * 0.5, collider.position[1] + collider.dimensions[1] * 0.5];
        if (collider.rotation == 0) { // AABB
            if (x >= upperLeft[0] && x <= lowerRight[0] && y >= upperLeft[1] && y <= lowerRight[1]) candidates.push(collider);
        }
        else {
            const diff = rotateBy([collider.position[0] - x, collider.position[1] - y], -collider.rotation);
            const newX = collider.position[0] + diff[0];
            const newY = collider.position[1] + diff[1];
            if (newX >= upperLeft[0] && newX <= lowerRight[0] && newY >= upperLeft[1] && newY <= lowerRight[1]) candidates.push(collider);
        }
    }
    let closestID = -1;
    let closestDist = Number.MAX_VALUE;
    let candXDiff = 0;
    let candYDiff = 0;
    for (let i = 0; i < candidates.length; i++) {
        const collider = candidates[i];
        const xDiff = collider.position[0] - x;
        const yDiff = collider.position[1] - y;
        const dist = xDiff * xDiff + yDiff * yDiff;
        if (dist < closestDist) {
            closestDist = dist;
            closestID = i;
            candXDiff = xDiff;
            candYDiff = yDiff;
        }
    }
    offsetX = candXDiff;
    offsetY = candYDiff;
    return closestID == -1 ? -1 : candidates[closestID].id;
}

function onKeyUp(e) {
    if (e.keyCode == 32) useOBB = !useOBB;
}

function update() {
    if (targetID != -1) {
        if (rotate) {
            shapes[targetID].rotation += Math.PI * 0.025;
        }
        if (translate) {
            shapes[targetID].position[0] = mouseX + offsetX;
            shapes[targetID].position[1] = mouseY + offsetY;
            colliders[targetID].position[0] = mouseX + offsetX;
            colliders[targetID].position[1] = mouseY + offsetY;
        }
        colliders[targetID] = useOBB ? capsuleOBB(shapes[targetID]) : capsuleAABB(shapes[targetID]);
        createOutline(colliders[targetID], outlines[targetID]);
        if (targetID != -1) graphics[targetID].setAttributeNS(null, "transform", "rotate(" + (shapes[targetID].rotation * rad2Deg) + " " + shapes[targetID].position[0] + " " + shapes[targetID].position[1] + ") " +
                                                                                 "translate(" + shapes[targetID].position[0] + ", " + shapes[targetID].position[1] + ")");
    }
    createLinesBetween(outlines[0], outlines[1], distanceLines[0]);
    requestAnimationFrame(update);
}
function start() {
    requestAnimationFrame(update);
}

svg.appendChild(cursor);

window.addEventListener("mousedown", onMouseDown);
window.addEventListener("mouseup", onMouseUp);
window.addEventListener("mousemove", onMouseMove);
window.addEventListener("keyup", onKeyUp);
window.oncontextmenu = (e) => e.preventDefault();
window.onload = start;
svg.style.cursor = "none";