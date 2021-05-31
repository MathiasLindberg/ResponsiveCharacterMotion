const svg = document.getElementById("svg");
const text = document.getElementById("text");
const center = [150, 50];
const p0 = document.createElementNS("http://www.w3.org/2000/svg", "circle");
p0.setAttributeNS(null, "cx", center[0]);
p0.setAttributeNS(null, "cy", center[1]);
p0.setAttributeNS(null, "r", 5);
svg.appendChild(p0);
const p1 = document.createElementNS("http://www.w3.org/2000/svg", "circle");
p1.setAttributeNS(null, "cx", center[0] + 50);
p1.setAttributeNS(null, "cy", center[1]);
p1.setAttributeNS(null, "r", 5);
svg.appendChild(p1);
const p2 = document.createElementNS("http://www.w3.org/2000/svg", "circle");
p2.setAttributeNS(null, "cx", center[0] - 50);
p2.setAttributeNS(null, "cy", center[1]);
p2.setAttributeNS(null, "r", 5);
svg.appendChild(p2);
const l1 = document.createElementNS("http://www.w3.org/2000/svg", "line");
l1.style.strokeWidth = 3;
l1.style.stroke = "green";
l1.setAttributeNS(null, "x1", center[0]);
l1.setAttributeNS(null, "y1", center[1]);
svg.appendChild(l1);
const l2 = document.createElementNS("http://www.w3.org/2000/svg", "line");
l2.style.strokeWidth = 3;
l2.style.stroke = "red";
l2.setAttributeNS(null, "x1", center[0]);
l2.setAttributeNS(null, "y1", center[1]);
svg.appendChild(l2);

let mouseDown = false;
let markedPoint = p1;
let markedLine = l1;

const rad2Deg = 57.2958;

function update() {
    let x1 = (p1.getAttributeNS(null, "cx") - center[0]) / 50;
    let y1 = (p1.getAttributeNS(null, "cy") - center[1]) / 50;
    let x2 = (p2.getAttributeNS(null, "cx") - center[0]) / 50;
    let y2 = (p2.getAttributeNS(null, "cy") - center[1]) / 50;
    angle = shortestSignedAngleBetween([x1, y1], [x2, y2]);
    text.innerText = "Shortest angle: " + (angle * rad2Deg).toFixed(2);
}

function rotateBy(vec, rad) {
    return [Math.cos(rad) * vec[0] - Math.sin(rad) * vec[1], Math.sin(rad) * vec[0] + Math.cos(rad) * vec[1]];
}

function shortestSignedAngleBetween(vec1, vec2) {
    let vecDiff = rotateBy(vec2, -Math.atan2(vec1[1], vec1[0]));
    let angle = Math.acos(vecDiff[0]);
    return Math.atan2(vecDiff[1], vecDiff[0]) < 0 ? angle : -angle;
}

function movePoint(e) {
    if (mouseDown) {
        x = Math.min(Math.max(e.clientX - 8, 0), svg.getAttributeNS(null, "width")) - center[0];
        y = Math.min(Math.max(e.clientY - 8, 0), svg.getAttributeNS(null, "height")) - center[1];
        magnitude = Math.sqrt(x * x + y * y);
        x = center[0] + x / magnitude * 50;
        y = center[1] + y / magnitude * 50;
        markedPoint.setAttributeNS(null, "cx", x);
        markedPoint.setAttributeNS(null, "cy", y);
        markedLine.setAttributeNS(null, "x2", x);
        markedLine.setAttributeNS(null, "y2", y);
        update();
    }
}

function moveInit(e) {
    x1 = e.clientX - p1.getAttributeNS(null, "cx");
    y1 = e.clientY - p1.getAttributeNS(null, "cy");
    x2 = e.clientX - p2.getAttributeNS(null, "cx");
    y2 = e.clientY - p2.getAttributeNS(null, "cy");
    if (x1 * x1 + y1 * y1 < x2 * x2 + y2 * y2) {
        markedPoint = p1;
        markedLine = l1;
    }
    else {
        markedPoint = p2;
        markedLine = l2;
    }
    mouseDown = true;
}


window.addEventListener("mousedown", moveInit);
window.addEventListener("mouseup", () => mouseDown = false);
window.addEventListener("mousemove", movePoint);