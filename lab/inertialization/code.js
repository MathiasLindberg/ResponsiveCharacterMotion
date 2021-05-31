const bt = document.querySelector("#bt");
const w = document.querySelector("#w");
const h = document.querySelector("#h");
const btText = document.querySelector("#bt_text");
const targetText = document.querySelector("#target_text");
const svg = document.querySelector("#svg");
let width = svg.getAttribute("width");
let height = svg.getAttribute("height");
const lineCount = 100;
const targetCircle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
const fromCircle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
const line1 = document.createElementNS("http://www.w3.org/2000/svg", "line");
const line2 = document.createElementNS("http://www.w3.org/2000/svg", "line");
const line3 = document.createElementNS("http://www.w3.org/2000/svg", "line");
const line4 = document.createElementNS("http://www.w3.org/2000/svg", "line");
const line5 = document.createElementNS("http://www.w3.org/2000/svg", "line");
const valText = document.createElementNS("http://www.w3.org/2000/svg", "text");
line1.setAttributeNS(null, "style", "stroke:black;stroke-width:4");
line2.setAttributeNS(null, "style", "stroke:black;stroke-width:4");
line3.setAttributeNS(null, "style", "stroke:black;stroke-width:4");
line4.setAttributeNS(null, "style", "stroke:black;stroke-width:4");
line5.setAttributeNS(null, "style", "stroke:black;stroke-width:1");
line1.setAttributeNS(null, "x1", 0);
line1.setAttributeNS(null, "x2", width);
line1.setAttributeNS(null, "y1", 0);
line1.setAttributeNS(null, "y2", 0);
line2.setAttributeNS(null, "x1", width);
line2.setAttributeNS(null, "x2", width);
line2.setAttributeNS(null, "y1", 0);
line2.setAttributeNS(null, "y2", height);
line3.setAttributeNS(null, "x1", width);
line3.setAttributeNS(null, "x2", 0);
line3.setAttributeNS(null, "y1", height);
line3.setAttributeNS(null, "y2", height);
line4.setAttributeNS(null, "x1", 0);
line4.setAttributeNS(null, "x2", 0);
line4.setAttributeNS(null, "y1", height);
line4.setAttributeNS(null, "y2", 0);
line5.setAttributeNS(null, "x1", 0);
line5.setAttributeNS(null, "x2", width);
line5.setAttributeNS(null, "y1", height * 0.5);
line5.setAttributeNS(null, "y2", height * 0.5);
valText.setAttributeNS(null, "fill", "black");
svg.appendChild(line1);
svg.appendChild(line2);
svg.appendChild(line3);
svg.appendChild(line4);
svg.appendChild(line5);
svg.appendChild(valText);

targetCircle.setAttributeNS(null, "stroke", "black");
targetCircle.setAttributeNS(null, "fill", "yellow");
fromCircle.setAttributeNS(null, "stroke", "black");
fromCircle.setAttributeNS(null, "fill", "blue");
let targetX = width - 10;
let targetY = 100;
let fromX = 10;
let fromY = -100;
let circleRad = 10;
let lastFixedTime = 0;
let running = true;
let lines = [];
let follow = -1;
let x = 0;
let hoverValueID = -1;
for (let i = 0; i < lineCount; i++) {
    let line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttributeNS(null, "style", "stroke:black;stroke-width:2");
    line.setAttributeNS(null, "x1", x);
    line.setAttributeNS(null, "y1", height * 0.5);
    x = i / lineCount * width;
    line.setAttributeNS(null, "x2", x);
    line.setAttributeNS(null, "y2", height * 0.5);
    svg.appendChild(line);
    lines.push(line);
}
targetCircle.setAttributeNS(null, "cx", targetX);
targetCircle.setAttributeNS(null, "cy", height * 0.5 - targetY);
targetCircle.setAttributeNS(null, "r", circleRad);
svg.appendChild(targetCircle);
fromCircle.setAttributeNS(null, "cx", fromX);
fromCircle.setAttributeNS(null, "cy", height * 0.5 - fromY);
fromCircle.setAttributeNS(null, "r", circleRad);
svg.appendChild(fromCircle);


function main() {
    document.onkeydown = e => {if (e.code === "Space") running = false;};
    document.onmousedown = e => {
        const rect = svg.getBoundingClientRect();
        let xPos = parseFloat(targetCircle.getAttributeNS(null, "cx")) + rect.x;
        let yPos = parseFloat(targetCircle.getAttributeNS(null, "cy")) + rect.y;
        if (((xPos - e.clientX) * (xPos - e.clientX) + (yPos - e.clientY) * (yPos - e.clientY)) < circleRad * circleRad) follow = 0;
        else {
            xPos = parseFloat(fromCircle.getAttributeNS(null, "cx")) + rect.x;
            yPos = parseFloat(fromCircle.getAttributeNS(null, "cy")) + rect.y;
            if (((xPos - e.clientX) * (xPos - e.clientX) + (yPos - e.clientY) * (yPos - e.clientY)) < circleRad * circleRad) follow = 1;
        }
    }
    document.onmouseup = () => follow = -1;
    document.onmousemove = e => {
        if (follow == 0)  {
            targetY = height * 0.5 - Math.max(0, Math.min(height, e.clientY + circleRad));
            targetCircle.setAttributeNS(null, "cy", height * 0.5 - targetY);
        }
        else if (follow == 1) {
            fromY = height * 0.5 - Math.max(0, Math.min(height, e.clientY + circleRad));
            fromCircle.setAttributeNS(null, "cy", height * 0.5 - fromY);
        }
        else {
            let closestID = -1;
            let closestDist = 1000000;
            let dist, x, y;
            const rect = svg.getBoundingClientRect();
            for (let i = 0; i < lineCount; i++) {
                x = parseFloat(lines[i].getAttributeNS(null, "x2")) + rect.x;
                y = parseFloat(lines[i].getAttributeNS(null, "y2")) + rect.y;
                dist = (x - e.clientX) * (x - e.clientX) + (y - e.clientY) * (y - e.clientY);
                if (dist < closestDist) {
                    closestDist = dist;
                    closestID = i;
                }
            }
            if (closestDist < circleRad * circleRad) {
                if (hoverValueID != closestID) {
                    hoverValueID = closestID;
                    x = parseFloat(lines[hoverValueID].getAttributeNS(null, "x2"));
                    y = height * 0.5 - (lines[hoverValueID].getAttributeNS(null, "y2"));
                    valText.innerHTML = "(" + x.toFixed(0) + ", " + y.toFixed(0) + ")";
                    valText.setAttributeNS(null, "x", e.clientX - 50);
                    valText.setAttributeNS(null, "y", e.clientY - 20);
                }
            }
            else if (hoverValueID >= 0) {
                hoverValueID = -1;
                valText.innerHTML = "";
            }
        }
    }
    requestAnimationFrame(update);
}

function update(t) {
    fixedTime = t * 0.001;
    let deltaTime = fixedTime - lastFixedTime;
    step(deltaTime);
    updateSVG(deltaTime);
    lastFixedTime = fixedTime;
    btText.innerHTML = "blend time: " + (bt.value * 0.01);
    targetText.innerHTML = "target value: " + targetY;
    width = parseFloat(w.value);
    height = parseFloat(h.value);
    targetX = width - 10;
    if (running) requestAnimationFrame(update);
}

function step(deltaTime) {
}

function updateSVG(deltaTime) {
    svg.setAttributeNS(null, "width", width);
    svg.setAttributeNS(null, "height", height);
    line1.setAttributeNS(null, "x2", width);
    line2.setAttributeNS(null, "x1", width);
    line2.setAttributeNS(null, "x2", width);
    line2.setAttributeNS(null, "y2", height);
    line3.setAttributeNS(null, "x1", width);
    line3.setAttributeNS(null, "y1", height);
    line3.setAttributeNS(null, "y2", height);
    line4.setAttributeNS(null, "y1", height);
    line5.setAttributeNS(null, "x2", width);
    line5.setAttributeNS(null, "y1", height * 0.5);
    line5.setAttributeNS(null, "y2", height * 0.5);
    targetCircle.setAttributeNS(null, "cx", targetX);
    targetCircle.setAttributeNS(null, "cy", height * 0.5 - targetY);
    fromCircle.setAttributeNS(null, "cx", fromX);
    fromCircle.setAttributeNS(null, "cy", height * 0.5 - fromY);
    let prevY = fromY;
    let y = fromY;
    let x = fromX;
    const dt = 1.0 / lineCount;
    const blendTime = bt.value * 0.01;
    const velocity = -20.0;
    for (let i = 0; i < lineCount; i++) {
        lines[i].setAttributeNS(null, "y1", height * 0.5 - y);
        y = inertialize((fromY - targetY), velocity, blendTime, i / lineCount) + targetY;
        //y = testInertialize(fromY - targetY, velocity, blendTime, i / lineCount) + targetY;
        //prevY = y;
        lines[i].setAttributeNS(null, "y2", height * 0.5 - y);
        lines[i].setAttributeNS(null, "x1", x);
        x = i / lineCount * width;
        lines[i].setAttributeNS(null, "x2", x);
    }
}

function inertialize(val, velocity, blendTime, t) {
    if (t >= blendTime) return 0; // no need to calculate the blend when time t is larger or equal to the blend time
    const negate = val < 0;
    if (negate) val = -val; // this method only works with positive values, negate input and output if value is negative
    if (velocity > 0) velocity = 0; // we want velocity to always decrease towards the target value
    blendTime = velocity == 0 ? 0 : Math.min(blendTime, -5.0 * val / velocity); // avoid overshoot
    let acc = (-8.0 * velocity * blendTime - 20.0 * val) / (blendTime * blendTime);
    if (acc < 0) acc = 0; // we want acceleration to counter-act the velocity
    let res = val + velocity * t; // + 0th + 1st polynomial
    let tN = t * t; // t^2
    res += acc / 2.0 * tN; // 2nd polynomial
    let blendTimeN = blendTime * blendTime; // blendTime^2
    let jerk = acc * blendTimeN;
    blendTimeN *= blendTime; // blendTime^3
    let inc = velocity * blendTime;
    tN *= t; // t^3
    res -= ((3.0 * jerk + 12.0 * inc + 20.0 * val) / (2.0 * blendTimeN)) * tN; // 3rd polynomial
    tN *= t; // t^4
    blendTimeN *= blendTime; // blendTime^4
    res += ((3.0 * jerk + 16.0 * inc + 30.0 * val) / (2.0 * blendTimeN)) * tN; // 4th polynomial
    tN *= t; // t^5
    blendTimeN *= blendTime; // blendTime^5
    res -= ((jerk + 6.0 * inc + 12.0 * val) / (2.0 * blendTimeN)) * tN; // 5th polynomial
    return negate ? -res : res;
}

window.onload = main;