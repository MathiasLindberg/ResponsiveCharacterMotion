const svg = document.querySelector("#svg");
const text = document.querySelector("#text");
const subtext = document.querySelector("#subtext");
const kd = document.querySelector("#kd");
const kp = document.querySelector("#kp");
const mass = document.querySelector("#mass");
let line1 = document.createElementNS("http://www.w3.org/2000/svg", "line");
let circle = document.createElementNS("http://www.w3.org/2000/svg", "circle");
line1.setAttributeNS(null, "x1", 150);
line1.setAttributeNS(null, "y1", 150);
line1.setAttributeNS(null, "x2", 150);
line1.setAttributeNS(null, "y2", 150);
line1.setAttributeNS(null, "style", "stroke:black;stroke-width:2");
circle.setAttributeNS(null, "r", 5);
circle.setAttributeNS(null, "stroke", "black");
circle.setAttributeNS(null, "fill", "grey");
svg.appendChild(line1);
svg.appendChild(circle);
let running = true;
let fixedTime = 0;
let lastFixedTime = 0;
let pos = {"x":150.0, "y":150.0};
let desiredPos = {"x":150.0, "y":150.0};
let vel = {"x":0.1, "y":0.1};
let control = {"x":0.0, "y":0.0};

function main() {
    document.onkeydown = e => {if (e.code === "Space") running = false;};
    document.onmousemove = e => {desiredPos.x = e.pageX; desiredPos.y = e.pageY;};
    svg.style.cursor = "none";
    requestAnimationFrame(update);
}

function update(t) {
    fixedTime = t * 0.001;
    let deltaTime = fixedTime - lastFixedTime;
    step(deltaTime);
    updateSVG(deltaTime);
    lastFixedTime = fixedTime;
    if (running) requestAnimationFrame(update);
}

// use next step to simulate stable proportional derivative control
function step(deltaTime) {
    const pro_gain = parseFloat(kp.value);
    const dev_gain = parseFloat(kd.value);
    const m = parseFloat(mass.value);
    control.x = (-pro_gain * (pos.x + deltaTime * vel.x - desiredPos.x) + dev_gain * vel.x) / (m + dev_gain * vel.x);
    control.y = (-pro_gain * (pos.y + deltaTime * vel.y - desiredPos.y) + dev_gain * vel.y) / (m + dev_gain * vel.y);
    pos.x += control.x;
    pos.y += control.y;
}

function updateSVG(deltaTime) {
    line1.setAttributeNS(null, "x2", pos.x);
    line1.setAttributeNS(null, "y2", pos.y);
    circle.setAttributeNS(null, "cx", desiredPos.x);
    circle.setAttributeNS(null, "cy", desiredPos.y);
    text.innerHTML = "(" + pos.x.toFixed(2) + ", " + pos.y.toFixed(2) + ")";
    subtext.innerHTML = "kp: " + kp.value + ", kd: " + kd.value + ", mass: " + mass.value;
}

window.onload = main;