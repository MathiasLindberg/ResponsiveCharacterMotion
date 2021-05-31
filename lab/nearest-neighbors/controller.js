const cursorLine1 = document.createElementNS("http://www.w3.org/2000/svg", "line");
const cursorLine2 = document.createElementNS("http://www.w3.org/2000/svg", "line");
cursorLine1.setAttributeNS(null, "stroke-linecap", "round");
cursorLine1.setAttributeNS(null, "stroke", "black");
cursorLine1.setAttributeNS(null, "stroke-width", 4);
cursorLine2.setAttributeNS(null, "stroke", "black");
cursorLine2.setAttributeNS(null, "stroke-width", 4);
cursorLine2.setAttributeNS(null, "stroke-linecap", "round");
svg.appendChild(cursorLine1);
svg.appendChild(cursorLine2);
svg.style.cursor = "none";

let alpha = 2;
let candidateCount = 20;

function onMouseClick(e) {
    if (featureCount != 2) return;
    const x = minVal + e.clientX / width * (maxVal - minVal);
    const y = minVal + (height - e.clientY) / height * (maxVal - minVal);
    const candidates = tree.searchTree([x,y], candidateCount, alpha);
    const overlays = [];
    candidates.forEach(e => {
        overlays.push(addPoint(featureToCanvas(e.featureVector[0], e.featureVector[1]), pointRad * 2, "transparent", "black", 3));
    });
    setTimeout(() => overlays.forEach(e => {
        e.remove();
    }), 2000)
}
function onMouseMove(e) {
    cursorLine1.setAttributeNS(null, "x1", e.clientX - 5);
    cursorLine1.setAttributeNS(null, "y1", e.clientY - 5);
    cursorLine1.setAttributeNS(null, "x2", e.clientX + 5);
    cursorLine1.setAttributeNS(null, "y2", e.clientY + 5);
    cursorLine2.setAttributeNS(null, "x1", e.clientX + 5);
    cursorLine2.setAttributeNS(null, "y1", e.clientY - 5);
    cursorLine2.setAttributeNS(null, "x2", e.clientX - 5);
    cursorLine2.setAttributeNS(null, "y2", e.clientY + 5);
}

svg.addEventListener("mouseup", onMouseClick);
svg.addEventListener("mousemove", onMouseMove);

const alphaSlider = document.getElementById("slider_alpha");
const candidateSlider = document.getElementById("slider_candidate");
const alphaText = document.getElementById("text_alpha");
const candidateText = document.getElementById("text_candidate");

function onAlphaChange(s) {
    alpha = s.target.value;
    alphaText.innerText = "alpha: " + alpha;
}
function onCandidateChange(s) {
    candidateCount = s.target.value;
    candidateText.innerText = "max candidate count: " + candidateCount;
}

alphaSlider.addEventListener("change", onAlphaChange);
candidateSlider.addEventListener("change", onCandidateChange);
alphaSlider.value = alpha;
candidateSlider.value = candidateCount;
alphaText.innerText = "alpha: " + alpha;
candidateText.innerText = "max candidate count: " + candidateCount;