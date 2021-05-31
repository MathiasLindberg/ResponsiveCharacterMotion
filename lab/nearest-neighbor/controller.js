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

function onMouseClick(e) {
    if (featureCount != 2) return;
    const x = minVal + e.clientX / width * (maxVal - minVal);
    const y = minVal + (height - e.clientY) / height * (maxVal - minVal);
    const [target, dist] = tree.searchTree([x,y]);
    if (DEBUG) console.log("best was " + target.label + " with distance " + dist);
    if (DEBUG_DRAW) {
        let overlay = null;
        setTimeout(() => overlay = addPoint(featureToCanvas(target.featureVector[0], target.featureVector[1]), 5, "transparent", "red", 10), 500);
        setTimeout(() => overlay.remove(), 1500);
    }
    
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