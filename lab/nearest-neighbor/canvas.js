const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
document.body.appendChild(svg);
const width = 1200;
const height = 680;
svg.setAttributeNS(null, "width", width);
svg.setAttributeNS(null, "height", height);
function addLine(from, to, color, width) {
    const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
    line.setAttributeNS(null, "x1", from[0]);
    line.setAttributeNS(null, "y1", from[1]);
    line.setAttributeNS(null, "x2", to[0]);
    line.setAttributeNS(null, "y2", to[1]);
    line.setAttributeNS(null, "stroke", color);
    line.setAttributeNS(null, "stroke-width", width);
    svg.appendChild(line);
    return line;
}
function addPoint(pos, rad, innerColor, outerColor, width) {
    const point = document.createElementNS("http://www.w3.org/2000/svg", "circle");
    point.setAttributeNS(null, "cx", pos[0]);
    point.setAttributeNS(null, "cy", pos[1]);
    point.setAttributeNS(null, "r", rad);
    point.setAttributeNS(null, "fill", innerColor);
    point.setAttributeNS(null, "stroke", outerColor);
    point.setAttributeNS(null, "stroke-width", width);
    svg.appendChild(point);
    return point;
}
const borderColor = "black";
const borderWidth = 10;
addLine([0, 0], [width, 0], borderColor, borderWidth);
addLine([width, 0], [width, height], borderColor, borderWidth);
addLine([width, height], [0, height], borderColor, borderWidth);
addLine([0, height], [0, 0], borderColor, borderWidth);
const pointInnerColor = "white";
const pointOuterColor = "black";
const pointRad = 5;
const pointWidth = 2;
function pctToCanvasWidth(pct) {
    return pct * (width - borderWidth - 1) + borderWidth * 0.5;
}
function pctToCanvasHeight(pct) {
    return pct * (height - borderWidth - 1) + borderWidth * 0.5;
}

const lineWidth = 2;
