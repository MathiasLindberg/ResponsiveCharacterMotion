const featureColors = ["red", "blue"];
const featureCount = featureColors.length;
const minVal = 0;
const maxVal = 7;

const DEBUG = true;
const DEBUG_DRAW = true;

function featureToCanvas(x, y) {
    return [pctToCanvasWidth((x - minVal) / (maxVal - minVal)), height - pctToCanvasHeight((y - minVal) / (maxVal - minVal))];
}
if (DEBUG) {
    for (let i = minVal; i <= maxVal; i++) {
        addLine(featureToCanvas(minVal, i), featureToCanvas(maxVal, i), "grey", 1);
    }
    for (let i = minVal; i <= maxVal; i++) {
        addLine(featureToCanvas(i, minVal), featureToCanvas(i, maxVal), "grey", 1);
    }
}
class KDNode {
    constructor(featureVector) {
        this.featureVector = featureVector;
        this.left = null;
        this.right = null;
    }
    addNode(node, depth, x1, y1, x2, y2) {
        const i = depth % featureCount;
        const horizontal = (i % 2); // whether the line being compared to is horizontal or not
        const thisVal = this.featureVector[i];
        const otherVal = node.featureVector[i];
        if (otherVal < thisVal) {
            if (horizontal) y2 = thisVal;
            else x2 = thisVal;
            if (this.left == null) {
                this.left = node;
                const nextVal = node.featureVector[(depth + 1) % featureCount];
                const nextCol = featureColors[(depth + 1) % featureCount];
                node.color = nextCol;
                if (horizontal) {
                    node.line = addLine(featureToCanvas(nextVal, y1), featureToCanvas(nextVal, y2), nextCol, lineWidth);
                    node.point = addPoint(featureToCanvas(nextVal, node.featureVector[i]), pointRad, nextCol, pointOuterColor, pointWidth);
                }
                else {
                    node.line = addLine(featureToCanvas(x1, nextVal), featureToCanvas(x2, nextVal), nextCol, lineWidth);
                    node.point = addPoint(featureToCanvas(node.featureVector[i], nextVal), pointRad, nextCol, pointOuterColor, pointWidth);
                }
            }
            else this.left.addNode(node, depth + 1, x1, y1, x2, y2);
        }
        else {
            if (horizontal) y1 = thisVal;
            else x1 = thisVal;
            if (this.right == null) {
                this.right = node;
                const nextVal = node.featureVector[(depth + 1) % featureCount];
                const nextCol = featureColors[(depth + 1) % featureCount];
                node.color = nextCol;
                if (horizontal) {
                    node.line = addLine(featureToCanvas(nextVal, y1), featureToCanvas(nextVal, y2), nextCol, lineWidth);
                    node.point = addPoint(featureToCanvas(nextVal, node.featureVector[i]), pointRad, nextCol, pointOuterColor, pointWidth);
                }
                else {
                    node.line = addLine(featureToCanvas(x1, nextVal), featureToCanvas(x2, nextVal), nextCol, lineWidth);
                    node.point = addPoint(featureToCanvas(node.featureVector[i], nextVal), pointRad, nextCol, pointOuterColor, pointWidth);    
                }
            }
            else this.right.addNode(node, depth + 1, x1, y1, x2, y2);
        }
    }
    squareDistanceTo(featureVector) {
        let distance = 0;
        for (let i = 0; i < featureCount; i++) distance += (this.featureVector[i] - featureVector[i]) * (this.featureVector[i] - featureVector[i]);
        return distance;
    }
    searchTree(queryVector, depth, bestNode, bestDist) {
        const i = depth % featureCount;
        const thisVal = this.featureVector[i];
        const otherVal = queryVector[i];
        let candidate = this;
        let otherCandidate = null;
        let line = null;
        let dist = Number.MAX_VALUE;
        if (DEBUG) console.log("nearest(" + this.label + ", (" + queryVector[0] + ", " + queryVector[1] + ")) - " + (Math.sqrt(this.squareDistanceTo(queryVector))).toFixed(1), "best is", bestNode.label, "with", (Math.sqrt(bestDist).toFixed(1)));
        if (DEBUG_DRAW) setTimeout(() => {
            this.point.setAttributeNS(null, "fill", "yellow");
            line = addLine(featureToCanvas(this.featureVector[0], this.featureVector[1]), featureToCanvas(candidate.featureVector[0], candidate.featureVector[1]), "green", 3);
        }, 500 * depth);
        if (DEBUG_DRAW) setTimeout(() => {
            this.point.setAttributeNS(null, "fill", "black");
            line.remove();
        }, 1000 + 500 * depth);
        const candidateDist = candidate.squareDistanceTo(queryVector);
        if (DEBUG) console.log("is", candidate.label, "better?");
        if (candidateDist < bestDist) {
            if (DEBUG) console.log(candidate.label, "is now best");
            bestDist = candidateDist;
            bestNode = candidate;
        }
        if (otherVal < thisVal) {
            if (DEBUG) console.log("going left from " + this.label, "best", (Math.sqrt(bestDist).toFixed(1)));
            if (this.left != null) {
                [candidate, dist] = this.left.searchTree(queryVector, depth + 1, bestNode, bestDist);
                if (DEBUG) console.log((Math.sqrt(bestDist).toFixed(1)));
                if (candidate != null && dist < bestDist) {
                    bestDist = dist;
                    bestNode = candidate;
                }
            }
            otherCandidate = this.right;
        }
        else {
            if (DEBUG) console.log("going right from " + this.label, "best", (Math.sqrt(bestDist).toFixed(1)));
            if (this.right != null) {
                [candidate, dist] = this.right.searchTree(queryVector, depth + 1, bestNode, bestDist);
                if (candidate != null && dist < bestDist) {
                    bestDist = dist;
                    bestNode = candidate;
                    if (DEBUG) console.log("returning new closest dist", (Math.sqrt(bestDist).toFixed(1)));
                }
            }
            otherCandidate = this.left;
        }
        
        let diff = this.featureVector[i] - otherVal;
        if (DEBUG) {
            console.log("comparing direct dist", "(" + this.featureVector[i] + " - " + otherVal + ")", "to", Math.sqrt(bestDist), "(" + ((Math.abs(diff) < Math.sqrt(bestDist)) ? "better" : "worse") + ")");
            if (otherCandidate == null) console.log("stopping at null");
        }
        if (otherCandidate != null && (diff * diff) < bestDist) {
            if (DEBUG) console.log("checking out " + otherCandidate.label);
            [candidate, dist] = otherCandidate.searchTree(queryVector, depth + 1, bestNode, bestDist);
            if (bestNode != null && dist < bestDist) {
                bestDist = dist;
                bestNode = candidate;
            }
        }
        if (DEBUG) console.log("returning", bestNode.label, (Math.sqrt(bestDist).toFixed(1)));
        return [bestNode, bestDist];
    }
}

class KDTree {
    root = null;
    addNode(node) {
        if (this.root == null) {
            this.root = node;
            const pos = featureToCanvas(node.featureVector[0], node.featureVector[1]);
            node.line = addLine([pos[0], pctToCanvasHeight(0)], [pos[0], pctToCanvasHeight(1)], featureColors[0], lineWidth);
            node.point = addPoint(pos, pointRad, featureColors[0], pointOuterColor, pointWidth);
            node.color = featureColors[0];
        } 
        else this.root.addNode(node, 0, minVal, minVal, maxVal, maxVal);
    }
    searchTree(queryVector) {
        if (DEBUG) queryVector = [Math.round(queryVector[0]), Math.round(queryVector[1])];
        if (DEBUG) console.log("\nquerying:", queryVector);
        let result = [null, Number.MAX_VALUE];
        if (this.root == null) return result;
        else {
            result = this.root.searchTree(queryVector, 0, this.root, Number.MAX_VALUE);
            result[1] = Math.sqrt(result[1]);
            return result;
        }
    }
}
const tree = new KDTree();

if (DEBUG) {
    let node = new KDNode([2,3]);
    node.label = "A";
    tree.addNode(node);
    node = new KDNode([4,2]);
    node.label = "B";
    tree.addNode(node);
    node = new KDNode([4,5]);
    node.label = "C";
    tree.addNode(node);
    node = new KDNode([3,3]);
    node.label = "D";
    tree.addNode(node);
    node = new KDNode([1,5]);
    node.label = "E";
    tree.addNode(node);
    node = new KDNode([4,4]);
    node.label = "F";
    tree.addNode(node);
}
else {
    for (let i = 0; i < 100; i++) {
        let features = [];
        for (let j = 0; j < featureCount; j++) features.push(minVal + Math.random() * (maxVal - minVal));
        tree.addNode(new KDNode(features));
    }
}

