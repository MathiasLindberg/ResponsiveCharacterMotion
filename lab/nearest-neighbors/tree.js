const featureColors = ["red", "blue"];
const featureCount = featureColors.length;
const minVal = 0;
const maxVal = 100;

function featureToCanvas(x, y) {
    return [pctToCanvasWidth((x - minVal) / (maxVal - minVal)), height - pctToCanvasHeight((y - minVal) / (maxVal - minVal))];
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
    searchTree(queryVector, depth, worstCandidateDist, worstCandidateInd, bestDist, alpha, candidates, candidateCount) {
        const i = depth % featureCount;
        const thisVal = this.featureVector[i];
        const otherVal = queryVector[i];
        let candidate = this;
        let otherCandidate = null;
        let subBestDist = Number.MAX_VALUE;
        let subWorstCandDist = Number.MIN_VALUE;
        let subWorstCandInd = -1;
        const candidateDist = candidate.squareDistanceTo(queryVector);
        if (candidates.length < candidateCount) { // not enough candidates are collected, add current
            if (candidateDist > worstCandidateDist) {
                worstCandidateDist = candidateDist;
                worstCandidateInd = candidates.length;
            }
            candidates.push(candidate);
        }
        else if (candidateDist < worstCandidateDist) { // enough candidates are collected, replace if current is closer
            candidates[worstCandidateInd] = candidate;
            for (let i = 0; i < candidateCount; i++) {
                let candDist = candidates[i].squareDistanceTo(queryVector);
                if (candDist > worstCandidateDist) { // new worse candidate
                    worstCandidateInd = i;
                    worstCandidateDist = candDist;
                }
            }
        }
        if (candidateDist < bestDist) { // new best
            bestDist = candidateDist;
        }
        if (otherVal < thisVal) { // left subtree is closer, check it first
            if (this.left != null) {
                [subBestDist, subWorstCandDist, subWorstCandInd] = this.left.searchTree(queryVector, depth + 1, worstCandidateDist, worstCandidateInd, bestDist, alpha, candidates, candidateCount);
                // propagate information found so far
                bestDist = subBestDist;
                worstCandidateDist = subWorstCandDist;
                worstCandidateInd = subWorstCandInd;
            }
            otherCandidate = this.right;
        }
        else { // otherwise check right subtree first
            if (this.right != null) {
                [subBestDist, subWorstCandDist, subWorstCandInd] = this.right.searchTree(queryVector, depth + 1, worstCandidateDist, worstCandidateInd, bestDist, alpha, candidates, candidateCount);
                // propagate information found so far
                bestDist = subBestDist;
                worstCandidateDist = subWorstCandDist;
                worstCandidateInd = subWorstCandInd;
            }
            otherCandidate = this.left;
        }
        
        let diff = this.featureVector[i] - otherVal;
        if (otherCandidate != null && (diff * diff) < bestDist / alpha) { // search unvisited subtree if it passes pruning
            [subBestDist, subWorstCandDist, subWorstCandInd] = otherCandidate.searchTree(queryVector, depth + 1, worstCandidateDist, worstCandidateInd, bestDist, alpha, candidates, candidateCount);
            // propagate information found so far
            bestDist = subBestDist;
            worstCandidateDist = subWorstCandDist;
            worstCandidateInd = subWorstCandInd;
        }
        return [bestDist, worstCandidateDist, worstCandidateInd];
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
    searchTree(queryVector, candidateCount, alpha) {
        let candidates = [];
        this.root.searchTree(queryVector, 0, Number.MIN_VALUE, -1, Number.MAX_VALUE, alpha, candidates, candidateCount);
        return candidates;
    }
}
const tree = new KDTree();

for (let i = 0; i < 500; i++) {
    let features = [];
    for (let j = 0; j < featureCount; j++) features.push(minVal + Math.random() * (maxVal - minVal));
    tree.addNode(new KDNode(features));
}
