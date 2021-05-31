const svg = document.getElementById("svg");
const agentG = document.createElementNS("http://www.w3.org/2000/svg", "circle");
const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
const trainButton = document.createElement("button");
const leftLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
const rightLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
const topLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
const bottomLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
path.setAttributeNS(null, "stroke", "black");
path.setAttributeNS(null, "stroke-width", 5);
path.setAttributeNS(null, "stroke-dasharray", "10 10");
path.setAttributeNS(null, "fill", "transparent");
trainButton.innerText = "Train";
trainButton.style.width = 200;
trainButton.style.height = 80;
trainButton.style.fontSize = 30;
trainButton.addEventListener("click", () => resetEnvironment(start));
document.body.appendChild(trainButton);
[leftLine, rightLine, topLine, bottomLine].forEach(line => {
    line.setAttributeNS(null, "stroke", "blue");
    line.setAttributeNS(null, "stroke-width", 3);
});
const gridWidth = 10;
const gridHeight = 10;
let d = "";
let startTime = 0;
let prevTime = 0;
let grid = [];
let gridGraphics = [];
let goalPos = [0,0];
let agentPos = [0,0];
const bBox = svg.getBoundingClientRect();
const fieldWidth = bBox.width / gridWidth;
const fieldHeight = bBox.height / gridHeight;
let dead = false;
let policy = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5];
let valueFunction = null;

const fieldType = {
    EMPTY: 0,
    TRAP: 1,
    COIN: 2,
    GOAL: 3,
    HOLE: 4
}
const actionTypes = {
    UP: 0,
    DOWN: 1,
    LEFT: 2,
    RIGHT: 3
}

// Training variables
let episode_done = false;
const AMOUNT_ACTIONS = 4;
const STATE_SIZE = 5;
const maxSteps = 24;
const possibleActions = [actionTypes.UP, actionTypes.DOWN, actionTypes.LEFT, actionTypes.RIGHT];
let agent = new A2CAgent(STATE_SIZE, AMOUNT_ACTIONS);
let total_reward = 0;
let episode_length = 0;
let state = null;
let episodeNum = 0;
let stepNum = 0;

async function update(time) {
    const elapsedTime = time - startTime;
    const dt = elapsedTime - prevTime;
    prevTime = elapsedTime;
    stepNum++;
    
    console.log("Episode " + episodeNum + " (" + stepNum + "/" + maxSteps + ")");
    let action = agent.get_action(state, possibleActions);
    switch (action) {
        case actionTypes.UP: move(0, 1); break;
        case actionTypes.DOWN: move(0, -1); break;
        case actionTypes.LEFT: move(-1, 0); break;
        case actionTypes.RIGHT: move(1, 0); break;
        default: console.error("Could not understand action: " + action);
    }
    let next_state = getState();
    let reward = calculateReward() - 1;
    total_reward += reward;
    let done = (agentPos[0] == goalPos[0] && agentPos[1] == goalPos[1]) || stepNum >= maxSteps;
    await agent.train_model(state, action, reward, next_state, done);

    if (done) {
        trainButton.style.visibility = "visible";
        console.log("Total reward: " + total_reward);
        stepNum = 0;
        total_reward = 0;
        return;
    }

    state = next_state;
    requestAnimationFrame(update);
}
function resetEnvironment() {
    for (let i = 0; i < gridGraphics.length; i++) {
        gridGraphics[i].remove();
    }
    svg.removeChild(agentG);
    svg.removeChild(path);
    svg.removeChild(leftLine);
    svg.removeChild(rightLine);
    svg.removeChild(topLine);
    svg.removeChild(bottomLine);
    gridGraphics = [];
    grid = [];
    requestAnimationFrame(start);
}
function start(time) {
    trainButton.style.visibility = "hidden";
    episodeNum++;
    dead = false;
    startTime = time;
    prevTime = startTime;

    goalPos = [Math.round(1 + Math.random() * (gridWidth - 3)), Math.round(1 + Math.random() * (gridHeight - 3))];
    agentPos = [Math.round(1 + Math.random() * (gridWidth - 3)), Math.round(1 + Math.random() * (gridHeight - 3))];

    for (let i = 0; i < gridHeight; i++) {
        for (let j = 0; j < gridWidth; j++) {
            let type = null;
            if (i == 0 || j == 0 || i == gridHeight - 1 || j == gridWidth - 1) type = fieldType.HOLE;
            else if (i == goalPos[1] && j == goalPos[0]) type = fieldType.GOAL;
            else if (i == agentPos[1] && j == agentPos[0]) type = fieldType.EMPTY;
            else type = randomField();
            const field = document.createElementNS("http://www.w3.org/2000/svg", "rect");
            field.setAttributeNS(null, "x", j * fieldWidth);
            field.setAttributeNS(null, "y", bBox.height - (i + 1) * fieldHeight);
            field.setAttributeNS(null, "width", fieldWidth);
            field.setAttributeNS(null, "height", fieldHeight);
            field.setAttributeNS(null, "fill", getColor(type));
            field.setAttributeNS(null, "stroke", "black");
            field.setAttributeNS(null, "stroke-width", 5);
            svg.appendChild(field);
            gridGraphics.push(field);
            grid.push(type);
        }
    }
    agentG.setAttributeNS(null, "cx", agentPos[0] * fieldWidth + (fieldWidth * 0.5));
    agentG.setAttributeNS(null, "cy", bBox.height - (agentPos[1] + 1) * fieldHeight + (fieldHeight * 0.5));
    agentG.setAttributeNS(null, "r", Math.min(fieldWidth, fieldHeight) * 0.5);
    agentG.setAttributeNS(null, "stroke", "black");
    agentG.setAttributeNS(null, "stroke-width", 2);
    agentG.setAttributeNS(null, "fill", "blue");
    d = "M " + agentG.getAttributeNS(null, "cx") + " " + agentG.getAttributeNS(null, "cy");
    svg.appendChild(path);
    svg.appendChild(agentG);
    svg.appendChild(leftLine);
    svg.appendChild(rightLine);
    svg.appendChild(topLine);
    svg.appendChild(bottomLine);

    state = getState();

    requestAnimationFrame(update);
}

function randomField() {
    const rn = Math.random();
    if (rn < 0.1) return fieldType.TRAP;
    else if (rn < 0.15) return fieldType.COIN;
    else return fieldType.EMPTY;
}
function getColor(type) {
    switch (type) {
        case fieldType.EMPTY: return "#cee1dd";
        case fieldType.TRAP: return "#ff0000";
        case fieldType.COIN: return "#f9cd1b";
        case fieldType.GOAL: return "#8b00ff";
        case fieldType.HOLE: return "#292929";
    }
}

function move(tx, ty) {
    if (dead) return;
    agentPos[0] += tx;
    agentPos[1] += ty;
    agentG.setAttributeNS(null, "cx", agentPos[0] * fieldWidth + (fieldWidth * 0.5));
    agentG.setAttributeNS(null, "cy", bBox.height - (agentPos[1] + 1) * fieldHeight + (fieldHeight * 0.5));
    field = getFieldAt(agentPos[0], agentPos[1]);
    if (field == fieldType.COIN) setFieldAt(agentPos[0], agentPos[1], fieldType.EMPTY);
    else if (field == fieldType.TRAP) {
        //dead = true;
        //agentG.setAttributeNS(null, "fill", "white");
    }
    d += " L " + agentG.getAttributeNS(null, "cx") + " " + agentG.getAttributeNS(null, "cy");
    path.setAttributeNS(null, "d", d);
    updateLines();
}

function getFieldAt(x, y) {
    return grid[clamp(y, 0, gridHeight - 1) * gridWidth + clamp(x, 0, gridWidth - 1)];
}
function setFieldAt(x, y, type) {
    grid[y * gridWidth + x] = type;
    gridGraphics[y * gridWidth + x].setAttributeNS(null, "fill", getColor(type));
}

function getState() {
    let state = [];
    for (let i = -1; i <= 1; i++) {
        let y = clamp(agentPos[1] + i, 0, gridHeight - 1);
        for (let j = -1; j <= 1; j++) {
            let x = clamp(agentPos[0] + j, 0, gridWidth - 1);
            state.push(getFieldAt(x, y));
        }
    }
    return state;
}

function getAction(state) {
    const value = model.predict(tf.tensor(state, [3,3]));
    let maxState = 0;
    let maxReturn = Number.MIN_VALUE;
    for (let i = 0; i < state.length; i++) {
        const currReturn = val;
    }
}

function calculateReward() {
    const type = getFieldAt(agentPos[0], agentPos[1]);
    switch (type) {
        case fieldType.EMPTY: return 0;
        case fieldType.TRAP: return -20;
        case fieldType.COIN: return 20;
        case fieldType.GOAL: return 500;
        case fieldType.HOLE: return -100;
    }
}

function clamp(val, min, max) {
    return Math.max(Math.min(val, max), min);
}

function onKeyUp(e) {
    switch (e.keyCode) {
        case 37: move(-1, 0); break;
        case 39: move(1, 0); break;
        case 40: move(0, -1); break;
        case 38: move(0, 1); break;
        case 83: saveModel(); break;
        case 76: loadModel(); break;
    }
}

window.addEventListener("keyup", onKeyUp);
window.onload = requestAnimationFrame(start);

/*function setupModel() {
    valueFunction = tf.sequential();
    model.add(tf.layers.dense({units: 1, inputShape: [3, 3, fieldType.values().length]}));
    model.compile({loss: 'meanSquaredError', optimizer: 'sgd'});
}
function initializeValueFunc() {
    xs = tf.tensor()
}*/

function updateLines() {
    let centerX = parseFloat(agentG.getAttributeNS(null, "cx"));
    let centerY = parseFloat(agentG.getAttributeNS(null, "cy"));
    let topLeft = [centerX - fieldWidth * 1.5, centerY - fieldHeight * 1.5];
    let bottomRight = [centerX + fieldWidth * 1.5, centerY + fieldHeight * 1.5];
    leftLine.setAttributeNS(null, "x1", topLeft[0]);
    leftLine.setAttributeNS(null, "y1", topLeft[1]);
    leftLine.setAttributeNS(null, "x2", topLeft[0]);
    leftLine.setAttributeNS(null, "y2", bottomRight[1]);
    rightLine.setAttributeNS(null, "x1", bottomRight[0]);
    rightLine.setAttributeNS(null, "y1", topLeft[1]);
    rightLine.setAttributeNS(null, "x2", bottomRight[0]);
    rightLine.setAttributeNS(null, "y2", bottomRight[1]);
    topLine.setAttributeNS(null, "x1", topLeft[0]);
    topLine.setAttributeNS(null, "y1", topLeft[1]);
    topLine.setAttributeNS(null, "x2", bottomRight[0]);
    topLine.setAttributeNS(null, "y2", topLeft[1]);
    bottomLine.setAttributeNS(null, "x1", topLeft[0]);
    bottomLine.setAttributeNS(null, "y1", bottomRight[1]);
    bottomLine.setAttributeNS(null, "x2", bottomRight[0]);
    bottomLine.setAttributeNS(null, "y2", bottomRight[1]);
}

// https://js.tensorflow.org/api/latest/#layers.softmax
/*function getValue(state) {
    stateOneHot = [];
    for (let i = 0; i < state; i++) {
        fieldOneHot = [];
    }
    xs = tf.tensor(state, [3, 3]);

}*/

async function saveModel() {
    await agent.actor.save('localstorage://model-agent');
    await agent.critic.save('localstorage://model-critic');
}
async function loadModel() {
    //agent = new A2CAgent(STATE_SIZE, AMOUNT_ACTIONS);
    agent.actor = await tf.loadLayersModel('localstorage://model-agent');
    agent.critic =  await tf.loadLayersModel('localstorage://model-critic');
    agent.actor.summary();
    agent.critic.summary();
    agent.actor.compile();
    agent.critic.compile();
    resetEnvironment();
}