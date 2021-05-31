document.body.style.backgroundColor = "white";
colors = [];
for (let i = 0; i < 20; i++) colors.push([Math.random() * 250, Math.random() * 250, Math.random() * 250]);

const deltaTime = 0.01;
const blendTime = 2;
let inertializeStart = 0;
let t = 0;
let r0 = 255;
let g0 = 255;
let b0 = 255;
let r1 = 255;
let g1 = 255;
let b1 = 255;
let intervalID = -1;

let blend = t => 0;

function inertializeColor(color) {
    if (intervalID != -1) clearInterval(intervalID);
    t = 0;
    const x0 = [r0 - color[0], g0 - color[1], b0 - color[2]];
    const x1 = [r1 - color[0], g1 - color[1], b1 - color[2]];
    const x0Magnitude = Math.sqrt(x0[0] * x0[0] + x0[1] * x0[1] + x0[2] * x0[2]);
    const x0Norm = [x0[0] / x0Magnitude, x0[1] / x0Magnitude, x0[2] / x0Magnitude];
    const x1Magnitude = x1[0] * x0Norm[0] + x1[1] * x0Norm[1] + x1[2] * x0Norm[2];
    const vel = (x0Magnitude - x1Magnitude) / deltaTime;
    inertialize(x0Magnitude, vel, blendTime);

    intervalID = setInterval(() => {
        if (t > blendTime) clearInterval(intervalID);
        t += deltaTime;
        r1 = r0;
        g1 = g0;
        b1 = b0;

        const xt = blend(t);
        r0 = xt * x0Norm[0] + color[0];
        g0 = xt * x0Norm[1] + color[1];
        b0 = xt * x0Norm[2] + color[2];
        document.body.style.backgroundColor = "rgb(" + r0 + "," + g0 + "," + b0 + ")";
    }, deltaTime * 1000);
}

function inertialize(val, velocity, blendTime) {
    const negate = val < 0;
    if (negate) val = -val; // this method only works with positive values, negate input and output if value is negative
    if (velocity > 0) velocity = 0; // we want velocity to always decrease towards the target value
    blendTime = velocity == 0 ? blendTime : Math.min(blendTime, -5.0 * val / velocity); // avoid overshoot
    let acc = (-8.0 * velocity * blendTime - 20.0 * val) / (blendTime * blendTime);
    if (acc < 0) acc = 0; // we want acceleration to counter-act the velocity
    let poly0 = val;
    let poly1 = velocity; // 1st polynomial
    let poly2 = acc / 2.0; // 2nd polynomial
    let blendTimeN = blendTime * blendTime; // blendTime^2
    let jerk = acc * blendTimeN;
    blendTimeN *= blendTime; // blendTime^3
    let inc = velocity * blendTime;
    let poly3 = -((3.0 * jerk + 12.0 * inc + 20.0 * val) / (2.0 * blendTimeN)); // 3rd polynomial
    blendTimeN *= blendTime; // blendTime^4
    let poly4 = ((3.0 * jerk + 16.0 * inc + 30.0 * val) / (2.0 * blendTimeN)); // 4th polynomial
    blendTimeN *= blendTime; // blendTime^5
    let poly5 = -((jerk + 6.0 * inc + 12.0 * val) / (2.0 * blendTimeN)); // 5th polynomial
    if (negate) {
        poly0 = -poly0;
        poly1 = -poly1;
        poly2 = -poly2;
        poly3 = -poly3;
        poly4 = -poly4;
        poly5 = -poly5;
    }
    blend = t => {
        let res = poly0;
        let tN = t;
        res += poly1 * tN;
        tN *= t;
        res += poly2 * tN;
        tN *= t;
        res += poly3 * tN;
        tN *= t;
        res += poly4 * tN;
        tN *= t;
        res += poly5 * tN;
        return res;
    }
}

colors.forEach(color => {
    button = document.createElement("button");
    button.style.width = 50;
    button.style.height = 50;
    button.style.backgroundColor = "rgb(" + color[0] + "," + color[1] + "," + color[2] + ")";
    document.body.appendChild(button);
    button.addEventListener("click", _ => inertializeColor(color));
});

/*window.addEventListener("load", () => {
    setInterval(() => {
        inertializeColor(colors[Math.floor(Math.random() * colors.length)])
    }, 3000)
});*/