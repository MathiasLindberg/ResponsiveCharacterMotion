//const tf = require('@tensorflow/tfjs-node-gpu');
let zeros = (w, h, v=0) => Array.from(new Array(h), _ => Array(w).fill(v));

class A2CAgent {
    constructor(state_size, action_size) {
        this.render = false;
        this.state_size = state_size;
        this.action_size = action_size;
        this.value_size = 1;

        this.discount_factor = 0.99;
        this.actor_learningr = 0.001;
        this.critic_learningr = 0.005;

        this.actor = this.build_actor();
        this.critic = this.build_critic();
    
    }

    build_actor() {
        const model = tf.sequential();
        
        model.add(tf.layers.dense({
            units: 24,
            activation: 'relu',
            kernelInitializer:'glorotUniform',
            inputShape:[9, 5], //oneHotShape
        }));
        
        model.add(tf.layers.flatten());

        model.add(tf.layers.dense({
            units: this.action_size,
            activation:'softmax',
            kernelInitializer:'glorotUniform',
        }));

        model.summary();

        model.compile({
            optimizer: tf.train.adam(this.actor_learningr),
            loss:tf.losses.softmaxCrossEntropy
        });

        return model;
    }

    build_critic() {
        const model = tf.sequential();
        
        
        model.add(tf.layers.dense({
            units: 24,
            activation: 'relu',
            kernelInitializer:'glorotUniform',
            inputShape: [9, 5], //oneHot shape
        }));

        model.add(tf.layers.flatten());

        model.add(tf.layers.dense({
            units: this.value_size,
            activation:'linear',
            kernelInitializer:'glorotUniform',
        }));

        model.summary();

        model.compile({
            optimizer: tf.train.adam(this.critic_learningr),
            loss:tf.losses.meanSquaredError,
        });

        return model;
    }

    format_state(state) {
        let copy_state = state.slice();
        for(let i=0; i < state.length; i++) {
            if(Array.isArray(copy_state[i])) {
                copy_state[i] = Math.ceil(state[i][1] / 10);
            }
        }

        return copy_state;

    }

    get_action(state, actions) {        
        let oneHotState = tf.oneHot(this.format_state(state), 5);
        
        let policy = this.actor.predict(oneHotState.reshape([1,9,5]), {
            batchSize:1,
        });
        
        let policy_flat = policy.dataSync();
        
        return weightedRandomItem(actions, policy_flat);
    }

    async train_model(state, action, reward, next_state, done) {
        let target = zeros(1, this.value_size);
        let advantages = zeros(1, this.action_size);

        let oneHotState = tf.oneHot(this.format_state(state), 5);
        let oneHotNextState = tf.oneHot(this.format_state(next_state), 5);
        oneHotState = oneHotState.reshape([1, 9, 5])
        oneHotNextState = oneHotNextState.reshape([1, 9, 5])
        let value = this.critic.predict(oneHotState).flatten().dataSync()[0];
        let next_value = this.critic.predict(oneHotNextState).dataSync()[0];
        console.log("Action: " + action);
        if(done) {
            advantages[action] = [reward - value];
            target[0] = reward;
        } else {
            advantages[action] = [reward +this.discount_factor * (next_value) - value];
            target[0] = reward + this.discount_factor * next_value;
        }


        await this.actor.fit(oneHotState, tf.tensor(advantages).reshape([1,4]), {
            epochs:1,
        });
        /*this.actor.fit(oneHotState, tf.tensor(advantages).reshape([1,2047]), {
            epochs:1,
        });*/

        await this.critic.fit(oneHotState, tf.tensor(target), {
            epochs:1,
        });
        
    }
}

function weightedRandomItem(actions, policy_flat) {
    if (Math.random() < 0.1) return actions[Math.round(Math.random() * (actions.length - 1))];
    let maxI = 0;
    let maxProb = 0;
    for (let i = 0; i < policy_flat.length; i++) {
        const prob = policy_flat[i] * Math.random();
        if (prob > maxProb) {
            maxI = i;
            maxProb = prob;
        }
    }
    return actions[maxI];
}