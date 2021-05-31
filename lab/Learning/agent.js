function actor_critic() {
    const tf = require('@tensorflow/tfjs-node-gpu');
    let zeros = (w, h, v=0) => Array.from(new Array(h), _ => Array(w).fill(v));

    class Agent {
        constructor(state_size, action_size) {
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

            model.add(tf.lauers.dense({
                units: 24,
                activation: 'relu',
                kernelInitializer: 'glorotUniform',
                inputShape: [9, 5]
            }));
            model.add(tf.layers.flatten());
            model.add(tf.layers.dense({
                units: this.action_size,
                activation: 'softmax',
                kernelInitializer: 'glorotUniform'
            }));
            model.summary();
            model.compile({
                optimizer: tf.train.adam(this.actor_learningr),
                loss: tf.losses.softmaxCrossEntropy
            });

            return model;
        }

        build_critic() {
            const model = tf.sequential();

            model.add(tf.layers.dense({
                units: 24,
                activation: 'relu',
                kernelInitializer:' glorotUniform',
                inputShape: [9, 5]
            }));
            model.add(tf.layers.flatten());
            model.add(tf.layers.dense({
                units: this.value_size,
                activation: 'linear',
                kernelInitializer: 'glorotUniform'
            }));
            model.summary();
            model.compile({
                optimizer: tf.train.adam(this.actor_learningr),
                loss: tf.losses.softmaxCrossEntropy
            });
        }

        format_state(state) {
            let copy_state = state.slice();
            for (let i = 0; i < state.length; i++) {
                if (Array.isArray(copy_state[i])) {
                    copy_state[i] = Math.ceil(state[i][1] / 10);
                }
            }

            return copy_state;
        }

    }

}