import { Constants } from "./constants.js";
export class Trajectory {
    constructor(poseSamples) {
        this.poseSamples = poseSamples;
        this.isDrawingReverse = false;
    }

    reverse() {
        for (let p of this.poseSamples)
            p.reverse();
    }

    mirror() {
    }

    intersect(x, y) {
        for (let p of this.poseSamples) {
            if (p.intersect(x, y))
                return p;
        }
        return null;
    }

    draw(ctx, config) {
        if (config.mode == "robot") {
            let constants = Constants.getInstance();
            let yrad = constants.drive.CenterToSide;
            let xrad = constants.drive.CenterToFront;
            let endT = this.poseSamples[this.poseSamples.length - 1].getSampleTime();
            let endTMS = endT * 1000;
            let currentTime = config.time;
            if (currentTime === "triangle-wave") {
                currentTime = 2 * Math.abs((Date.now() % endTMS) / 1000 - endTMS / 1000 / 2);
            } else if (currentTime === "sawtooth-wave" || currentTime === undefined) {
                currentTime = (Date.now() % endTMS) / 1000;
            }

            // draw the pose that matches our time
            let currentRobotDrawn = false;
            for (let p of this.poseSamples) {
                if (config.colors["body"] !== "transparent") {
                    this._drawRobot(p, xrad, yrad, ctx, config);
                }
                if (!currentRobotDrawn && p.getSampleTime() > currentTime) {
                    this._drawCurrentRobot(p, xrad, yrad, ctx, config);
                    currentRobotDrawn = true;
                    if (config.stopDrawingAtRobot) {
                        break;
                    }
                }
                this._drawWheels("back", p, xrad, yrad, ctx,
                    config.colors["backwheels"]);
                this._drawWheels("front", p, xrad, yrad, ctx,
                    config.colors["frontwheels"]);
            }

            if (!currentRobotDrawn) {
                let p = this.poseSamples[this.poseSamples.length - 1];

                this._drawCurrentRobot(p, xrad, yrad, ctx, config);
            }
        } else if (config.mode == "velocity") {
            for (let i = 5; i < this.poseSamples.length - 5; i++) {
                let p = this.poseSamples[i];
                if (p.getSampleTime() > config.time)
                    break;

                this._drawPlot(p.getSampleTime(), p.velocity, ctx, config);
            }
        } else if (config.mode == "trajectory") {
            for (let p of this.poseSamples)
                p.draw(ctx, config.color);
        }
    }

    _drawPlot(x, y, ctx, config) {
        ctx.save();

        ctx.beginPath();
        ctx.arc(x * config.xScale, ctx.canvas.clientHeight - y, config.radius, 0, 2 * Math.PI, false);
        ctx.fillStyle = config.color;
        ctx.fill();

        ctx.restore();
    }

    _drawRobot(p, xrad, yrad, ctx, config) {
        ctx.save();
        ctx.fillStyle = config.colors["body"];
        ctx.translate(p.translation.x, p.translation.y);
        ctx.rotate(p.rotation.getRadians());
        ctx.fillRect(-xrad, -yrad, 2 * xrad, 2 * yrad);
        ctx.restore();
    }

    _drawCurrentRobot(p, xrad, yrad, ctx, config) {
        ctx.save();
        ctx.fillStyle = config.colors["body/active"];
        ctx.translate(p.translation.x, p.translation.y);
        ctx.rotate(p.rotation.getRadians());
        ctx.fillRect(-xrad, -yrad, 2 * xrad, 2 * yrad);
        ctx.restore();
    }

    _drawWheels(subset, p, xrad, yrad, ctx, color) {
        ctx.save();
        ctx.translate(p.translation.x, p.translation.y);
        ctx.rotate(p.rotation.getRadians());
        if (subset == "back" || subset == "all") {
            // back left (x is front)
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(-xrad, -yrad, 2, 0, 2 * Math.PI, false);
            ctx.fill();
            // back right
            ctx.beginPath();
            ctx.arc(-xrad, yrad, 2, 0, 2 * Math.PI, false);
            ctx.fill();
        }
        if (subset == "front" || subset == "all") {
            ctx.fillStyle = color;
            // front left
            ctx.beginPath();
            ctx.arc(xrad, -yrad, 2, 0, 2 * Math.PI, false);
            ctx.fill();
            // front right
            ctx.beginPath();
            ctx.arc(xrad, yrad, 2, 0, 2 * Math.PI, false);
            ctx.fill();
        }
        ctx.restore();
    }

    getSamples() {
        return this.poseSamples;
    }

    // returns a trajectory:
    //   array of pose2d,curvature,dcurvature,time
    //  cf: timeParameterizeTrajectory (java implementation)
    static generate(samples, timingConstraints, stepSize,
        startVelocity, endVelocity, maxVelocity, maxAbsAccel) {
        // Resample with equidistant steps along the trajectory. 
        // Note that we may have sampled the spline with the same 
        // value for stepSize. In that case, we were working on the xy 
        // plane. Now, we're working along the robot trajectory. 
        //
        let result = [];
        let totalDist = 0;
        samples[0].distance = 0.0;
        result.push(samples[0]);
        let last = samples[0];
        let next;
        for (let i = 1; i < samples.length; i++) {
            next = samples[i];
            let dist = next.getDistance(last);
            totalDist += dist;
            if (dist >= stepSize) {
                let pct = stepSize / dist;
                let ipose = last.interpolate(next, pct);
                ipose.distance = pct * dist; // should be stepSize;
                result.push(ipose);
                last = ipose;
            }
            else
                if (i == samples.length - 1) {
                    // last sample isn't as far as stepSize, but it
                    // is important, so lets just append it for now.
                    result.push(next);
                }
        }
        result.totalDist = totalDist;

        if (timingConstraints) {
            // apply time constraints to deliver per-sample  velocity 
            // target. (tbd)
            Trajectory.applyTimingConstraints(result, timingConstraints,
                startVelocity, endVelocity, maxVelocity, maxAbsAccel);
        }

        return new Trajectory(result);
    }

    static applyTimingConstraints(samples, constraints,
        vel0, vel1, maxV, maxAbsAccel) {
        // Forward pass. We look at pairs of consecutive states, where the start 
        // state has already been assigned a velocity (though we may adjust 
        // the velocity downwards during the backwards pass). We wish to find an
        // acceleration that is admissible at both the start and end state, as 
        // well as an admissible end velocity. If there is no admissible end 
        // velocity or acceleration, we set the end velocity to the state's 
        // maximum allowed velocity and will repair the acceleration during the 
        // backward pass (by slowing down the predecessor).
        const kEpsilon = 1e-6;
        let last = samples[0];
        last.distance = 0;
        last.maxVel = vel0;
        last.accelLimits = [-maxAbsAccel, maxAbsAccel];
        for (let i = 1; i < samples.length; i++) {
            let s = samples[i];
            const ds = s.getDistance(last);
            s.distance = ds + last.distance;

            // Enforce global maxvel and max reachable vel by global
            // accel limits. (we may need to interate to find the max vel1
            // and common accel limits may be a functionof vel)
            while (true) {
                // Enforce global max velocity and max reachable velocity by 
                // global acceleration limit. vf = sqrt(vi^2 + 2*a*d)
                s.maxVel = Math.min(maxV,
                    Math.sqrt(last.maxVel * last.maxVel +
                        2 * last.accelLimits[1] * ds));
                s.accelLimits = [-maxAbsAccel, maxAbsAccel];

                // At this point s is ready, but no constraints have been 
                // applied aside from last state max accel.

                // lets first apply velocity constraints
                for (let c of constraints) {
                    s.maxVel = Math.min(s.maxVel, c.getMaxVel(s));
                    if (s.maxVel < 0) {
                        throw (`trajectory velocity underflow ${s.maxVel} sample ${i}`);
                    }
                }

                // now enforce accel constraints
                for (const c of constraints) {
                    let minmax = c.getMinMaxAccel(s, s.maxVel);
                    if (minmax[1] < minmax[0])
                        throw "trajectory bogus minmax accel 0";
                    // reverse could be applied here... (subtle)
                    s.accelLimits[0] = Math.max(s.accelLimits[0],
                        minmax[0]);
                    s.accelLimits[1] = Math.min(s.accelLimits[1],
                        minmax[1]);
                }
                if (s.accelLimits[0] > s.accelLimits[1]) {
                    throw "trajectory bogus minmax accel 1";
                }

                if (ds < kEpsilon) break;

                // If the max acceleration for this state is more conservative 
                // than what we had applied, we need to reduce the max accel at 
                // the predecessor state and try again.
                // TODO: Simply using the new max acceleration is guaranteed to 
                //  be valid, but may be too conservative. Doing a search would 
                //  be better
                let actualAccel = (s.maxVel * s.maxVel - last.maxVel * last.maxVel)
                    / (2.0 * ds);
                if (s.accelLimits[1] < (actualAccel - kEpsilon))
                    last.accelLimits[1] = s.accelLimits[1];
                else {
                    if (actualAccel > (last.accelLimits[0] + kEpsilon)) {
                        last.accelLimits[1] = actualAccel;
                    }
                    // if actual accel is less than last minaccel,
                    // we repapir it in backward pass.
                    break; // while 1 loop
                }
            } /* end while */
            last = s;
        } /* end foreach sample */

        // Backward Pass
        let next = samples[samples.length - 1];
        if (next.distance == undefined)
            throw "trajectory: bogus backward state";
        next.maxVel = vel1;
        next.accelLimits = [-maxAbsAccel, maxAbsAccel];
        for (let i = samples.length - 2; i >= 0; --i) {
            let s = samples[i]; // 
            const ds = s.distance - next.distance;
            while (true) {
                const newMaxVel = Math.sqrt(next.maxVel * next.maxVel +
                    2 * next.accelLimits[0] * ds);
                if (newMaxVel >= s.maxVel)
                    break; //  no new limits to impose
                s.maxVel = newMaxVel;
                for (const c of constraints) {
                    let minmax = c.getMinMaxAccel(s, s.maxVel);
                    if (minmax[1] < minmax[0])
                        throw "trajectory bogus minmax accel 2";
                    // xxx: reverse not implemented
                    s.accelLimits[0] = Math.max(s.accelLimits[0],
                        minmax[0]);
                    s.accelLimits[1] = Math.min(s.accelLimits[1],
                        minmax[1]);
                }
                if (s.accelLimits[0] > s.accelLimits[1])
                    throw "trajectory bogus minmax accel 1";
                if (ds < kEpsilon)
                    break;
                const actualAccel = (s.maxVel * s.maxVel - next.maxVel * last.maxVel)
                    / (2.0 * ds);
                if (s.accelLimits[0] > (actualAccel + kEpsilon))
                    next.accelLimits[0] = s.accelLimits[0];
                else {
                    next.accelLimits[0] = actualAccel;
                    break; // out of while
                }
            } // end while
            next = s;
        } /* end foreach sample */

        // Integrate constrained states forward in time to obtain
        // timed states
        let t = 0, s = 0, v = 0;
        for (let i = 0; i < samples.length; i++) {
            let samp = samples[i];
            samp.t = t;
            const ds = samp.distance - s;
            const accel = (samp.maxVel * samp.maxVel - v * v) / (2 * ds);
            let dt = 0;
            if (i > 0) {
                samples[i - 1].accel = accel; // todo: reverse?
                if (Math.abs(accel > kEpsilon))
                    dt = (samp.maxVel - v) / accel;
                else
                    if (Math.abs(v) > kEpsilon)
                        dt = ds / v;
                    else {
                        throw "trajectory invalid sample " + samp;
                    }
            }
            t += dt;
            v = samp.maxVel;
            s = samp.distance;
            samp.velocity = v;
            samp.acceleration = accel;
            delete samp.maxVel;
        }
    }
}

export default Trajectory;

