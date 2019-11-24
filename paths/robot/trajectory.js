import { Constants } from "./constants.js";
import DifferentialDrive from "./paths/robot/drive.js";
import { Rotation2d, Twist2d } from "./paths/geo/pose2d.js";
import Units from "./paths/geo/units.js";
import Pose2d, { Translation2d } from "../geo/pose2d.js";

const differentialDrive = new DifferentialDrive(Constants.getInstance(), null, null);

export class Trajectory {
    constructor(poseSamples) {
        this.poseSamples = poseSamples;
        this.isDrawingReverse = false;
        // transmissions being null is ok because we just use this for kinematics
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
        let endT = this.poseSamples[this.poseSamples.length - 1].getSampleTime();
        let endTMS = endT * 1000;

        if (config.mode == "robot") {
            let constants = Constants.getInstance();
            let yrad = constants.drive.CenterToSide;
            let xrad = constants.drive.CenterToFront;
            let currentTime = config.time;
            if (currentTime === "triangle-wave") {
                currentTime = 2 * Math.abs((Date.now() % endTMS) / 1000 - endTMS / 1000 / 2);
            } else if (currentTime === "sawtooth-wave" || currentTime === undefined) {
                currentTime = (Date.now() % endTMS) / 1000;
            }

            // draw the pose that matches our time
            let currentRobotDrawn = false;
            let pose = Pose2d.clone(this.poseSamples[0] || Pose2d.fromIdentity());
            let lastPose = Pose2d.fromIdentity();
            for (let p of this.poseSamples) {
                if (config.sim !== "none") {
                    let vel = (lastPose.getSampleTime() - p.getSampleTime()) * p.velocity;
                    pose = pose.transformBy(Pose2d.fromXYTheta(vel * p.rotation.cos, vel * p.rotation.sin, lastPose.rotation.inverse().rotateBy(p.rotation).getRadians()))
                    p.translation = Translation2d.clone(pose.translation);
                    p.rotation = Rotation2d.clone(pose.rotation);
                }
                
                if (config.colors["velvector"] !== "transparent" && p.getSampleTime() % (config.vectorTimestep || 0.5) < 0.2) {
                    this._drawVelVector(p.translation.x, p.translation.y, p.velocity, p.getRotation(), ctx, config);
                }
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
            let samples = this.poseSamples.slice(5, this.poseSamples.length - 5);

            let index = samples.length;
            let reversedIterator = {
                next: function () {
                    index--;
                    return {
                        done: index < 0,
                        value: samples[index]
                    }
                }
            }
            reversedIterator[Symbol.iterator] = function() {
                return this;
            }

            for (let p of config.pass === "backward" ? reversedIterator : samples) {
                if (config.pass === "backward" && p.getSampleTime() < endT - config.time) {
                    break;
                } else if (config.pass !== "backward" && p.getSampleTime() > config.time)
                    break;

                let vel = p.getVelocityForPass(config.pass);
                if (config.side === "left") {
                    vel = p.wheelStates.left;
                } else if (config.side === "right") {
                    vel = p.wheelStates.right;
                }

                this._drawPlot(p.getSampleTime(), vel, ctx, config);
            }
        } else if (config.mode == "trajectory") {
            for (let p of this.poseSamples)
                p.draw(ctx, config.color);
        }
    }


    _drawVelVector(x, y, vel, rotation, ctx, config) {
        vel /= 10;

        let fromx = x, fromy = y;
        let tox = Math.cos(rotation.getRadians()) * vel + x;
        let toy = Math.sin(rotation.getRadians()) * vel + y;

        ctx.save();
        ctx.lineWidth = 1;

        ctx.strokeStyle = config.colors["velvector"];
        ctx.beginPath();
        ctx.moveTo(fromx, fromy);
        ctx.lineTo(tox, toy);
        ctx.stroke();

        ctx.restore();
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
        if (timingConstraints) {
            // apply time constraints to deliver per-sample  velocity 
            // target. (tbd)
            Trajectory.applyTimingConstraints(samples, timingConstraints,
                startVelocity, endVelocity, maxVelocity, maxAbsAccel);
        }

        return new Trajectory(samples);
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
                    break; // while loop
                }
            } /* end while */
            last = s;
            
            s.forwardPassVel = s.maxVel;
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

            s.backwardPassVel = s.maxVel;
        } /* end foreach sample */

        // Integrate constrained states forward in time to obtain
        // timed states
        let t = 0, s = 0, v = 0;
        let currentPose = Pose2d.fromXYTheta(0, 0, 0);
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

            samp.getVelocityForPass = (pass) => pass === "forward" ? samp.forwardPassVel || samp.velocity : samp.backwardPassVel || samp.velocity;
            samp.getMaxVelocityForPass = () => Math.max(samp.getVelocityForPass("forward"), samp.getVelocityForPass("backward"));

            if (i > 1) {
                let wheelStates = differentialDrive.solveInverseKinematics({
                    linear: Units.inchesToMeters(samp.velocity),
                    angular: samples[i - 1].rotation.inverse().rotateBy(samp.rotation).getRadians() / (samp.t - samples[i - 1].t)
                });
                samp.wheelStates = { left: Units.metersToInches(wheelStates.left), right: Units.metersToInches(wheelStates.right) };
            } else {
                samp.wheelStates = { left: 0, right: 0 };
            }
            

            delete samp.maxVel;
        }
    }
}

export default Trajectory;

