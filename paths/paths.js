/* global app */
import Spline2Array from "./geo/spline2array.js";
import { Spline2Sampler, kMaxDX } from "./geo/spline2sampler.js";
import { Trajectory } from "./robot/trajectory.js";
import { Pose2d } from "./geo/pose2d.js";
import { CentripetalMax } from "./robot/timing.js";
import { Constants } from "./robot/constants.js";
import { DCMotorTransmission } from "./robot/dcmotor.js";
import { DifferentialDrive } from "./robot/drive.js";
import { DifferentialDriveDynamics } from "./robot/timing.js";
import { VelocityLimitRegionConstraint } from "./robot/timing.js";

// an individual path, borne from waypoints
export default class Path {
    constructor(name, waypoints, config) {
        this.name = name;
        let constants = Constants.getInstance();
        let defaultPathConfig =
        {
            maxDX: kMaxDX, // from spline sampler
            startVelocity: 0,
            endVelocity: 0,
            maxVelocity: constants.paths.MaxVelocity,
            maxAbsAccel: constants.paths.MaxAccel,
            regionConstraints: [] // list of {maxVel,xmin,ymin,xmax,ymax}
        };
        this.config = Object.assign({}, defaultPathConfig, config);
        this.waypoints = waypoints;
        this.splines = null;
        this.osplines = null;
        this.trajectory = null;
        this._reverse = false;
    }

    serialize() {
        let path = { name: this.name, reverse: this._reverse };
        let pts = [];
        for (let wp of this.waypoints) {
            let h = wp.rotation.getDegrees();
            let o = { x: wp.translation.x, y: wp.translation.y, heading: h };
            pts.push(o);
        }
        path.waypoints = pts;
        // XXX: constraints!
        return JSON.stringify(path, null, 4);
    }

    static deserialize(txt) {
        let path = null;
        try {
            let o = JSON.parse(txt);
            let waypoints = [];
            for (let wp of o.waypoints)
                waypoints.push(Pose2d.fromXYTheta(wp.x, wp.y, wp.heading));
            path = new Path(o.name, waypoints);
            if (o.reverse)
                path.reverse();
            // XXX: constraints!
        }
        catch (err) {
            app.error("" + err);
            app.alertuser("" + err);
        }
        return path;
    }

    reverse() {
        this._reverse = true;
        return this;
    }

    intersect(config, x, y) {
        switch (config.mode) {
            case "waypoints":
                for (let p of this.waypoints) {
                    if (p.intersect(x, y)) {
                        if (this._reverse) {
                            let pp = Pose2d.clone(p);
                            pp.reverse();
                            return pp;
                        }
                        else
                            return p;
                    }
                }
                break;
            case "spline":
                for (let p of this.getSplineSamples()) {
                    if (p.intersect(x, y))
                        return p;
                }
                break;
            case "optspline":
                for (let p of this.getOptimizedSplineSamples()) {
                    if (p.intersect(x, y))
                        return p;
                }
                break;
            case "splineCtls":
                app.warning("splineCtls.intersection not implemented");
                break;
            case "optsplineCtls":
                app.warning("optsplineCtls.intersection not implemented");
                break;
            case "robot":
            case "trajectory":
                return this.getTrajectory().intersect(x, y);
            default:
                app.warning("Path.draw unknown mode " + config.mode);
                break;
        }
        return null;
    }

    draw(ctx, config, firstDrawTime) {
        switch (config.mode) {
            case "waypoints":
                if (config.pointerColor === undefined) {
                    config.pointerColor = "blue";
                }

                for (let p of this.waypoints)
                    p.draw(ctx, config);
                break;
            case "spline":
                this.drawToTime(this.getSplineSamples(config), ctx, config, firstDrawTime);
                break;
            case "optspline":
                this.drawToTime(this.getOptimizedSplineSamples(config), ctx, config, firstDrawTime);
                break;
            case "optspline-dt":
                this.drawToTime(this.getOptimizedSplineConstantDtSamples(config), ctx, config, firstDrawTime);
                break;
            case "optspline-ds":
                this.drawToTime(this.getOptimizedSplineReparamSamples(config), ctx, config, firstDrawTime);
                break;
            case "splineCtls":
                this.getSplines().draw(ctx, config.color);
                break;
            case "optsplineCtls":
                this.getOptimizedSplines().draw(ctx, config.color);
                break;
            case "trajectory":
            case "robot":
                if (config.time === "one-shot") {
                    config.time = Date.now() / 1000 - firstDrawTime;
                }
                this.getTrajectory().draw(ctx, config);
                break;
            default:
                app.warning("Path.draw unknown mode " + config.mode);
        }
    }

    drawToTime(samples, ctx, config, firstDrawTime) {
        for (let i = 0; i < samples.length; i++) {
            if (config.time === "one-shot" && i / samples.length > (Date.now() / 1000 - firstDrawTime) / (config.animationTime || 0.5)) {
                return;
            }
            samples[i].draw(ctx, config);
        }
    }

    getWaypoints() {
        return this.waypoints;
    }

    getSplines() {
        if (!this.splines)
            this.splines = Spline2Array.fromPose2Array(this.waypoints);
        return this.splines;
    }

    getSplineSamples(config = {}) {
        if (!this.splinesamps) {
            let splines = this.getSplines();
            this.splinesamps =
                Spline2Sampler.sampleSplines(splines, config.maxDx, config.maxDy, config.maxDTheta);
        }
        return this.splinesamps;
    }

    getOptimizedSplines() {
        if (!this.osplines) {
            this.osplines = Spline2Array.fromPose2Array(this.waypoints);
            this.osplines.optimizeCurvature();
        }
        return this.osplines;
    }

    getOptimizedSplineSamples(config = {}) {
        if (!this.osplinesamps) {
            let osplines = this.getOptimizedSplines();
            this.osplinesamps = Spline2Sampler.sampleSplines(osplines, config.maxDx, config.maxDy, config.maxDTheta);
        }
        return this.osplinesamps;
    }

    getOptimizedSplineConstantDtSamples(config = {}) {
        if (!this.constDtOSplineSamps) {
            let osplines = this.getSplines();
            let samples = [];
            for (let spline of osplines.splines) {
                for (let t = 0; t <= 1; t += Number(config.dt) || 0.01) {
                    samples.push(spline.getPose2dWithCurvature(t));
                }
            }
            this.constDtOSplineSamps = samples;
        }
        return this.constDtOSplineSamps;
    }

    getOptimizedSplineReparamSamples(config = {}) {
        if (!this.reparamOSplineSamps) {
            const osplines = this.getSplines();
            const dt = config.dt || 0.01;
            const ds = config.ds || 1;

            let s = 0;
            let lastS = 0;
            let lastSampledS = 0;
            let samples = [];
            for (let spline of osplines.splines) {
                for (let t = 0; t <= 1; t += dt) {
                    lastS = s;
                    s += spline.getVelocity(t) * dt;

                    let newTargetS = lastSampledS + ds;
                    while (lastS <= newTargetS && s >= newTargetS) {
                        let interpPercent = (newTargetS - s) / ds;
                        samples.push(spline.getPose2dWithCurvature(t + interpPercent * dt));
                        lastSampledS = newTargetS;

                        newTargetS += ds;
                    }
                }
            }
            this.reparamOSplineSamps = samples;
        }
        return this.reparamOSplineSamps;
    }

    getTrajectory() {
        if (this.trajectory == null) {
            let constants = Constants.getInstance();
            let osamps = this.getOptimizedSplineSamples();
            let timing = [];

            let leftTrans = new DCMotorTransmission(
                constants.drive.LeftTransmission.Ks,
                constants.drive.LeftTransmission.Kv,
                constants.drive.LeftTransmission.Ka,
                constants.drive.WheelRadius,
                constants.robot.LinearInertia);
            let rightTrans = new DCMotorTransmission(
                constants.drive.RightTransmission.Ks,
                constants.drive.RightTransmission.Kv,
                constants.drive.RightTransmission.Ka,
                constants.drive.WheelRadius,
                constants.robot.LinearInertia);
            let drive = new DifferentialDrive(constants, leftTrans, rightTrans);

            for (let rc of this.config.regionConstraints) {
                timing.push(new VelocityLimitRegionConstraint(
                    rc.maxVel, rc.xmin, rc.xmax, rc.ymin, rc.ymax
                ));
            }
            timing.push(new CentripetalMax(constants.paths.MaxCentripetalAccel));
            timing.push(new DifferentialDriveDynamics(drive,
                constants.paths.MaxVoltage));

            this.trajectory = Trajectory.generate(
                osamps,
                timing,  // timing constraints tbd
                this.config.maxDX,
                this.config.startVelocity,
                this.config.endVelocity,
                this.config.maxVelocity,
                this.config.maxAbsAccel
            );
            if (this._reverse)
                this.trajectory.reverse();
        }
        return this.trajectory;
    }
}