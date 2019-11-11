/* global app */
import { Pose2d } from "./pose2d.js";

export const kMaxDX = 2.0; //inches
export const kMaxDY = 0.05; //inches
export const kMaxDTheta = 0.1; //radians!
export const kMinSampleSize = 1;
const kRecursionDepthLimit = 500;

export class Spline2Sampler {
    static sampleSplines(spline2array, maxDx, maxDy, maxDTheta) {
        let accum = []; // array of post2d with curvature
        maxDx = maxDx || kMaxDX;
        maxDy = maxDy || kMaxDY;
        maxDTheta = maxDTheta || kMaxDTheta;
        accum.push(spline2array.get(0).getPose2dWithCurvature(0.0));
        for (let i = 0; i < spline2array.length; i++) {
            let s = spline2array.get(i);
            Spline2Sampler.sampleSpline(s, accum, 0.0, 1.0,
                maxDx, maxDy, maxDTheta,
                                        /*skip first*/ true);
        }
        return accum;
    }

    static sampleSpline(spline2, accum, t0, t1, maxDx, maxDy, maxDTheta, skipFirst) {
        if (t0 == undefined) t0 = 0.0;
        if (t1 == undefined) t1 = 1.0;
        if (maxDx == undefined) maxDx = kMaxDX;
        if (maxDy == undefined) maxDy = kMaxDY;
        if (maxDTheta == undefined) maxDTheta = kMaxDTheta;
        const dt = (t1 - t0) / kMinSampleSize;
        if (skipFirst == undefined || !skipFirst)
            accum.push(spline2.getPose2dWithCurvature(0.0));
        for (let t = t0; t < t1; t += dt) {
            Spline2Sampler.getSegmentArc(spline2, accum, t, t + dt,
                maxDx, maxDy, maxDTheta);
        }
    }

    static getSegmentArc(spline2, accum, t0, t1, maxDx, maxDy, maxDTheta, depth) {
        const p0 = spline2.getPose(t0);
        const p1 = spline2.getPose(t1);
        const twist = Pose2d.getTwist(p0, p1);
        if (depth == undefined) depth = 0;
        if (Math.abs(twist.dy) > maxDy || Math.abs(twist.dx) > maxDx ||
            Math.abs(twist.dtheta) > maxDTheta) {
            // subdivide
            if (depth > kRecursionDepthLimit) {
                throw "recursion depth limit exceeded";
                return;
            }
            Spline2Sampler.getSegmentArc(spline2, accum, t0, (t0 + t1) / 2,
                maxDx, maxDy, maxDTheta, depth + 1);
            Spline2Sampler.getSegmentArc(spline2, accum, (t0 + t1) / 2, t1,
                maxDx, maxDy, maxDTheta, depth + 1);
        }
        else {
            accum.push(spline2.getPose2dWithCurvature(t1));
        }
    }
}

export default Spline2Sampler;
