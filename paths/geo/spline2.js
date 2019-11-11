import { Pose2d, Translation2d, Rotation2d } from "./pose2d.js";
import { Spline1 } from "./spline1.js";

export class Spline2 {
    constructor(splineX, splineY) {
        this.x = splineX;
        this.y = splineY;
        this.evalCache = null;
    }

    static fromControlPoints(xk0, yk0, xdk0, ydk0, xddk0, yddk0,
        xk1, yk1, xdk1, ydk1, xddk1, yddk1) {
        return new Spline2(
            new Spline1(xk0, xdk0, xddk0, xk1, xdk1, xddk1),
            new Spline1(yk0, ydk0, yddk0, yk1, ydk1, yddk1)
        );
    }

    static fromPoses(p0, p1) {
        const p0Xlate = p0.getTranslation();
        const p0Rotate = p0.getRotation();
        const p1Xlate = p1.getTranslation();
        const p1Rotate = p1.getRotation();
        // produce a tangent vector proportional to the distance between poses
        const scale = 1.2 * p0Xlate.getDistance(p1Xlate);
        return Spline2.fromControlPoints(
            p0Xlate.x, p0Xlate.y,
            p0Rotate.cos * scale, p0Rotate.sin * scale, // p0 tangent
            0, 0, // no curvature from poses
            p1Xlate.x, p1Xlate.y,
            p1Rotate.cos * scale, p1Rotate.sin * scale, // p1 tangent
            0, 0 // no curvature from poses
        );
    }

    // used to evaluate perform partial derivs along x
    static fromSpline2VaryDDX(spline, ddx0, ddx1) {
        return new Spline2(
            Spline1.fromSplineVaryDD(spline.x, ddx0, ddx1), spline.y);
    }

    // used to evaluate perform partial derivs along y
    static fromSpline2VaryDDY(spline, ddy0, ddy1) {
        return new Spline2(spline.x,
            Spline1.fromSplineVaryDD(spline.y, ddy0, ddy1));
    }

    getStartPose() {
        return this.getPose(0);
    }

    getEndPose() {
        return this.getPose(1);
    }

    getPose2dWithCurvature(t) {
        let pose = this.getPose(t);
        pose.curvature = this.getCurvature(t);
        pose.dcurvature = this.getDCurvature(t) / this.getVelocity(t);
        return pose;
    }

    getPose(t) {
        if (this.evalCache == null || this.evalCache.t != t)
            this.evalCache = { t: t };
        if (this.evalCache.x == undefined) {
            this.evalCache.x = this.x.getPosition(t);
            this.evalCache.y = this.y.getPosition(t);
        }
        if (this.evalCache.dx == undefined) {
            this.evalCache.dx = this.x.getTangent(t);
            this.evalCache.dy = this.y.getTangent(t);
        }
        return new Pose2d(new Translation2d(this.evalCache.x,
            this.evalCache.y),
            new Rotation2d(this.evalCache.dx,
                this.evalCache.dy, true));
    }

    draw(ctx, color, radius) {
        // draw positions, tangent and curvature 
        radius = radius | 2;

        this.getPose(0);
        this.getCurvature(0); // populate the evalCache
        this._draw(ctx, color, radius);

        this.getPose(1);
        this.getCurvature(1); // populate the evalCache
        this._draw(ctx, color, radius);
    }

    _draw(ctx, color, radius) {
        let c = this.evalCache;
        ctx.beginPath();
        ctx.arc(c.x, c.y, radius, 0, 2 * Math.PI, false);
        ctx.fill();

        ctx.lineCap = "round";
        ctx.lineWidth = 2;
        ctx.strokeStyle = "red";
        ctx.beginPath();
        ctx.moveTo(c.x, c.y);
        ctx.lineTo(c.x + c.dx, c.y + c.dy);
        ctx.stroke();

        ctx.strokeStyle = "yellow";
        ctx.beginPath();
        ctx.moveTo(c.x, c.y);
        ctx.lineTo(c.x + c.ddx, c.y + c.ddy);
        ctx.stroke();
    }

    getVelocity(t) // a scalar quantity, ie: tangential speed
    {
        if (this.evalCache == null || this.evalCache.t != t)
            this.evalCache = { t: t };
        if (this.evalCache.dx == undefined) {
            this.evalCache.dx = this.x.getTangent(t);
            this.evalCache.dy = this.y.getTangent(t);
        }
        return Math.hypot(this.evalCache.dx, this.evalCache.dy);
    }

    getHeading(t) {
        if (this.evalCache == null || this.evalCache.t != t)
            this.evalCache = { t: t };
        if (this.evalCache.dx == undefined) {
            this.evalCache.dx = this.x.getTangent(t);
            this.evalCache.dy = this.y.getTangent(t);
        }
        return new Rotation2d(this.evalCache.dx, this.evalCache.dy, true);
    }

    getCurvature(t) {
        /* http://mathworld.wolfram.com/Curvature.html, eq 13 */
        if (this.evalCache == null || this.evalCache.t != t)
            this.evalCache = { t: t };
        if (this.evalCache.dx == undefined) {
            this.evalCache.dx = this.x.getTangent(t);
            this.evalCache.dy = this.y.getTangent(t);
        }
        if (this.evalCache.ddx == undefined) {
            this.evalCache.ddx = this.x.getCurvature(t);
            this.evalCache.ddy = this.y.getCurvature(t);
        }
        const c = this.evalCache;
        const lensq = c.dx * c.dx + c.dy * c.dy;
        let curvature = (c.dx * c.ddy - c.ddx * c.dy) / (lensq * Math.sqrt(lensq));
        return curvature; // curvature may be infinite, this is okay (turn in place)
    }

    getDCurvature(t) {
        if (this.evalCache == null || this.evalCache.t != t)
            this.evalCache = { t: t };
        if (this.evalCache.dx == undefined) {
            this.evalCache.dx = this.x.getTangent(t);
            this.evalCache.dy = this.y.getTangent(t);
        }
        if (this.evalCache.ddx == undefined) {
            this.evalCache.ddx = this.x.getCurvature(t);
            this.evalCache.ddy = this.y.getCurvature(t);
        }
        if (this.evalCache.dddx == undefined) {
            this.evalCache.dddx = this.x.getDCurvature(t);
            this.evalCache.dddy = this.y.getDCurvature(t);
        }
        const c = this.evalCache;
        const dx2dy2 = (c.dx * c.dx + c.dy * c.dy);
        const num = (c.dx * c.dddy - c.dddx * c.dy) * dx2dy2 -
            3 * (c.dx * c.ddy - c.ddx * c.dy) * (c.dx * c.ddx + c.dy * c.ddy);
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    getDCurvatureSq(t) // a little faster than getDCurvature
    {
        if (this.evalCache == null || this.evalCache.t != t)
            this.evalCache = { t: t };
        if (this.evalCache.dx == undefined) {
            this.evalCache.dx = this.x.getTangent(t);
            this.evalCache.dy = this.y.getTangent(t);
        }
        if (this.evalCache.ddx == undefined) {
            this.evalCache.ddx = this.x.getCurvature(t);
            this.evalCache.ddy = this.y.getCurvature(t);
        }
        if (this.evalCache.dddx == undefined) {
            this.evalCache.dddx = this.x.getDCurvature(t);
            this.evalCache.dddy = this.y.getDCurvature(t);
        }
        const c = this.evalCache;
        const dx2dy2 = (c.dx * c.dx + c.dy * c.dy);
        const num = (c.dx * c.dddy - c.dddx * c.dy) * dx2dy2 -
            3 * (c.dx * c.ddy - c.ddx * c.dy) * (c.dx * c.ddx + c.dy * c.ddy);
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    sumDCurveSq(numSamples) {
        const dt = 1.0 / numSamples;
        let sum = 0.;
        for (let t = 0; t < 1.0; t += dt) {
            sum += dt * this.getDCurvatureSq(t);
        }
        return sum;
    }
}

export default Spline2;
