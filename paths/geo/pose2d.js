/* global app */

let kEpsilon = 1E-9;

export function lerp(a, b, pct) {
    return a + (b - a) * pct;
}

function sq(x) { return x * x; }
function distSq(x0, x1, y0, y1) {
    return sq(x1 - x0) + sq(y1 - y0);
}

export class Translation2d {
    constructor(x, y) {
        this.x = x;
        this.y = y;
    }

    static clone(other) {
        return new Translation2d(other.x, other.y);
    }

    static fromIdentity() {
        return new Translation2d(0, 0);
    }

    static fromDelta(x0, x1) {
        return new Translation2d(x1.x - x0.x, x1.y - x0.y);
    }

    static dot(a, b) {
        return a.x * b.x + a.y * b.y;
    }

    static getAngleBetween(a, b)  // returns Rotation2d
    {
        let cos = this.dot(a, b) / (a.length() * b.length());
        if (Number.isNaN(cos))
            return new Rotation2d(1, 0, false);
        else
            return Rotation2d.fromRadians(
                Math.acos(Math.min(1.0, Math.max(cos, -1.0))));
    }

    static cross(a, b) {
        return a.x * b.y - a.y * b.x;
    }

    static subtract(t2, t1) {
        return new Translation2d(t2.x - t1.x, t2.y - t1.y);
    }

    static add(t1, t2) {
        return new Translation2d(t2.x + t1.x, t2.y + t1.y);
    }

    length() {
        return Math.hypot(this.x, this.y);
    }

    lengthSq() {
        return this.x * this.x + this.y * this.y;
    }

    translateBy(other) {
        return Translation2d.add(this, other);
    }

    translateByXY(x, y) {
        return Translation2d.add(this, new Translation2d(x, y));
    }

    rotateBy(rotation) {
        return new Translation2d(this.x * rotation.cos - this.y * rotation.sin,
            this.x * rotation.sin + this.y * rotation.cos);
    }

    direction() {
        return new Rotation2d(this.x, this.y, true);
    }

    inverse() {
        return new Translation2d(-this.x, -this.y);
    }

    interpolate(other, x) {
        if (x <= 0)
            return new Translation2d(this.x, this.y);
        else
            if (x >= 1)
                return new Translation2d(other.x, other.y);
            else
                return this.extrapolate(other, x);
    }

    extrapolate(other, pct) {
        return new Translation2d(this.x + pct * (other.x - this.x),
            this.y + pct * (other.y - this.y));
    }

    scale(s) {
        return new Translation2d(this.x * s, this.y * s);
    }

    getDistance(other) {
        return this.inverse().translateBy(other).length();
    }

    equals(other, epsilon) {
        if (epsilon == undefined)
            epsilon = kEpsilon;
        if (Math.abs(other.x - this.x) > epsilon) return false;
        if (Math.abs(other.y - this.y) > epsilon) return false;
        return true;
    }
}

export class Rotation2d {
    static clone(other) {
        return new Rotation2d(other.cos, other.sin);
    }

    static fromIdentity() {
        return new Rotation2d(1, 0, false);
    }

    static fromRotation2d(rot) {
        return new Rotation2d(rot.cos, rot.sin, false);
    }

    static fromRadians(rads) {
        return new Rotation2d(Math.cos(rads), Math.sin(rads), false);
    }

    static fromDegrees(deg) {
        return this.fromRadians(Rotation2d.d2r(deg));
    }

    static d2r(d) {
        return d * (Math.PI / 180);
    }

    static r2d(r) {
        return r * (180 / Math.PI);
    }

    constructor(x, y, donormalize) {
        this.cos = x;
        this.sin = y;
        if (donormalize)
            this.normalize();
    }

    normalize() {
        let magnitude = Math.hypot(this.cos, this.sin);
        if (magnitude > kEpsilon) {
            this.cos /= magnitude;
            this.sin /= magnitude;
        }
        else {
            this.sin = 0;
            this.cos = 1;
        }
    }

    tan() {
        if (Math.abs(this.cos) < kEpsilon) {
            if (this.sin >= 0.0)
                return Number.POSITIVE_INFINITY;
            else
                return Number.NEGATIVE_INFINITY;
        }
        return this.sin / this.cos;
    }

    getRadians() {
        return Math.atan2(this.sin, this.cos);
    }

    getDegrees() {
        return Rotation2d.r2d(this.getRadians());
    }

    reverse() {
        // other.cos(180) = -1 
        // other.sin(180) = 0
        // reverse == rotate by 180 degrees
        this.cos = this.cos * -1;
        this.sin = this.sin * -1;
    }

    rotateBy(other) {
        return new Rotation2d(
            this.cos * other.cos - this.sin * other.sin,
            this.cos * other.sin + this.sin * other.cos, true);
    }

    perp()  // perpendicular ie: normal
    {
        return new Rotation2d(-this.sin, this.cos, false);
    }

    inverse() {
        return new Rotation2d(this.cos, -this.sin, false);
    }

    isParallel(other) {
        // angles are the same (hm: we assume normalized)
        if (Math.abs(other.cos - this.cos) > kEpsilon) return false;
        if (Math.abs(other.sin - this.sin) > kEpsilon) return false;
        return true;
    }

    toTranslation() {
        return new Translation2d(this.cos, this.sin);
    }

    interpolate(other, x) {
        if (x <= 0)
            return new Rotation2d(this.cos, this.sin, false);
        else
            if (x >= 1)
                return new Rotation2d(other.cos, other.sin, false);
            else {
                let dtheta = this.inverse().rotateBy(other).getRadians();
                return this.rotateBy(Rotation2d.fromRadians(dtheta * x));
            }
    }

    getDistance(other)  // dtheta in radians
    {
        return this.inverse().rotateBy(other).getRadians();
    }
}

export class Twist2d {
    constructor(dx, dy, dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    static fromIdentity() {
        return Twist2d(0, 0, 0);
    }

    static clone(t) {
        return Twist2d(t.dx, t.dy, t.dtheta);
    }

    scaled(scale) {
        return new Twist2d(this.dx * scale, this.dy * scale,
            this.dtheta * scale);
    }

    length() {
        if (this.dy == 0) // common case for diff-drive robot
            return Math.abs(this.dx);
        else
            return Math.hypot(this.dx, this.dy);
    }

    curvature() {
        if (Math.abs(this.dtheta) < kEpsilon)
            return 0;
        else
            return this.dtheta / this.length(); // length of zero means straight
    }

    // interpolating twist is invalid for common cases, consider
    // using Pose2d.interpolate.
    interpolate(other, pct) {
        if (pct <= 0)
            return Twist2d.clone(this);
        else
            if (pct >= 1)
                return Twist2d.clone(other);
            else {
                return new Twist2d(
                    this.dx + pct(other.dx - this.dx),
                    this.dy + pct(other.dy - this.dy),
                    this.dtheta + pct(other.dtheta - this.dtheta));
            }
    }
}

export class Pose2d  /* this is also a Pose2dWithCurvature when values are present */ {
    static fromIdentity() {
        return new Pose2d(Translation2d.fromIdentity(),
            Rotation2d.fromIdentity());
    }

    static clone(other) {
        let newPose = new Pose2d(
            Translation2d.clone(other.translation),
            Rotation2d.clone(other.rotation));
        if (other.t != undefined)
            newPose.t = other.t;
        if (other.velocity != undefined)
            newPose.velocity = other.velocity;
        if (other.accel != undefined)
            newPose.accel = other.accel;
        if (other.curvature != undefined) {
            newPose.curvature = other.curvature;
            newPose.dcurvature = other.dcurvature;
        }
        if (other.distance != undefined)
            newPose.distance = other.distance;
        return newPose;
    }

    static fromXYTheta(x, y, theta) {
        return new Pose2d(new Translation2d(x, y),
            Rotation2d.fromDegrees(theta));
    }

    static fromDelta(p0, p1) {
        return new Pose2d(
            Translation2d.fromDelta(p0.translation, p1.translation).rotateBy(p0.rotation.inverse()),
            p1.rotation.rotateBy(p0.rotation.inverse()));
    }

    static getTwist(p0, p1) {
        const xform = Pose2d.fromDelta(p0, p1);
        return Pose2d.log(xform);
    }

    static exp(twist) {
        let cosTwist = Math.cos(twist.dtheta);
        let sinTwist = Math.sin(twist.dtheta);
        let s, c;

        if (Math.abs(twist.dtheta) < kEpsilon) {
            s = 1.0 - 1.0 / 6.0 * twist.dtheta * twist.dtheta;
            c = .5 * twist.dtheta;
        }
        else {
            s = sinTwist / twist.dtheta;
            c = (1.0 - cosTwist) / twist.dtheta;
        }

        return new Pose2d(new Translation2d(twist.dx * s - twist.dy * c,
            twist.dx * c + twist.dy * s),
            new Rotation2d(cosTwist, sinTwist, false));
    }

    static log(transform) {
        let dtheta = transform.getRotation().getRadians();
        let half_dtheta = 0.5 * dtheta;
        let cos_minus_one = transform.getRotation().cos - 1.0;
        let halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < kEpsilon) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        }
        else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().sin) / cos_minus_one;
        }
        let translation_part = transform.getTranslation()
            .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta, false));
        return new Twist2d(translation_part.x, translation_part.y, dtheta);
    }

    constructor(translation, rotation, comment) {
        this.translation = translation;
        this.rotation = rotation;
        this.comment = comment || "";
    }

    getTranslation() {
        return this.translation;
    }

    getRotation() {
        return this.rotation;
    }

    // offsetBy simply moves pose along its heading to another location
    offsetBy(x, y) {
        return new Pose2d(this.translation.translateBy(
            new Translation2d(x, y).rotateBy(this.rotation)),
            Rotation2d.clone(this.rotation));
    }

    transformBy(other) {
        return new Pose2d(
            this.translation.translateBy(other.translation.rotateBy(this.rotation)),
            this.rotation.rotateBy(other.rotation));
    }

    inverse() {
        let invRot = this.rotation.inverse();
        return new Pose2d(this.translation.inverse().rotateBy(invRot), invRot);
    }

    perp()  // perpendicular, ie: normal
    {
        return new Pose2d(this.translation, this.rotation.perp());
    }

    isColinear(otherPose) {
        // Return true if this pose is (nearly) colinear with the another.
        //  we can compare our heading with than of another pose.
        //  We must also verify that that heading is nearly the same as
        //  the vector between our two positions.
        if (!this.rotation.isParallel(otherPose.rotation))
            return false;
        let dp = Translation2d.subtract(otherPose.translation, this.translation);
        let dir = dp.direction();
        if (!this.rotation.isParallel(dir))
            return false;
        else
            return true;
        // was:
        // const twist = Pose2d.log(this.inverse().transformBy(otherPose));
        // return (Math.abs(twist.dy) < kEpsilon) && (Math.abs(twist.dtheta)< kEpsilon);
    }

    intersection(otherPose) {
        app.warning("pose2d intersection isn't implemented");
        // used by pure-pursuit
    }

    interpolate(otherPose, x) {
        if (x <= 0) {
            return Pose2d.clone(this);
        }
        else if (x >= 1) {
            return Pose2d.clone(otherPose);
        }
        let twist = Pose2d.log(this.inverse().transformBy(otherPose));
        let newpose = this.transformBy(Pose2d.exp(twist.scaled(x)));
        if (this.curvature != undefined) {
            newpose.curvature = lerp(this.curvature, otherPose.curvature, x);
            newpose.dcurvature = lerp(this.dcurvature, otherPose.dcurvature, x);
        }
        if (otherPose.distance != undefined) {
            // distance is presumed to be the distance between points
            newpose.distance = x * otherPose.distance;
        }
        return newpose;
    }

    getDistance(other) {
        let twist = Pose2d.log(this.inverse().transformBy(other));
        return twist.length();
    }

    getHeading(other) {
        return Math.atan2(this.translation.y - other.translation.y,
            this.translation.x - other.translation.x);
    }

    intersect(mx, my, radius) {
        let rsq = sq(radius || 2);
        return distSq(mx, this.translation.x, my, this.translation.y) < rsq;
    }

    draw(ctx, config) {
        let x = this.translation.x;
        let y = this.translation.y;
        ctx.beginPath();
        ctx.arc(x, y, config.radius || 2, 0, 2 * Math.PI, false);
        ctx.fillStyle = config.color || "#2CFF2C";
        ctx.lineCap = "round";
        ctx.fill();

        let len = (config.reverse ? -1 : 1) * (config.radius || 2) * 2;
        ctx.strokeStyle = config.pointerColor || "transparent"; // color;
        ctx.beginPath();
        ctx.moveTo(x, y);
        ctx.lineTo(x + len * this.rotation.cos,
            y + len * this.rotation.sin);
        ctx.lineWidth = config.radius || 2;
        ctx.stroke();
    }

    reverse() {
        // reverse motion and heading, not position, not time, not distance
        this.rotation.reverse();
        if (this.velocity != undefined)
            this.velocity *= -1;
        if (this.accel != undefined)
            this.accel *= -1;
        // curvature?
    }

    asInfo() {
        let x = this.translation.x.toFixed(1);
        let y = this.translation.y.toFixed(1);
        let deg = this.rotation.getDegrees().toFixed(0);
        return `${x} ${y} ${deg}°`;
    }

    getSampleTime() {
        return this.t; // may be undefined
    }

    asDetails() {
        let result = this.asInfo();
        if (this.t != undefined)
            result += ` t: ${this.t.toFixed(1)}`;
        if (this.velocity != undefined)
            result += ` v: ${this.velocity.toFixed(2)}`;
        if (this.accel != undefined)
            result += ` a: ${this.accel.toFixed(2)}`;
        if (this.curvature != undefined)
            result += ` curv: ${this.curvature.toFixed(4)}`;
        if (this.distance != undefined)
            result += ` dist: ${this.distance.toFixed(1)}`;
        return result;
    }

    asArray() {
        return [this.translation.x, this.translation.y,
        this.rotation.getRadians()];
    }

    toString() {
        return "new Pose2d(" +
            `new Translation2d(${this.translation.x.toFixed(3)},` +
            ` ${this.translation.y.toFixed(3)}), ` +
            `new Rotation2d(${this.rotation.cos.toFixed(2)},` +
            ` ${this.rotation.sin.toFixed(2)}));`;
    }

    transform(other) {
        other.position.rotate(this.rotation);
        this.translation.translate(other.translation);
        this.rotation.rotate(other.rotation);
    }
}

export default Pose2d;