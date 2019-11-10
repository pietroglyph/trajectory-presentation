/* global app */
import { Pose2d, Translation2d, Rotation2d, Twist2d } from "./pose2d.js";
import { Spline2Sampler } from "./spline2sampler.js";
import { Spline2Array } from "./spline2array.js";
import { Spline2 } from "./spline2.js";

export function epsilonEquals(x, y, epsilon = 1e-10) {
    if (Math.abs(x - y) < epsilon)
        return true;
    else
        return false;
}

export function assertEquals(a, b, msg, epsilon = .0001) {
    if (Math.abs(a - b) > epsilon) {
        console.error(msg);
    }
}

export function assertTrue(b, msg) {
    if (!b)
        console.error(msg);
}

export class Test {
    constructor() {
    }

    runAll() {
        app.info("testpose2d");
        this.testPose2d();

        app.info("testtwist");
        this.testTwist();

        app.info("splineTest1");
        this.splineTest1();

        app.info("testSplineOptimizer");
        this.testSplineOptimizer();

        app.info("testSplineSampler");
        this.testSplineSampler();
    }

    testPose2d() {
        // Test transformation
        var pose1 = new Pose2d(new Translation2d(3, 4),
            Rotation2d.fromDegrees(90));
        var pose2 = new Pose2d(new Translation2d(1, 0),
            Rotation2d.fromDegrees(0));
        var pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.getTranslation().x, "x0");
        assertEquals(5, pose3.getTranslation().y, "y0");
        assertEquals(90, pose3.getRotation().getDegrees(), "r0");

        pose1 = new Pose2d(new Translation2d(3, 4),
            Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(1, 0),
            Rotation2d.fromDegrees(-90));
        pose3 = pose1.transformBy(pose2);
        assertEquals(3, pose3.getTranslation().x, "x1");
        assertEquals(5, pose3.getTranslation().y, "y1");
        assertEquals(0, pose3.getRotation().getDegrees(), "r1");

        // A pose times its inverse should be the identity
        var identity = Pose2d.fromIdentity();
        pose1 = new Pose2d(new Translation2d(3.51512152, 4.23),
            Rotation2d.fromDegrees(91.6));
        pose2 = pose1.transformBy(pose1.inverse());
        assertEquals(identity.getTranslation().x, pose2.getTranslation().x, "x2");
        assertEquals(identity.getTranslation().y, pose2.getTranslation().y, "y2");
        assertEquals(identity.getRotation().getDegrees(),
            pose2.getRotation().getDegrees(), "rot2");

        // Movement from pose1 to pose2 is along a circle with radius of 10 units centered at (3, -6)
        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(13, -6), Rotation2d.fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .5);

        var expectedAngleRads = Math.PI / 4;
        assertEquals(3.0 + 10.0 * Math.cos(expectedAngleRads), pose3.getTranslation().x, "x3");
        assertEquals(-6.0 + 10.0 * Math.sin(expectedAngleRads), pose3.getTranslation().y, "y3");
        assertEquals(expectedAngleRads, pose3.getRotation().getRadians(), "r3");

        pose1 = new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(90));
        pose2 = new Pose2d(new Translation2d(13, -6), Rotation2d.fromDegrees(0.0));
        pose3 = pose1.interpolate(pose2, .75);
        expectedAngleRads = Math.PI / 8;
        assertEquals(3.0 + 10.0 * Math.cos(expectedAngleRads), pose3.getTranslation().x, "x4");
        assertEquals(-6.0 + 10.0 * Math.sin(expectedAngleRads), pose3.getTranslation().y, "y4");
        assertEquals(expectedAngleRads, pose3.getRotation().getRadians(), "r4");
    }

    testTwist() {
        // Exponentiation (integrate twist to obtain a Pose2d)
        var twist = new Twist2d(1.0, 0.0, 0.0);
        var pose = Pose2d.exp(twist);
        assertEquals(1.0, pose.getTranslation().x, "x0");
        assertEquals(0.0, pose.getTranslation().y, "y0");
        assertEquals(0.0, pose.getRotation().getDegrees(), "r0");

        // Scaled.
        twist = new Twist2d(1.0, 0.0, 0.0);
        pose = Pose2d.exp(twist.scaled(2.5));
        assertEquals(2.5, pose.getTranslation().x, "x1");
        assertEquals(0.0, pose.getTranslation().y, "y1");
        assertEquals(0.0, pose.getRotation().getDegrees(), "r1");

        // Logarithm (find the twist to apply to obtain a given Pose2d)
        pose = new Pose2d(new Translation2d(2.0, 2.0),
            Rotation2d.fromRadians(Math.PI / 2));
        twist = Pose2d.log(pose);
        assertEquals(Math.PI, twist.dx, "dx0");
        assertEquals(0.0, twist.dy, "dy0");
        assertEquals(Math.PI / 2, twist.dtheta, "dtheta0");

        // Logarithm is the inverse of exponentiation.
        var pose2 = Pose2d.exp(twist);
        assertEquals(pose2.getTranslation().x, pose.getTranslation().x, "x3");
        assertEquals(pose2.getTranslation().y, pose.getTranslation().y, "y3");
        assertEquals(pose2.getRotation().getDegrees(), pose.getRotation().getDegrees(), "r3");
    }

    splineTest1() {
        var p1 = new Pose2d(new Translation2d(0, 0),
            Rotation2d.fromRadians(0));
        var p2 = new Pose2d(new Translation2d(15, 10),
            new Rotation2d(1, -5, true));
        var s = Spline2.fromPoses(p1, p2);
        var samples = [];
        Spline2Sampler.sampleSpline(s, samples);

        let arclength = 0;
        let lastPose = samples[0];
        for (let i = 1; i < samples.length; i++) {
            let s = samples[i];
            const twist = Pose2d.log(lastPose.inverse().transformBy(s));
            arclength += twist.dx;
            lastPose = s;
        }
        assertEquals(lastPose.getTranslation().x, 15.0, "x0");
        assertEquals(lastPose.getTranslation().y, 10.0, "y0");
        assertEquals(lastPose.getRotation().getDegrees(), -78.69006752597981, "r0");
        assertEquals(arclength, 23.17291953186379, "arclength");
    }

    testSplineOptimizer() {
        let a = new Pose2d(new Translation2d(0, 100),
            Rotation2d.fromDegrees(270));
        let b = new Pose2d(new Translation2d(50, 0),
            Rotation2d.fromDegrees(0));
        let c = new Pose2d(new Translation2d(100, 100),
            Rotation2d.fromDegrees(90));
        let splines0 = Spline2Array.fromPose2Array([a, b, c]);

        let startTime = Date.now();
        assertTrue(splines0.optimizeCurvature() < .014, "sumCurvature0");
        console.log("Optimization time (ms): " + (Date.now() - startTime));

        let d = new Pose2d(new Translation2d(0, 0),
            Rotation2d.fromDegrees(90));
        let e = new Pose2d(new Translation2d(0, 50),
            Rotation2d.fromDegrees(0));
        let f = new Pose2d(new Translation2d(100, 0),
            Rotation2d.fromDegrees(90));
        let g = new Pose2d(new Translation2d(100, 100),
            Rotation2d.fromDegrees(0));

        let splines1 = Spline2Array.fromPose2Array([d, e, f, g]);
        startTime = Date.now();
        assertTrue(splines1.optimizeCurvature() < 0.16, "sumCurvature1");
        console.log("Optimization time (ms): " + (Date.now() - startTime));

        let h = new Pose2d(new Translation2d(0, 0),
            Rotation2d.fromDegrees(0));
        let i = new Pose2d(new Translation2d(50, 0),
            Rotation2d.fromDegrees(0));
        let j = new Pose2d(new Translation2d(100, 50),
            Rotation2d.fromDegrees(45));
        let k = new Pose2d(new Translation2d(150, 0),
            Rotation2d.fromDegrees(270));
        let l = new Pose2d(new Translation2d(150, -50),
            Rotation2d.fromDegrees(270));

        let splines2 = Spline2Array.fromPose2Array([h, i, j, k, l]);
        startTime = Date.now();
        assertTrue(splines2.optimizeCurvature() < 0.05, "sumCurvature2");
        assertEquals(splines2.get(0).getCurvature(1.0), 0.0, "curvature0");
        assertEquals(splines2.get(2).getCurvature(1.0), 0.0, "curvature2");
        console.log("Optimization time (ms): " + (Date.now() - startTime));
    }

    testSplineSampler() {
        // 4 points on a circle, centered at 0
        const radius = 40;
        let waypoints = [];
        waypoints.push(new Pose2d(new Translation2d(0, radius),
            Rotation2d.fromDegrees(0)));
        waypoints.push(new Pose2d(new Translation2d(radius, 0),
            Rotation2d.fromDegrees(-90)));
        waypoints.push(new Pose2d(new Translation2d(0, -radius),
            Rotation2d.fromDegrees(-180)));
        waypoints.push(new Pose2d(new Translation2d(radius, 0),
            Rotation2d.fromDegrees(-270)));
        waypoints.push(new Pose2d(new Translation2d(0, radius),
            Rotation2d.fromDegrees(0)));

        const s2array = Spline2Array.fromPose2Array(waypoints);
        return Spline2Sampler.sampleSplines(s2array);
    }
}

export default Test;
