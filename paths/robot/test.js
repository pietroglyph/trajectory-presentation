/* global app */
import { Pose2d } from "../geo/pose2d.js";
import { Trajectory } from "./trajectory.js";

export class Test {
    // Please validate geo/Test before these.
    constructor() {
    }


    runAll() {
        app.info("testTrajectory");
        this.testTrajectory();
    }

    testTrajectory() {
        let maxDx = 1; // inches
        let maxDy = 1; // inches
        let maxDTheta = .01; // radians
        let timingConstraints = null;
        let startVelocity = 0;
        let endVelocity = 0;
        let maxVelocity = 80; // ips
        let maxAbsAccel = 20; // ips/sec
        let pose2Array = [
            Pose2d.fromXYTheta(-100, 100, 0),
            Pose2d.fromXYTheta(-50, 100, 0),
            Pose2d.fromXYTheta(-50, 50, 270),
            Pose2d.fromXYTheta(-100, 50, 360),
        ];
        let traj = Trajectory.generateTrajectory(pose2Array,
            maxDx, maxDy, maxDTheta,
            timingConstraints,
            startVelocity, endVelocity,
            maxVelocity, maxAbsAccel);
    }
}

export default Test;
