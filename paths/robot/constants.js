
let instances = {};
let s_defaultRobotId = "default";
export class Constants {
    static setRobotId(id) {
        s_defaultRobotId = id;
    }

    static getInstance(robotid) {
        if (!robotid) robotid = s_defaultRobotId;
        if (instances[robotid] == undefined)
            instances[robotid] = new Constants(robotid);
        return instances[robotid];
    }

    constructor(robotid) {
        this.paths =
            {
                MaxVelocity: 240, // in/s (20 ft/s)
                MaxAccel: 120, // in/s*s
                MaxCentripetalAccel: 30,  // in/s*s (?)
                MaxVoltage: 9,  // V
            };
        switch (robotid) {
            case "TestChassis":
                this.drive =
                    {
                        WheelBase: 23.75,
                        WheelDiameter: 6,
                        WheelRadius: 3,
                        TrackScrubFactor: 1.063,
                        RightTranmission: {
                            Ks: .7714, // V
                            Kv: .192,  // V/radians/s
                            Ka: .0533,
                        },
                        LeftTransmission: {
                            Ks: .794,
                            Kv: .185,
                            Ka: .035,
                        },
                        CenterToFront: 16.125,
                        CenterToSide: 13.75,
                        // hm what about CenterToBack?
                        VelocityKp: .2,
                    };
                this.robot =
                    {
                        LinearInertia: 27.93, // kg
                        AngularInertia: 1.74, // moment of inertia
                        AngularDrag: 12,   // N*m/(rad/sec)
                    };
                break;
            case "SecondRobot":
                this.drive =
                    {
                        WheelBase: 25.75,
                        WheelDiameter: 6,
                        WheelRadius: 3,
                        TrackScrubFactor: 1.1982,
                        RightTranmission: {
                            Ks: .9167,
                            Kv: .2405,
                            Ka: .0651,
                        },
                        LeftTransmission: {
                            Ks: .9238,
                            Kv: .2448,
                            Ka: .0643,
                        },
                        CenterToFront: 17.625, // in
                        CenterToSide: 18.75,
                        // hm what about CenterToBack?
                        VelocityKp: .3,
                    };
                this.robot =
                    {
                        LinearInertia: 67.812 + 5, // kg
                        AngularInertia: 5.58, // moment of inertia
                        AngularDrag: 12,   // N*m/(rad/sec)
                    };
                break;
            case "FirstRobot":
            default:
                this.drive =
                    {
                        WheelBase: 25.75, // diameter in inches
                        TrackScrubFactor: 1.037, // to produce "effective wheelbase"
                        WheelDiameter: 6, // inches
                        WheelRadius: 3,
                        RightTransmission: {
                            Ks: 1.582,
                            Kv: .244,
                            Ka: .076,
                        },
                        LeftTransmission: {
                            Ks: 1.476,
                            Kv: .216,
                            Ka: .092,
                        },
                        CenterToFront: 16.125, // not measured
                        CenterToSide: 13.75,
                        // hm what about CenterToBack?
                        VelocityKp: .3,
                    };
                this.robot =
                    {
                        LinearInertia: 67.81, // kg
                        AngularInertia: 4.97, // moment of inertia
                        AngularDrag: 12,   // N*m/(rad/sec)
                    };
                break;

        }
    }
}

export default Constants;
