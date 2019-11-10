export class Units {
    static rpmToRadsPerSec(rpm) {
        return rpm * 2.0 * Math.PI / 60.0;
    }

    static radsPerSecToRpm(rps) {
        return rps * 60.0 / (2.0 * Math.PI);
    }

    static degreesToRads(degrees) {
        return Math.PI * degrees / 180.;
    }

    static radsToDegrees(radians) {
        return 180.0 * radians / Math.PI;
    }

    static metersToInches(meters) {
        return meters / 0.0254;
    }

    static metersPerInch() {
        return Units.inchesToMeters(1.);
    }

    static metersToFeet(meters) {
        return Units.metersToInches(meters) / 12.0;
    }

    static metersPerFoot() {
        return Units.feetToMeters(1);
    }

    static mmToInches(mm) {
        return mm / 25.4;
    }

    static inchesToMeters(inches) {
        return inches * 0.0254;
    }

    static inchesPerMeter() {
        return Units.metersPerInch(1);
    }

    static inchesToMM(inches) {
        return inches * 25.4;
    }

    static feetToMeters(feet) {
        return Units.inchesToMeters(feet * 12.0);
    }
}

export default Units;