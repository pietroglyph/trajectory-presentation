import { Units } from "../geo/units.js";
let kEpsilon = 1e-9;
export class DCMotorTransmission {
    constructor(Ks, Kv, Ka, WheelRadiusIn, RobotMass) {
        this.frictionV = Ks;
        this.speedPerV = 1.0 / Kv;
        const r = Units.inchesToMeters(WheelRadiusIn);

        // newton's second law:
        // https://www.khanacademy.org/science/physics/torque-angular-momentum/torque-tutorial/v/rotational-version-of-newtons-second-law
        //  F = ma (tangential)
        //  torque = F*r
        //  torque = r*m*a
        //  a = alpha*r (a is linear, alpha is angular acceleration)
        //  torque = r*r*m*alpha  
        //  alpha = torque / I  (I is rotational inertia (moi), rsq*m)
        //  I units: kg*m*m
        //  but force distributed through the wheel, perfect 
        //  cylinder/disk as I = 1/2 m*rsq
        this.torquePerV = r * r * RobotMass / (2 * Ka);
    }

    freeSpeedAtVolts(V) // returns rad/sec
    {
        if (V > kEpsilon)
            return Math.max(0, (V - this.frictionV) * this.speedPerV);
        else
            if (V < -kEpsilon)
                return Math.min(0, (V + this.frictionV) * this.speedPerV);
            else
                return 0;
    }

    getTorqueForVolts(currentSpeed, V) {
        let effectiveV = V;
        if (currentSpeed > kEpsilon) // fwd motion, rolling friction
            effectiveV -= this.frictionV;
        else
            if (currentSpeed < -kEpsilon) // rev motion, rolling friction
                effectiveV += this.frictionV;
            else
                if (V > kEpsilon) // static, fwd torque
                    effectiveV = Math.max(0.0, V - this.frictionV);
                else
                    if (V < -kEpsilon) // static, rev torque
                        effectiveV = Math.min(0.0, V + this.frictionV);
                    else
                        return 0; // Idle
        return this.torquePerV * (effectiveV - currentSpeed / this.speedPerV);
    }

    getVoltsForTorque(currentSpeed, torque) {
        let frictionV;
        if (currentSpeed > kEpsilon)
            frictionV = this.frictionV;
        else
            if (currentSpeed < -kEpsilon)
                frictionV = -this.frictionV;
            else
                if (torque > kEpsilon)
                    frictionV = this.frictionV;
                else
                    if (torque < -kEpsilon)
                        frictionV = -this.frictionV;
                    else
                        return 0;
        return (torque / this.torquePerV) + currentSpeed / this.speedPerV + frictionV;
    }
}

export default DCMotorTransmission;