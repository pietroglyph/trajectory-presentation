/* Quintic 1D Hermite Spline */

export class Spline1 {
    static fromSplineVaryDD(spline, dd0, dd1) {
        // varyDD is used as part of scheme to auto-compute curvature
        return new Spline1(
            spline.k0, spline.dk0, spline.ddk0 + dd0,
            spline.k1, spline.dk1, spline.ddk1 + dd1
        );
    }

    constructor(k0, dk0, ddk0, k1, dk1, ddk1) {
        this.k0 = k0; this.dk0 = dk0; this.ddk0 = ddk0;
        this.k1 = k1; this.dk1 = dk1; this.ddk1 = ddk1;
        this._computeCoefs();
    }

    tweakCurvature(ddk0, ddk1) {
        this.ddk0 += ddk0;
        this.ddk1 += ddk1;
        this._computeCoefs();
    }

    _computeCoefs() {
        // position has 6
        // a*t^5 + b*t^4 + c*t^3 + d*t^2 + e^t + f
        this.coeffs = [];
        this.coeffs.push(
            -6 * this.k0 - 3 * this.dk0 - 0.5 * this.ddk0 +
            0.5 * this.ddk1 - 3 * this.dk1 + 6 * this.k1);
        this.coeffs.push(15 * this.k0 + 8 * this.dk0 + 1.5 * this.ddk0 -
            this.ddk1 + 7 * this.dk1 - 15 * this.k1);
        this.coeffs.push(-10 * this.k0 - 6 * this.dk0 - 1.5 * this.ddk0 +
            0.5 * this.ddk1 - 4 * this.dk1 + 10 * this.k1);
        this.coeffs.push(0.5 * this.ddk0);
        this.coeffs.push(this.dk0);
        this.coeffs.push(this.k0);

        // tangent has 5
        // 5*a*t^4 + 4*b*t^3 + 3*c*t^2 + 2*d*t + e;
        this.dcoeffs = [];
        this.dcoeffs.push(5 * this.coeffs[0]);
        this.dcoeffs.push(4 * this.coeffs[1]);
        this.dcoeffs.push(3 * this.coeffs[2]);
        this.dcoeffs.push(2 * this.coeffs[3]);
        this.dcoeffs.push(this.coeffs[4]);

        // curvature has 4
        // 20*a*t^3 + 12*b*t^2 + 6*c*t + 2*d;
        this.ddcoeffs = [];
        this.ddcoeffs.push(20 * this.coeffs[0]);
        this.ddcoeffs.push(12 * this.coeffs[1]);
        this.ddcoeffs.push(6 * this.coeffs[2]);
        this.ddcoeffs.push(2 * this.coeffs[3]);

        // dcurvature has 3
        // 60*a*t^2 + 24*b*t + 6*c;
        this.dddcoeffs = [];
        this.dddcoeffs.push(60 * this.coeffs[0]);
        this.dddcoeffs.push(24 * this.coeffs[1]);
        this.dddcoeffs.push(6 * this.coeffs[2]);
    }

    getPosition(t) {
        if (t <= 0)
            return this.k0;
        else
            if (t >= 1)
                return this.k1;
            else {
                // horner's rule rules
                return this.coeffs[5] +
                    t * (this.coeffs[4] +
                        t * (this.coeffs[3] +
                            t * (this.coeffs[2] +
                                t * (this.coeffs[1] +
                                    t * this.coeffs[0]))));
            }
    }

    getTangent(t) {
        if (t <= 0)
            return this.dk0;
        else
            if (t >= 1)
                return this.dk1;
            else {
                return this.dcoeffs[4] +
                    t * (this.dcoeffs[3] +
                        t * (this.dcoeffs[2] +
                            t * (this.dcoeffs[1] +
                                t * this.dcoeffs[0])));
            }
    }

    getCurvature(t) {
        if (t <= 0)
            return this.ddk0;
        else
            if (t >= 1)
                return this.ddk1;
            else {
                return this.ddcoeffs[3] +
                    t * (this.ddcoeffs[2] +
                        t * (this.ddcoeffs[1] +
                            t * (this.ddcoeffs[0])));
            }
    }

    getDCurvature(t) {
        if (t <= 0)
            return this.dddcoeffs[2];
        else
            if (t >= 1) {
                return this.dddcoeffs[2] + this.dddcoeffs[1] + this.dddcoeffs[0];

            }
            else {
                return this.dddcoeffs[2] +
                    t * (this.dddcoeffs[1] +
                        t * (this.dddcoeffs[0]));
            }
    }
}

export default Spline1;
