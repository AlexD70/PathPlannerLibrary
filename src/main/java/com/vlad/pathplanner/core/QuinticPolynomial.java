package com.vlad.pathplanner.core;

public class QuinticPolynomial {
    double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0;
    QuinticPolynomial firstDerivative;
    QuinticPolynomial secondDerivative;

    public QuinticPolynomial(double a, double b, double c, double d, double e, double f) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
        this.e = e;
        this.f = f;
        // firstDerivative = new QuinticPolynomial(0, 5 * a, 4 * b, 3 * c, 2 * d, e);
        // secondDerivative = new QuinticPolynomial(0, 0, 20 * a, 12 * b, 6 * c, 2 * d);
    }

    public QuinticPolynomial() {
    }

    public QuinticPolynomial(double[] coeffs) {
        this.setCoeffs(coeffs);
    }

    public void setCoeffs(double[] coeffs) {
        this.a = coeffs[0];
        this.b = coeffs[1];
        this.c = coeffs[2];
        this.d = coeffs[3];
        this.e = coeffs[4];
        this.f = coeffs[5];
        firstDerivative = new QuinticPolynomial(0, 5 * a, 4 * b, 3 * c, 2 * d, e);
        secondDerivative = new QuinticPolynomial(0, 0, 20 * a, 12 * b, 6 * c, 2 * d);
    }

    double eval(double t) {
        return a * Math.pow(t, 5) + b * Math.pow(t, 4) + c * Math.pow(t, 3) + d * Math.pow(t, 2) + e * t + f;
    }

    QuinticPolynomial getFirstDerivative() {
        return firstDerivative;
    }

    QuinticPolynomial getSecondDerivative() {
        return secondDerivative;
    }
}
