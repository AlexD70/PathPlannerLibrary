package com.vlad.pathplanner.core;

import org.apache.commons.math3.linear.*;

import org.apache.commons.math3.analysis.integration.*;
import org.apache.commons.math3.analysis.UnivariateFunction;

public class Segment {

    /**
     * A segment is made up of two parametrized quintic polynomials
     * and a heading interpolator (TODO)
     */
    private double len = -1;

    QuinticPolynomial x_poly, y_poly;

    double[][] coeffsArray = { { 0, 0, 0, 0, 0, 1 }, { 0, 0, 0, 0, 1, 0 }, { 0, 0, 0, 2, 0, 0 }, { 1, 1, 1, 1, 1, 1 },
            { 5, 4, 3, 2, 1, 0 }, { 20, 12, 6, 2, 0, 0 } };
    RealMatrix coeffsMatrix = MatrixUtils.createRealMatrix(coeffsArray);
    DecompositionSolver solver = new LUDecomposition(coeffsMatrix).getSolver();

    BaseAbstractUnivariateIntegrator integrator = new SimpsonIntegrator();
    UnivariateFunction function = new UnivariateFunction() {
        public double value(double tau) {
            return getUnitArcLength(tau);
        }
    };

    public Segment() {
        x_poly = new QuinticPolynomial();
        y_poly = new QuinticPolynomial();
    }

    public double length() {
        if (len == -1)
            len = getDisplacementAtParameter(1);
        return len;
    }

    public Vector2D getPointAtParameter(double t) {
        return new Vector2D(x_poly.eval(t), y_poly.eval(t));
    }

    public Vector2D getPointAtDisplacement(double s) {
        if (!Utilities.inRange(s, 0, this.length(), 0.01)) {
            System.out.println("Incorrect displacement provided");
            System.exit(-1); //bail
        }
        return this.getPointAtParameter(this.getParameterAtDisplacement(s));
    }

    public Vector2D getFirstDerivAtParameter(double t) {
        return new Vector2D(x_poly.getFirstDerivative().eval(t), y_poly.getFirstDerivative().eval(t));
    }

    public Vector2D getSecondDerivAtParameter(double t) {
        return new Vector2D(x_poly.getSecondDerivative().eval(t), y_poly.getSecondDerivative().eval(t));
    }

    public double getSlopeAtParameter(double t) {
        Vector2D tangent_vector = this.getFirstDerivAtParameter(t);
        if (Math.abs(tangent_vector.x) < 0.01)
            return Double.POSITIVE_INFINITY;
        return tangent_vector.y / tangent_vector.x;
    }

    private double getUnitArcLength(double tau) {
        double xsquared = this.x_poly.getSecondDerivative().eval(tau) * this.x_poly.getSecondDerivative().eval(tau);
        double ysquared = this.y_poly.getSecondDerivative().eval(tau) * this.y_poly.getSecondDerivative().eval(tau);
        return Math.sqrt(xsquared + ysquared);
    }

    /*
     * Integrate sqrt(dx/dt ^2 + dy/dt^2) from 0 to t
     * to find the arc length
     * Returns s(t) *
     */
    public double getDisplacementAtParameter(double t) {
        return integrator.integrate(Integer.MAX_VALUE, function, 0, t);
    }

    /*
     * Calculates at which t in [0,1] we have a particular displacement
     * Returns t(s)
     * Somewhat inefficient at the moment but its fine
     */
    public double getParameterAtDisplacement(double s) {
        if (!Utilities.inRange(s, 0, this.length(), 0.01)) {
            System.out.println("Incorrect displacement provided");
            System.exit(-1); //bail
        }
        double t = customTrapezoidal(0, 1, 2000, s);
        if (t == -1) {
            System.out.println("This should not show. Check the getParameterAtDisplacement function");
            System.exit(-1); //bail
        }
        return t;
    }

    public void calculateXCoeffs(double x, double x1, double x2, double xn, double xn1, double xn2) {
        double[] freeTerms = { x, x1, x2, xn, xn1, xn2 };
        RealVector constants = new ArrayRealVector(freeTerms, false);
        RealVector solution = solver.solve(constants);
        this.x_poly.setCoeffs(solution.toArray());
    }

    public void calculateYCoeffs(double y, double y1, double y2, double yn, double yn1, double yn2) {
        double[] freeTerms = { y, y1, y2, yn, yn1, yn2 };
        RealVector constants = new ArrayRealVector(freeTerms, false);
        RealVector solution = solver.solve(constants);
        this.y_poly.setCoeffs(solution.toArray());
    }

    private double customTrapezoidal(double x0, double xn, int n, double stop) {
        double eps = 0.3;
        double h = (xn - x0) / n;

        double integration = (getUnitArcLength(x0) + getUnitArcLength(xn)) / 2;

        for (int i = 1; i < n; i++) {
            double k = x0 + i * h;
            integration += getUnitArcLength(k) * 2;
            //System.out.println((integration * h / 2) + " " + stop);
            if (Math.abs(integration * h / 2 - stop) <= eps)
                return  k - h;
        }
        integration *= h / 2;
        return -1;
    }
}