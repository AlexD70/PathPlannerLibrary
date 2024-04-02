package com.vlad.pathplanner.core;

import com.vlad.pathplanner.v2.Point2d;

import org.apache.commons.math3.linear.*;

import org.apache.commons.math3.analysis.integration.*;
import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.solvers.BrentSolver;
import org.apache.commons.math3.exception.TooManyEvaluationsException;

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
        return len; // who is len? what does it mean?
    }

    public Point2d getPointAtTimeMoment(double t) {
        return new Point2d(x_poly.eval(t), y_poly.eval(t));
    }

    public Point2d getPointAtDisplacement(double s) { // maybe theres a better way of doing this one???
        if (!Utilities.inRange(s, 0, this.length(), 0)) {
//            System.out.println("Incorrect displacement provided");
//            System.exit(-1); //bail this should raise some exception. For now let it be a RuntimeException
            throw new RuntimeException("Displacement out of range!");
        }

        return this.getPointAtTimeMoment(this.getTimeMomentAtDisplacement(s));
    }

    public Point2d getFirstDerivAtTimeMoment(double t) { // maybe map these as sth else rather than points
        return new Point2d(x_poly.getFirstDerivative().eval(t), y_poly.getFirstDerivative().eval(t));
    }

    public Point2d getSecondDerivAtTimeMoment(double t) { // see above
        return new Point2d(x_poly.getSecondDerivative().eval(t), y_poly.getSecondDerivative().eval(t));
    }

    public double getSlopeAtTimeMoment(double t) {
        Point2d tangentPoint = this.getFirstDerivAtTimeMoment(t);
//        if (Math.abs(tangentPoint.x) < 0.01) // redundant check
//            return Double.POSITIVE_INFINITY;
        return tangentPoint.y / tangentPoint.x; // dy/dx slope
    }

    private double getUnitArcLength(double tau) { // what does this have to do with the unit circle???
        // this is just the absolute value of the position vector in xOy at moment tau
        double xsquared = Math.pow(this.x_poly.getSecondDerivative().eval(tau), 2);
        double ysquared = Math.pow(this.y_poly.getSecondDerivative().eval(tau), 2);
        return Math.sqrt(xsquared + ysquared);
    }

    /*
     * Integrate sqrt(dx/dt ^2 + dy/dt^2) from 0 to t
     * to find the arc length
     * Returns s(t) *
     */
    public double getDisplacementAtParameter(double t) {
        if (t == 0.0) return 0;
        return integrator.integrate(Integer.MAX_VALUE, function, 0, t);
        // isnt the integral of a quintic polynomial determined by a formula?
        // why use an integrator for it???

        // ax^5 + bx^4 + cx^3 + dx^2 + ex + f
        // indefinite integral: g(x) = ax^6/6 + bx^5/5 + cx^4/4 + dx^3/3 + ex^2/2 + fx + C
        // integral from 0 to t: g(t) - g(0)
    }

    /*
     * Calculates at which t in [0,1] we have a particular displacement s0
     * Returns t such that s(t) = s0
     * Finds the root of the function s(t) - s0 = 0 using Brent's method
     */
    public double getTimeMomentAtDisplacement(double s0) {
        if (!Utilities.inRange(s0, 0, this.length(), 0.01)) {
            System.out.println("Incorrect displacement provided");
            System.exit(-1); //bail
        }
        // Stop these edge cases from reaching the Brent solver
        if (s0 < 0.01) return 0;
        if (this.length() - s0 < 0.01) return 1;
        UnivariateFunction f = t -> (getDisplacementAtParameter(t) - s0);
        BrentSolver brentSolver = new BrentSolver();
        try {
            double root = brentSolver.solve(1000, f, 0.0, 1.0);
            return root;
        } catch (TooManyEvaluationsException e) {
            System.out.println("Brent solver reached the maximum number of evaluations.");
            System.exit(-1); // bail
        }
        return -1;
        // double t = customTrapezoidal(0, 1, 2000, s);
        // if (t == -1) {
        //     System.out.println("This should not show. Check the getParameterAtDisplacement function");
        //     System.exit(-1); //bail
        // }
        // return t;
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

    @Deprecated
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