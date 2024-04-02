package com.vlad.pathplanner.core;

import java.util.ArrayList;

public class MotionProfile {
    private double maxVelocity;
    private double maxAcceleration;
    private double maxAngularVelocity;
    private double maxAngularAcceleration;

    public ArrayList<KinematicState> timeProfileX;
    public ArrayList<KinematicState> timeProfileY;
    public ArrayList<Vector2D> displacement_profile;

    private Path path;

    public double duration;

    public MotionProfile(Path path, double maxVelocity, double maxAcceleration, double maxAngularVelocity, double maxAngularAcceleration) {
        this.path = path;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;

        timeProfileX = new ArrayList<>();
        timeProfileY = new ArrayList<>();
        displacement_profile = new ArrayList<>();

        duration = 0;
    }

    public void makeProfile(double startVelocity, double endVelocity, double startAcceleration, double endAcceleration, int numPoints) {
        long startTime = System.currentTimeMillis();
        double[] space = Utilities.generateLinear(0.0, this.path.getLength(), numPoints);
        // Place velocity planning points on the path
        for (double s : space) {
            double planningVelocity = this.maxVelocity;
            double ci = this.path.getCurvatureAtDisplacement(s);
            double vmaxOmega = 0.0;
            if (Math.abs(ci) < 0.01)
                vmaxOmega = Double.MAX_VALUE;
            else
                vmaxOmega = this.maxAngularVelocity / Math.abs(ci);
            planningVelocity = Math.min(planningVelocity, vmaxOmega);
            this.displacement_profile.add(new Vector2D(s, planningVelocity));
        }
        // Add the start and end velocities
        this.displacement_profile.set(0, new Vector2D(0.0, startVelocity));
        this.displacement_profile.set(numPoints - 1, new Vector2D(this.path.getLength(), endVelocity));
        
        // ----- FORWARD PASS -----
        for (int i = 1; i < numPoints; i++) {
            Vector2D currentPoint = this.displacement_profile.get(i);
            Vector2D prevPoint = this.displacement_profile.get(i - 1);
            // The velocity of the current point must be adjusted if its too high
            // i.e. we cannot accelerate from the previous point to this one
            if (currentPoint.y <= prevPoint.y)
                // Nothing to do here because its less or equal (we don't accelerate)
                continue;
            double currentVel = currentPoint.y;
            double prevVel = prevPoint.y;
            double ds = currentPoint.x - prevPoint.x;
            double currentMaxVel = Math.sqrt(prevVel * prevVel + 2 * this.maxAcceleration * ds);
            double finalVel = Math.min(currentMaxVel, currentVel);
            this.displacement_profile.set(i, new Vector2D(currentPoint.x, finalVel));
        }

        // ----- BACKWARDS PASS -----
        for (int i = numPoints - 1; i > 0; i--) {
            Vector2D currentPoint = this.displacement_profile.get(i);
            Vector2D prevPoint = this.displacement_profile.get(i - 1);
            // The velocity of the previous point must be adjusted if its too high
            // i.e. we cannot decelerate from the previous point to the current one
            if (currentPoint.y >= prevPoint.y)
                // Nothing to do here because its greater or equal (we don't decelerate)
                continue;

            double currentVel = currentPoint.y;
            double prevVel = prevPoint.y;
            double ds = currentPoint.x - prevPoint.x;
            double prevMaxVel = Math.sqrt(currentVel * currentVel + 2 * this.maxAcceleration * ds);

            this.displacement_profile.set(i - 1, new Vector2D(prevPoint.x, Math.min(prevMaxVel, prevVel)));
        }

        // ----- TIME PROFILE CREATION-----
        Vector2D tangentVector = this.path.getFirstDerivAtDisplacement(0);
        tangentVector.div(tangentVector.abs());

        this.timeProfileX.add(new KinematicState(0, 0, startVelocity * tangentVector.x, 
            startAcceleration * tangentVector.x, 0));
        this.timeProfileY.add(new KinematicState(0, 0, startVelocity * tangentVector.y, 
            startAcceleration * tangentVector.y, 0));

        double prevTime = 0, currentTime = 0;
        for (int i = 1; i < numPoints; i++) {
            Vector2D currentPoint = this.displacement_profile.get(i);
            Vector2D prevPoint = this.displacement_profile.get(i - 1);

            double currentVel = currentPoint.y;
            double prevVel = prevPoint.y;

            double ds = currentPoint.x - prevPoint.x;
            double dt = 2 * ds / (currentVel + prevVel);

            currentTime += dt;

            tangentVector = this.path.getFirstDerivAtDisplacement(currentPoint.x);
            tangentVector.div(tangentVector.abs());

            double vx = currentVel * tangentVector.x;
            double vy = currentVel * tangentVector.y;

            double currentAcc = (currentVel - prevVel) / (currentTime - prevTime);

            this.timeProfileX.add(new KinematicState(currentTime, currentPoint.x,
                vx, currentAcc * tangentVector.x, 0));

            this.timeProfileY.add(new KinematicState(currentTime, currentPoint.x,
                vy, currentAcc * tangentVector.y, 0));

            prevTime = currentTime;
        }
        this.duration = currentTime;
        long buildTime = System.currentTimeMillis() - startTime;
        //System.out.println("Profile built in " + buildTime + " ms.");
    }
}
