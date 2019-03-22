package frc.robot;

import java.util.ArrayList;

import frc.robot.Constants;

// PID Loop.
public class VelocityTracker {
    private double prevPos;

    public VelocityTracker() {
        this.prevPos = 0;
    }

    public double calculateVelocity(double currPos, double deltaTime) {
        double velocity = ((currPos - this.prevPos) / (deltaTime / 1000.0));
        //if (velocity == 0) {
            System.out.println(deltaTime / 1000.0);
        //}
        this.prevPos = currPos;
        return velocity;
    }
}