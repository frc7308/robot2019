package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;

public class AnalogEncoder {
    public AnalogInput m_encoderInput;
    private double m_angle;
    private double m_revolutions;
    private double m_prevAngle;
    private int m_offset;

    private int k_ticksPerRevolution;
    private double k_ticksPerInch;

    private boolean inverted;

    public AnalogEncoder(int port, int ticksPerRevolution, double ticksPerInch, boolean inverted) {
        this.m_encoderInput = new AnalogInput(port);
        this.k_ticksPerRevolution = ticksPerRevolution;
        this.k_ticksPerInch = ticksPerInch;
        this.inverted = inverted;
    }

    public void zero() {
        this.m_offset = this.m_encoderInput.getValue();
        this.m_revolutions = 0;
    }

    public void update() {
        this.m_angle = this.m_encoderInput.getValue();
        this.m_revolutions += calculateRevolutions(this.m_angle, this.m_prevAngle);
        this.m_prevAngle = this.m_angle;
    }

    public double getPosition() {
        return (this.m_revolutions * this.k_ticksPerRevolution + this.m_encoderInput.getValue() - this.m_offset) / this.k_ticksPerInch;
    }

    private int calculateRevolutions(double currAngle, double prevAngle) {
        double skipThreshold = this.k_ticksPerRevolution / 4;
        if (prevAngle > this.k_ticksPerRevolution - skipThreshold && currAngle < skipThreshold) {
            return 1;
        } else if (prevAngle < skipThreshold && currAngle > this.k_ticksPerRevolution - skipThreshold) {
            return -1;
        }
        return 0;
    }
}