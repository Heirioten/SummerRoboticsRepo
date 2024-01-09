package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface DriveSubsystemIO {
    @AutoLog
    public static class DriveSubsystemIOInputs {
        public double leftEncoderAverage = 0.0;
        public double leftVelocity = 0.0;
        public double leftAppliedVolts = 0.0;
        public double[] leftCurrentAmps = new double[] {};

        public double rightEncoderAverage = 0.0;
        public double rightVelocity = 0.0;
        public double rightAppliedVolts = 0.0;
        public double[] rightCurrentAmps = new double[] {};

        public double gyroYaw = 0.0;
    }

    public default void updateInputs(DriveSubsystemIOInputs inputs) {

    }

    public default void setVoltage(double leftVoltage, double rightVoltage) {
        setLeftVoltage(leftVoltage);
        setRightVoltage(rightVoltage);
    }

    public default void setLeftVoltage(double leftVoltage) {}
    public default void setRightVoltage(double rightVoltage) {}
}
