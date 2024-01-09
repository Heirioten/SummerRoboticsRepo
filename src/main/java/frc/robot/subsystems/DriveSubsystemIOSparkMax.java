package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.OperatorConstants;


public class DriveSubsystemIOSparkMax implements DriveSubsystemIO {
    CANSparkMax fl, fr, bl, br;
    RelativeEncoder encoderFl, encoderFr, encoderBl, encoderBr;
    MotorControllerGroup left, right;
    DifferentialDrive drive;

    PigeonIMU pigeon;

    public DriveSubsystemIOSparkMax() {
        fl = new CANSparkMax(1, MotorType.kBrushless);
        fr = new CANSparkMax(2, MotorType.kBrushless);
        bl = new CANSparkMax(3, MotorType.kBrushless);
        br = new CANSparkMax(4, MotorType.kBrushless);

        fl.restoreFactoryDefaults();
        fr.restoreFactoryDefaults();
        bl.restoreFactoryDefaults();
        br.restoreFactoryDefaults();

        fl.setInverted(false);
        fr.setInverted(true);
        bl.setInverted(false);
        br.setInverted(true);

        encoderFl = fl.getEncoder();
        encoderFr = fr.getEncoder();
        encoderBl = bl.getEncoder();
        encoderBr = br.getEncoder();

        encoderFl.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
        encoderFr.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
        encoderBl.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);
        encoderBr.setPositionConversionFactor(OperatorConstants.kDistancePerPulse);

        encoderFl.setVelocityConversionFactor(OperatorConstants.kVelocityConversionFactor);
        encoderFr.setVelocityConversionFactor(OperatorConstants.kVelocityConversionFactor);
        encoderBl.setVelocityConversionFactor(OperatorConstants.kVelocityConversionFactor);
        encoderBr.setVelocityConversionFactor(OperatorConstants.kVelocityConversionFactor);


        left = new MotorControllerGroup(fl, bl);
        right = new MotorControllerGroup(fr, br);

        fl.setIdleMode(IdleMode.kCoast);
        fr.setIdleMode(IdleMode.kCoast);
        bl.setIdleMode(IdleMode.kCoast);
        br.setIdleMode(IdleMode.kCoast);  

        drive = new DifferentialDrive(left, right);
        drive.setSafetyEnabled(false);

        pigeon = new PigeonIMU(0);
    }

    @Override
    public void updateInputs(DriveSubsystemIOInputs inputs) {
        inputs.leftEncoderAverage = (encoderFl.getPosition() + encoderBl.getPosition()) / 2;
        inputs.leftVelocity = (encoderFl.getVelocity() + encoderBl.getVelocity()) / 2;
        inputs.leftAppliedVolts = ((fl.get() + bl.get()) / 2) * RobotController.getBatteryVoltage();
        inputs.leftCurrentAmps = new double[] {fl.getOutputCurrent(), bl.getOutputCurrent()};

        inputs.rightEncoderAverage = (encoderFr.getPosition() + encoderBr.getPosition()) / 2;
        inputs.rightVelocity = (encoderFr.getVelocity() + encoderBl.getVelocity()) / 2;
        inputs.rightAppliedVolts = ((fr.get() + br.get()) / 2) * RobotController.getBatteryVoltage();
        inputs.rightCurrentAmps = new double[] {fr.getOutputCurrent(), br.getOutputCurrent()};

        inputs.gyroYaw = pigeon.getYaw();
    }

    @Override
    public void setLeftVoltage(double leftV) {
        left.setVoltage(leftV);
    }

    @Override
    public void setRightVoltage(double rightV) {
        right.setVoltage(rightV);
    }
}