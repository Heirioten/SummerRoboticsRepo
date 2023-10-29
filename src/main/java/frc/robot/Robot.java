// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

// https://docs.wpilib.org/en/stable/docs/software/commandbased/what-is-command-based.html

/* Recommended viewing order (for a brief introduction to command-based programming / WPILib):

- Robot.java (this)
- RobotContainer.java (up until GetAutonomousCommand)
- GripperSubsystem.java
- PivotSubsystem.java
- ExtensionArmSubsystem.java
- TeleopCommand.java
- The remainder of RobotContainer.java


A less commented version of this is available at Heirioten/SummerRoboticsRepo:f4556537fb7b634db5cc6b881deb2e6aeed3f4a5
*/
public class Robot extends TimedRobot 
{
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  // PID - Product/Integral/Derivative
  // A method for controlling motors to move to an exact position or to an exact velocity, efficiently.
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html
  // Integral and Derivative can be entirely ignored for the sake of this project.
  // Product is the important one. Imagine our motor is at an arbitrary position of 0, and we want to get to 1.
  // How do we get our motor to 1?
  // By using P control, imagine our P=0.1
  // Our distance from our goal (setpoint) is 1.
  // 1 * 0.1 = 0.1
  // We will set our motor to 0.1 (10%)
  // In 20ms, at our next update, maybe our position is 0.2.
  // Our new distance is 0.8.
  // 0.8 * 0.1 = 0.08
  // Our new motor speed is 8%.
  // By tuning this value, we can find the most efficient way to meet our goal. Too fast, and our speed will ramp up so quickly the inertia will have us overshoot and it will bounce back and forth before finally settling at the setpoint.
  // Too slow, and well -- it's too slow.
  // The rest of the comments are not written as well as this one.


  // we want to know when robot is disabled so that our PID controllers dont make our robot have a seizure when we enable/disable/reenable adn the arm moves around during the disable (which happened a lot during testing in 2023 season)
  // because our arms are on constant PID control, our solution to this was, during the disable period, every loop we set the intended position of the arm to the actual position of the arm, such that when we reenable, the setpoint and actual position are equal
  public static boolean robotDisabled;

  @Override
  public void robotInit() 
  {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() 
  {
    robotDisabled = true;
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() 
  {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() 
  {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }

    robotDisabled = false;
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() 
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static boolean isRobotDisabled()
  {
    return robotDisabled;
  }
}
