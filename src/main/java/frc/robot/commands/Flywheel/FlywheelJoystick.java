package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Flywheel.FlywheelJoystick;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class FlywheelJoystick extends CommandBase {

  public FlywheelJoystick() {
    addRequirement(Robot.mFlywheel);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
    // Get the joystick inputs

    double xFlywheelSpin = (RobotContainer.mOperatorJoystick.getRawAxis(0));

    Robot.mFlywheel.setOpenLoopOutput(xFlywheelSpin);

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}