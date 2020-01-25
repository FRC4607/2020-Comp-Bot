package frc.robot.commands.Flywheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Flywheel.FlywheelJoystick;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class FlywheelJoystick extends CommandBase {

  public FlywheelJoystick() {
    addRequirements(Robot.mFlywheel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // Get the joystick inputs
    double xFlywheelSpin = (RobotContainer.mOperatorJoystick.getRawAxis(1));

    Robot.mFlywheel.setOpenLoopOutput(xFlywheelSpin);

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}