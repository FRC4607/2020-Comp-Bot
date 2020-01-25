package frc.robot.commands.TransferWheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.TransferWheel.TransferWheelJoystick;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class TransferWheelJoystick extends CommandBase {

  public TransferWheelJoystick() {
    addRequirements(Robot.mTransferWheel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    // Get the joystick inputs

    double xTransferWheelSpin = (RobotContainer.mOperatorJoystick.getRawAxis(5));

    Robot.mTransferWheel.setOpenLoopOutput(xTransferWheelSpin);

  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
  }
}