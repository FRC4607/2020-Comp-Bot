// package frc.robot.commands.Hopper;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.commands.Hopper.HopperJoystick;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;

// public class HopperJoystick extends CommandBase {

//   public HopperJoystick() {
//     addRequirements(Robot.mTransferWheel);
//     System.out.println("TransferWheelJoystick created");
//   }

//   @Override
//   public void initialize() {
//   }

//   @Override
//   public void execute() {
//     // Get the joystick inputs
//     // decide best axis to use, probably change to button
//     double xHopperSpin = (RobotContainer.mOperatorJoystick.getRawAxis(4));

//     Robot.mHopper.setOpenLoopOutput(xHopperSpin);

//     System.out.println("HopperJoystick executed");
//   }

//   @Override
//   public boolean isFinished() {
//     return false;
//   }

//   @Override
//   public void end(boolean interrupted) {
//   }
// }

