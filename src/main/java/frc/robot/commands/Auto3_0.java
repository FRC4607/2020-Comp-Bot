package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.TransferWheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto3_0 extends SequentialCommandGroup {
  /**
   * Creates a new Auto3.
   */
  public Auto3_0( Drivetrain drivetrain, Flywheel flywheel, Hopper hopper, Indexer indexer, 
                          TransferWheel transferWheel, Intake intake/*, Limelight limelight */) {

    super(
      new InstantCommand( () -> intake.mShifter.set( DoubleSolenoid.Value.kForward ) ),
      new FlywheelToSetRPM_Auto( flywheel, 31000 ).withTimeout( 3 ),

      parallel(
        new FlywheelToSetRPM_Auto( flywheel, 31000 ).withTimeout( 7 ),
        new InstantCommand( () -> hopper.Spin() ).withTimeout( 7 ),
        new InstantCommand( () -> indexer.Spin() ).withTimeout( 7 ),
        new InstantCommand( () -> transferWheel.Spin() ).withTimeout( 7 )
      ),
        new InstantCommand( () -> hopper.Stop() ),
        new InstantCommand( () -> indexer.Stop() ),
        new InstantCommand( () -> transferWheel.Stop() ),
        new DriveForDistance( drivetrain, 2, -.5, 0.0 )
    );
  }
}