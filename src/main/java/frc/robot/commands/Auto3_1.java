/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.drivers.Limelight;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.TransferWheel;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto3_1 extends SequentialCommandGroup {
  /**
   * Creates a new Auto3.
   */
  public Auto3_1( Drivetrain drivetrain, Flywheel flywheel, Hopper hopper, Indexer indexer, 
                  TransferWheel transferWheel, Intake intake/*, Limelight limelight */) {

    super();
    addCommands(
      new InstantCommand( () -> intake.SetUp( !intake.IsUp() ) ).withTimeout( 3 ),
      new FlywheelToSetRPM_Auto( flywheel, 31000 ).withTimeout( 3 )
    ); 

    parallel(
        new FlywheelToSetRPM_Auto(flywheel, 31000),
        new InstantCommand( () -> hopper.Spin() ).withTimeout( 7 ),
        new InstantCommand( () -> indexer.Spin() ).withTimeout( 7 ),
        new InstantCommand( () -> transferWheel.Spin()).withTimeout( 7 )
    );

    addCommands(
    new InstantCommand( () -> hopper.Stop() ),
    new InstantCommand( () -> indexer.Stop() ),
    new InstantCommand( () -> transferWheel.Stop() ),
    new DriveForDistance( drivetrain, 5, -.5, 0.0 ).withTimeout( 2 )
    );
  }
}