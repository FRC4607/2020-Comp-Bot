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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto3_1 extends SequentialCommandGroup {
  /**
   * Creates a new Auto3.
   */
  public Auto3_1( Drivetrain drivetrain, Flywheel flywheel, Hopper hopper, Indexer indexer, TransferWheel transferWheel, Limelight limelight ) {

    super();
    addCommands(
     
        parallel(
          new FlywheelToSetRPM_Auto(flywheel, 31000),
          new InstantCommand( () -> hopper.Spin() ),
          new InstantCommand( () -> indexer.Spin() ),
          new TransferWheelRun( transferWheel, flywheel, limelight, true )
        ).withTimeout(7)
      );
    }
}
