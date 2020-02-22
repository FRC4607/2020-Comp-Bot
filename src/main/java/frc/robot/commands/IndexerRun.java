/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Flywheel;
import frc.robot.lib.drivers.Limelight;

public class IndexerRun extends CommandBase {
  /**
   * Creates a new IndexerRun.
   */
  
  private Indexer mIndexer;
  private Flywheel mFlywheel;
  private Limelight mLimelight;

  public IndexerRun( Indexer indexer, Flywheel flywheel, Limelight limelight ) {
    mIndexer = indexer;
    mFlywheel = flywheel;
    mLimelight = limelight;
    addRequirements(mIndexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentFlywheelSetpoint = mFlywheel.getSetPoint();
    double currentFlywheel = mFlywheel.GetCurrentVelocity_UPMS();
    boolean targetAquired = mLimelight.foundTarget();
    double allowedError = 1000;

    if( Math.abs( currentFlywheelSetpoint - currentFlywheel ) < allowedError && targetAquired) {
      mIndexer.Spin();
    } else {
      mIndexer.Stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mIndexer.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
