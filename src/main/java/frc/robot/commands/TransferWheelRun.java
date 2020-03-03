/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransferWheel;
import frc.robot.subsystems.Flywheel;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.drivers.Limelight.camMode;

public class TransferWheelRun extends CommandBase {
  /**
   * Creates a new IndexerRun.
   */
  
  private TransferWheel mTransferWheel;
  private Flywheel mFlywheel;
  private Limelight mLimelight;
  private boolean mUseLimelight;

  public TransferWheelRun ( TransferWheel transferWheel, Flywheel flywheel, Limelight limelight, boolean useLimelight ) {
    mTransferWheel = transferWheel;
    mFlywheel = flywheel;
    mLimelight = limelight;
    mUseLimelight = useLimelight;
    addRequirements( mTransferWheel );
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize () {
    mLimelight.setCamMode( camMode.kVision );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute () {
    double currentFlywheelSetpoint = mFlywheel.getSetPoint();
    double currentFlywheel = mFlywheel.GetCurrentVelocity_UPMS();
    boolean targetAquired = mLimelight.foundTarget();
    double allowedError = 5000;
    double currentError = Math.abs( currentFlywheelSetpoint - currentFlywheel );

    SmartDashboard.putNumber( "FlyWheel Error", currentError );

    if( currentError < allowedError && ( targetAquired || !mUseLimelight ) ) {
      SmartDashboard.putBoolean( "Transfer Ready", true );
      mTransferWheel.Spin();
    } else {
      SmartDashboard.putBoolean( "Transfer Ready", false );
      mTransferWheel.Stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end( boolean interrupted ) {
    mTransferWheel.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished () {
    return false;
  }
}
