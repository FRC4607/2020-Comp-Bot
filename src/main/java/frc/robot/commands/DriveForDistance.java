/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveForDistance extends CommandBase {

  private Drivetrain mDrivetrain;
  private double mSetpoint;
  private double mSpeed;
  private double mTurn;
  /**
   * Creates a new DriveForDistance.
   */
  public DriveForDistance( Drivetrain drivetrain, double setpoint, double speed, double turn ) {
    mDrivetrain = drivetrain;
    mSetpoint = setpoint;
    mTurn = turn;
    mSpeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivetrain.zeroDistanceTraveled();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivetrain.mDifferentialDrive.arcadeDrive( mSpeed, mTurn );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs( mDrivetrain.getLeftEncoder() ) > Math.abs( mSetpoint );
  }
}
