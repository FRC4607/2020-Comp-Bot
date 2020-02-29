/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.lib.controllers.Vision.State;
import frc.robot.subsystems.Turret;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class TurretManual extends CommandBase {

  private Turret mTurret;
  private final XboxController mOperatorXbox;

  public TurretManual( Turret turret, XboxController operatXboxController ) {
    mTurret = turret;
    mOperatorXbox = operatXboxController;
    addRequirements(mTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //mTurret.mVision.setState( State.kPnP );
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = mOperatorXbox.getX( Hand.kLeft ) * 0.2 ;
    mTurret.setOpenLoop( value );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTurret.Stop();
    //mTurret.mVision.setState( State.kTurn );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
