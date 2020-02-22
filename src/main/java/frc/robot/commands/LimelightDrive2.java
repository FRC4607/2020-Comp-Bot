package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.lib.drivers.Limelight;
import frc.robot.lib.drivers.Limelight.camMode;

public class LimelightDrive2 extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final XboxController mDriverXbox;
    private final Limelight mLimelight;

    @Override
    public void initialize () {
        mLimelight.setCamMode(camMode.kVision);
    }

    @Override
    public void execute() {
        // Arcade drive driven off of the left Xbox joystick only 

        double kp = .05;
        double tx = mLimelight.horizontalToTargetDeg();
        double xturn = tx*kp;
        SmartDashboard.putNumber("tx", tx);
            mDrivetrain.mDifferentialDrive.arcadeDrive(mDriverXbox.getY(Hand.kLeft), -xturn);
     
    }

    @Override
    public void end ( boolean interrupted ) {
        mDrivetrain.mDifferentialDrive.arcadeDrive( 0, 0 );
    } 

    @Override
    public boolean isFinished () {
      return false;
    }

    public LimelightDrive2 ( Drivetrain drivetrain, XboxController driverXbox, Limelight limelight ) {
        mDrivetrain = drivetrain;
        mDriverXbox = driverXbox;
        mLimelight = limelight;
        addRequirements( mDrivetrain );
    }

}

