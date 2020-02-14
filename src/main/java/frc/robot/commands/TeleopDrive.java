package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final XboxController mDriverXbox;


    @Override
    public void initialize () {}

    @Override
    public void execute() {
        // Arcade drive driven off of the left Xbox joystick only 
        if ( mDrivetrain.IsReversed() ) {
            mDrivetrain.mDifferentialDrive.arcadeDrive( mDriverXbox.getY( Hand.kLeft ), mDriverXbox.getX( Hand.kLeft ) );
        } else {
            mDrivetrain.mDifferentialDrive.arcadeDrive( mDriverXbox.getY( Hand.kLeft ), -mDriverXbox.getX( Hand.kLeft ) );
        }
    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return false;
    }

    public TeleopDrive ( Drivetrain drivetrain, XboxController driverXbox ) {
        mDrivetrain = drivetrain;
        mDriverXbox = driverXbox;
        addRequirements( mDrivetrain );
    }

}

