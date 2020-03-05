package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVETRAIN;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Drivetrain mDrivetrain;
    private final XboxController mDriverXbox;

    @Override
    public void initialize () {
    }

    @Override
    public void execute () { 
        // drive with square input
        // double speed = squareInputs( deadband ( mDriverXbox.getY( Hand.kLeft) ) );
        // double turn = squareInputs( deadband ( mDriverXbox.getX( Hand.kLeft ) ) );

        // drive without square input
        double speed = deadband( mDriverXbox.getY( Hand.kLeft ) ); 
        double turn = deadband( mDriverXbox.getX( Hand.kLeft ) ); 

        // Arcade drive driven off of the left Xbox joystick only 
        if ( mDrivetrain.IsReversed () ) {
            mDrivetrain.mDifferentialDrive.arcadeDrive( speed, -turn );
        } else {
            mDrivetrain.mDifferentialDrive.arcadeDrive( speed , turn );
        }
    }

    

    @Override
    public void end ( boolean interrupted ) {
    } 

    @Override
    public boolean isFinished () {
      return false;
    }

    public TeleopDrive ( Drivetrain drivetrain, XboxController driverXbox ) {
        mDrivetrain = drivetrain;
        mDriverXbox = driverXbox;
        addRequirements( mDrivetrain );
    }

    private double deadband ( double value ) { 
        if ( Math.abs( value ) > DRIVETRAIN.DEADBAND ) {
            return value;
        } else {
            return 0;
        }
    }

    // private double squareInputs ( double value ) {
    //    return Math.copySign( value * value, value );
    // }

}

