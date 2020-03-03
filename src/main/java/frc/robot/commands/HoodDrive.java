package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.Constants.HOOD;

public class HoodDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Hood mHood;
    private final XboxController mOperatorXbox;

    @Override
    public void initialize () {}

    @Override
    public void execute() {

        // drive off joystick 
        double xHood = ( -0.2 * mOperatorXbox.getRawAxis( 5 ) );
        double xHoodR = ( 0.2 * mOperatorXbox.getRawAxis( 5 ) );

        if ( xHood < -HOOD.DEADBAND ) {
            mHood.setOpenLoop( xHood );
        } else if ( xHoodR < HOOD.DEADBAND ) {
            mHood.setOpenLoopR( xHoodR );
        } else {
            mHood.Stop();
        }
        
    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return false;
    }

    public HoodDrive ( Hood hood, XboxController operatorXbox ) {
        mHood = hood;
        mOperatorXbox = operatorXbox;
        addRequirements( mHood );
    }

}

