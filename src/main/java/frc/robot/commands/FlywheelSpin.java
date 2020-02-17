package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.Constants.FLYWHEEL;

public class FlywheelSpin extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Flywheel mFlywheel; 
    private final XboxController mOperatorXbox;

    @Override
    public void initialize () {
    }

    @Override
    public void execute() {

        // drive off right trigger on operator 
        double xSpin = ( -0.6 * mOperatorXbox.getRawAxis( 3 ) );
        if ( xSpin < -FLYWHEEL.DEADBAND ) {
            mFlywheel.setOpenLoop( xSpin );
        } else { 
            mFlywheel.Stop();
        }
    
    }

    @Override
    public void end ( boolean interrupted ) {
    }
    
    @Override
    public boolean isFinished () {
      return false;
    }

    public FlywheelSpin ( Flywheel flywheel, XboxController operatorXbox ) {
        mFlywheel = flywheel;
        mOperatorXbox = operatorXbox;
        addRequirements( mFlywheel );
    }

}

