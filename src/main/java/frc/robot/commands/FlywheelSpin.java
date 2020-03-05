 package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.Constants.FLYWHEEL;
import frc.robot.subsystems.Flywheel.ControlState_t;

public class FlywheelSpin extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Flywheel mFlywheel; 
    private final XboxController mOperatorXbox;

    @Override
    public void initialize () {
        mFlywheel.SetControlState( ControlState_t.OpenLoop );
    }

    @Override
    public void execute () {

        // drive off right trigger on operator 
        final double xSpin = ( -0.6 * mOperatorXbox.getRawAxis ( 3 ) );
        if ( xSpin < -FLYWHEEL.DEADBAND ) {
            mFlywheel.setOpenLoop( xSpin );
        } else { 
            mFlywheel.Stop ();
        }
    
    }

    @Override
    public void end ( final boolean interrupted ) {
    }
    
    @Override
    public boolean isFinished () {
      return false;
    }

    public FlywheelSpin ( final Flywheel flywheel, final XboxController operatorXbox ) {
        mFlywheel = flywheel;
        mOperatorXbox = operatorXbox;
        addRequirements( mFlywheel );
    }

}

