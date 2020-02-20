package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.Constants.FLYWHEEL;
import frc.robot.subsystems.Flywheel.ControlState_t;

public class FlywheelToSetRPM extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Flywheel mFlywheel; 
    private final XboxController mOperatorXbox;

    @Override
    public void initialize () {
        mFlywheel.SetControlState( ControlState_t.ClosedLoop );
    }

    @Override
    public void execute() {

        // drive off right trigger on operator 
        // dylan wants 60 percent right now 
        // Max RPM is 18730 
        // REV through born encoder quadature resolution is 2048 cycles per revolution (8192 counts per revolution)
        // 0 to 60 of trigger * max RPM * units per rev * units per 100 ms 
        double xSpin = ( ( -0.6 * mOperatorXbox.getRawAxis( 3 )) * 18730 * 2048 / 600); 
        if ( xSpin < -FLYWHEEL.DEADBAND ) {
            mFlywheel.setCloseLoop( xSpin );
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

    public FlywheelToSetRPM ( Flywheel flywheel, XboxController operatorXbox ) {
        mFlywheel = flywheel;
        mOperatorXbox = operatorXbox;
        addRequirements( mFlywheel );
    }

}

