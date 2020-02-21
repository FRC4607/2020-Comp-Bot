package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.Constants.FLYWHEEL;
import frc.robot.Constants.GLOBAL;
import frc.robot.subsystems.Flywheel.ControlState_t;

public class FlywheelToSetRPM extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Flywheel mFlywheel; 
    private final XboxController mOperatorXbox;

    /* String for output */
    StringBuilder _sb = new StringBuilder();

    /* Loop tracker for prints */
    int _loops = 0;


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
        double targetVelocity_UnitsPer100ms  =  15000 + (mOperatorXbox.getRawAxis( 3 ) * 15000); 
        if (++_loops >= 10) {
			_loops = 0;
            System.out.println(_sb.toString());
            
         /* Reset built string */
		_sb.setLength(0);
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

