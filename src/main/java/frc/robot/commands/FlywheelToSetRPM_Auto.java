package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Flywheel;
import frc.robot.Constants.FLYWHEEL;
import frc.robot.Constants.GLOBAL;
import frc.robot.subsystems.Flywheel.ControlState_t;

public class FlywheelToSetRPM_Auto extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Flywheel mFlywheel; 
    private final double mSetpoint;

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
        double targetVelocity_UnitsPer100ms  =  mSetpoint; 
        mFlywheel.setCloseLoop(targetVelocity_UnitsPer100ms);
    
    }

    @Override
    public void end ( boolean interrupted ) {
        mFlywheel.SetControlState(ControlState_t.OpenLoop);
        mFlywheel.setOpenLoop(0.0);
    }
    
    @Override
    public boolean isFinished () {
      return false;
    }

    public FlywheelToSetRPM_Auto ( Flywheel flywheel, double setpoint ) {
        mFlywheel = flywheel;
        mSetpoint = setpoint;
        addRequirements( mFlywheel );
    }

}

