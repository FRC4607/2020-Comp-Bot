package frc.robot.subsystems;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;

public class Hopper extends SubsystemBase {

    public static CANSparkMax mHopper;
    
    private boolean mIsBrakeMode;

    public boolean isBrakeMode() {
      return mIsBrakeMode;
    }
    
    public void ApplyDriveSignal(double throttle) {
      double mThrottle = throttle;
      mHopper.set(mThrottle);
    }

    public void setOpenLoopControl() {
      setBrakeMode(true);
    }

    public void setOpenLoopOutput(double zHopper) {
      ApplyDriveSignal(zHopper);
    }

    private void setBrakeMode(boolean b) {
    }

    public Hopper() {
      mIsBrakeMode = false;
      setBrakeMode(true);
  
      mHopper = new CANSparkMax (Constants.kHopper, Constants. kHopperType);

      System.out.println("Hopper created");

      // This is inverted alongside the joystick inputs in order to create working limit switches
      mHopper.setInverted(true);
  
      // Start off in open loop control
      setOpenLoopControl();
    }
  
    public static void initDefaultSetup() {
    }
		
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mHopper.set(RobotContainer.mOperatorJoystick.getRawAxis(4));
  }

}
      
