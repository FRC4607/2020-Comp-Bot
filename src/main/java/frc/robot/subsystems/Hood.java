package frc.robot.subsystems;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;

public class Hood extends SubsystemBase {

    public static CANSparkMax mHood;
    
    private boolean mIsBrakeMode;

    public boolean isBrakeMode() {
      return mIsBrakeMode;
    }
    
    public void ApplyDriveSignal(double throttle) {
      double mThrottle = throttle;
      mHood.set(mThrottle);
    }

    public void setOpenLoopControl() {
      setBrakeMode(true);
    }

    public void setOpenLoopOutput(double zHood) {
      ApplyDriveSignal(zHood);
    }

    private void setBrakeMode(boolean b) {
    }

    public Hood() {
      mIsBrakeMode = false;
      setBrakeMode(true);
  
      mHood = new CANSparkMax (Constants.kHood, Constants.kHoodType);

      System.out.println("Hood created");
      System.out.print("Can ID: ");
      System.out.println(Constants.kHood);

      // This is inverted alongside the joystick inputs in order to create working limit switches
      mHood.setInverted(true);
  
      // Start off in open loop control
      setOpenLoopControl();
    }
  
    public static void initDefaultSetup() {
    }
		
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.print("Joystick value: ");
    // System.out.println(RobotContainer.mOperatorJoystick.getRawAxis(5));

    if (RobotContainer.mOperatorJoystick.getRawAxis(5) < -0.2 || RobotContainer.mOperatorJoystick.getRawAxis(5) > 0.2) {
      mHood.set(RobotContainer.mOperatorJoystick.getRawAxis(5) * 0.75);
    } 
  }

}
      
