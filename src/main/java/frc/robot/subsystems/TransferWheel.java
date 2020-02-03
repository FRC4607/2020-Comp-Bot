package frc.robot.subsystems;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.revrobotics.CANSparkMax;

public class TransferWheel extends SubsystemBase {

    public static CANSparkMax mTransferWheel;
    
    private boolean mIsBrakeMode;

    public boolean isBrakeMode() {
      return mIsBrakeMode;
    }
    
    public void ApplyDriveSignal(double throttle) {
      double mThrottle = throttle;
      mTransferWheel.set(mThrottle);
    }

    public void setOpenLoopControl() {
      setBrakeMode(true);
    }

    public void setOpenLoopOutput(double zTransferWheel) {
      ApplyDriveSignal(zTransferWheel);
    }

    private void setBrakeMode(boolean b) {
    }

    public TransferWheel() {
      mIsBrakeMode = false;
      setBrakeMode(true);
  
      mTransferWheel = new CANSparkMax (Constants.kTransferWheel, Constants. kTransferWheelType);

      System.out.println("TransferWheel created");

      // This is inverted alongside the joystick inputs in order to create working limit switches
      mTransferWheel.setInverted(true);
  
      // Start off in open loop control
      setOpenLoopControl();
    }
  
    public static void initDefaultSetup() {
    }
		
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mTransferWheel.set(RobotContainer.mOperatorJoystick.getRawAxis(3) * -1.0);
  }

}
      
