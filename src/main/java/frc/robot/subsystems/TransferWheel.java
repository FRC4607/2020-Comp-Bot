package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Flywheel.FlywheelJoystick;
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

    public TransferWheel(CANSparkMax master) {
      mTransferWheel = master;
  
      mIsBrakeMode = false;
      setBrakeMode(true);
  
      System.out.println("TransferWheel created");

      // This is inverted alongside the joystick inputs in order to create working limit switches
      mTransferWheel.setInverted(true);
  
      // Start off in open loop control
      setOpenLoopControl();
    }

    private void setBrakeMode(boolean b) {
    }

    public TransferWheel() {
    }
  
    public static TransferWheel create() {
      CANSparkMax mTransferMotor = new CANSparkMax (Constants.kTransferWheel, Constants. kTransferWheelType);
     
      return new TransferWheel(mTransferMotor);
    }
  
    
    public static void initDefaultSetup() {
    }
		
	}
      
