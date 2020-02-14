package frc.robot.subsystems;

import frc.robot.Constants.TRANSFER;
import frc.robot.lib.drivers.Photoeye;
import frc.robot.lib.drivers.SparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class TransferWheel extends SubsystemBase {

    // Just to test pull request

    // Hardware
    private final CANSparkMax mTransferMotor;
    private final Photoeye mTransferPhotoeye = new Photoeye( TRANSFER.TRANSFER_PHOTOEYE_ANALOG_CHANNEL );

    // Hardware states
    private boolean mIsBrakeMode;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( TransferWheel.class );

    public TransferWheel ( CANSparkMax master ) {
        mMaster = master;
    }

    public static TransferWheel create () {
        // Talon's and Victor's go through a custom wrapper for creation
        CANSparkMax transferMotor = new CANSparkMax( TRANSFER.TRANSFER_MOTOR_ID, MotorType.kBrushless );
        return new TransferWheel( transferMotor );
    }

    @Override
    public void periodic () {

    }

}

