package frc.robot.subsystems;

import frc.robot.Constants.INTAKE;
import frc.robot.lib.drivers.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public class Intake extends SubsystemBase {

    // Hardware
    private WPI_TalonSRX mMaster;

    // Logging
    private final Logger mLogger = LoggerFactory.getLogger( Intake.class );

    public Intake ( WPI_TalonSRX master ) {
        mMaster = master;
    }

    public static Intake create () {
        WPI_TalonSRX master = TalonSRX.createTalonSRXWithEncoder( new WPI_TalonSRX( INTAKE.MASTER_ID) );
        return new Intake( master );
    }

    @Override
    public void periodic () {

    }

}