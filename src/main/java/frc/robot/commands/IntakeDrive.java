package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.INTAKE;

public class IntakeDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Intake mIntake;
    private final XboxController mDriverXbox;

    @Override
    public void initialize () {}

    @Override
    public void execute() {

        // drive off triggers 
        double xOuttake = ( -1.0 * mDriverXbox.getRawAxis(3));
        double xIntake = ( 1.0 * mDriverXbox.getRawAxis(2));

        if (xOuttake < -INTAKE.DEADBAND) {
            mIntake.setOpenLoop(xOuttake);
        } else if (xIntake > INTAKE.DEADBAND) {
            mIntake.setOpenLoop(xIntake);
        } else {
            mIntake.Stop();
        }
    
    }

    @Override
    public void end ( boolean interrupted ) {}

    @Override
    public boolean isFinished () {
      return false;
    }

    public IntakeDrive ( Intake intake, XboxController driverXbox ) {
        mIntake = intake;
        mDriverXbox = driverXbox;
        addRequirements( mIntake );
    }

}

