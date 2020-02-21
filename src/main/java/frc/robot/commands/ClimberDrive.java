package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ClimberDrive extends CommandBase {
    @SuppressWarnings( { "PMD.UnusedPrivateField", "PMD.SingularField" } )
    private final Climber mClimber;
    private final XboxController mOperatorXbox;

    @Override
    public void initialize () {}

    @Override
    public void execute() {
            // Climber( mOperatorXbox.getY( Hand.kLeft ), mOperatorXbox.getX( Hand.kLeft ) );
    }

    @Override
    public void end ( boolean interrupted ) {
    } 

    @Override
    public boolean isFinished () {
      return false;
    }

    public ClimberDrive ( Climber climber, XboxController operatorXbox ) {
        mClimber = climber;
        mOperatorXbox = operatorXbox;
        addRequirements( mClimber );
    }

}

