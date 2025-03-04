package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake.CoralIntakeSubsystem;

public class CoralIntakeWheelsCommand extends Command {
    CoralIntakeSubsystem m_CoralIntakeSubsystem;

    public CoralIntakeWheelsCommand(CoralIntakeSubsystem input) {
        m_CoralIntakeSubsystem = input;
    }
    public InstantCommand turnMotor(double speed) {
        // double speed;
        // switch (m_CoralIntakeSubsystem.currentMode()) {
        //     case FLOOR:
        //         speed = Constants.CoralArmConstants.coral_intake_floor_speed;
        //         break;

        //     case HUMAN_STATION:
        //         speed = Constants.CoralArmConstants.coral_intake_humanStation_speed;
        //         break;

        //     case REEF:
        //         speed = Constants.CoralArmConstants.coral_reef_l1;
        //         break;
        //     default:
        //         speed = 0;
        // }
        return new InstantCommand(()-> 
            m_CoralIntakeSubsystem.intakeSpinWheels(speed)
        );
    }

    public InstantCommand stopMotors(){
        return new InstantCommand(()->
            m_CoralIntakeSubsystem.turnOffIntakeWheel()
        );
    }
}
