package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

public class IntakeSubsystem extends SubsystemBase {

    public Motor intake;
    private boolean direction;
    public static final double intakeSpeed = 0.77; // 1150 RPM * 0.77 (or / 77%) = 885.5

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1150);

        intake.setInverted(true); // change to true if initially out-taking
        direction = intake.getInverted();
    }

    public void runIntake() {
        intake.set(intakeSpeed);
    }

    public void stopIntake() {
        intake.stopMotor();
    }

    public void invertIntake() {
        if(direction) {
            direction = false;
        } else {
            direction = true;
        }

        intake.setInverted(direction);
    }

    public RunCommand runIntakeCommand() {
        return new RunCommand(() -> runIntake());
    }

    public InstantCommand stopIntakeCommand() {
        return new InstantCommand(() -> stopIntake());
    }

    public InstantCommand invertIntakeCommand() {
        return new InstantCommand(() -> invertIntake());
    }
}
