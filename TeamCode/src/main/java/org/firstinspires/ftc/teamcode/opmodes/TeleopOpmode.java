package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

//@Autonomous(name = "Blank")
@TeleOp(name = "Teleop")
//@Disabled

public class TeleopOpmode extends CommandOpMode {

    TelemetryManager telemetryM;
    GamepadEx driverGamepad;
    MecanumDriveSubsystem drive;
    IntakeSubsystem intake;

    @Override
    public void initialize() {

        driverGamepad = new GamepadEx(gamepad1);
        drive = new MecanumDriveSubsystem(this.hardwareMap, new Pose());
        intake = new IntakeSubsystem(hardwareMap);

        drive.setDefaultCommand(new DriveCommand(drive,
                () -> driverGamepad.getLeftY(),
                () -> driverGamepad.getLeftX(),
                () -> driverGamepad.getRightX()));

        driverGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
            .whenHeld(intake.runIntakeCommand())
            .whenReleased(intake.stopIntakeCommand());

        driverGamepad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(intake.invertIntakeCommand());


        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        telemetryM.update(telemetry);


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();

            drive.setRobotCentric(driverGamepad.getGamepadButton(
                    GamepadKeys.Button.RIGHT_BUMPER).get());

            telemetryM.update(telemetry);
        }
        reset();
    }

}