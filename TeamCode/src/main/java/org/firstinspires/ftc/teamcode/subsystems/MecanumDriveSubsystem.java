package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.utils.Drawing;
import org.firstinspires.ftc.teamcode.utils.Constants;

/**
 * Mecanum drivebase subsystem.
 *
 * Hardware/localization is delegated entirely to a Pedro Pathing {@link Follower}, which
 * owns the four drive motors and the goBILDA Pinpoint odometry computer (wired up in
 * {@link Constants}). This class is the SolversLib-facing wrapper: it registers with the
 * {@code CommandScheduler} as a normal {@code SubsystemBase}, drives the Follower's update
 * loop from {@link #periodic()}, pushes live pose/path data to the FTC Panels dashboard,
 * and exposes simple drive/path-following methods for Commands to call.
 *
 * Only one Command should control this subsystem at a time; the scheduler enforces that
 * automatically as long as commands declare it with {@code addRequirements(driveSubsystem)}.
 */
public class MecanumDriveSubsystem extends SubsystemBase {

    private final Follower follower;
    private final TelemetryManager panelsTelemetry;
    private boolean teleopDriveActive = false;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        Drawing.init();
    }

    public MecanumDriveSubsystem(HardwareMap hardwareMap, Pose startingPose) {
        this(hardwareMap);
        follower.setStartingPose(startingPose);
    }

    /**
     * Runs once per scheduler loop (SubsystemBase auto-registers this). Advances the
     * Follower's localization + path-following state machine and pushes telemetry to
     * both the Driver Station and the FTC Panels dashboard.
     */
    @Override
    public void periodic() {
        follower.update();
        Drawing.drawDebug(follower);

        panelsTelemetry.debug("drive/x (in)", follower.getPose().getX());
        panelsTelemetry.debug("drive/y (in)", follower.getPose().getY());
        panelsTelemetry.debug("drive/heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("drive/busy", follower.isBusy());
        panelsTelemetry.update();
    }

    /**
     * Switches the Follower into open, driver-controlled mode. Call this once before
     * feeding it stick inputs (a default teleop DriveCommand should do this in its
     * initialize()). Automatically re-armed by {@link #driveRobotCentric} /
     * {@link #driveFieldCentric} if a path-following command just finished.
     */
    public void startTeleopDrive() {
        follower.startTeleopDrive();
        teleopDriveActive = true;
    }

    /** Drives with stick input relative to the robot's own heading. */
    public void driveRobotCentric(double forward, double strafe, double turn) {
        if (!teleopDriveActive) startTeleopDrive();
        follower.setTeleOpDrive(forward, strafe, turn, true);
    }

    /** Drives with stick input relative to the field (forward is always "away from driver"). */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        if (!teleopDriveActive) startTeleopDrive();
        follower.setTeleOpDrive(forward, strafe, turn, false);
    }

    public void stop() {
        driveRobotCentric(0, 0, 0);
    }

    /** Follows an autonomous path and holds its end heading/position once finished. */
    public void followPath(PathChain path) {
        followPath(path, true);
    }

    public void followPath(PathChain path, boolean holdEnd) {
        teleopDriveActive = false;
        follower.followPath(path, holdEnd);
    }

    /** True while the Follower is actively executing a path. */
    public boolean isBusy() {
        return follower.isBusy();
    }

    public Pose getPose() {
        return follower.getPose();
    }

    /** Overwrites the Follower's pose estimate, e.g. after an AprilTag correction. */
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    /** Escape hatch for building PathChains (follower.pathBuilder()...) outside this class. */
    public Follower getFollower() {
        return follower;
    }
}
