package org.firstinspires.ftc.teamcode.utils;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Central tuning/config file for the mecanum drivebase.
 *
 * This wires the goBILDA Pinpoint odometry computer and four drive motors into a
 * Pedro Pathing {@link Follower}. The Follower does all of the localization math and
 * PID-based path following; {@code MecanumDriveSubsystem} just wraps it in a
 * SolversLib {@code SubsystemBase} so it plays nicely with the command-based framework.
 *
 * TODO: Every numeric value below is a placeholder. Follow the Pedro Pathing tuning
 * guide (<a href="https://pedropathing.com/docs/pathing/tuning">...</a>) to tune forwardZeroPowerAcceleration,
 * lateralZeroPowerAcceleration, xVelocity/yVelocity, and the PIDF coefficients for your robot.
 * TODO: Update the hardware map config names below to match your actual robot configuration.
 */
public class Constants {

    // ---- Hardware map device names (must match your Driver Station robot config) ----
    public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    public static final String BACK_RIGHT_MOTOR_NAME = "backRight";
    public static final String PINPOINT_NAME = "pinpoint";

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.0) // kg - weigh your robot
            .forwardZeroPowerAcceleration(-25.9346931313679598) // TODO: tune
            .lateralZeroPowerAcceleration(-67.342491844080064) // TODO: tune
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.03, 0, 0, 0.015
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.4, 0, 0.005, 0.0006
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.8, 0, 0, 0.01
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2.5, 0, 0.1, 0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.1, 0, 0.00035, 0.6, 0.015
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02, 0, 0.000005, 0.6, 0.01
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName(FRONT_LEFT_MOTOR_NAME)
            .leftRearMotorName(BACK_LEFT_MOTOR_NAME)
            .rightFrontMotorName(FRONT_RIGHT_MOTOR_NAME)
            .rightRearMotorName(BACK_RIGHT_MOTOR_NAME)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(78.26) // TODO: tune, in inches/sec
            .yVelocity(61.49); // TODO: tune, in inches/sec

    // goBILDA Pinpoint odometry computer configuration.
    // forwardPodY / strafePodX are the offsets (in inches) of the odometry pods from the
    // robot's center of rotation. See https://pedropathing.com/docs/pathing/tuning/localization/pinpoint
    public static PinpointConstants localizerConstantsPinpoint = new PinpointConstants()
            .hardwareMapName(PINPOINT_NAME)
            .forwardPodY(0.75) // TODO: measure on your robot
            .strafePodX(-6.6) // TODO: measure on your robot
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);


    public static ThreeWheelConstants localizer3ConstantsThreeWheel = new ThreeWheelConstants()
            .forwardTicksToInches(.001989436789)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(.001989436789)
            .leftPodY(1)
            .rightPodY(-1)
            .strafePodX(-2.5)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);

    /*
     * PathConstraints, in order: tValueConstraint, velocityConstraint, translationalConstraint,
     * headingConstraint, timeoutConstraint, brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart.
     * Leave BEZIER_CURVE_SEARCH_LIMIT at 10 unless you know you need to change it.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.995, 0.1, 0.1, 0.009, 50, 1.25, 10, 1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizer3ConstantsThreeWheel)
              //  .pinpointLocalizer(localizerConstantsPinpoint)
                .pathConstraints(pathConstraints)
                .build();
    }
}
