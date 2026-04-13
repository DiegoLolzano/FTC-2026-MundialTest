package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class Constants {
    /*
     * TODO: PLACEHOLDER VALUES
     *  VALUES THAT NEED TO BE CHECKED:
     *  -MASS
     *  -DRIVE MOTOR DIRECTIONS
     *  -LOCALIZATION
     *  -PIDFs CONSTANTS
     */
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.5)
            .forwardZeroPowerAcceleration(-51.54322464112613)
            .lateralZeroPowerAcceleration(-110.48020786969806)
            .translationalPIDFCoefficients(new PIDFCoefficients(0,0,0,0))//.translationalPIDFCoefficients(new PIDFCoefficients(0.11,0,0.009,0.044))
            .headingPIDFCoefficients(new PIDFCoefficients(0,0,0,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0, 0, 0, 0, 0))
            .centripetalScaling(0.005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftFrontMotorName("lf")
            .leftRearMotorName("lr")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .useBrakeModeInTeleOp(true)
            .xVelocity(77.79242183655266)
            .yVelocity(52.73848838505783);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-4.33)//6.08
            .strafePodX(1.5)//0.2
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .yawScalar(1);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.999,
            0.1,
            0.1,
            0.009,
            50,
            0.9,
            10,
            0.9
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
