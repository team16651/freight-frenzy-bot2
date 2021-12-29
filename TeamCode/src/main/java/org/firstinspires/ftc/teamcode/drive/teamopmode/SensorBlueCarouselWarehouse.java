package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives back and forth in a straight line indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin moving forward and
 * backward. You should observe the target position (green) and your pose estimate (blue) and adjust
 * your follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 *
 * This opmode is designed as a convenient, coarse tuning for the follower PID coefficients. It
 * is recommended that you use the FollowerPIDTuner opmode for further fine tuning.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class SensorBlueCarouselWarehouse extends LinearOpMode {

    CarouselSpinner carouselSpinner = null;
    DcMotor carouselSpinnerMotor = null;

    Arm arm = null;
    DcMotor armMotor = null;
    Servo handServo = null;

    ShippingElementDetector pringles = null;
    DistanceSensor leftSensor = null;
    DistanceSensor rightSensor = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo, true);
        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        pringles = new ShippingElementDetector(leftSensor, rightSensor);

        Trajectory toCarouselSpinner = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d( 0,-3.5))
                .build();

        Trajectory toCarouselSpinner2 = drive.trajectoryBuilder(toCarouselSpinner.end())
                .lineToLinearHeading(new Pose2d(-5.1, -3.5, 5.6),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        Trajectory avoidDuck = drive.trajectoryBuilder(toCarouselSpinner2.end())
                .lineToLinearHeading(new Pose2d(-4.1, -8.5, 5.6))
                .build();

        Trajectory toDetect = drive.trajectoryBuilder(avoidDuck.end())
                .lineToLinearHeading(new Pose2d(14.834, -18.752, 4.712))
                .build();

        Trajectory toShippingMid = drive.trajectoryBuilder(toDetect.end())
                .lineToLinearHeading(new Pose2d(36.049, -3.5, 5.05))
                .build();



        waitForStart();

        arm.grab();
        this.sleep(1500);
        arm.move(Arm.MID_POSITION);
        this.sleep(500);
        drive.followTrajectory(toCarouselSpinner);
        this.sleep(500);
        drive.followTrajectory(toCarouselSpinner2);
        carouselSpinner.spin(false, 0.5);
        this.sleep(2500);
        carouselSpinner.stop();
        drive.followTrajectory(avoidDuck);
        this.sleep(500);
        drive.followTrajectory(toDetect);

        String location = pringles.detectPringle();
        double yOffset = 0;
        if (location.equals(ShippingElementDetector.RIGHT)){
            arm.move(Arm.HIGH_POSITION);
            yOffset = -2;
        }else if (location.equals(ShippingElementDetector.NEITHER)){
            arm.move(Arm.LOW_POSITION);
            yOffset = -3;
        }

        Trajectory toShippingMid2 = drive.trajectoryBuilder(toShippingMid.end())
                .lineToLinearHeading(new Pose2d(36.049, -19.385 + yOffset, 5.05))
                .build();

        Trajectory toPark = drive.trajectoryBuilder(toShippingMid2.end())
                .lineToLinearHeading(new Pose2d(41.688, 0.75, 0.2))
                .build();

        Trajectory toPark2 = drive.trajectoryBuilder(toPark.end())
                .lineToLinearHeading(new Pose2d(96.044, 0.75, 0.2))
                .build();

        drive.followTrajectory(toShippingMid);
        this.sleep(500);
        drive.followTrajectory(toShippingMid2);
        arm.release();
        this.sleep(1500);
        drive.followTrajectory(toPark);
        this.sleep(500);
        drive.followTrajectory(toPark2);
        arm.move(Arm.PARK_POSITION);

    }
}