package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

@Config
@Autonomous(group = "drive")
public class SensorBlueCarouselStorage extends LinearOpMode {

    private CarouselSpinner carouselSpinner = null;
    private DcMotor carouselSpinnerMotor = null;

    private Arm arm = null;
    private DcMotor armMotor = null;
    private Servo handServo = null;

    private ShippingElementDetector shippingElementDetector = null;
    private DistanceSensor leftSensor = null;
    private DistanceSensor rightSensor = null;

    private Pose2d poseHome = new Pose2d(0, 0, 0);
    private Pose2d poseRunAway = new Pose2d(2, 0, 0);
    private Pose2d poseDetectRandomization = new Pose2d(13,0,0);
    private Pose2d poseShippingHub1 = new Pose2d(8.5, 20.08, 0);
    private Pose2d poseShippingHub2 = new Pose2d(23.32, 20.08, 0.52);
    private Pose2d poseCarousel1 = new Pose2d(8.5,0,0);
    private Pose2d poseCarousel2 = new Pose2d(17.14, -24.05, 3.21);
    private Pose2d poseCarousel3 = new Pose2d(9, -24.05, 3.21);
    private Pose2d poseStorage = new Pose2d(34.17, -22.05, 3.21);

    private Trajectory trajectoryHomeToRunAway = null;
    private Trajectory trajectoryRunAwayToDetect = null;
    private Trajectory trajectoryDetectToShippingHub1 = null;
    private Trajectory trajectoryShippingHub1ToShippingHub2 = null;
    private Trajectory trajectoryShippingHub2ToCarousel1 = null;
    private Trajectory trajectoryCarousel1ToCarousel2 = null;
    private Trajectory trajectoryCarosoel2ToCarousel3 = null;
    private Trajectory trajectoryCarousel3ToStorage = null;

    private int armPosition = Arm.MID_POSITION;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);

        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo);

        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        shippingElementDetector = new ShippingElementDetector(leftSensor, rightSensor);

        waitForStart();

        runAutonomous(drive, carouselSpinner, arm, shippingElementDetector);
    }

    private void runAutonomous(SampleMecanumDrive drive, CarouselSpinner carouselSpinner, Arm arm, ShippingElementDetector shippingElementDetector) {
        arm.grab();
        sleep(1500);

        driveToDetect(drive, arm);
        double xOffset = detectShippingElementAndReturnXOffset(shippingElementDetector);
        driveToShippingHub(drive, arm, xOffset);
        arm.release();
        sleep(1000);
        driveToCarousel(drive, arm);
        spinCarousel(carouselSpinner);
        driveToPark(drive);
    }



    private void driveToDetect(SampleMecanumDrive drive, Arm arm){
        trajectoryHomeToRunAway = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseRunAway)
                .build();
        trajectoryRunAwayToDetect = drive.trajectoryBuilder(poseRunAway)
                .lineToLinearHeading(poseDetectRandomization,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        drive.followTrajectory(trajectoryHomeToRunAway);
        arm.move(Arm.MID_POSITION);
        sleep(1000);
        drive.followTrajectory(trajectoryRunAwayToDetect);
    }

    private double detectShippingElementAndReturnXOffset(ShippingElementDetector shippingElementDetector){
        String location = shippingElementDetector.detectPringle();
        double xOffset = 3.5;

        if (ShippingElementDetector.RIGHT.equals(location)){
            armPosition = Arm.HIGH_POSITION;
            xOffset = 5;
        }
        else if (ShippingElementDetector.NEITHER.equals(location)){
            armPosition = Arm.LOW_POSITION;
            xOffset = 2;
        }

        return xOffset;
    }

    private void driveToShippingHub(SampleMecanumDrive drive, Arm arm, double xOffset){
        poseShippingHub2 = poseShippingHub2.plus(new Pose2d(xOffset, 0, 0));

        trajectoryDetectToShippingHub1 = drive.trajectoryBuilder(trajectoryRunAwayToDetect.end())
                .lineToLinearHeading(poseShippingHub1)
                .build();

        trajectoryShippingHub1ToShippingHub2 = drive.trajectoryBuilder(trajectoryDetectToShippingHub1.end())
                .lineToLinearHeading(poseShippingHub2)
                .build();

        drive.followTrajectory(trajectoryDetectToShippingHub1);

        arm.move(armPosition);

        sleep(1000);

        drive.followTrajectory(trajectoryShippingHub1ToShippingHub2);
    }

    private void driveToCarousel(SampleMecanumDrive drive, Arm arm){
        trajectoryShippingHub2ToCarousel1 = drive.trajectoryBuilder(trajectoryShippingHub1ToShippingHub2.end())
                .lineToLinearHeading(poseCarousel1)
                .build();

        trajectoryCarousel1ToCarousel2 = drive.trajectoryBuilder(trajectoryShippingHub2ToCarousel1.end())
                .lineToLinearHeading(poseCarousel2)
                .build();

        trajectoryCarosoel2ToCarousel3 = drive.trajectoryBuilder(trajectoryCarousel1ToCarousel2.end())
                .lineToLinearHeading(poseCarousel3,
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL/2))
                .build();

        drive.followTrajectory(trajectoryShippingHub2ToCarousel1);
        arm.grab();
        arm.move(Arm.PARK_POSITION);
        drive.followTrajectory(trajectoryCarousel1ToCarousel2);
        drive.followTrajectory(trajectoryCarosoel2ToCarousel3);
    }

    private void spinCarousel(CarouselSpinner carouselSpinner){
        carouselSpinner.spin(false, 0.4);
        this.sleep(3000);
        carouselSpinner.stop();
    }

    private void driveToPark(SampleMecanumDrive drive){
        trajectoryCarousel3ToStorage = drive.trajectoryBuilder(trajectoryCarosoel2ToCarousel3.end())
                .lineToLinearHeading(poseStorage)
                .build();

        drive.followTrajectory(trajectoryCarousel3ToStorage);
    }
}
