package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

@Config
@Autonomous(group = "drive")
public class SensorBlueWarehouseOutward extends LinearOpMode {

    private Arm arm = null;
    private DcMotor armMotor = null;
    private Servo handServo = null;

    private ShippingElementDetector shippingElementDetector = null;
    private DistanceSensor leftSensor = null;
    private DistanceSensor rightSensor = null;

    private Pose2d poseHome = new Pose2d(0, 0, 0);
    private Pose2d poseRunAway = new Pose2d(2, 0, 0);
    private Pose2d poseDetectRandomization = new Pose2d(20.68,0,0);
    private Pose2d poseShippingHub1 = new Pose2d(10, -5.54, 0);
    private Pose2d poseShippingHub2 = new Pose2d(25.81, -9.13, 5.76);
    private Pose2d poseHome2 = new Pose2d(0,0,1.57);
    private Pose2d poseWarehousePose = new Pose2d(0, 35, 1.57);

    protected Trajectory trajectoryHomeToRunAway = null;
    protected Trajectory trajectoryRunAwayToDetect = null;
    protected Trajectory trajectoryDetectToShippingHub1 = null;
    protected Trajectory trajectoryShippingHub1ToShippingHub2 = null;
    protected Trajectory trajectoryShippingHub2ToHome2 = null;
    protected Trajectory trajectoryHomeToWarehouse = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        arm = new Arm(armMotor, handServo);

        leftSensor = (DistanceSensor)hardwareMap.get("distanceLeft");
        rightSensor = (DistanceSensor)hardwareMap.get("distanceRight");
        shippingElementDetector = new ShippingElementDetector(leftSensor, rightSensor);

        waitForStart();

        runAutonomous(drive, arm, shippingElementDetector);
    }

    protected void runAutonomous(SampleMecanumDrive drive, Arm arm, ShippingElementDetector shippingElementDetector){
        arm.grab();
        sleep(1500);
        driveToDetect(drive, arm);
        double xOffset = detectShippingElementAndReturnXOffset(shippingElementDetector, arm);
        driveToShippingHub(drive, xOffset);
        arm.release();
        sleep(1000);
        driveToHome(drive);
        driveToWarehouse(drive);
        arm.move(Arm.PARK_POSITION);
        sleep(1000);
    }

    private void driveToDetect(SampleMecanumDrive drive, Arm arm){
        trajectoryHomeToRunAway = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseRunAway)
                .build();

        trajectoryRunAwayToDetect = drive.trajectoryBuilder(poseRunAway)
                .lineToLinearHeading(poseDetectRandomization)
                .build();

        drive.followTrajectory(trajectoryHomeToRunAway);
        arm.move(Arm.MID_POSITION);
        drive.followTrajectory(trajectoryRunAwayToDetect);
    }

    private double detectShippingElementAndReturnXOffset(ShippingElementDetector shippingElementDetector, Arm arm){
        String location = shippingElementDetector.detectPringle();
        double xOffset = 3.5;

        if (ShippingElementDetector.RIGHT.equals(location)){
            arm.move(Arm.HIGH_POSITION);
            xOffset = 5;
        }
        else if (ShippingElementDetector.NEITHER.equals(location)){
            arm.move(Arm.LOW_POSITION);
            xOffset = 2;
        }

        return xOffset;
    }

    private void driveToShippingHub(SampleMecanumDrive drive, double xOffset){

        poseShippingHub2 = poseShippingHub2.plus(new Pose2d(xOffset, 0, 0));

        trajectoryDetectToShippingHub1 = drive.trajectoryBuilder(trajectoryRunAwayToDetect.end())
                .lineToLinearHeading(poseShippingHub1)
                .build();

        trajectoryShippingHub1ToShippingHub2 = drive.trajectoryBuilder(trajectoryDetectToShippingHub1.end())
                .lineToLinearHeading(poseShippingHub2)
                .build();

        drive.followTrajectory(trajectoryDetectToShippingHub1);
        drive.followTrajectory(trajectoryShippingHub1ToShippingHub2);
    }

    private void driveToHome(SampleMecanumDrive drive){
        trajectoryShippingHub2ToHome2 = drive.trajectoryBuilder(trajectoryShippingHub1ToShippingHub2.end())
                .lineToLinearHeading(poseHome2)
                .build();

        drive.followTrajectory(trajectoryShippingHub2ToHome2);
    }

    private void driveToWarehouse(SampleMecanumDrive drive){
        trajectoryHomeToWarehouse = drive.trajectoryBuilder(trajectoryShippingHub2ToHome2.end())
                .lineToLinearHeading(poseWarehousePose)
                .build();

        drive.followTrajectory(trajectoryHomeToWarehouse);
    }
}