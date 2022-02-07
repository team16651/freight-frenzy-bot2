package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.ShippingElementDetector;

@Config
@Autonomous(group = "drive")
public class SensorRedWarehouseInward extends SensorRedWarehouseOutward {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
    }

    @Override
    protected void runAutonomous(SampleMecanumDrive drive, Arm arm, ShippingElementDetector shippingElementDetector){
        super.runAutonomous(drive, arm, shippingElementDetector);

        Vector2d parkingPose = new Vector2d(26, -35);

        Trajectory trajectoryWarehouseToParking = drive.trajectoryBuilder(trajectoryHomeToWarehouse.end())
                .strafeTo(parkingPose)
                .build();

        drive.followTrajectory(trajectoryWarehouseToParking);
    }
}