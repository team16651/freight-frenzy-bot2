package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.CarouselSpinner;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class RedTeleOp extends LinearOpMode {

    static final double THROTTLE = 0.75;
    static final double STRAFE_THROTTLE = 0.5;
    static final double ROTATION_THROTTLE = 0.5;

    CarouselSpinner carouselSpinner;
    DcMotor carouselSpinnerMotor;

    Arm arm;
    DcMotor armMotor;
    Servo handServo;
    DigitalChannel armTouchSensor;
    Servo cappingServo;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        carouselSpinnerMotor = (DcMotor) hardwareMap.get("carouselSpinner");
        carouselSpinner = new CarouselSpinner(carouselSpinnerMotor);

        armMotor = (DcMotor)hardwareMap.get("armMotor");
        handServo = (Servo)hardwareMap.get("handServo");
        armTouchSensor = (DigitalChannel)hardwareMap.get("touch");
        cappingServo = (Servo)hardwareMap.get("cappingServo");
        arm = new Arm(armMotor, handServo, armTouchSensor, cappingServo);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        cappingServo.setPosition(1);

        while (!isStopRequested()) {
            double throttle = THROTTLE;
            double strafeThrottle = STRAFE_THROTTLE;
            double rotateThrottle = ROTATION_THROTTLE;
            if (arm.isExtended()){
                throttle = throttle/2;
                strafeThrottle = strafeThrottle/2;
                rotateThrottle = rotateThrottle/2;
            }

            /* Controller 1 */
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * throttle,
                            -gamepad1.left_stick_x * strafeThrottle,
                            -gamepad1.right_stick_x * rotateThrottle
                    )
            );

            drive.update();

            if (gamepad1.right_trigger > 0){
                spinCarousel(gamepad1.right_trigger);
            }
            else{
                carouselSpinner.stop();
            }

            /* Controller 2 */
            if (gamepad2.left_trigger > 0){
                arm.rotate(gamepad2.left_trigger, true);
            }
            else if (gamepad2.right_trigger > 0){
                arm.rotate(gamepad2.right_trigger, false);
            }
            else {
                arm.rotate(0.0, true);
            }

            if (gamepad2.y){
                arm.release();
            }

            if (gamepad2.a){
                arm.grab();
            }

            if (gamepad2.x){
                arm.extend();
            }

            if (gamepad2.b){
                arm.retract();
            }

            telemetry.addData("arm motor position: ", armMotor.getCurrentPosition());
            telemetry.addData("servo position", handServo.getPosition());
            telemetry.addData("capping servo position", cappingServo.getPosition());
            telemetry.update();
        }
    }

    protected void spinCarousel(double speed){
        carouselSpinner.spin(true, speed);
    }
}

