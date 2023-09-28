package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import CupertinoRobotics.support.Hardware.Gyro;

import java.util.List;

@Autonomous (name = "CV FTC Test-TF")
public class ComputerVisionFTC_TF extends LinearOpMode {

    OpenCvCamera cam;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor backRightDrive;
    private DcMotor backLeftDrive;
    private final double ANGLE_TOLERANCE = 2;
    private final double SPIN_TOLERANCE = 1;
    private DcMotor armMotor = null;
    private TouchSensor touchSensor;
    private Servo leftServo;
    private Servo rightServo;

    static final double COUNTS_PER_MOTOR_REV = 580;
    //static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    BNO055IMU imu; //The gyro is a IMU (inertial measurement unit), and the class we can use to interface it is BNO055IMU
    private Gyro gyro = null;

    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName camera = hardwareMap.get(WebcamName.class, "Webcam 1");

        waitForStart();

        //Create AprilTag processor
        TfodProcessor tfodProcessor = TfodProcessor.easyCreateWithDefaults();
        //Create visionPortal with AprilTag processor
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, tfodProcessor);
        //Get detected tags from the AprilTag processor

        while (opModeIsActive()) {
            List<Recognition> recognitions = tfodProcessor.getRecognitions();
            telemetry.addData("num detections", recognitions.size());

            //Iterate through each detected tag in the list
            for (Recognition recognition : recognitions){
            //Get label of this recognized object
            String label = recognition.getLabel();
            //Get label of this recognized object
            float confidence = recognition.getConfidence();
            //Add this label and confidence to the telemetry
            telemetry.addLine("Recognized object: "+ label);
            telemetry.addLine("Confidence: " + confidence);
        }
            telemetry.update();
            sleep(50);
        }
    }
}
