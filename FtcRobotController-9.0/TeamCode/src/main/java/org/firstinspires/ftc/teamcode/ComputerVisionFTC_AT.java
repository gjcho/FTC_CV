package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import CupertinoRobotics.support.Hardware.Gyro;

import java.util.List;

@Autonomous (name = "CV FTC Test-AT")
public class ComputerVisionFTC_AT extends LinearOpMode {

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
//        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, visionProcessors);

        // Get current frame rate to estimate CPU load
//        float fps = visionPortal.getFps();
        //Disable features to manage the CPU load
//        visionPortal.setProcessorEnabled(visionProcessor, false);
//        visionPortal.stopLiveView();
//        visionPortal.stopStreaming();
//        //Close VisionPortal to stop everything
//        visionPortal.close(); //**

        //Create TensorFlow Processor
//        TfodProcessor tfodProcessor = TfodProcessor.easyCreateWithDefaults();
//        //Create visionPortal with TensorFlow processor
//        VisionPortal visionPortalTF = VisionPortal.easyCreateWithDefaults(camera, tfodProcessor);
//
//        visionPortalTF.setProcessorEnabled(tfodProcessor, true);
//
//        //get recognized objects from TensorFlow
//        List<Recognition> recognitions = tfodProcessor.getRecognitions();
//        //Iterate through each recognized object in the list
//        for (Recognition recognition : recognitions){
//            //Get label of this recognized object
//            String label = recognition.getLabel();
//            //Get label of this recognized object
//            float confidence = recognition.getConfidence();
//            //Add this label and confidence to the telemetry
//            telemetry.addLine("Recognized object: "+ label);
//            telemetry.addLine("Confidence: " + confidence);
//        }

        waitForStart();

        //Create AprilTag processor
        AprilTagProcessor aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        //Create visionPortal with AprilTag processor
        VisionPortal visionPortal = VisionPortal.easyCreateWithDefaults(camera, aprilTagProcessor);
        //Get detected tags from the AprilTag processor

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            telemetry.addData("num detections", detections.size());

            //Iterate through each detected tag in the list
            for(AprilTagDetection detection : detections){
                //Get tag ID number
                int id = detection.id;

                //Get post information of this tag
                AprilTagPoseFtc tagPose = detection.ftcPose;

                //Add this tag's information to the telemetry
                telemetry.addLine("Detection tag ID: " + id);
                telemetry.addLine("Distance to tag: " + tagPose.range);
                telemetry.addLine("Bearing to tag: " + tagPose.bearing);
                telemetry.addLine("Angle to tag " + tagPose.yaw);
            }
            telemetry.update();
            sleep(50);
        }
    }
}
