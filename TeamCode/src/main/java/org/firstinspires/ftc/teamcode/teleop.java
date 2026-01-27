package org.firstinspires.ftc.teamcode;

import com.google.gson.Gson;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import java.net.InetSocketAddress;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TelemetryWebsocketsServer;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.java_websocket.server.WebSocketServer;
//import org.json.simple.JSONObject;

import java.text.DecimalFormat;
import java.util.LinkedHashMap;
import java.util.List;

@TeleOp(name = "AndroidStudioTeleop")
public class teleop extends LinearOpMode {
    private AprilTagProcessor myAprilTagProcessor;

    private VisionPortal visionPortal;

    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;
    private DcMotor flywheelMotorL;
    private DcMotor flywheelMotorR;
//    private Servo distanceSensorServo;
    private IMU imu_IMU;

    private CRServo sweeperServoA;
    private CRServo sweeperServoB;
    private CRServo sweeperServoC;
    private DistanceSensor backDistanceSensor;

//    private CRServo sweeperServoD;
//    private CRServo sweeperServo;
//    private DistanceSensor turningDistanceSensor;
//    private DistanceSensor rightDistanceSensor;

    double headingDiff;
    double targetHeading = 0;
    double currentHeading = 0;
    double rightJoyReleasedAt = 0;


    boolean rightJoyWasActive = false;
    double SIDEWAYS_CORRECTION_FACTOR = 0.09;
    int ANGLE_TOLERANCE = 1;
    double TURN_SETTLE_SECS = 0.2;
    boolean targetHeadingRecorded = true; //its set just above to 0 when the robot starts

    boolean websocketsTelemetryEnabled = false;
    WebSocketServer server;
    LinkedHashMap<String, LinkedHashMap<String, Object>> telemetryDataDict = new LinkedHashMap<>();


//

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */


    @Override
    public void runOpMode() {


        setup();
        waitForStart();
        if (opModeIsActive()) {
//            distanceSensorServo.setPosition(0.25);
            while (opModeIsActive()) {
                main_loop();
            }
            if (websocketsTelemetryEnabled) {
                try {
                    server.stop();
                } catch (Exception e) {
                    //pass
                }
            }
        }
    }


    private void collectTelemetry(String category, String key, Object value){
        String valueToAdd;
        if (value instanceof String) {
            valueToAdd = (String) value;
        } else if (value instanceof Integer) {
            valueToAdd = value.toString();
        } else if (value instanceof Double) {
            DecimalFormat df = new DecimalFormat("0.00");
            valueToAdd =  df.format(value);
        } else {
            valueToAdd = value.toString();
        }


        if (!telemetryDataDict.containsKey(category)) {
            telemetryDataDict.put(category, new LinkedHashMap<String, Object>());
        }

        telemetryDataDict.get(category).put(key, valueToAdd);
        //Add it to the controller screen too.
        telemetry.addData(key, value);
    }

    private void submitTelemetry() {
        telemetry.update();

        if (websocketsTelemetryEnabled) {
            Gson gson = new Gson();
            server.broadcast(gson.toJson(telemetryDataDict));
        }
        telemetryDataDict.clear();

    }

    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        flywheelMotorL = hardwareMap.get(DcMotor.class, "flywheelMotorL");
        flywheelMotorR = hardwareMap.get(DcMotor.class, "flywheelMotorR");
//        distanceSensorServo = hardwareMap.get(Servo.class, "distanceSensorServo");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        sweeperServoA = hardwareMap.get(CRServo.class, "sweeperServoA");
        sweeperServoB = hardwareMap.get(CRServo.class, "sweeperServoB");
        sweeperServoC = hardwareMap.get(CRServo.class, "sweeperServoC");

        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");
        //        sweeperServoD = hardwareMap.get(CRServo.class, "sweeperServoD");
//        turningDistanceSensor = hardwareMap.get(DistanceSensor.class, "turningDistanceSensor");
//        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
    }
    private void initializeIMU() {
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
//        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu_IMU.resetYaw();
    }

    private void configureMotors() {
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Enable the encoders on them.
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Make right motors go reverse
        FrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotor.Direction.REVERSE);
        // Make all the manipulating motors brake

        flywheelMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        sweeperServoA.setPower(0);
        sweeperServoB.setPower(0);
        sweeperServoC.setPower(0);
    }

    private void setup() {
        initializeHardware();
        initializeIMU();
        initAprilTag();
        configureMotors();
        telemetry.setNumDecimalPlaces(0, 4);
        if (websocketsTelemetryEnabled) {
           server = new TelemetryWebsocketsServer(new InetSocketAddress("0.0.0.0", 8765));
            server.start();
        }
        collectTelemetry("Message", "Ready: ", "Waiting for start.");
        submitTelemetry();
    }

    private void main_loop() {
        update_manipulators();
        update_movers();
        update_telemetry();
        telemetryAprilTag();

    }

    private double scaleSpeed(double somethingToScale) {
        double scaledValue;

        if (somethingToScale >= 0) {
            scaledValue = 1 * (Math.log(9 * Math.min(Math.max(somethingToScale, 0), 1) + 1) / Math.log(10));
        } else {
            scaledValue = -(1 * (Math.log(9 * Math.min(Math.max(-somethingToScale, 0), 1) + 1) / Math.log(10)));
        }
        return scaledValue;
    }

    private double getHeading() {
        double botHeadingDeg = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        collectTelemetry("IMU","Yaw (heading)", botHeadingDeg);
        collectTelemetry("IMU", "YawPitchRoll", imu_IMU.getRobotYawPitchRollAngles().toString());
        return botHeadingDeg;
    }

    private double getHeadingDiff(double targetHeading, double currentHeading) {
        if (Math.abs(targetHeading) == 180) {
            // Since 180 = -180, you can pretty much go both ways, depending on which is closer
            if (currentHeading >= 0) {
                headingDiff = 180 - currentHeading;
            } else {
                headingDiff = -180 - currentHeading;
            }
        } else {
            if ((targetHeading < -90) && currentHeading > 90) {
                targetHeading += 360;
            } else if ((targetHeading > 90) && currentHeading < -90) {
                targetHeading -= 360;
            }
            headingDiff = targetHeading - currentHeading;
        }
        return headingDiff;
    }

    private double getCorrectionSpeed(double currentSpeed) {
        double coastingSpeed;
        currentSpeed = Math.abs(currentSpeed);
        if (currentSpeed == 0) {
            return 0.05; //Make it correct quickly when it stops
        }

        if (currentSpeed < 0.219512) {
            coastingSpeed = currentSpeed / 2;
        }
        else {
            coastingSpeed = 0.09 * currentSpeed + 0.09;
        }
        collectTelemetry("Steering", "calculatedCorrectionSpeed", coastingSpeed);
        return coastingSpeed;
    }


    private void setModeAll_stopAndReset() {
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setModeAll_runUsingEncoder() {
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetMotors() {
        setModeAll_stopAndReset();
        setModeAll_runUsingEncoder();
    }
    private void initAprilTag() {

        // Create the AprilTag processor.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                //.build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "LogitechC310"));
        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(myAprilTagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = myAprilTagProcessor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
    private void update_telemetry() {
//        DecimalFormat df = new DecimalFormat("0.00");
        collectTelemetry("Decorative", "SECTION", "POWER");
        // Front Motors
        collectTelemetry("Power", "Front Left", FrontLeftMotor.getPower());
        collectTelemetry("Power", "Front Right", FrontRightMotor.getPower());
        // Back Motors
        collectTelemetry("Power", "Back Left", BackLeftMotor.getPower());
        collectTelemetry("Power", "Back Right", BackRightMotor.getPower());
        // Manipulating Motors
        collectTelemetry("Power", "Flywheel", flywheelMotorL.getPower() + " & " + flywheelMotorR.getPower());
        // POSITIONING
//        collectTelemetry("Position", "SECTION", "ARM POSITIONS");
        collectTelemetry("Position", "SECTION", "WHEEL POSITIONS");
        collectTelemetry("Position", "Front", FrontLeftMotor.getCurrentPosition() + "     " + FrontRightMotor.getCurrentPosition());
        collectTelemetry("Position", "Back", BackLeftMotor.getCurrentPosition() + "     " + BackRightMotor.getCurrentPosition());
        // SERVOS& SENSORS
        collectTelemetry("Servo", "SweepyPower", sweeperServoA.getPower() + " & " + sweeperServoB.getPower() + " & "  + sweeperServoC.getPower());// + " & " + sweeperServoD.getPower());
        collectTelemetry("Distance", "Back", backDistanceSensor.getDistance(DistanceUnit.CM));

//        collectTelemetry("sensors", "TurningDistSensorCM", turningDistanceSensor.getDistance(DistanceUnit.CM));
//        collectTelemetry("sensors", "RightDistSensorCM", rightDistanceSensor.getDistance(DistanceUnit.CM));

        // Send Updates
        submitTelemetry();
    }


    private void update_manipulators() {

        if (gamepad1.x) {
            flywheelMotorL.setPower(0.8);
            flywheelMotorR.setPower(-0.8);
//            sweeperServo.setPower(1);
        } else if (gamepad1.y) {
            flywheelMotorL.setPower(0);
            flywheelMotorR.setPower(0);
        } else {
            //Don't do anything.
        }

        if (gamepad1.a) {
            sweeperServoA.setPower(-1);
            sweeperServoB.setPower(-1);
            sweeperServoC.setPower(-1);
//            sweeperServoD.setPower(0);
        } else if (gamepad1.b) {
//            trafficStopServo.setPosition(0.15);
            sweeperServoA.setPower(0);
            sweeperServoB.setPower(0);
            sweeperServoC.setPower(0);


        } else {
            // pass
        }
    }


    private void update_movers() {
        //Movers use gamepad1

        double speedMulti;
        double flTargetPower;
        double frTargetPower;
        float joyLeftX;
        double blTargetPower;
        double brTargetPower;
        float joyLeftY;
        double joyRightX;
        double joyRightY;
        double maxCalculatedPower;
        boolean correctionDisabled = false;

        if (gamepad1.right_bumper) {
            speedMulti = 0.3;
        } else {
            speedMulti = 1;
        }
        // Get Joystick Value
        joyLeftX = (float) scaleSpeed(-gamepad1.left_stick_x * 0.9);
        joyLeftY = (float) scaleSpeed(gamepad1.left_stick_y * 0.9);
        joyRightX = (float) scaleSpeed(-gamepad1.right_stick_x * 0.4);
        joyRightY = gamepad1.right_stick_y * 0.6;
        // Set target motor powers based on x, y, and yaw
        flTargetPower = (joyLeftY + joyLeftX + joyRightX) * speedMulti;
        frTargetPower = (joyLeftY + -joyLeftX + -joyRightX) * speedMulti;
        blTargetPower = (joyLeftY + -joyLeftX + joyRightX) * speedMulti;
        brTargetPower = (joyLeftY + joyLeftX + -joyRightX) * speedMulti;
        // Normalize Values if they happen to go above 1
        maxCalculatedPower = JavaUtil
                .maxOfList(JavaUtil.createListWith(flTargetPower, frTargetPower, blTargetPower, brTargetPower));
        if (maxCalculatedPower > 1) {
            flTargetPower = flTargetPower / maxCalculatedPower;
            frTargetPower = frTargetPower / maxCalculatedPower;
            blTargetPower = blTargetPower / maxCalculatedPower;
            brTargetPower = brTargetPower / maxCalculatedPower;

            targetHeadingRecorded = true;
        }
        // Activate heading corrections only when we're not steering
        if (joyRightX == 0) {

            // The robot (with its inertia) will continue to turn for a bit more even after we lift
            // our hands from the control and the turning power is cut. This code block checks if
            // the joystick was just released  more than 0.2s (TURN_SETTLE_SECS) and decides if the
            // heading correction should be activated.

            // Essentially, it should go like this:
            // 1. Upon initialization, the target heading is initialized as 0 (straight ahead), so
            // targetHeadingRecorded is set to true;
            // 2. The robot is moved using the left joystick (up/down/left/right). No right joystick
            // was inputted = no intention to change heading = same target heading is still valid.
            // 3. When right joy has input, the heading will be intentionally changed. The current
            // target heading is now invalid. However, we can't record the new target heading yet as
            // the joystick controls was just lifted and the robot still have turning motion. So, we
            // simply note down that we still need to record the heading (targetHeadingRecorded = false)
            // and record the current time for reference.
            // 4. The next time we arrive at this function, we check if the heading was recorded yet.
            // if we see that it's not, we check if the joystick has been released for more than 0.2s
            // (TURN_SETTLE_SECS), and if so, we record the heading and mark that the heading is
            // up-to-date. If not, we just do nothing, keeping the heading correction disabled until
            // the timer has passed and the robot has (hopefully) stopped turning.


            if (rightJoyWasActive) {
                rightJoyReleasedAt = getRuntime();
                collectTelemetry("Corrections", "Angle Timer", "Started");
                correctionDisabled = true;
                targetHeadingRecorded = false;
                rightJoyWasActive = false;
            } else if (!targetHeadingRecorded) {
                if ((getRuntime() - rightJoyReleasedAt) > TURN_SETTLE_SECS) {
                    targetHeading = getHeading();
                    collectTelemetry("Corrections", "Angle Timer", "Done");
                    targetHeadingRecorded = true;
                } else {
                    collectTelemetry("Corrections", "Angle Timer", "Waiting");
                    correctionDisabled = true;
                }
            }

            if (!correctionDisabled) {
                currentHeading = getHeading();
                headingDiff = getHeadingDiff(targetHeading, currentHeading);
                if (Math.abs(headingDiff) < ANGLE_TOLERANCE) {
                    // Within Threshold
                    collectTelemetry("Corrections","Turning Correction", "N/A");
                } else if (headingDiff < 0) {
                    // Need to go right
                    double correctionSpeed = getCorrectionSpeed(flTargetPower) * 0.8;
                    flTargetPower = flTargetPower - correctionSpeed;
                    frTargetPower = frTargetPower + correctionSpeed;
                    blTargetPower = blTargetPower - correctionSpeed;
                    brTargetPower = brTargetPower + correctionSpeed;
                    collectTelemetry("Corrections","Turning Correction", "Adjusting Right");
                } else if (headingDiff > 0) {
                    // Need to go left
                    double correctionSpeed = getCorrectionSpeed(flTargetPower);
                    flTargetPower = flTargetPower + correctionSpeed;
                    frTargetPower = frTargetPower - correctionSpeed;
                    blTargetPower = blTargetPower + correctionSpeed;
                    brTargetPower = brTargetPower - correctionSpeed;
                    collectTelemetry("Corrections","Turning Correction", "Adjusting Left");
                }
            }
        } else {
            rightJoyWasActive = true;
            //Right joystick is being controlled.
        }
        collectTelemetry("Corrections","TargetHeading", targetHeading);
//        collectTelemetry("CurrentHeading", currentHeading);
        maxCalculatedPower = JavaUtil
                .maxOfList(JavaUtil.createListWith(flTargetPower, frTargetPower, blTargetPower, brTargetPower));
        if (maxCalculatedPower > 1) {
            flTargetPower = flTargetPower / maxCalculatedPower;
            frTargetPower = frTargetPower / maxCalculatedPower;
            blTargetPower = blTargetPower / maxCalculatedPower;
            brTargetPower = brTargetPower / maxCalculatedPower;
        }
        // Set Motors
        FrontLeftMotor.setPower(flTargetPower);
        FrontRightMotor.setPower(frTargetPower);
        BackLeftMotor.setPower(blTargetPower);
        BackRightMotor.setPower(brTargetPower);
    }

}