package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


import java.net.HttpURLConnection;
import java.net.InetSocketAddress;
import java.net.URL;

import org.firstinspires.ftc.teamcode.util.TelemetryWebsocketsServer;
import org.java_websocket.server.WebSocketServer;
import org.json.simple.JSONObject;

import java.io.OutputStreamWriter;
import java.text.DecimalFormat;

@TeleOp(name = "helloAndroidStudio2")
public class experimentalTeleop extends LinearOpMode {

    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;
    private DcMotor hMotor;
    private DcMotor mMotor;
    private Servo distanceSensorServo;
    private IMU imu_IMU;
    private Servo clawServo;
    private CRServo sweeperServo;
    private DistanceSensor turningDistanceSensor;
    private DistanceSensor rightDistanceSensor;
    private DcMotor pulleyMotor;

    double headingDiff;
    double targetHeading = 0;
    double currentHeading = 0;
    double rightJoyReleasedAt = 0;


    boolean rightJoyWasActive = false;
    double SIDEWAYS_CORRECTION_FACTOR = 0.09;
    int ANGLE_TOLERANCE = 1;
    double TURN_SETTLE_SECS = 0.2;
    boolean setHeadingDone = true; //its set just above to 0 when the robot starts
    WebSocketServer server = new TelemetryWebsocketsServer(new InetSocketAddress("0.0.0.0", 8765));
    JSONObject telemetryDataDict  = new JSONObject();

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */


    @Override
    public void runOpMode() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");
        hMotor = hardwareMap.get(DcMotor.class, "hMotor");
        mMotor = hardwareMap.get(DcMotor.class, "mMotor");
        distanceSensorServo = hardwareMap.get(Servo.class, "distanceSensorServo");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        sweeperServo = hardwareMap.get(CRServo.class, "sweeperServo");
        turningDistanceSensor = hardwareMap.get(DistanceSensor.class, "turningDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        pulleyMotor = hardwareMap.get(DcMotor.class, "pulleyMotor");

        setup();
        collectTelemetry("Ready: ", "Waiting for start.");
        submitTelemetry();
        waitForStart();
        if (opModeIsActive()) {
            distanceSensorServo.setPosition(0.25);
            while (opModeIsActive()) {
                main_loop();
            }
        }
    }


    private void collectTelemetry(String key, Object value){
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
        telemetryDataDict.put(key, valueToAdd);
        telemetry.addData(key, value);
    }

    private void submitTelemetry() {
        telemetry.update();
        server.broadcast(telemetryDataDict.toJSONString());
        telemetryDataDict.clear();
    }

    private void initializeIMU() {
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu_IMU.resetYaw();
    }

    private void setup() {
        // Put initialization blocks here.
        telemetry.setNumDecimalPlaces(0, 4);
        // Make all moving motors float
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
        hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Enter your comment here!
        initializeIMU();
        server.start();
    }

    private void main_loop() {
        update_manipulators();
        update_movers();
//        update_values();
        update_telemetry();

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
        collectTelemetry("Yaw (heading)", botHeadingDeg);
        collectTelemetry("YawPitchRoll", imu_IMU.getRobotYawPitchRollAngles().toString());
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
            return 0.04; //Make it correct quickly when it stops
        }

        if (currentSpeed < 0.1) {
            coastingSpeed = 0.025;
        }
//        } else if (currentSpeed >= 0.1 && currentSpeed <=0.75) {
        else {
            coastingSpeed = 0.085 * currentSpeed + 0.0165;
//        } else {
//            coastingSpeed = 0.08;
        }
        collectTelemetry("calculatedCorrectionSpeed", coastingSpeed);
        return coastingSpeed;
    }
    // SECTION: EXPERIMENTAL CODE

    private int getHTTP(String targetUrl) {
        collectTelemetry("HTTP", "GETting " + targetUrl);
        submitTelemetry();
        collectTelemetry("HTTP", "GETting " + targetUrl);
        try {
            URL url = new URL(targetUrl);
            HttpURLConnection con = (HttpURLConnection) url.openConnection();
            con.setRequestMethod("GET");
            con.setRequestProperty("Content-Type", "application/json");
            int status = con.getResponseCode();
            con.disconnect();
            return status;
        } catch (Exception e) {
            e.printStackTrace();
            collectTelemetry("Exception", e);
            return -1;
        }
    }

    private int postHTTP(String targetUrl, JSONObject jsonData) {
        collectTelemetry("HTTP", "POSTing to " + targetUrl);
        submitTelemetry();
        collectTelemetry("HTTP", "POSTing to " + targetUrl);
        try {
            URL url = new URL(targetUrl);
            HttpURLConnection con = (HttpURLConnection) url.openConnection();
            con.setConnectTimeout(1000);
            con.setReadTimeout(1000);
            con.setRequestMethod("POST");
            con.setRequestProperty("Content-Type", "application/json");

            OutputStreamWriter wr = new OutputStreamWriter(con.getOutputStream());

            // jsonData.put("message", "Hi there!");
            wr.write(jsonData.toString());
            wr.flush();

            int status = con.getResponseCode();
            con.disconnect();
            return status;
        } catch (Exception e) {
            e.printStackTrace();
            collectTelemetry("Exception", e);
            return -1;
        }
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

//    private void sendDataAndReset() {
//        String LOG_ENDPOINT = "http://192.168.43.151:8000/addPoint";
//        JSONObject jsonData = new JSONObject();
//        JSONObject motorTicks = new JSONObject();
//        JSONObject distanceSensors = new JSONObject();
//
//        motorTicks.put("flMotorTicks", FrontLeftMotor.getCurrentPosition());
//        motorTicks.put("frMotorTicks", FrontRightMotor.getCurrentPosition());
//        motorTicks.put("blMotorTicks", BackLeftMotor.getCurrentPosition());
//        motorTicks.put("brMotorTicks", BackRightMotor.getCurrentPosition());
//
//        distanceSensors.put("leftDistance", null);
//        distanceSensors.put("backDistance", turningDistanceSensor.getDistance(DistanceUnit.CM));
//        distanceSensors.put("rightDistance", rightDistanceSensor.getDistance(DistanceUnit.CM));
//
//        jsonData.put("motorTicks", motorTicks);
//        jsonData.put("distanceSensors", distanceSensors);
//
//        postHTTP(LOG_ENDPOINT, jsonData);
//        resetMotors();
//    }

//    private void sendCurrentPower() {
//        String LOG_ENDPOINT = "http://192.168.43.151:8000/addPoint";
//        JSONObject jsonData = new JSONObject();
//        JSONObject motorTicks = new JSONObject();
//        JSONObject distanceSensors = new JSONObject();
//
//    }

    private void update_telemetry() {
        collectTelemetry("SECTION", "POWER");
        // Front Motors
        collectTelemetry("Front Left", FrontLeftMotor.getPower());
        collectTelemetry("Front Right", FrontRightMotor.getPower());
        // Back Motors
        collectTelemetry("Back Left", BackLeftMotor.getPower());
        collectTelemetry("Back Right", BackRightMotor.getPower());
        // Manipulating Motors
        collectTelemetry("Middle", mMotor.getPower());
        collectTelemetry("High Actual", hMotor.getPower());
        // POSITIONING
        collectTelemetry("SECTION", "ARM POSITIONS");
        collectTelemetry("Middle", mMotor.getCurrentPosition() + " / " + mMotor.getTargetPosition());
        collectTelemetry("High", hMotor.getCurrentPosition() + " / " + hMotor.getTargetPosition());
        collectTelemetry("SECTION", "WHEEL POSITIONS");
        collectTelemetry("Front", FrontLeftMotor.getCurrentPosition() + "     " + FrontRightMotor.getCurrentPosition());
        collectTelemetry("Back", BackLeftMotor.getCurrentPosition() + "     " + BackRightMotor.getCurrentPosition());
        // SERVOS& SENSORS
        collectTelemetry("GrabbyPosition", clawServo.getPosition());
        collectTelemetry("SweepyPower", sweeperServo.getPower());
        collectTelemetry("DistanceSensor", "Back:" + turningDistanceSensor.getDistance(DistanceUnit.CM) + "cm | Right: " + rightDistanceSensor.getDistance(DistanceUnit.CM) + "cm.");
        // Send Updates
        submitTelemetry();
    }


    private void update_manipulators() {

        //Manipulators use gamepad 2

        double manipulatingPower = 0;
        double mTargetPower = 0;
        boolean mMotorAnchored = false;
        double hTargetPower = 0;
        boolean hMotorAnchored = false;

        // Check power boost
        if (gamepad2.left_bumper) {
            manipulatingPower = 1;
        } else {
            manipulatingPower = 0.5;
        }
        // Get input for extenders
        if (gamepad2.dpad_up) {
            mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mTargetPower = manipulatingPower;
            mMotorAnchored = false;
        } else if (gamepad2.dpad_down) {
            mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            mTargetPower = -manipulatingPower;
            mMotorAnchored = false;
        } else {
            if (!mMotorAnchored) {
                mMotor.setTargetPosition(mMotor.getCurrentPosition());
                mMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mTargetPower = 0.5;
                mMotorAnchored = true;
            }
        }
        if (gamepad2.dpad_left) {
            hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hTargetPower = -manipulatingPower;
            hMotorAnchored = false;
        } else if (gamepad2.dpad_right) {
            hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hTargetPower = manipulatingPower;
            hMotorAnchored = false;
        } else {
            if (!hMotorAnchored) {
                hMotor.setTargetPosition(hMotor.getCurrentPosition());
                hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                hTargetPower = 0.5;
                hMotorAnchored = true;
            }
        }
        // Move Extenders
        mMotor.setPower(mTargetPower);
        hMotor.setPower(hTargetPower);
        // Deal with manipulators
        if (gamepad2.x) {
            clawServo.setPosition(0.75);
        } else if (gamepad2.y) {
            clawServo.setPosition(0.55);
        }
        if (gamepad2.a) {
            sweeperServo.setPower(1);
        } else if (gamepad2.b) {
            sweeperServo.setPower(-1);
        } else {
            sweeperServo.setPower(0);
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
            speedMulti = 0.2;
        } else {
            speedMulti = 1;
        }
        // Get Joystick Value
        joyLeftX = -gamepad1.left_stick_x;
        joyLeftY = gamepad1.left_stick_y;
        joyRightX = -(gamepad1.right_stick_x * 0.8);
        joyRightY = gamepad1.right_stick_y * 0.8;
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

            setHeadingDone = true;
        }
        // Activate heading corrections only when we're not steering
        if (joyRightX == 0) {

            if (rightJoyWasActive) {
                rightJoyReleasedAt = getRuntime();
                collectTelemetry("AngleTimer", "Started");
                correctionDisabled = true;
                setHeadingDone = false;
                rightJoyWasActive = false;
            } else if (!setHeadingDone) {
                if ((getRuntime() - rightJoyReleasedAt) > TURN_SETTLE_SECS) {
                    targetHeading = getHeading();
                    collectTelemetry("AngleTimer", "Done");
                    setHeadingDone = true;
                } else {
                    collectTelemetry("AngleTimer", "Waiting");
                    correctionDisabled = true;
                }
            }

            if (!correctionDisabled) {
                currentHeading = getHeading();
                headingDiff = getHeadingDiff(targetHeading, currentHeading);
                if (Math.abs(headingDiff) < ANGLE_TOLERANCE) {
                    // Within Threshold
                    collectTelemetry("Turning Correction", "N/A");
                } else if (headingDiff < 0) {
                    // Need to go right
                    double correctionSpeed = getCorrectionSpeed(flTargetPower);
                    flTargetPower = flTargetPower - correctionSpeed;
                    frTargetPower = frTargetPower + correctionSpeed;
                    blTargetPower = blTargetPower - correctionSpeed;
                    brTargetPower = brTargetPower + correctionSpeed;
                    collectTelemetry("Turning Correction", "Adjusting Right");
                } else if (headingDiff > 0) {
                    // Need to go left
                    double correctionSpeed = getCorrectionSpeed(flTargetPower);
                    flTargetPower = flTargetPower + correctionSpeed;
                    frTargetPower = frTargetPower - correctionSpeed;
                    blTargetPower = blTargetPower + correctionSpeed;
                    brTargetPower = brTargetPower - correctionSpeed;
                    collectTelemetry("Turning Correction", "Adjusting Right");
                }
            }
        } else {
            rightJoyWasActive = true;
            //Right joystick is being controlled.
        }
        collectTelemetry("TargetHeading", targetHeading);
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
        if (gamepad1.right_trigger > 0) {
            pulleyMotor.setPower(gamepad1.right_trigger);
        } else if (gamepad1.left_trigger > 0) {
            pulleyMotor.setPower(-gamepad1.left_trigger);
        } else {
            pulleyMotor.setPower(0);
        }
    }

//    private void update_values() {
//        if (gamepad1.x) {
//            sendDataAndReset();
//        } else if (gamepad1.y) {
//            resetMotors();
//        }
//
//        if (gamepad1.left_bumper) {
//            sendCurrentPower();
//
//        }
//    }

}