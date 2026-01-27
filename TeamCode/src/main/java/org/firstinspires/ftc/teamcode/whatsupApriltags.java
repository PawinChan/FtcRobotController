package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "whatsupApriltags")
public class whatsupApriltags extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor BackRightMotor;
    private DcMotor mMotor;
    private IMU imu_IMU;
    private DcMotor hMotor;
    private Servo distanceSensorServo;
    private DistanceSensor turningDistanceSensor;
    private DistanceSensor rightDistanceSensor;
    private CRServo sweeperServo;
    private Servo clawServo;

    double currentHeading;
    boolean DEBUG_ARM_DISABLED;
    double coastingSpeed;
    double headingDiff;
    double CLAW_OPEN_POS;
    double CLAW_CLOSED_POS;
    int TICKS_IN_A_METER;
    double SENSOR_BACK_POSITION;
    int ARM_LOWEST_POSITION;
    int ANGLE_TOLERANCE;
    int TICKS_TOLERANCE;
    double SIDEWAYS_CORRECTION_FACTOR;

    // START OF FUNCTIONS AREA

    // SECTION: INITIALIZATION CODE

    private void initializeHardware() {
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "FrontLeftMotor");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "FrontRightMotor");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "BackLeftMotor");
        BackRightMotor = hardwareMap.get(DcMotor.class, "BackRightMotor");

        mMotor = hardwareMap.get(DcMotor.class, "mMotor");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        hMotor = hardwareMap.get(DcMotor.class, "hMotor");
        distanceSensorServo = hardwareMap.get(Servo.class, "distanceSensorServo");
        turningDistanceSensor = hardwareMap.get(DistanceSensor.class, "turningDistanceSensor");
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        sweeperServo = hardwareMap.get(CRServo.class, "sweeperServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    private void configureMotors() {
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        hMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ((DcMotorEx) FrontLeftMotor).setTargetPositionTolerance(10);
        ((DcMotorEx) FrontRightMotor).setTargetPositionTolerance(10);
        ((DcMotorEx) BackLeftMotor).setTargetPositionTolerance(10);
        ((DcMotorEx) BackRightMotor).setTargetPositionTolerance(10);
    }

    private void initConstants() {
        double SENSOR_LEFT_POSITION;

        DEBUG_ARM_DISABLED = false;
        CLAW_OPEN_POS = 0.5;
        CLAW_CLOSED_POS = 0.7;
        ARM_LOWEST_POSITION = mMotor.getCurrentPosition();
        TICKS_IN_A_METER = 1205; //2200 for 1:20 ratio, 1205 for 1:12
        ANGLE_TOLERANCE = 1;
        TICKS_TOLERANCE = 11;
        SENSOR_BACK_POSITION = 0.25;
        SENSOR_LEFT_POSITION = 0.63;
        SIDEWAYS_CORRECTION_FACTOR = 0.1;
    }

    private void notifyError(String errorMsg) {
        telemetry.addLine(errorMsg);
        telemetry.update();
        sleep_new(120);
    }

    private void sleep_new(int sleep_sec) {
        sleep(sleep_sec * 1000);
    }

    private double convert_cm(double input_cm) {
        // Returns distance in ticks given CMs.
        // Note: constant defined in initConst. Can be obtained by running the
        // calibrateTicks opmode
        return input_cm * (TICKS_IN_A_METER / 100);
    }

    private double convert_ticks(double input_ticks) {
        // Returns distance in cm given ticks
        return input_ticks / (TICKS_IN_A_METER / 100);
    }

    private double abs_avg(List listToAverage) {
        double absAvgResult;

        // Get the average of absolute values of all items in the list.
        // (like avg([abs(i) for i in listToAverage]))
        telemetry.addData("abs_avg input", listToAverage);
        absAvgResult = 0;
        for (Object listItem : listToAverage) {
            double itemValue = ((Number) listItem).doubleValue();
            absAvgResult += Math.abs(itemValue);
        }
        absAvgResult = Double.parseDouble(JavaUtil.formatNumber(absAvgResult / JavaUtil.listLength(listToAverage), 2));
        return absAvgResult;
    }

    private List subtractLists(List listA, List listB) {
        List subtractedList;
        double i;

        // Piecewise subtraction.
        if (JavaUtil.listLength(listA) != JavaUtil.listLength(listB)) {
            notifyError("List A and B must be equal in length to subtract.");
        }
        subtractedList = JavaUtil.createListWith();
        for (int index = 0; index < JavaUtil.listLength(listA); index++) {
            double valueA = ((Number) listA.get(index)).doubleValue();
            double valueB = ((Number) listB.get(index)).doubleValue();
            subtractedList.add(valueA - valueB);
        }
        return subtractedList;
    }
    // SECTION: COASTING FUNCTIONS

    private double getCoastingSpeedFromDistance(double distanceForCoasting) {
        // Returns the speed which should be used (between 0 and 1) given the remaining
        // distance in centimeters.
        // This function DOES NOT handle and ignore directions. Make sure to flip the
        // power accordingly (if required) from the calling function.
        // Feel free to customize or play with this in Desmos. It's just y=mx+b capped
        // to a range
        distanceForCoasting = Math.abs(distanceForCoasting);
        if (distanceForCoasting < 10) {
            coastingSpeed = 0.1;
        } else if (distanceForCoasting >= 10 && distanceForCoasting <= 30) {
            coastingSpeed = 0.0325 * distanceForCoasting + -0.225;
        } else {
            coastingSpeed = 0.75;
        }
        telemetry.addData("coastingSpeedD", coastingSpeed);
        return coastingSpeed;
    }

    private double getCoastingSpeedFromAngle(double angleForCoasting) {
        // Returns the speed (between 0 and 1) given the angle to turn in degree.
        // This function DOES NOT handle and ignore directions. Make sure to flip the
        // power accordingly (if required) from the calling function.
        // Feel free to customize or play with this in Desmos. It's just y=mx+b capped
        // to a range
        angleForCoasting = Math.abs(angleForCoasting);
        if (angleForCoasting < 10) {
            coastingSpeed = 0.05;
        } else if (angleForCoasting >= 10 && angleForCoasting <= 30) {
            coastingSpeed = 0.015 * angleForCoasting + -0.05;
        } else {
            coastingSpeed = 0.6;
        }
        telemetry.addData("coastingSpeedA", coastingSpeed);
        return coastingSpeed;
    }

    private double getCorrectionSpeed(double currentSpeed) {
        currentSpeed = Math.abs(currentSpeed);
        if (currentSpeed < 0.1) {
            coastingSpeed = 0.05;
        } else if (currentSpeed >= 0.1 && currentSpeed <=0.9) {
            coastingSpeed = 0.0625 * currentSpeed + 0.04375;
        } else {
            coastingSpeed = 0.1;
        }
        telemetry.addData("calculatedCorrectionSpeed", coastingSpeed);
        return coastingSpeed;
    }
    // SECTION: MOTOR OPERATIONS

    private void setModeAll_runToPos() {
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setPowerAll(double targetPower) {
        FrontLeftMotor.setPower(targetPower);
        FrontRightMotor.setPower(targetPower);
        BackLeftMotor.setPower(targetPower);
        BackRightMotor.setPower(targetPower);
    }

    private void setPowerByIndex(double motorIndex, double targetPower) {
        if (motorIndex == 1) {
            FrontLeftMotor.setPower(targetPower);
        } else if (motorIndex == 2) {
            FrontRightMotor.setPower(targetPower);
        } else if (motorIndex == 3) {
            BackLeftMotor.setPower(targetPower);
        } else if (motorIndex == 4) {
            BackRightMotor.setPower(targetPower);
        } else {
            notifyError("Error: Invalid motor index of: " + motorIndex);
        }
    }

    private void setPowerRight(double targetPower) {
        FrontRightMotor.setPower(targetPower);
        BackRightMotor.setPower(targetPower);
    }

    private void setModeAll_runUsingEncoder() {
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setPowerLeft(double targetPower) {
        FrontLeftMotor.setPower(targetPower);
        BackLeftMotor.setPower(targetPower);
    }

    private void setModeAll_stopAndReset() {
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void setPowerSlide(double targetPower) {
        FrontLeftMotor.setPower(targetPower);
        FrontRightMotor.setPower(-targetPower);
        BackLeftMotor.setPower(-targetPower);
        BackRightMotor.setPower(targetPower);
    }

    private boolean motorsAreBusy() {
        return FrontLeftMotor.isBusy() || FrontRightMotor.isBusy() || BackLeftMotor.isBusy() || BackRightMotor.isBusy();
    }
    // SECTION: OBTAINING HEADINGS

    private void initializeIMU() {
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
        imu_IMU.resetYaw();
    }

    private double getHeading() {
        double botHeadingDeg;

        botHeadingDeg = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw (heading)", botHeadingDeg);
        telemetry.addData("YawPitchRoll", imu_IMU.getRobotYawPitchRollAngles().toString());
        return botHeadingDeg;
    }

    private long getAlignedHeading() {
        long botAlignedHeading;

        // Get closest angle in 90 degrees interval, eg. N, E, S, W
        botAlignedHeading = Math.round(Math.min(Math.max(getHeading(), -180), 180) / 90) * 90;
        telemetry.addData("AlignedHeading", botAlignedHeading);
        return botAlignedHeading;
    }

    private double getHeadingDiff(double targetHeading, double currentHeading) {
        if (Math.abs(targetHeading) == 180) {
            // Since 180 = -180, you can pretty much go both ways, depending on which is
            // closer
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
    }// SECTION: CHANGING HEADINGS

    private void faceNorth() {
        headTo(0, 0, 0);
    }

    private void faceEast() {
        headTo(-90, 0, 0);
    }

    private void faceSouth() {
        headTo(180, 0, 0);
    }

    private void faceWest() {
        headTo(90, 0, 0);
    }

    private void headTo(int targetHeading, int driveBlindSec, int minHeadingChangeSpeed) {
        getHeading();
        telemetry.update();
        setModeAll_runUsingEncoder();
        setPowerAll(0);
        while (opModeIsActive()) {
            currentHeading = getHeading();
            headingDiff = getHeadingDiff(targetHeading, currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("headingDiff", headingDiff);
            if (Math.abs(headingDiff) < ANGLE_TOLERANCE) {
                // Within Treshold
                setPowerAll(0);
                telemetry.addData("Action", "Done Turning");
                telemetry.update();
                break;
            } else if (headingDiff < 0) {
                // Need to go right
                setPowerLeft(Math.max(getCoastingSpeedFromAngle(headingDiff), minHeadingChangeSpeed));
                setPowerRight(-Math.max(getCoastingSpeedFromAngle(headingDiff), minHeadingChangeSpeed));
                telemetry.addData("Action", "Turning Right");
            } else if (headingDiff > 0) {
                // Need to go left
                setPowerLeft(-Math.max(getCoastingSpeedFromAngle(headingDiff), minHeadingChangeSpeed));
                setPowerRight(Math.max(getCoastingSpeedFromAngle(headingDiff), minHeadingChangeSpeed));
                telemetry.addData("Action", "Turning Left");
            }
            if (driveBlindSec > 0) {
                telemetry.addData("Delaying for", driveBlindSec + "sec(s)");
                telemetry.update();
                sleep_new(driveBlindSec);
                driveBlindSec = 0;
            }
            telemetry.update();
        }
        headingDiff = targetHeading - getHeading();
        telemetry.addData("targetHeading", targetHeading);
        telemetry.addData("headingDiffFinal", headingDiff);
        telemetry.addData("action", "Finished.");
        telemetry.update();
    }// SECTION: MANIPULATORS

    private void armTo(int armTargetPosition, int armMoveDelay) {
        if (DEBUG_ARM_DISABLED) {
            return;
        }
        armTargetPosition = ARM_LOWEST_POSITION + armTargetPosition;
        telemetry.addLine("Moving arm to:" + armTargetPosition);
        mMotor.setTargetPosition(armTargetPosition);
        mMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mMotor.setPower(0.8);
        // Define waiting behavior. -1 for no waiting, 0 is until finish, else num secs
        // to wait
        if (armMoveDelay < 0) {
            return;
        }
        if (armMoveDelay == 0) {
            while (mMotor.isBusy()) {
                telemetry.addData("mMotorPosition", mMotor.getCurrentPosition());
                telemetry.update();
            }
        } else {
            sleep_new(armMoveDelay);
        }
    }

    private void handTo(int handTargetPosition) {
        telemetry.addLine("Moving hand to:" + handTargetPosition);
        hMotor.setTargetPosition(handTargetPosition);
        hMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hMotor.setPower(0.8);
        while (hMotor.isBusy()) {
            telemetry.addData("hMotorPosition", hMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    private void startSweepIn() {
        sweeperServo.setPower(1);
    }

    private void grabSpecimen() {
        clawServo.setPosition(CLAW_CLOSED_POS);
    }

    private void startSweepOut() {
        sweeperServo.setPower(-1);
    }

    private void releaseSpecimen() {
        clawServo.setPosition(CLAW_OPEN_POS);
    }

    private void stopSweep() {
        sweeperServo.setPower(0);
    }
    // SECTION: MANEUVERING ROBOTS

    private void maneuverRobotTicks(double moveMagnitude, int frontLeftSign, int frontRightSign, int backLeftSign, int backRightSign) {
        double motorIndex;
        double targetHeading;

        // int FrontLeftTargetTicks;
        // int FrontRightTargetTicks;
        // int BackLeftTargetTicks;
        // int BackRightTargetTicks;

        // int FrontLeftCurrentTicks;
        // int FrontRightCurrentTicks;
        // int BackLeftCurrentTicks;
        // int BackRightCurrentTicks;

        double frontLeftMotorSpeed = 0;
        double frontRightMotorSpeed = 0;
        double backLeftMotorSpeed = 0;
        double backRightMotorSpeed = 0;

        List allTargetMotorTicks;
        // List allTargetMotorSpeeds;
        boolean positioningIsFinished;
        List allCurrentMotorTicks;
        List allMotorTickDiffs;
        double avgMotorTickDiffs;
        double motorTargetSpeed = 0;
        double exitCondition1;
        boolean exitCondition2;

        // Ensure that they're all just signs
        if (Math.abs(frontLeftSign) != 1 || Math.abs(frontRightSign) != 1 || Math.abs(backLeftSign) != 1
                || Math.abs(backRightSign) != 1) {
            notifyError("magnitude must only be +1 or -1 in maneuverRobotTicks");
        }
        setModeAll_stopAndReset();
        setModeAll_runUsingEncoder();
        setPowerAll(0);
        targetHeading = getAlignedHeading();
        allTargetMotorTicks = JavaUtil.createListWith(
                frontLeftSign * moveMagnitude, frontRightSign * moveMagnitude,
                backLeftSign * moveMagnitude,  backRightSign * moveMagnitude
        );

        positioningIsFinished = false;
        while (opModeIsActive()) {

            // Get information
            currentHeading = getHeading();
            headingDiff = getHeadingDiff(targetHeading, currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("headingDiff", headingDiff);
            allCurrentMotorTicks = JavaUtil.createListWith(
                    FrontLeftMotor.getCurrentPosition(), FrontRightMotor.getCurrentPosition(),
                    BackLeftMotor.getCurrentPosition(),  BackRightMotor.getCurrentPosition()
            );
            allMotorTickDiffs = subtractLists(allTargetMotorTicks, allCurrentMotorTicks);
            avgMotorTickDiffs = abs_avg(allMotorTickDiffs);

            // Deal with positioning.
            if (!positioningIsFinished) {
                // Movement regarding positioning required.
                motorTargetSpeed = getCoastingSpeedFromDistance(convert_ticks(avgMotorTickDiffs));
                frontLeftMotorSpeed = frontLeftSign * motorTargetSpeed;
                frontRightMotorSpeed = frontRightSign * motorTargetSpeed;
                backLeftMotorSpeed = backLeftSign * motorTargetSpeed;
                backRightMotorSpeed = backRightSign * motorTargetSpeed;
            } else {
                // Movement regarding positioning is done. Just need to handle angles
                // Since the power was already set to 0 in the beginning, we can just do nothing and proceed...
            }
            // Deal with angles.
            if (Math.abs(headingDiff) < ANGLE_TOLERANCE) {
                // Within Treshold
                telemetry.addData("Turning Correction", "N/A");
            } else if (headingDiff < 0) {
                // Need to go right
                frontLeftMotorSpeed += getCorrectionSpeed(motorTargetSpeed);
                backLeftMotorSpeed += getCorrectionSpeed(motorTargetSpeed);
                frontRightMotorSpeed -= getCorrectionSpeed(motorTargetSpeed);
                backRightMotorSpeed -= getCorrectionSpeed(motorTargetSpeed);

                telemetry.addData("Turning Correction", "Adjusting Right");
            } else if (headingDiff > 0) {
                // Need to go left
                frontLeftMotorSpeed -= getCorrectionSpeed(motorTargetSpeed);
                backLeftMotorSpeed -= getCorrectionSpeed(motorTargetSpeed);
                frontRightMotorSpeed += getCorrectionSpeed(motorTargetSpeed);
                backRightMotorSpeed += getCorrectionSpeed(motorTargetSpeed);
                telemetry.addData("Turning Correction", "Adjusting Right");
            }
            // Send Telemetry
            telemetry.addData("allTargetMotorSpeeds", JavaUtil.createListWith(frontLeftMotorSpeed, frontRightMotorSpeed, backLeftMotorSpeed, backRightMotorSpeed));
            telemetry.addData("allTargetMotorTicks", allTargetMotorTicks);
            telemetry.addData("allCurrentMotorTicks", allCurrentMotorTicks);
            telemetry.addData("allMotorTickDiffs", allMotorTickDiffs);
            telemetry.addData("avgMotorTickDiffs", avgMotorTickDiffs);
            telemetry.addData("positioningIsFinished", positioningIsFinished);

            exitCondition1 = Math.abs(moveMagnitude - abs_avg(allCurrentMotorTicks));
            exitCondition2 = Math.abs(headingDiff) < ANGLE_TOLERANCE;

            if (exitCondition1 < TICKS_TOLERANCE) {
                positioningIsFinished = true; //sticky
            }
            telemetry.addData("ExitCondition1", exitCondition1);
            telemetry.addData("ExitCondition2", exitCondition1);
            telemetry.update();
            // Decide if we need to continue moving
            if (positioningIsFinished && exitCondition2) {
                // If positioning is finished and we're aligned, break out of loop!
                break;
            } else {
                // Else, send speed to motors and repeat.
                FrontLeftMotor.setPower(frontLeftMotorSpeed);
                FrontRightMotor.setPower(frontRightMotorSpeed);
                BackLeftMotor.setPower(backLeftMotorSpeed);
                BackRightMotor.setPower(backRightMotorSpeed);
            }
        }
        setPowerAll(0);
    }

    private void moveLeftSmart(int moveCM) {
        maneuverRobotTicks(convert_cm(moveCM), -1, 1, 1, -1);
    }

    private void moveFrontSmart(int moveCM) {
        maneuverRobotTicks(convert_cm(moveCM), 1, 1, 1, 1);
    }

    private void moveBackSmart(int moveCM) {
        maneuverRobotTicks(convert_cm(moveCM), -1, -1, -1, -1);
    }

    private void moveRightSmart(int moveCM) {
        maneuverRobotTicks(convert_cm(moveCM), 1, -1, -1, 1);
    }

    private void moveWithSensor(double targetBackCm) {
        double currentBackCm;
        double backCmDiff;

        setModeAll_runUsingEncoder();
        distanceSensorServo.setPosition(SENSOR_BACK_POSITION);
        while (opModeIsActive()) {
            currentBackCm = Math.min(Math.max(turningDistanceSensor.getDistance(DistanceUnit.CM), 1), 140);
            backCmDiff = targetBackCm - currentBackCm;
            if (Math.abs(backCmDiff) < 1) {
                setPowerAll(0);
                telemetry.addData("Movement Done", currentBackCm + " / " + targetBackCm + " (diff: " + backCmDiff + ", final: "
                        + turningDistanceSensor.getDistance(DistanceUnit.CM) + ")");
                telemetry.update();
                break;
            } else if (backCmDiff >= 0) {
                setPowerAll(getCoastingSpeedFromDistance(backCmDiff));
            } else {
                setPowerAll(-getCoastingSpeedFromDistance(backCmDiff));
            }
            telemetry.addData("Distance", currentBackCm + " / " + targetBackCm + " (diff: " + backCmDiff + ") / speed: "
                    + getCoastingSpeedFromDistance(backCmDiff));
            telemetry.update();
        }
    }

    private void slideWithSensor(double targetRightCm) {
        double currentRightCm;
        double rightCmDiff;

        setModeAll_runUsingEncoder();
        while (opModeIsActive()) {
            currentRightCm = Math.min(Math.max(rightDistanceSensor.getDistance(DistanceUnit.CM), 1), 140);
            rightCmDiff = targetRightCm - currentRightCm;
            if (Math.abs(rightCmDiff) < 1) {
                setPowerAll(0);
                telemetry.addData("Movement Done", currentRightCm + " / " + targetRightCm + " (diff: " + rightCmDiff
                        + ", final: " + turningDistanceSensor.getDistance(DistanceUnit.CM) + ")");
                telemetry.update();
                break;
            } else if (rightCmDiff >= 0) {
                // Slide to da right
                setPowerSlide(-getCoastingSpeedFromDistance(rightCmDiff));
            } else {
                // Slide to da left
                setPowerSlide(getCoastingSpeedFromDistance(rightCmDiff));
            }
            telemetry.addData("Distance", currentRightCm + " / " + targetRightCm + " (diff: " + rightCmDiff + ") / speed: "
                    + getCoastingSpeedFromDistance(rightCmDiff));
            telemetry.update();
        }
    }

    private void moveRobotTicks(double numTicks) {
        double avgCmRemaining;

        FrontLeftMotor.setTargetPosition((int) (FrontLeftMotor.getCurrentPosition() + numTicks));
        FrontRightMotor.setTargetPosition((int) (FrontRightMotor.getCurrentPosition() + numTicks));
        BackLeftMotor.setTargetPosition((int) (BackLeftMotor.getCurrentPosition() + numTicks));
        BackRightMotor.setTargetPosition((int) (BackRightMotor.getCurrentPosition() + numTicks));
        setModeAll_runToPos();
        avgCmRemaining = convert_ticks(Math.abs(JavaUtil
                .averageOfList(JavaUtil.createListWith(FrontLeftMotor.getCurrentPosition() - FrontLeftMotor.getTargetPosition(),
                        FrontRightMotor.getCurrentPosition() - FrontRightMotor.getTargetPosition(),
                        BackLeftMotor.getCurrentPosition() - BackLeftMotor.getTargetPosition(),
                        BackRightMotor.getCurrentPosition() - BackRightMotor.getTargetPosition()))));
        while (opModeIsActive() && avgCmRemaining > 1) {
            avgCmRemaining = convert_ticks(Math.abs(JavaUtil.averageOfList(
                    JavaUtil.createListWith(FrontLeftMotor.getCurrentPosition() - FrontLeftMotor.getTargetPosition(),
                            FrontRightMotor.getCurrentPosition() - FrontRightMotor.getTargetPosition(),
                            BackLeftMotor.getCurrentPosition() - BackLeftMotor.getTargetPosition(),
                            BackRightMotor.getCurrentPosition() - BackRightMotor.getTargetPosition()))));
            setPowerAll(getCoastingSpeedFromDistance(avgCmRemaining));
            telemetry.addData("FRONT", FrontLeftMotor.getCurrentPosition() + " / " + FrontLeftMotor.getTargetPosition()
                    + "     " + FrontRightMotor.getCurrentPosition() + " / " + FrontRightMotor.getTargetPosition());
            telemetry.addData("BACK", BackLeftMotor.getCurrentPosition() + " / " + BackLeftMotor.getTargetPosition() + "     "
                    + BackRightMotor.getCurrentPosition() + " / " + BackRightMotor.getTargetPosition());
            telemetry.update();
        }
    }

    private void moveRobotBasic(double frontLeftTicks, double frontRightTicks, double backLeftTicks,
                                double backRightTicks) {
        FrontLeftMotor.setTargetPosition((int) (FrontLeftMotor.getCurrentPosition() + frontLeftTicks));
        FrontRightMotor.setTargetPosition((int) (FrontRightMotor.getCurrentPosition() + frontRightTicks));
        BackLeftMotor.setTargetPosition((int) (BackLeftMotor.getCurrentPosition() + backRightTicks));
        BackRightMotor.setTargetPosition((int) (BackRightMotor.getCurrentPosition() + backRightTicks));
        setModeAll_runToPos();
        setPowerAll(0.4);
        while (motorsAreBusy()) {
            telemetry.addData("FRONT", FrontLeftMotor.getCurrentPosition() + " / " + FrontLeftMotor.getTargetPosition()
                    + "     " + FrontRightMotor.getCurrentPosition() + " / " + FrontRightMotor.getTargetPosition());
            telemetry.addData("BACK", BackLeftMotor.getCurrentPosition() + " / " + BackLeftMotor.getTargetPosition() + "     "
                    + BackRightMotor.getCurrentPosition() + " / " + BackRightMotor.getTargetPosition());
            telemetry.update();
        }
    }

    private void moveRobotBy(double numTicks) {
        moveRobotBasic(numTicks, numTicks, numTicks, numTicks);
    }

    // END OF FUNCTIONS AREA

    // SECTION: MAIN OPMODE
    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3
     * blue
     * Comment Blocks show where to place Initialization code (runs once, after
     * touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once,
     * after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active,
     * namely not
     * Stopped).
     */

    @Override
    public void runOpMode() {

        telemetry.setMsTransmissionInterval(50);
        telemetry.setNumDecimalPlaces(0, 4);
        initializeHardware();
        initConstants(); //needs to be after initializeHardware since it gets mMotor data
        configureMotors();
        grabSpecimen();
        initializeIMU();
        initAprilTag();
        telemetry.addLine("Waiting for start signal...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            imu_IMU.resetYaw();
//            moveRightSmart(250);
//            moveFrontSmart(250);
//            moveLeftSmart(250);
//            moveBackSmart(250);
//            sleep_new(5);
            while (opModeIsActive()) {
                telemetryAprilTag();
            }
            visionPortal.close();
            telemetry.update();
            sleep(20);
        }
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

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

                .build();

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
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "LogitechC310"));
        } else {
            builder.setCamera(BuiltinCameraDirection.FRONT);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
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

    }   // end method telemetryAprilTag()
}
