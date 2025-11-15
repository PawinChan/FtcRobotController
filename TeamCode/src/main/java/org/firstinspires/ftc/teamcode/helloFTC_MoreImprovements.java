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

@TeleOp(name = "helloAndroidStudio")
public class helloFTC_MoreImprovements extends LinearOpMode {

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
    waitForStart();
    if (opModeIsActive()) {
      distanceSensorServo.setPosition(0.25);
      while (opModeIsActive()) {
        main_loop();
      }
    }
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
  }

  private void main_loop() {
    update_manipulators();
    update_movers();
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
    double botHeadingDeg;

    botHeadingDeg = imu_IMU.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    telemetry.addData("Yaw (heading)", botHeadingDeg);
    telemetry.addData("YawPitchRoll", imu_IMU.getRobotYawPitchRollAngles().toString());
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
      }
      else {
        if ((targetHeading < -90) && currentHeading > 90) {
          targetHeading += 360;
        } else if ((targetHeading > 90) && currentHeading < -90) {
          targetHeading -= 360;
        }
        headingDiff = targetHeading - currentHeading;
      }
      return headingDiff;
    }



  private void update_telemetry() {
    telemetry.addData("SECTION", "POWER");
    // Front Motors
    telemetry.addData("Front Left", FrontLeftMotor.getPower());
    telemetry.addData("Front Right", FrontRightMotor.getPower());
    // Back Motors
    telemetry.addData("Back Left", BackLeftMotor.getPower());
    telemetry.addData("Back Right", BackRightMotor.getPower());
    // Manipulating Motors
    telemetry.addData("Middle", mMotor.getPower());
    telemetry.addData("High Actual", hMotor.getPower());
    // POSITIONING
    telemetry.addData("SECTION", "ARM POSITIONS");
    telemetry.addData("Middle", mMotor.getCurrentPosition() + " / " + mMotor.getTargetPosition());
    telemetry.addData("High", hMotor.getCurrentPosition() + " / " + hMotor.getTargetPosition());
    telemetry.addData("SECTION", "WHEEL POSITIONS");
    telemetry.addData("Front", FrontLeftMotor.getCurrentPosition() + "     " + FrontRightMotor.getCurrentPosition());
    telemetry.addData("Back", BackLeftMotor.getCurrentPosition() + "     " + BackRightMotor.getCurrentPosition());
    // SERVOS& SENSORS
    telemetry.addData("GrabbyPosition", clawServo.getPosition());
    telemetry.addData("SweepyPower", sweeperServo.getPower());
    telemetry.addData("DistanceSensor", "Back:" + turningDistanceSensor.getDistance(DistanceUnit.CM) + "cm | Right: " + rightDistanceSensor.getDistance(DistanceUnit.CM) + "cm.");
    // Send Updates
    telemetry.update();
  }


  private void update_manipulators() {
    double manipulatingPower = 0;
    double mTargetPower = 0;
    boolean mMotorAnchored = false;
    double hTargetPower = 0;
    boolean hMotorAnchored = false;

    // Check power boost
    if (gamepad1.left_bumper) {
      manipulatingPower = 1;
    } else {
      manipulatingPower = 0.5;
    }
    // Get input for extenders
    if (gamepad1.dpad_up) {
      mMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      mTargetPower = manipulatingPower;
      mMotorAnchored = false;
    } else if (gamepad1.dpad_down) {
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
    if (gamepad1.dpad_left) {
      hMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      hTargetPower = -manipulatingPower;
      hMotorAnchored = false;
    } else if (gamepad1.dpad_right) {
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
    if (gamepad1.x) {
      clawServo.setPosition(0.75);
    } else if (gamepad1.y) {
      clawServo.setPosition(0.55);
    }
    if (gamepad1.a) {
      sweeperServo.setPower(1);
    } else if (gamepad1.b) {
      sweeperServo.setPower(-1);
    } else {
      sweeperServo.setPower(0);
    }
  }


  private void update_movers() {
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
    maxCalculatedPower = JavaUtil.maxOfList(JavaUtil.createListWith(flTargetPower, frTargetPower, blTargetPower, brTargetPower));
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
        telemetry.addData("AngleTimer", "Started");
        correctionDisabled = true;
        setHeadingDone = false;
        rightJoyWasActive = false;
      }
      else if (!setHeadingDone) {
        if ((getRuntime() - rightJoyReleasedAt) > TURN_SETTLE_SECS) {
          targetHeading = getHeading();
          telemetry.addData("AngleTimer", "Done");
          setHeadingDone = true;
        } else {
          telemetry.addData("AngleTimer", "Waiting");
          correctionDisabled = true;
        }
      }

      if (!correctionDisabled) {
        currentHeading = getHeading();
        headingDiff = getHeadingDiff(targetHeading, currentHeading);
        if (Math.abs(headingDiff) < ANGLE_TOLERANCE) {
          // Within Treshold
          telemetry.addData("Turning Correction", "N/A");
        } else if (headingDiff < 0) {
          // Need to go right
          flTargetPower = flTargetPower - SIDEWAYS_CORRECTION_FACTOR;
          frTargetPower = frTargetPower + SIDEWAYS_CORRECTION_FACTOR;
          blTargetPower = blTargetPower - SIDEWAYS_CORRECTION_FACTOR;
          brTargetPower = brTargetPower + SIDEWAYS_CORRECTION_FACTOR;

//          flTargetPower = flTargetPower - SIDEWAYS_CORRECTION_FACTOR;
//          frTargetPower = frTargetPower + SIDEWAYS_CORRECTION_FACTOR;
//          blTargetPower = blTargetPower - SIDEWAYS_CORRECTION_FACTOR;
//          brTargetPower = brTargetPower + SIDEWAYS_CORRECTION_FACTOR;
          telemetry.addData("Turning Correction", "Adjusting Right");
        } else if (headingDiff > 0) {
          // Need to go left
         flTargetPower = flTargetPower + SIDEWAYS_CORRECTION_FACTOR;
         frTargetPower = frTargetPower - SIDEWAYS_CORRECTION_FACTOR;
         blTargetPower = blTargetPower + SIDEWAYS_CORRECTION_FACTOR;
         brTargetPower = brTargetPower - SIDEWAYS_CORRECTION_FACTOR;

//          flTargetPower = flTargetPower + SIDEWAYS_CORRECTION_FACTOR;
//          frTargetPower = frTargetPower - SIDEWAYS_CORRECTION_FACTOR;
//          blTargetPower = blTargetPower + SIDEWAYS_CORRECTION_FACTOR;
//          brTargetPower = brTargetPower - SIDEWAYS_CORRECTION_FACTOR;
          telemetry.addData("Turning Correction", "Adjusting Right");
        }
      }
    } else {
      rightJoyWasActive = true;
      //Right joystick is being controlled.
    }
    telemetry.addData("TargetHeading", targetHeading);
    telemetry.addData("CurrentHeading", currentHeading);
    maxCalculatedPower = JavaUtil.maxOfList(JavaUtil.createListWith(flTargetPower, frTargetPower, blTargetPower, brTargetPower));
    if (maxCalculatedPower > 1) {
      flTargetPower = flTargetPower / maxCalculatedPower;
      frTargetPower = frTargetPower / maxCalculatedPower;
      blTargetPower = blTargetPower / maxCalculatedPower;
      brTargetPower = brTargetPower / maxCalculatedPower;
    }
    // Set Motors
    FrontLeftMotor.setPower(flTargetPower);
    FrontRightMotor.setPower(frTargetPower);
    BackLeftMotor.setPower(blTargetPower * 0.8);
    BackRightMotor.setPower(brTargetPower * 0.8);
    if (gamepad1.right_trigger > 0) {
      pulleyMotor.setPower(gamepad1.right_trigger);
    } else if (gamepad1.left_trigger > 0) {
      pulleyMotor.setPower(-gamepad1.left_trigger);
    } else {
      pulleyMotor.setPower(0);
    }
  }

}