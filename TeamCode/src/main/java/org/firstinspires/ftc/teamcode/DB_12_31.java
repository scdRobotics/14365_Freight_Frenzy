package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Happy new year (DB_12_31)", group = "Current")

public class DB_12_31 extends OpMode {
  // Declare OpMode members.
  private ElapsedTime runtime = new ElapsedTime();

  // Create drive motor variables
  private DcMotor frontLeft = null;
  private DcMotor frontRight = null;
  private DcMotor backLeft = null;
  private DcMotor backRight = null;

  // Create intake motor variable
  private DcMotor intake = null;

  // Create launcher motor varibles
  private DcMotor launchLeft = null;
  private DcMotor launchRight = null;

  private boolean running = true;
  private boolean pressed = false;

  private double launchPower = 0.45;

  private double pusherPos = 0.35;
  private Servo pusher = null;

  @Override
  public void init() {
    telemetry.addData("Status", "Initializing");

    // Assign motor variables to HardwareMap motors
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

    // Assign intake variable to HardwareMap motor
    intake = hardwareMap.get(DcMotor.class, "intake");

    // Assign launcher variables to HardwareMap motors
    launchLeft = hardwareMap.get(DcMotor.class, "launchLeft");
    launchRight = hardwareMap.get(DcMotor.class, "launchRight");

    // Assign advancer variable to HardwareMap Servo
    pusher = hardwareMap.get(Servo.class, "intakeAdvance");

    // Assign a default directon for the drive motors
    frontLeft.setDirection(DcMotor.Direction.FORWARD);
    backLeft.setDirection(DcMotor.Direction.FORWARD);
    frontRight.setDirection(DcMotor.Direction.FORWARD);
    backRight.setDirection(DcMotor.Direction.FORWARD);

    // Assign a default direction for the launcher motors
    launchLeft.setDirection(DcMotor.Direction.FORWARD);
    launchRight.setDirection(DcMotor.Direction.FORWARD);

    // Tell the driver that initialization is complete.
    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void init_loop() {
    // Code that runs in a loop until start. Leave blank for now.
  }

  @Override
  public void start() {
    runtime.reset();
    pusher.setPosition(pusherPos);
  }

  @Override
  public void loop() {
    telemetry.addData("runtime",runtime);

    double norm = -gamepad1.left_stick_y;
    double strafe = gamepad1.left_stick_x;
    double yaw = gamepad1.right_stick_x;

    telemetry.addData("norm",norm);
    telemetry.addData("strafe",strafe);
    telemetry.addData("yaw",yaw);

    frontLeft.setPower((norm - yaw + strafe));
    backLeft.setPower(-(-norm + yaw + strafe));
    frontRight.setPower(-(norm + yaw - strafe));
    backRight.setPower((-norm - yaw - strafe));

    telemetry.addData("frontLeft",(norm + strafe + yaw));
    telemetry.addData("backLeft",(norm - strafe + yaw));
    telemetry.addData("frontRight",(norm - strafe - yaw));
    telemetry.addData("backRight",(norm + strafe - yaw));

    if(gamepad1.dpad_up) {
      launchPower += 0.005;
    } else if (gamepad1.dpad_down) {
      launchPower -= 0.005;
    }

    telemetry.addData("launchPower",launchPower);


    if(!gamepad1.y) {

      launchLeft.setPower(-(launchPower - (gamepad1.left_trigger / 2)));
      launchRight.setPower((launchPower - (gamepad1.left_trigger / 2)));

      if(gamepad1.x) {
        pusherPos = 0.2;
      } else {
        pusherPos = 0.35;
      }
      pusher.setPosition(pusherPos);

      if(gamepad1.right_bumper) {
        intake.setPower(-1);
      } else {
        intake.setPower(1);
      }
    }
  }

  /*
   * Code to run ONCE after the driver hits STOP
   */
  @Override
  public void stop() {}

}