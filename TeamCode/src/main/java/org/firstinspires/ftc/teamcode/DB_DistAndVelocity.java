//*********************************************************************************
//                             -- CHANGELOG --                                    *
//  ____________________________________________________________________________  *
// |   DATE    |     NAME     |                   DESCRIPTION                   | *
// |___________|______________|_________________________________________________| *
// | 1/7/2021  | Jack         | Changed the drive system to utilize 2 gamepads  | *
// |           |              | to allow for 2 drivers                          | *
// |___________|______________|_________________________________________________| *
// | 1/22/2021 | Jack         | Updated to the second iteration of the advancer,| *
// |           |              | assorted bug fixes and style improvements       | *
// |___________|______________|_________________________________________________| *
//*********************************************************************************

package org.firstinspires.ftc.teamcode;

// TODO: Add distance sensor aided/automatic high goal targeting; we may need distance sensors on all sides for this to work

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "--TestOpMode--", group = "Current")

public class DB_DistAndVelocity extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private DcMotor intake = null;

    private DcMotorEx launchLeft = null;
    private DcMotorEx launchRight = null;

    private boolean running = true;
    private boolean pressed = false;

    private boolean latched = false;

    private double launchPower = 0.42;

    private double pusherPos = 0.35;
    private Servo pusher = null;

    protected DcMotorEx grabber = null;
    private Servo latch = null;

    double DriveSpeed=1;

    protected BNO055IMU imu;
    protected BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    protected Orientation lastAngles = new Orientation();
    protected double globalAngle;
    protected double initialAngle;

    protected final double countPerRotation=537.6;
    protected final double countPerDegree=0.07205357141;

    protected DistanceSensor backDist;
    protected double readBackDist;

    protected DistanceSensor frontDist;
    protected double readFrontDist;

    protected DistanceSensor leftDist;
    protected double readLeftDist;

    protected DistanceSensor rightDist;
    protected double readRightDist;

    protected double savedRightDist;
    protected double savedLeftDist;
    protected double savedFrontDist;
    protected double savedBackDist;

    //protected double idealRightWall=74.6; //71
    //protected double idealLeftWall=123.9; //128
    //protected double idealFrontWall=209.0; //216
    //protected double idealBackWall=109.8; //103

    protected double idealRightWall=71; //71
    protected double idealLeftWall=128; //128
    protected double idealFrontWall=126; //216
    protected double idealBackWall=103; //103

    public static final double TICKS_PER_REV = 28;
    public static final double MAX_RPM = 6000;

    double grabberPos = 0;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        intake = hardwareMap.get(DcMotor.class, "intake");

        launchLeft = hardwareMap.get(DcMotorEx.class, "launchLeft");
        launchRight = hardwareMap.get(DcMotorEx.class, "launchRight");

        pusher = hardwareMap.get(Servo.class, "intakeAdvance");

        grabber=hardwareMap.get(DcMotorEx.class,"grabber");
        latch=hardwareMap.get(Servo.class,"latch");

        backDist=hardwareMap.get(DistanceSensor.class, "backDist");
        readBackDist=backDist.getDistance(DistanceUnit.CM);

        rightDist=hardwareMap.get(DistanceSensor.class, "rightDist");
        readRightDist=backDist.getDistance(DistanceUnit.CM);

        frontDist=hardwareMap.get(DistanceSensor.class, "frontDist");
        readFrontDist=backDist.getDistance(DistanceUnit.CM);

        leftDist=hardwareMap.get(DistanceSensor.class, "leftDist");
        readLeftDist=backDist.getDistance(DistanceUnit.CM);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        launchLeft.setDirection(DcMotor.Direction.REVERSE);
        launchRight.setDirection(DcMotor.Direction.FORWARD);

        //**** The IMU and associated variables ************
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        initialAngle = getAngle();
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
        grabber.setMode(DcMotor.RunMode.RESET_ENCODERS);
        grabber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //launchLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //launchRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }

    @Override
    public void loop() {

        // <Driver 1>

        telemetry.addData("Left Velocity: ", launchLeft.getVelocity());
        //telemetry.addData("Right Velocity: ", launchRight.getVelocity());

        updateDist();
        telemetry.addData("Front Dist: ", readFrontDist);
        telemetry.addData("Back Dist: ", readBackDist);
        telemetry.update();

        double norm = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        if(gamepad1.right_trigger >= 0.2) {
            DriveSpeed=0.5;
        }
        else{
            DriveSpeed=1;
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setPower((norm - yaw + strafe)*DriveSpeed);
        backLeft.setPower(-(-norm + yaw + strafe)*DriveSpeed);
        frontRight.setPower(-(norm + yaw - strafe)*DriveSpeed);
        backRight.setPower((-norm - yaw - strafe)*DriveSpeed);

        /*telemetry.addData("frontLeft",(norm + strafe + yaw));
        telemetry.addData("backLeft",(norm - strafe + yaw));
        telemetry.addData("frontRight",(norm - strafe - yaw));
        telemetry.addData("backRight",(norm + strafe - yaw));*/

        /*if (gamepad1.dpad_left) {
            initialAngle = getAngle();
            updateDist();
            savedBackDist=readBackDist;
            savedFrontDist=readFrontDist;
            savedRightDist=readBackDist;
            savedLeftDist=readFrontDist;


            double count = 0;
            if(readBackDist>500 && readFrontDist>500 || readRightDist>500 && readLeftDist>500 && count<4){
                updateDist();
                count++;
            }
            if(count==4){
                count=0;
                telemetry.addData("Angle Get: ", "Failed!");
                telemetry.update();
            }
        }*/





        if (gamepad1.dpad_left){
            initialAngle = getAngle();
        }
        if(gamepad1.dpad_right){
            zeroBotEncoder(1);
            updateDist();

            if(readFrontDist> 500){
                double moveForwardDist = readBackDist-idealBackWall;
                forwardEncoder(moveForwardDist,1);
                zeroBotEncoder(1);
            }
            else{
                double moveBackDist = idealFrontWall-readFrontDist;
                reverseEncoder(moveBackDist,1);
                zeroBotEncoder(1);
            }
            updateDist();

            if(readLeftDist>500){
                double moveLeftDist= readRightDist-idealRightWall;
                strafeLeftEncoder(moveLeftDist,1);
                zeroBotEncoder(1);
            }
            else {
                double moveRightDist = readLeftDist-idealLeftWall;
                strafeRightEncoder(moveRightDist,1);
                zeroBotEncoder(1);
            }

            /*pusher.setPosition(0.2);
            pause(1.25);
            pusher.setPosition(0.35);
            pause(1.25);
            pusher.setPosition(0.2);
            pause(1.25);
            pusher.setPosition(0.35);
            pause(1.25);
            pusher.setPosition(0.2);
            pause(1.25);
            pusher.setPosition(0.35);
            pause(1.25);*/
            //pause(0.2);



        }
        /*if (gamepad1.dpad_right){
            boolean checkMove = false;
            zeroBotEncoder(1);
            updateDist();
            if(readRightDist<readLeftDist && readRightDist>idealRightWall){
                strafeLeftEncoder(10, 1);
                updateDist();
                checkMove = true;
            }
            else if(readLeftDist<readRightDist && readLeftDist>idealLeftWall && checkMove==true){
                strafeLeftEncoder(10, 1);
                updateDist();
            }
            checkMove = false;

            /*zeroBotEncoder(1);
            updateDist();
            if(readFrontDist>500 && readBackDist>500){
                reverseEncoder(30, 1);
                updateDist();
            }
            else{
                if(readBackDist>500){
                    double moveBackDist=readBackDist-idealBackWall;
                    reverseEncoder(moveBackDist, 1);
                }
                else if(readFrontDist>500){
                    double moveFrontDist=readFrontDist-idealFrontWall;
                    moveFrontDist=moveFrontDist*-1;
                    strafeLeftEncoder(moveFrontDist, 1);
                }
            }*/

            /*if(savedRightDist>500){
                double moveDist = 182.88-(readLeftDist+37);
                strafeRightEncoder(moveDist, 1);
            }else{
                double moveDist = readRightDist - savedRightDist;
                strafeRightEncoder(moveDist, 1);
            }
            zeroBotEncoder(1);

            if(savedBackDist>500){
                double moveDist = 365.76-(readFrontDist+37);
                reverseEncoder(moveDist, 1);
            }else{
                double moveDist = readBackDist - savedBackDist;
                reverseEncoder(moveDist, 1);
            }
            zeroBotEncoder(1);*/
        //}

        if(gamepad1.a){
            reverseEncoder(20, 1);
        }
        if(gamepad1.b){
            strafeRightEncoder(20, 1);
        }




        // <Driver 2>
        // CONTROLS:
        // left trigger:  slow down the launcher
        // x:             run advancer
        // right bumper:  run intake backward
        // y:             KILL EVERYTHING

        if(gamepad2.dpad_up) {
            if(launchPower < 0.75) {
                launchPower += 0.00005;
            }
        }
        else if (gamepad2.dpad_down) {
            launchPower -= 0.00005;
        }

        else if(gamepad2.dpad_right) {
            if(launchPower < 0.75) {
                launchPower += 0.0005;
            }
        }
        else if (gamepad2.dpad_left) {
            launchPower -= 0.0005;
        }

        if(gamepad2.y) {
            launchPower = 0.387;
        }
        else if(gamepad2.b){
            launchPower = 0.42;
        }
        telemetry.addData("launchPower",launchPower);

        //launchLeft.setVelocity(1);
        launchLeft.setPower(launchPower);

        //launchRight.setVelocity(1);
        launchRight.setPower(launchPower);

        //launchLeft.setRunMode(DcMotor.RunMode.);

            if(gamepad2.x || gamepad1.x) {
                pusherPos = 0.2;
            } else {
                pusherPos = 0.35;
            }
            pusher.setPosition(pusherPos);
            if(!gamepad2.left_bumper && !gamepad1.left_bumper) {
                if(gamepad2.right_bumper) {
                    //intake.setPower(-1);
                } else {
                    //intake.setPower(1);
                }
            } else {
                //intake.setPower(0);
            }

        if(gamepad2.right_stick_y < 0.5){
            grabberPos+=1.5;
            grabber.setTargetPosition((int)(grabberPos));

        }
        if(gamepad2.right_stick_y > -0.5){
            grabberPos-=1.5;
            grabber.setTargetPosition((int)(grabberPos));
        }
        grabber.setPower(0.8);
        grabber.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(gamepad2.left_trigger>0.05){
            latch.setPosition(0);
        } else {
            latch.setPosition(0.4);
        }

        if(gamepad2.right_stick_button){
            //wobble
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {}

    public void updateDist(){
        readBackDist=backDist.getDistance(DistanceUnit.CM);
        readRightDist=rightDist.getDistance(DistanceUnit.CM);
        readFrontDist=frontDist.getDistance(DistanceUnit.CM);
        readLeftDist=leftDist.getDistance(DistanceUnit.CM);
        telemetry.addData("Back Dist, ",readBackDist);
        telemetry.addData("Right Dist, ",readRightDist);
        telemetry.addData("Left Dist, ",readLeftDist);
        telemetry.addData("Front Dist, ",readFrontDist);
        telemetry.update();
    }
    public void zeroBotEncoder(double MotorPower){
        double newAngle = getAngle();
        telemetry.addData("zeroBot Initial ",initialAngle);

        telemetry.addData("New ",newAngle);
        telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
        telemetry.update();
        while (Math.abs(newAngle - initialAngle) > 1){
            telemetry.addData("Zerobot Adj Initial ",initialAngle);
            telemetry.addData("New ",newAngle);
            telemetry.addData("Diff ",Math.abs(newAngle - initialAngle));
            telemetry.update();
            newAngle = getAngle();
            ///*** Better to come up with a formula for how long to turn based on difference in the angle
            //** ie how many degress would rightTurn(0.1) get me and calculate value based on Math.abs(newAngle - initialAngle)
            //if (newAngle > initialAngle + 10 || newAngle > initialAngle - 10){
            if (newAngle > initialAngle + 10 || newAngle > initialAngle-10){
                rightEncoder(Math.abs(newAngle - initialAngle), MotorPower);
            }else {
                leftEncoder(Math.abs(newAngle - initialAngle), MotorPower);
            }
        }

    }
    public double getAngle()
    {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }
    public void leftEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        frontRight.setTargetPosition((int)(pos/countPerDegree));
        frontLeft.setTargetPosition((int)(pos/countPerDegree));
        backRight.setTargetPosition((int)(pos/countPerDegree));
        backLeft.setTargetPosition((int)(pos/countPerDegree));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(MotorPower);
        frontLeft.setPower(MotorPower);
        backRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);

        while (frontLeft.isBusy()){

        }
    }
    public void strafeLeftEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(-cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(-cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void strafeRightEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(-cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(-cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void reverseEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(-cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(-cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void forwardEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        double cmOffset = pos/25;

        frontLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        frontRight.setTargetPosition((int)(-cmOffset*countPerRotation));
        backLeft.setTargetPosition((int)(cmOffset*countPerRotation));
        backRight.setTargetPosition((int)(-cmOffset*countPerRotation));

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backRight.setPower(MotorPower);
        frontRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);
        frontLeft.setPower(MotorPower);

        while (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy()){

        }
    }
    public void rightEncoder(double pos, double MotorPower){
        frontLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RESET_ENCODERS);

        frontRight.setTargetPosition((int)(-pos/countPerDegree));
        frontLeft.setTargetPosition((int)(-pos/countPerDegree));
        backRight.setTargetPosition((int)(-pos/countPerDegree));
        backLeft.setTargetPosition((int)(-pos/countPerDegree));

        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontRight.setPower(MotorPower);
        frontLeft.setPower(MotorPower);
        backRight.setPower(MotorPower);
        backLeft.setPower(MotorPower);

        while (frontLeft.isBusy()){

        }
    }
    public void launch(){
        pusher.setPosition(0.2);
        pause(1);
        pusher.setPosition(0.35);
        pause(1);
    }
    public void pause(double secs){
        ElapsedTime mRuntime = new ElapsedTime();
        while(mRuntime.time()< secs){

        }
    }
}