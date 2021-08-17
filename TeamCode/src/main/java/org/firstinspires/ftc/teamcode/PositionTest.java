package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.SwitchableLight;

@Autonomous(name="PositionTest", group="linearOpMode")
public class PositionTest extends AutonomousPrime2020 {
    @Override
    public void runOpMode(){
        mapObjects();
        waitForStart();

        while(opModeIsActive()){
            updateDist();
            telemetry.addData("Left Dist: ", readLeftDist);
            telemetry.addData("Right Dist: ", readRightDist);
            telemetry.addData("Front Dist: ", readFrontDist);
            telemetry.addData("Back Dist: ", readBackDist);
            telemetry.update();
        }

        /*forwardEncoder(130, 1);
        pause(0.1);
        VennisFunctEnhanced(220, 1, 0.35);*/

        //telemetry.addData("Hello!", " World!");
        //pause(10);
        /*leftEncoder(90, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);

        leftEncoder(180, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);

        leftEncoder(270, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);

        leftEncoder(360, 0.25);
        pause(0.75);
        zeroBotEncoder(0.25);
        pause(0.75);*/

        /*wobbleLock(); //Servo locks to wobble
        intakeAdvance.setPosition(0.35); //Set intake advance arm to neutral position
        velocitySpin(1, 1000);
        forwardEncoder(160, 1); //Approach first PS
        zeroBotEncoder(1); //Zero angle
        //updateDist(); //Get updated distances
        for(int i = 0; i<3; i++) {
            updateRightDist();
            double rightWallDist = readRightDist - 105; //Calculate how much & what direction to move in
            //Was 125, 105
            strafeRightEncoder(rightWallDist, 0.5); //Move the distance above
            pause(0.5);
            zeroBotEncoder(0.5);
            pause(0.5);
        }
        zeroBotEncoder(1);
        launchAdvanceFast(); //Hit first PS

        updateLeftDist();
        double leftWallDistFirst = readLeftDist-78;
        strafeLeftEncoder(leftWallDistFirst, 0.5);
        zeroBotEncoder(1); //Zero angle
        pause(0.1); //Pause for launch arm to move
        safeLaunch(1000); //Hit second PS

        updateLeftDist();
        double leftWallDistSecond = readLeftDist-65;
        strafeLeftEncoder(leftWallDistSecond, 0.5);
        zeroBotEncoder(1); //Zero angle
        pause(0.1); //Pause for launch arm to move
        safeLaunch(1000); //Hit third PS*/

        /*strafeLeftEncoder(20, 0.5); //Strafe to third PS
        zeroBotEncoder(1); //Zero angle
        pause(0.1); //Pause for launch arm to move
        safeLaunch(1000); //Hit third PS

        updateDist();
        double readRightDistSecond = readRightDist;
        double readLeftDistSecond = readLeftDist;
        telemetry.addData("Third Right Dist: ", readRightDistSecond);
        telemetry.addData("Third Left Dist: ", readLeftDistSecond);
        telemetry.update();
        pause(100);*/

        /*latch.setPosition(0);
        pause(1.5);
        latch.setPosition(0.1);
        pause(1.5);
        latch.setPosition(0.2);
        pause(1.5);
        latch.setPosition(0.3);
        pause(1.5);
        latch.setPosition(0.4);
        pause(1.5);
        latch.setPosition(0.5);
        pause(1.5);
        latch.setPosition(0.6);
        pause(1.5);
        latch.setPosition(0.7);
        pause(1.5);
        latch.setPosition(0.8);
        pause(1.5);
        latch.setPosition(0.9);
        pause(1.5);
        latch.setPosition(1);
        pause(1.5);*/


        //velocitySpin(1, 1060);
        /*while(opModeIsActive()){
            telemetry.addData("Right Launch Wheel: ", launchRight.getVelocity());
            telemetry.addData("Left Launch Wheel: ", launchLeft.getVelocity());
            telemetry.update();
            double launchLeftVelocity = launchLeft.getVelocity();
            double launchRightVelocity = launchRight.getVelocity();
            if((launchLeftVelocity<=1080 && launchLeftVelocity>=1040) && (launchRightVelocity<=1080 && launchRightVelocity>=1040)){
                telemetry.addData("Ready to spin at: ", launchLeftVelocity);
            }
        }*/


        /*updateDist();
        double ILD = readLeftDist;
        double IBD = readBackDist;
        VennisFunctEnhanced(50, 0.25, 0.75);
        updateDist();
        double NLD = readLeftDist;
        double NBD = readBackDist;
        double rightDistMoved = NLD-ILD;
        double backDistMoved = NBD-IBD;
        double radiansTurned = Math.atan2(backDistMoved, rightDistMoved);
        double degreesTurned = Math.toDegrees(radiansTurned);
        telemetry.addData("Initial Left Dist: ", ILD);
        telemetry.addData("Initial Back Dist: ", IBD);
        telemetry.addData("Second Left Dist: ", NLD);
        telemetry.addData("Second Back Dist: ", NBD);
        telemetry.addData("Dist Moved Right: ", rightDistMoved);
        telemetry.addData("Dist Moved Back: ", backDistMoved);
        telemetry.addData("Radians Turned: ", radiansTurned);
        telemetry.addData("Degrees Turned: ", degreesTurned);
        telemetry.update();
        pause(30);*/



        //wobbleGrabDown(1);
        //pause(100);

        /*latch.setPosition(0);
        pause(2);
        latch.setPosition(0.1);
        pause(2);
        latch.setPosition(0.2);
        pause(2);
        latch.setPosition(0.3);
        pause(2);
        latch.setPosition(0.4);
        pause(2);
        latch.setPosition(0.5);
        pause(2);
        latch.setPosition(0.6);
        pause(2);
        latch.setPosition(0.7);
        pause(2);
        latch.setPosition(0.8);
        pause(2);
        latch.setPosition(0.9);
        pause(2);
        latch.setPosition(1);
        pause(2);*/

    }
}