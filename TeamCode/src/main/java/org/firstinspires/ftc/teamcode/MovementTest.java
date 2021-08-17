package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutonomousTestCode", group="linearOpMode")
public class MovementTest extends AutonomousPrime2020 {
    @Override
    public void runOpMode(){
        mapObjects();
        waitForStart();

        //launch speed start = 0.42
        //launch speed second = 0.465
        //launch function is launchAdvanceFast

        double version = 0;

        //Version 0 is 4 rings
        //Version 1 is 1 ring
        //Version 2 is 0 rings

        if(version==0){
            wobbleLock();
            intakeAdvance.setPosition(0.35);
            startLaunch(0.42);
            forwardEncoder(160, 1);
            zeroBotEncoder(1);
            updateDist();
            double rightWallDist=readRightDist-115; //was 30
            //rightWallDist=Math.abs(rightWallDist);
            strafeRightEncoder(rightWallDist, 0.5);
            //pause(0.2);
            launchAdvanceFast();
            //launchAdvanceFast();
            strafeLeftEncoder(25, 1);
            zeroBotEncoder(1);
            pause(0.1);
            launchAdvanceFast();
            //launchAdvanceFast();
            strafeLeftEncoder(20, 1);
            zeroBotEncoder(1);
            pause(0.1);
            launchAdvanceFast();
            //pause(0.2);

            intakeStart(1);
            startLaunch(0.4455);
            //strafeRightEncoder(87, 0.75);
            strafeRightEncoder(87, 1);

            zeroBotEncoder(1);
            pause(0.2);
            /*updateDist();
            if(rightWallDist>500){
                //strafeRightEncoder(15, 1);
            } else{
                rightWallDist=readRightDist-40; //was 30
                //rightWallDist=Math.abs(rightWallDist);
                strafeRightEncoder(rightWallDist, 0.5);
            }*/
            //pause(0.1);
            //reverseEncoder(45, 0.35);
            reverseEncoder(45, 0.4);

            zeroBotEncoder(1);
            pause(0.5);
            launchAdvanceFast();
            reverseEncoder(9, 0.5);
            pause(0.5);
            launchAdvanceFast();
            reverseEncoder(9, 0.5);
            startLaunch(0.4405); //go slower for last 2

            startLaunch(0.45);

            pause(0.5);
            launchAdvanceFast ();
            reverseEncoder(9, 0.5);
            pause(0.5);
            launchAdvanceFast();
            pause(0.5);
            launchAdvanceFast();
            pause(0.25);
            intakeEnd();

            rightEncoder(0.2, 1); //lighter turn
            pause(0.1);
            forwardEncoder(210, 1);
            wobbleRelease();
            pause(0.1);
            reverseEncoder(95, 1);
        }
        else if(version==1){
            wobbleLock();
            intakeAdvance.setPosition(0.35);
            startLaunch(0.42);
            forwardEncoder(160, 1);
            zeroBotEncoder(1);

            updateDist();

            double rightWallDist=readRightDist-115; //was 30
            //rightWallDist=Math.abs(rightWallDist);
            strafeRightEncoder(rightWallDist, 0.5);
            //pause(0.2);
            launchAdvanceFast();
            //launchAdvanceFast();
            strafeLeftEncoder(25, 1);
            zeroBotEncoder(1);
            pause(0.1);
            launchAdvanceFast();
            //launchAdvanceFast();
            strafeLeftEncoder(20, 1);
            zeroBotEncoder(1);
            pause(0.1);
            launchAdvanceFast();
            //pause(0.2);


            //strafeRightEncoder(87, 0.75);

            intakeAdvance.setPosition(0.35);
            wobbleGrabDown(1);
            forwardEncoder(75, 1);
            pause(0.1);
            strafeRightEncoder(50, 1); //was 55
            wobbleRelease();
            intakeStart(1);
            startLaunch(0.45);
            reverseEncoder(50, 1);
            pause(0.1);

            updateDist();

            rightWallDist=readRightDist-72; //was 30
            rightWallDist=Math.abs(rightWallDist);
            strafeRightEncoder(rightWallDist, 0.5);

            //strafeRightEncoder(34, 1);
            pause(0.1);
            reverseEncoder(80, 1);
            zeroBotEncoder(1);
            pause(0.5); //was 1.25
            launchAdvanceFast();
            intakeEnd();


            updateDist();
            if(rightWallDist>500){
                strafeRightEncoder(15, 1);
            } else{
                rightWallDist=readRightDist-40; //was 30
                rightWallDist=Math.abs(rightWallDist);
                strafeRightEncoder(rightWallDist, 0.5);
            }
            pause(0.1);
            reverseEncoder(37, 1);
            updateDist();

            double count = 0;

            while(readRightDist>=10 && readBackDist >=30) {
                strafeRightEncoder(3, 1);
                updateDist();
                count++;
                if(count==5){
                    break;
                }
            }
            //updateDist();
            reverseEncoder(5, 0.25);
            strafeRightEncoder(1, 1);
            //reverseEncoder(15, 0.25);
            //pause(0.1);
            wobbleLatch();
            //pause(0.25);
            wobbleGrabUp(1);
            forwardEncoder(145, 1); //was 175
            pause(0.1);
            rightEncoder(3.35, 1);
            wobbleLatchRelease();
            //pause(0.2);
            forwardEncoder(10, 1);
            wobbleGrabUpLarge(1);


            /*zeroBotEncoder(1);
            pause(0.1);
            //reverseEncoder(45, 0.35);
            reverseEncoder(45, 1);
            zeroBotEncoder(1);
            pause(1);
            launchAdvanceFast();
            pause(0.1);
            leftEncoder(0.35, 1); //was 0.375
            intakeEnd();
            pause(0.1);
            forwardEncoder(115, 1); //was 105
            pause(0.1);
            wobbleRelease();
            wobbleGrabDown(1);
            reverseEncoder(100, 1);
            zeroBotEncoder(1);
            updateDist();
            double rightWallDist=readRightDist-40; //was 30
            rightWallDist=Math.abs(rightWallDist);
            strafeRightEncoder(rightWallDist, 0.5);
            pause(0.1);
            reverseEncoder(50, 1);
            updateDist();
            while(readRightDist>=10 && readBackDist >=30) {
                strafeRightEncoder(2, 1);
                updateDist();
            }
            reverseEncoder(15, 0.25);
            pause(0.1);
            wobbleLatch();
            pause(0.25);
            forwardEncoder(100, 1);
            rightEncoder(3, 1);
            wobbleLatchRelease();
            pause(0.2);
            forwardEncoder(10, 1);
            //intakeEnd();
            //pause(0.25);
            /*wobbleGrabDown(1);
            reverseEncoder(165, 1); //was 175
            pause(0.5);
            //reverseEncoder(10, 0.25);
            //pause(0.25);
            strafeRightEncoder(30, 0.25); //was 35
            wobbleLatch();
            //pause(0.75);
            //launchAdvanceFast();
            //leftEncoder(0.25, 1);
            pause(0.1);
            forwardEncoder(140, 1); //was 160
            pause(0.1);
            rightEncoder(4.25, 1); //was 3.5
            //reverseEncoder(150, 1);
            wobbleLatchRelease();
            //pause(0.15);
            forwardEncoder(10, 1);
            //pause(1);
            //forwardEncoder(20, 1);
            /*startLaunch(0.42);
            wobbleLock();
            forwardEncoder(160, 1);
            zeroBotEncoder(1);
            pause(0.2);
            launchAdvanceFast();
            launchAdvanceFast();
            strafeLeftEncoder(25, 1);
            zeroBotEncoder(1);
            pause(0.2);
            launchAdvanceFast();
            strafeLeftEncoder(20, 1);
            zeroBotEncoder(1);
            pause(0.2);
            launchAdvanceFast();
            pause(0.2);
            intakeStart(1);
            startLaunch(0.4455);
            strafeRightEncoder(87, 1);
            zeroBotEncoder(1);
            pause(0.2);
            reverseEncoder(45, 1);
            pause(0.5);
            launchAdvanceFast();
            pause(0.2);
            intakeEnd();
            forwardEncoder(120, 1);
            zeroBotEncoder(1);
            pause(0.1);
            strafeLeftEncoder(50, 1);
            zeroBotEncoder(1);
            wobbleRelease();
            pause(0.25);
            reverseEncoder(70, 1);
            zeroBotEncoder(1);
            pause(0.1);
            strafeRightEncoder(95, 1);
            zeroBotEncoder(1);
            pause(0.1);
            reverseEncoder(85, 1);
            pause(2);
            rightEncoder(1.75, 1);
            pause(0.2);
            strafeLeftEncoder(150, 1);*/
        }
        else if(version==2){
            wobbleLock();
            intakeAdvance.setPosition(0.35);
            startLaunch(0.42);
            forwardEncoder(160, 1);
            zeroBotEncoder(1);
            updateDist();
            double rightWallDist=readRightDist-115; //was 30
            //rightWallDist=Math.abs(rightWallDist);
            strafeRightEncoder(rightWallDist, 0.5);
            //pause(0.2);
            //launchAdvanceFast();
            launchAdvanceFast();
            strafeLeftEncoder(25, 1);
            zeroBotEncoder(1);
            pause(0.3); //was 0.2
            launchAdvanceFast();
            //launchAdvanceFast();
            strafeLeftEncoder(20, 1);
            zeroBotEncoder(1);
            pause(0.1);
            launchAdvanceFast();

            forwardEncoder(20, 1);
            pause(0.2);
            strafeRightEncoder(112, 1); //was 127, 112
            pause(0.2);
            wobbleRelease();
            pause(0.75);
            zeroBotEncoder(1);

            wobbleGrabDown(1);
            reverseEncoder(115, 0.5); //was 70, 118.35
            //strafeRightEncoder(43, 0.25); //was 40
            double count = 0;
            while(readRightDist>=10 && readBackDist >=30) {
                strafeRightEncoder(3, 1);
                updateDist();
                count++;
                if(count==13){
                    break;
                }
            }
            pause(0.5);
            //reverseEncoder(5, 1);
            wobbleLatch();
            pause(1);
            wobbleGrabUp(1);
            forwardEncoder(85, 1);
            rightEncoder(5, 1); //was 5
            wobbleLatchRelease();
            forwardEncoder(20, 1);
            strafeRightEncoder(50, 1);
            reverseEncoder(55, 1);

            //strafeRightEncoder(10, 1)


            /*startLaunch(0.42);
            forwardEncoder(160, 1);
            zeroBotEncoder(1);
            pause(0.2);
            launchAdvanceFast();
            strafeLeftEncoder(25, 1);
            zeroBotEncoder(1);
            pause(0.2);
            launchAdvanceFast();
            strafeLeftEncoder(20, 1);
            zeroBotEncoder(1);
            pause(0.2);
            launchAdvanceFast();
            pause(0.2);
            forwardEncoder(20, 1);
            pause(0.2);
            strafeRightEncoder(87, 1);
            pause(0.2);
            reverseEncoder(70, 1);
            pause(3); //Grab wobble
            leftEncoder(1.5, 1);
            pause(0.2);
            strafeLeftEncoder(120, 1);
            wobbleRelease();
            pause(1.5);
            strafeRightEncoder(60, 1);*/
        }
    }
}