package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import java.util.List;
// TODO: Add distance sensor checks to high goal & two other power shots; eventually, wobble delivery for 4 rings
@Autonomous(name="COLE", group="linearOpMode")
public class UltimateColeFast extends AutonomousPrime2020 {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "Aba+gBH/////AAABma/0sYDZakYVhtjb1kH5oBVmYfYsDZXTuEZL9m7EdnFKZN/0v/LvE/Yr0NsXiJo0mJmznKAA5MK6ojvgtV1e1ODodBaMYZpgE1YeoAXYpvvPGEsdGv3xbvgKhvwOvqDToPe3x5w6gsq7a4Ullp76kLxRIoZAqaRpOuf1/tiJJQ7gTBFf8MKgbCDosmMDj7FOZsclk7kos4L46bLkVBcD9E0l7tNR0H0ShiOvxBwq5eDvzvmzsjeGc1aPgx9Br5AbUwN1T+BOvqwvZH2pM2HDbybgcWQJKH1YvXH4O62ENsYhD9ubvktayK8hSuu2CpUd1FVU3YQp91UrCvaKPYMiMFu7zeQCnoc7UOpG1P/kdFKP";
    private static String labelName;
    private static int noLabel;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    @Override
    public void runOpMode() {
        mapObjects();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                updateDist();
                double backWallDist = readBackDist;
                double leftWallDist = readLeftDist;
                double rightWallDist = readRightDist;
                double count = 0;
                wobbleGrabDown(1);//This is just for testing, will probably be somewhere else in the actual code
                pause(7);//Waiting for the wobble grabber to be in place
//                while(readBackDist>backWallDist||readBackDist==backWallDist||readBackDist>backWallDist-1){
//                    strafeRightEncoder(3, 1); //Strafe right slightly closer to in line with wobble
//                    updateDist();
//                    count++;
//                    if (count==5){
//                        break;
//                    }
                while(readRightDist>=10 && readBackDist >=30){ //When the wobble is detected (With a failsafe to avoid being too close to the wall)
                    strafeRightEncoder(3, 1);  //Strafe right in small increments
                    updateDist(); //Update distance sensor values
                    count++; //Increase the count variable
                    if(count==5){ //If you've strafed 13 times (missed the wobble)...
                        break; //...then break out of the loop
                    }
                }
                while(readBackDist>10) {
                    reverseEncoder(5,1); //Goes closer to the robot, until it's in range, this value may have to change
                    updateDist();
                }
                if(readBackDist<10) { //If the robot is too close to the wobble it will readjust to be in the right place
                    double reAdjust = 10 - readBackDist; //Subtracting 10 gives us how far to go
                    forwardEncoder(reAdjust, 1);
                    updateDist();
                }
//                    while (newBackWallDist>10) {
//                        reverseEncoder(10, 1);
//                        updateDist();
//                    }
//                    strafeRightEncoder(1, 1); //Strafe to ensure hook latches
                wobbleLatch(); //Grab the wobble
                forwardEncoder(5,1);
                updateDist();
                if (readBackDist>12) { //checking to make sure it has the wobble, may have to adjust this value
                    reverseEncoder(4,1); //Going back towards the wobble, since we already went forwad 5, the 4 is to cover most of the distance without overdoing it
                    while(readBackDist>10) {
                        reverseEncoder(2,1);//May need to adjust
                        updateDist();
                    }
                    wobbleLatch();
                }
                forwardEncoder(5,1);
                updateDist();
                if (readBackDist>12) { //If the robot doesn't have the wobble at this point, it just goes to park, instead of dropping off a wobble it doesn't have
                    forwardEncoder(160, 1);//This value may change
                }
            }
        }
    }
}