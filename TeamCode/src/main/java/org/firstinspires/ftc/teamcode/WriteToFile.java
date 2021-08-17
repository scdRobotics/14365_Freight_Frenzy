package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

/**
 * Created by Gabriel on 2018-01-03.
 * A simple way to log data to a file.
 */
@Autonomous(name="WriteFileTest", group="Linear Opmode")
public class WriteToFile extends AutonomousPrime2020{

    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line = "";
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;


    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    /*
    Probably will throw an error- see what stuff Android Studio suggests to make it
    */

    public void Log(String filename, boolean logTime) {
        if (logTime) startTime = System.nanoTime();
        this.logTime = logTime;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void update() {
        if (disabled) return;
        try {
            if (logTime) {
                long timeDifference = System.nanoTime()-startTime;
                line = timeDifference/1E9+","+line;
            }
            fileWriter.write(line+"\n");
            line = "";
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String data) {
        if (disabled){
            return;
        }
        if (!line.equals("")) {
            line += ",";
        }
        line += data;
    }
    public void addData(Object data) {
        addData(data.toString());
    }
    public void addData(boolean data) {
        addData(String.valueOf(data));
    }
    public void addData(byte data) {
        addData(String.valueOf(data));
    }
    public void addData(char data) {
        addData(String.valueOf(data));
    }
    public void addData(short data) {
        addData(String.valueOf(data));
    }
    public void addData(int data) {
        addData(String.valueOf(data));
    }
    public void addData(long data) {
        addData(String.valueOf(data));
    }
    public void addData(float data) {
        addData(String.valueOf(data));
    }
    public void addData(double data) {
        addData(String.valueOf(data));
    }

    public void runOpMode(){
        mapObjects();
        waitForStart();
        Log("DistanceSensorTest", false); //Make file

        //addData("LaunchRight Velocity");
        //addData("LaunchLeft Velocity");
        //addData("Elapsed Time");
        //addData("Launch Left Error");
        addData("Right Sensor Value (Cardboard)");
        addData("Right Sensor Time");
        addData("Back Sensor Value (Wall)");
        addData("Back Sensor Time");
        update(); //Update File
        System.out.println("Start Loop");
        timer.reset(); //Reset timer
        boolean loop = true;
        int count = 0;
        while (opModeIsActive() && loop) { //Loop
            //addData(launchRight.getVelocity()); //Record launch right motor velocity
            //System.out.println("In Loop");
            //telemetry.addData("Y'ello?", "Who dis?");
            //telemetry.update();
            //addData(launchLeft.getVelocity()); //Record launch left wheel motor velocity
            //addData(timer.time()); //Add timer when recording happens

            //addData(1020-launchLeft.getVelocity()); //Add timer when recording happens
            //addData(1020-launchRight.getVelocity()); //Add timer when recording happens

            updateRightDist();
            addData(readRightDist);
            addData(timer.time());
            timer.reset();

            updateBackDist();
            addData(readBackDist);
            addData(timer.time());
            timer.reset();

            count++;
            telemetry.addData("Loops Through: ", count);
            telemetry.update();


            //addData("\n"); //New line
            update(); //Update File- might want to update file when is stop requested to save time?? Risky if function fails to complete before stop, though



            if(count>=5000){
                update();
                System.out.println("Loop End");
                telemetry.update();
                close();
                loop=false;
            }
        }
    }

}