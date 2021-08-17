package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(name="RGB To YCrCb", group="linearOpMode")
public class RGB_To_YCrCb extends AutonomousPrime2020 {
    Mat redDetect = new Mat();

    @Override
    public void runOpMode() {
        mapObjects();

        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        camera.openCameraDevice();
        camera.startStreaming(160, 120, OpenCvCameraRotation.UPRIGHT);

        camera.setPipeline(new OpenCvPipeline() {
            @Override
            public Mat processFrame(Mat input) {
                Imgproc.cvtColor(input, redDetect, Imgproc.COLOR_RGB2YCrCb);
                int rows = redDetect.rows(); //Calculates number of rows
                int cols = redDetect.cols(); //Calculates number of columns
                int ch = redDetect.channels(); //Calculates number of channels (Grayscale: 1, RGB: 3, etc.)
                for (int i=0; i<rows; i++) {
                    for (int j=0; j<cols; j++) {
                        double[] data = redDetect.get(i, j); //Stores element in an array
                        for (int k = 0; k < ch; k++){ //Runs for the available number of channels
                            data[k] = data[k] * 2; //Pixel modification done here
                        }
                        redDetect.put(i, j, data); //Puts element back into matrix
                    }
                }
                return redDetect;
            }
        });

        waitForStart();

        if (opModeIsActive()) {

        }
    }
}