package org.firstinspires.ftc.teamcode;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;

public class FullGoldPipe {
    private GoldFirstPipe goldPipe1 = new GoldFirstPipe(640, 480);
    private GoldSecondPipe goldPipe2 = new GoldSecondPipe();
    private ArrayList<MatOfPoint> finalContours;


    public GoldFirstPipe getGoldFirstPipe() {
        return goldPipe1;
    }
    public GoldSecondPipe getGoldSecondPipe() {
        return goldPipe2;
    }
    public ArrayList<MatOfPoint> getFinalContours() {
        return finalContours;
    }

    public void process(Mat source0) {
        goldPipe1.process(source0);
        Mat source0Copy = source0.clone();
        ArrayList<MatOfPoint> firstContours =  goldPipe1.findContoursOutput();
        Imgproc.fillPoly(source0, firstContours, new Scalar( 255, 255, 255 ));
        goldPipe2.process(source0);
        finalContours =  goldPipe2.findContoursOutput();
    }
}



