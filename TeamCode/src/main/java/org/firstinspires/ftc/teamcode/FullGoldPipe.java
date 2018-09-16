package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.vuforia.Frame;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgcodecs.Imgcodecs;
import java.util.ArrayList;

public class FullGoldPipe {

    private GoldFirstPipe goldPipe1;
    private GoldSecondPipe goldPipe2 = new GoldSecondPipe();
    private ArrayList<MatOfPoint> finalContours;
    private VuforiaLocalizer vuforiaPass;

    public FullGoldPipe(int width, int height, VuforiaLocalizer passedVuforia) {
        goldPipe1 = new GoldFirstPipe(width, height);
        vuforiaPass = passedVuforia;
    }

    public GoldFirstPipe getGoldFirstPipe() {
        return goldPipe1;
    }
    public GoldSecondPipe getGoldSecondPipe() {
        return goldPipe2;
    }
    public ArrayList<MatOfPoint> getFinalContours() {
        return finalContours;
    }

    public void process(Frame frame) {
        Bitmap bmp = vuforiaPass.convertFrameToBitmap(frame);

        Mat source0 = new Mat(bmp.getWidth(), bmp.getHeight(), CvType.CV_8UC3);
        Utils.bitmapToMat(bmp, source0);


        goldPipe1.process(source0);
        Mat source0Copy = source0.clone();
        ArrayList<MatOfPoint> firstContours =  goldPipe1.findContoursOutput();
        Imgproc.fillPoly(source0, firstContours, new Scalar( 255, 255, 255 ));
        goldPipe2.process(source0);
        finalContours =  goldPipe2.findContoursOutput();
    }
}



