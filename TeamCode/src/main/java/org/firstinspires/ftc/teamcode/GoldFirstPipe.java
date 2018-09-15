package org.firstinspires.ftc.teamcode;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
//import java.util.stream.Collectors;
import java.util.HashMap;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.objdetect.*;
public class GoldFirstPipe {

    //Outputs
    private Mat resizeImageOutput = new Mat();
    private Mat cvCvtcolor0Output = new Mat();
    private Mat cvCvtcolor1Output = new Mat();
    private Mat rgbThresholdOutput = new Mat();
    private Mat blurOutput = new Mat();
    private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
    private int passedWidth;
    private int passedHeight;

    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }
    public GoldFirstPipe(int width, int height) {
        passedWidth = width;
        passedHeight = height;
    }
    /**
     * This is the primary method that runs the entire pipeline and updates the outputs.
     */
    public void process(Mat source0) {
        // Step Resize_Image0:
        Mat resizeImageInput = source0;
        double resizeImageWidth = passedWidth;
        double resizeImageHeight = passedHeight;
        int resizeImageInterpolation = Imgproc.INTER_AREA;
        resizeImage(resizeImageInput, resizeImageWidth, resizeImageHeight, resizeImageInterpolation, resizeImageOutput);

        // Step CV_cvtColor0:
        Mat cvCvtcolor0Src = resizeImageOutput;
        int cvCvtcolor0Code = Imgproc.COLOR_BGRA2BGR;
        cvCvtcolor(cvCvtcolor0Src, cvCvtcolor0Code, cvCvtcolor0Output);

        // Step CV_cvtColor1:
        Mat cvCvtcolor1Src = cvCvtcolor0Output;
        int cvCvtcolor1Code = Imgproc.COLOR_RGB2YCrCb;
        cvCvtcolor(cvCvtcolor1Src, cvCvtcolor1Code, cvCvtcolor1Output);

        // Step RGB_Threshold0:
        Mat rgbThresholdInput = cvCvtcolor1Output;
        double[] rgbThresholdRed = {147.62629917427392, 255.0};
        double[] rgbThresholdGreen = {0.0, 255.0};
        double[] rgbThresholdBlue = {0.0, 255.0};
        rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);

        // Step Blur0:
        Mat blurInput = rgbThresholdOutput;
        BlurType blurType = BlurType.get("Median Filter");
        double blurRadius = 8.408407692436699;
        blur(blurInput, blurType, blurRadius, blurOutput);

        // Step Find_Contours0:
        Mat findContoursInput = blurOutput;
        boolean findContoursExternalOnly = false;
        findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);

    }

    /**
     * This method is a generated getter for the output of a Resize_Image.
     * @return Mat output from Resize_Image.
     */
    public Mat resizeImageOutput() {
        return resizeImageOutput;
    }

    /**
     * This method is a generated getter for the output of a CV_cvtColor.
     * @return Mat output from CV_cvtColor.
     */
    public Mat cvCvtcolor0Output() {
        return cvCvtcolor0Output;
    }

    /**
     * This method is a generated getter for the output of a CV_cvtColor.
     * @return Mat output from CV_cvtColor.
     */
    public Mat cvCvtcolor1Output() {
        return cvCvtcolor1Output;
    }

    /**
     * This method is a generated getter for the output of a RGB_Threshold.
     * @return Mat output from RGB_Threshold.
     */
    public Mat rgbThresholdOutput() {
        return rgbThresholdOutput;
    }

    /**
     * This method is a generated getter for the output of a Blur.
     * @return Mat output from Blur.
     */
    public Mat blurOutput() {
        return blurOutput;
    }

    /**
     * This method is a generated getter for the output of a Find_Contours.
     * @return ArrayList<MatOfPoint> output from Find_Contours.
     */
    public ArrayList<MatOfPoint> findContoursOutput() {
        return findContoursOutput;
    }


    /**
     * Scales and image to an exact size.
     * @param input The image on which to perform the Resize.
     * @param width The width of the output in pixels.
     * @param height The height of the output in pixels.
     * @param interpolation The type of interpolation.
     * @param output The image in which to store the output.
     */
    private void resizeImage(Mat input, double width, double height,
                             int interpolation, Mat output) {
        Imgproc.resize(input, output, new Size(width, height), 0.0, 0.0, interpolation);
    }

    /**
     * Converts an image from one color space to another.
     * @param src Image to convert.
     * @param code conversion code.
     * @param dst converted Image.
     */
    private void cvCvtcolor(Mat src, int code, Mat dst) {
        Imgproc.cvtColor(src, dst, code);
    }

    /**
     * Segment an image based on color ranges.
     * //@param input The image on which to perform the RGB threshold.
     * //@param red The min and max red.
     * //@param green The min and max green.
     * //@param blue The min and max blue.
     * /@param output The image in which to store the output.
     */
    private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
                              Mat out) {
        Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
        Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
                new Scalar(red[1], green[1], blue[1]), out);
    }

    /**
     * An indication of which type of filter to use for a blur.
     * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
     */
    enum BlurType{
        BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
        BILATERAL("Bilateral Filter");

        private final String label;

        BlurType(String label) {
            this.label = label;
        }

        public static BlurType get(String type) {
            if (BILATERAL.label.equals(type)) {
                return BILATERAL;
            }
            else if (GAUSSIAN.label.equals(type)) {
                return GAUSSIAN;
            }
            else if (MEDIAN.label.equals(type)) {
                return MEDIAN;
            }
            else {
                return BOX;
            }
        }

        @Override
        public String toString() {
            return this.label;
        }
    }

    /**
     * Softens an image using one of several filters.
     * @param input The image on which to perform the blur.
     * @param type The blurType to perform.
     * @param doubleRadius The radius for the blur.
     * @param output The image in which to store the output.
     */
    private void blur(Mat input, BlurType type, double doubleRadius,
                      Mat output) {
        int radius = (int)(doubleRadius + 0.5);
        int kernelSize;
        switch(type){
            case BOX:
                kernelSize = 2 * radius + 1;
                Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
                break;
            case GAUSSIAN:
                kernelSize = 6 * radius + 1;
                Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
                break;
            case MEDIAN:
                kernelSize = 2 * radius + 1;
                Imgproc.medianBlur(input, output, kernelSize);
                break;
            case BILATERAL:
                Imgproc.bilateralFilter(input, output, -1, radius, radius);
                break;
        }
    }

    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     * @param //input The image on which to perform the Distance Transform.
     * @param //type The Transform.
     * @param //maskSize the size of the mask.
     * @param //output The image in which to store the output.
     */
    private void findContours(Mat input, boolean externalOnly,
                              List<MatOfPoint> contours) {
        Mat hierarchy = new Mat();
        contours.clear();
        int mode;
        if (externalOnly) {
            mode = Imgproc.RETR_EXTERNAL;
        }
        else {
            mode = Imgproc.RETR_LIST;
        }
        int method = Imgproc.CHAIN_APPROX_SIMPLE;
        Imgproc.findContours(input, contours, hierarchy, mode, method);
    }




}
