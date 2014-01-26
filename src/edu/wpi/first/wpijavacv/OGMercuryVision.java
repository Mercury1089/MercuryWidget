package edu.wpi.first.wpijavacv;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.util.ArrayList;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JSpinner;
import javax.swing.SpinnerNumberModel;

public class OGMercuryVision extends WPICameraExtension {
    
    //Camera constants used for distance calculation
    private final static int X_RES = 640;		
    private final static int Y_RES = 480;		
    private final static double VERT_FOV = Math.toRadians(49.0);     //Axis M1013
    private final static double HOR_FOV = Math.toRadians(67.0);  //Axis M1013 camera
    
    public static final String NAME = "OG Vision Tracking";

    private static final double maxAngleError = 0.4; //TODO
    private static final double maxHorError = 0.6; //TODO
    private static final double maxVertError = 0.6; //TODO
    private static final double maxRatioError = 0.3; //TODO
    private static final double kNearlyHorizontalSlope = Math.tan(Math.toRadians(20));
    private static final double kNearlyVerticalSlope = Math.tan(Math.toRadians(90-20));
    private static final int kMinHorWidth = 30; 
    private static final int kMinHorHeight = 5; 
    private static final int kMaxHorWidth = 150;
    private static final int kMaxHorHeight = 20;
    private static final int kMinVertWidth = 5;
    private static final int kMinVertHeight = 45;
    private static final int kMaxVertWidth = 30;
    private static final int kMaxVertHeight = 200;
    private static final double kRangeOffset = 0.0; //TODO
    private double maxHue = 90, minHue = 45, 
                   maxSat = 255, minSat = 200, 
                   maxVal = 255, minVal = 55;

    private static final double kShooterOffsetDeg = 0; //TODO

    private static final double kCameraHeightIn = 24; //TODO
    private static final double kTopTargetHeightIn = 101.25;
    
    // Store JavaCV temporaries as members to reduce memory management during processing
    private opencv_core.CvSize size = null;
    private WPIContour[] contours;
    private ArrayList<WPIPolygon> horiz;
    private ArrayList<WPIPolygon> vert;
    private opencv_imgproc.IplConvKernel morphKernel;
    private opencv_core.IplImage bin;
    private opencv_core.IplImage hsv;
    private opencv_core.IplImage hue;
    private opencv_core.IplImage sat;
    private opencv_core.IplImage val;
    private WPIPoint linePt1;
    private WPIPoint linePt2;
    private int horizontalOffsetPixels;
    private WPIBinaryImage binWPI;
    private static opencv_core.CvMemStorage storage;
    
    private ArrayList<PairedTarget> pairs;
    
    private JLabel sideDir, distanceVal, numHoriz, numVert;
    private JLabel hueID, satID, valID;
    private JSpinner hueUpper, hueLower, satUpper, satLower, valUpper, valLower;
    private JCheckBox dispBinary;
    
    public OGMercuryVision() {
        morphKernel = opencv_imgproc.IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
        storage = opencv_core.CvMemStorage.create();
        
        pairs = new ArrayList<>();
    }
    
    @Override
    public void init() {
        super.init();
        this.setLayout(new BorderLayout());
        
        distanceVal = new JLabel();
        sideDir = new JLabel();
        numHoriz = new JLabel();
        numVert = new JLabel();
        hueID = new JLabel("Hue Limits:");
        satID = new JLabel("Sat Limits:");
        valID = new JLabel("Val Limits:");
        
        dispBinary = new JCheckBox("Display Binary");
        
        hueUpper = new JSpinner(new SpinnerNumberModel(maxHue, 0, 255, 1));
        hueLower = new JSpinner(new SpinnerNumberModel(minHue, 0, 255, 1));
        satUpper = new JSpinner(new SpinnerNumberModel(maxSat, 0, 255, 1));
        satLower = new JSpinner(new SpinnerNumberModel(minSat, 0, 255, 1));
        valUpper = new JSpinner(new SpinnerNumberModel(maxVal, 0, 255, 1));
        valLower = new JSpinner(new SpinnerNumberModel(minVal, 0, 255, 1));
        
        JPanel controls = new JPanel();
        controls.setLayout(new GridLayout(9, 2));
        controls.add(dispBinary);
        controls.add(new JLabel());
        controls.add(sideDir);
        controls.add(distanceVal);
        controls.add(numHoriz);
        controls.add(numVert);
        controls.add(hueID);
        controls.add(hueUpper); 
        controls.add(new JLabel());
        controls.add(hueLower);
        controls.add(satID);
        controls.add(satUpper);
        controls.add(new JLabel());
        controls.add(satLower);
        controls.add(valID);
        controls.add(valUpper);
        controls.add(new JLabel());
        controls.add(valLower);
        
        add(BorderLayout.WEST, controls);
    }
    
    @Override
    public WPIImage processImage(WPIColorImage raw) {
        //if fields are wrong, initialize them
        if(size == null || size.width() != raw.getWidth() || size.height() != raw.getHeight()) {
            size = opencv_core.cvSize(raw.getWidth(),raw.getHeight());
            bin = opencv_core.IplImage.create(size, 8, 1);
            hsv = opencv_core.IplImage.create(size, 8, 3);
            hue = opencv_core.IplImage.create(size, 8, 1);
            sat = opencv_core.IplImage.create(size, 8, 1);
            val = opencv_core.IplImage.create(size, 8, 1);
            horizontalOffsetPixels =  (int)Math.round(kShooterOffsetDeg*(size.width()/HOR_FOV));
            linePt1 = new WPIPoint(size.width()/2+horizontalOffsetPixels,size.height()-1);
            linePt2 = new WPIPoint(size.width()/2+horizontalOffsetPixels,0);
        }
        
        minHue = (Double)hueLower.getValue();
        maxHue = (Double)hueUpper.getValue();
        
        minSat = (Double)satLower.getValue();
        maxSat = (Double)satUpper.getValue();
        
        minVal = (Double)valLower.getValue();
        maxVal = (Double)valUpper.getValue();
        
        //split the image into H, S, and V layers
        IplImage input = raw.image;
        opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV);
        opencv_core.cvSplit(hsv, hue, sat, val, null);
        
        //remove pixels where H < 45 or H > 75
        opencv_imgproc.cvThreshold(hue, bin, minHue, 255, opencv_imgproc.CV_THRESH_BINARY); //TODO change values maybe
        opencv_imgproc.cvThreshold(hue, hue, maxHue, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        // remove pixels that aren't colorful enough
        opencv_imgproc.cvThreshold(sat, sat, minSat, maxSat, opencv_imgproc.CV_THRESH_BINARY);

        // remove pixels that are too dim
        opencv_imgproc.cvThreshold(val, val, minVal, maxVal, opencv_imgproc.CV_THRESH_BINARY);
        
        //combine all images, hopefully remaining pixels are targets
        opencv_core.cvAnd(hue, bin, bin, null);
        opencv_core.cvAnd(bin, sat, bin, null);
        opencv_core.cvAnd(bin, val, bin, null);

        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, 1);

        binWPI = makeWPIBinaryImage(bin);
        contours = findConvexContours(binWPI);
        
        horiz = new ArrayList<>();
        vert = new ArrayList<>();
        for (WPIContour c : contours) {
            double ratio = ((double) c.getHeight()) / ((double) c.getWidth());
            if (relativeError(ratio, 4/23.5) < maxHorError && contains(kMinHorWidth, kMaxHorWidth, c.getWidth())) {
                horiz.add(c.approxPolygon(20));
            } else if (relativeError(ratio, 32.0/4) < maxVertError && contains(kMinVertHeight, kMaxVertHeight, c.getHeight())) {
                vert.add(c.approxPolygon(20));
            }
        }
        numHoriz.setText("Horiz: " + horiz.size());
        numVert.setText("Vert: " + vert.size());
        
        if(vert.size() > 0) {
            distanceVal.setText("Distance:\n" + getDistance(vert.get(0))); //TODO multiple verticals
        } else {
            distanceVal.setText("Distance:\nNONE");
        }
        if(pairs.size() > 0) {
            sideDir.setText("Side:" + (pairs.get(0).isRight? "Right" : "Left")); //TODO mutiple targets
        } else {
            sideDir.setText("Side:\nNONE");
        }
        
        opencv_core.cvClearMemStorage(storage);
        if(dispBinary.isSelected()) {
            return binWPI;
        } else
            return raw;
    }
    
    public static double relativeError(double result, double expected) {
        return (result - expected) / expected;
    }
    
    public static boolean contains(double lower, double upper, double value) {
        return lower <= value && upper >= value;
    }
    
    public static WPIBinaryImage makeWPIBinaryImage(IplImage arr) {
        IplImage tempImage = IplImage.create(arr.cvSize(), arr.depth(), 1);
        opencv_core.cvCopy(arr, tempImage);
        return new WPIBinaryImage(tempImage);
    }
    
    public static WPIContour[] findConvexContours(WPIBinaryImage image) {
        image.validateDisposed();

        IplImage tempImage = IplImage.create(image.image.cvSize(), image.image.depth(), 1);

        opencv_core.cvCopy(image.image, tempImage);

        opencv_core.CvSeq contours = new opencv_core.CvSeq();
        opencv_imgproc.cvFindContours(tempImage, storage, contours, 256, opencv_imgproc.CV_RETR_LIST, opencv_imgproc.CV_CHAIN_APPROX_TC89_KCOS);
        ArrayList<WPIContour> results = new ArrayList();
        while (!WPIDisposable.isNull(contours)) {
            opencv_core.CvSeq convexContour = opencv_imgproc.cvConvexHull2(contours, storage, opencv_imgproc.CV_CLOCKWISE, 1);
            WPIContour contour = new WPIContour(opencv_core.cvCloneSeq(convexContour, storage));
            results.add(contour);
            contours = contours.h_next();
        }

        tempImage.release();
        WPIContour[] array = new WPIContour[results.size()];
        return results.toArray(array);
    }
    
    public static double getDistance(WPIPolygon polygon){
        double distance = (Y_RES * 32 / 12.0) / (polygon.getHeight() * 2 * Math.tan(VERT_FOV / 2.0));
        
        return (int)(distance * 100) / 100.0;
    }
    
    private class PairedTarget {
        protected WPIPolygon vert, horiz;
        protected boolean isRight;
        public PairedTarget(WPIPolygon vert, WPIPolygon horiz) {
            this.vert = vert;
            this.horiz = horiz;
            isRight = isRight();
        }
        
        private boolean isRight() {
            return horiz.getX()- vert.getX() > 0;
        }
    }
}