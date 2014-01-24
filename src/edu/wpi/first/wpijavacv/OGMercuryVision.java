package edu.wpi.first.wpijavacv;

import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import java.awt.Polygon;
import java.util.ArrayList;

public class OGMercuryVision extends WPICameraExtension {
    
    //Camera constants used for distance calculation
    private final int X_RES = 640;		
    private final int Y_RES = 480;		
    private final double VERT_FOV = Math.toRadians(49.0);     //Axis M1013
    private final double HOR_FOV = Math.toRadians(67.0);  //Axis M1013 camera
    
    public static final String NAME = "OG Vision Tracking";

    // Constants that need to be tuned
<<<<<<< HEAD
    private static final double maxHorError = 0.9; //TODO
    private static final double maxVertError = 0.6; //TODO
=======
    private static final double maxRatioError = 0.3; //TODO
>>>>>>> caa213b9af27d899a07b439c16d943d92ceafffb
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
    private CanvasFrame result = new CanvasFrame("binary");
    
    private static opencv_core.CvMemStorage storage;
    
    public OGMercuryVision() {
        morphKernel = opencv_imgproc.IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
        storage = opencv_core.CvMemStorage.create();
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
        //split the image into H, S, and V layers
        IplImage input = raw.image;
        opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV);
        opencv_core.cvSplit(hsv, hue, sat, val, null);
        
        //remove pixels where H < 45 or H > 75
        opencv_imgproc.cvThreshold(hue, bin, 60-15, 255, opencv_imgproc.CV_THRESH_BINARY); //TODO change values maybe
        opencv_imgproc.cvThreshold(hue, hue, 60+25, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        
        // remove pixels that aren't colorful enough
        opencv_imgproc.cvThreshold(sat, sat, 200, 255, opencv_imgproc.CV_THRESH_BINARY);

        // remove pixels that are too dim
        opencv_imgproc.cvThreshold(val, val, 55, 255, opencv_imgproc.CV_THRESH_BINARY);
        
        //combine all images, hopefully remaining pixels are targets
        opencv_core.cvAnd(hue, bin, bin, null);
        opencv_core.cvAnd(bin, sat, bin, null);
        opencv_core.cvAnd(bin, val, bin, null);

        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel, opencv_imgproc.CV_MOP_CLOSE, 1);

        WPIBinaryImage binWPI = makeWPIBinaryImage(bin);
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
        if(vert.size() > 0) {
            result.setTitle("H: " + horiz.size() + " V: " + vert.size() + " D: " + getDistance(vert.get(0)));
        } else {
            result.setTitle("no vertical targets");
        }
        
        result.showImage(bin.getBufferedImage());
        
        opencv_core.cvClearMemStorage(storage);
        return raw;
    }
    
    public static double relativeError(double result, double expected) {
        return Math.abs((result - expected) / expected);
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
    
    public double getDistance(WPIPolygon polygon){
        double distance = (Y_RES * 32 / 12.0) / (polygon.getHeight() * 2 * Math.tan(VERT_FOV / 2.0));
        
        return (int)(distance * 100) / 100.0;
    }
}