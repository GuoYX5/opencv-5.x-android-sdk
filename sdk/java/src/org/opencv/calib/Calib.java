//
// This file is auto-generated. Please don't modify it!
//
package org.opencv.calib;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.utils.Converters;

// C++: class Calib

public class Calib {

    // C++: enum <unnamed>
    public static final int
            CALIB_CB_ADAPTIVE_THRESH = 1,
            CALIB_CB_NORMALIZE_IMAGE = 2,
            CALIB_CB_FILTER_QUADS = 4,
            CALIB_CB_FAST_CHECK = 8,
            CALIB_CB_EXHAUSTIVE = 16,
            CALIB_CB_ACCURACY = 32,
            CALIB_CB_LARGER = 64,
            CALIB_CB_MARKER = 128,
            CALIB_CB_SYMMETRIC_GRID = 1,
            CALIB_CB_ASYMMETRIC_GRID = 2,
            CALIB_CB_CLUSTERING = 4,
            CALIB_NINTRINSIC = 18,
            CALIB_USE_INTRINSIC_GUESS = 0x00001,
            CALIB_FIX_ASPECT_RATIO = 0x00002,
            CALIB_FIX_PRINCIPAL_POINT = 0x00004,
            CALIB_ZERO_TANGENT_DIST = 0x00008,
            CALIB_FIX_FOCAL_LENGTH = 0x00010,
            CALIB_FIX_K1 = 0x00020,
            CALIB_FIX_K2 = 0x00040,
            CALIB_FIX_K3 = 0x00080,
            CALIB_FIX_K4 = 0x00800,
            CALIB_FIX_K5 = 0x01000,
            CALIB_FIX_K6 = 0x02000,
            CALIB_RATIONAL_MODEL = 0x04000,
            CALIB_THIN_PRISM_MODEL = 0x08000,
            CALIB_FIX_S1_S2_S3_S4 = 0x10000,
            CALIB_TILTED_MODEL = 0x40000,
            CALIB_FIX_TAUX_TAUY = 0x80000,
            CALIB_USE_QR = 0x100000,
            CALIB_FIX_TANGENT_DIST = 0x200000,
            CALIB_FIX_INTRINSIC = 0x00100,
            CALIB_SAME_FOCAL_LENGTH = 0x00200,
            CALIB_ZERO_DISPARITY = 0x00400,
            CALIB_USE_LU = (1 << 17),
            CALIB_USE_EXTRINSIC_GUESS = (1 << 22),
            fisheye_CALIB_USE_INTRINSIC_GUESS = 1 << 0,
            fisheye_CALIB_RECOMPUTE_EXTRINSIC = 1 << 1,
            fisheye_CALIB_CHECK_COND = 1 << 2,
            fisheye_CALIB_FIX_SKEW = 1 << 3,
            fisheye_CALIB_FIX_K1 = 1 << 4,
            fisheye_CALIB_FIX_K2 = 1 << 5,
            fisheye_CALIB_FIX_K3 = 1 << 6,
            fisheye_CALIB_FIX_K4 = 1 << 7,
            fisheye_CALIB_FIX_INTRINSIC = 1 << 8,
            fisheye_CALIB_FIX_PRINCIPAL_POINT = 1 << 9,
            fisheye_CALIB_ZERO_DISPARITY = 1 << 10,
            fisheye_CALIB_FIX_FOCAL_LENGTH = 1 << 11;


    // C++: enum GridType (cv.CirclesGridFinderParameters.GridType)
    public static final int
            CirclesGridFinderParameters_SYMMETRIC_GRID = 0,
            CirclesGridFinderParameters_ASYMMETRIC_GRID = 1;


    // C++: enum HandEyeCalibrationMethod (cv.HandEyeCalibrationMethod)
    public static final int
            CALIB_HAND_EYE_TSAI = 0,
            CALIB_HAND_EYE_PARK = 1,
            CALIB_HAND_EYE_HORAUD = 2,
            CALIB_HAND_EYE_ANDREFF = 3,
            CALIB_HAND_EYE_DANIILIDIS = 4;


    // C++: enum RobotWorldHandEyeCalibrationMethod (cv.RobotWorldHandEyeCalibrationMethod)
    public static final int
            CALIB_ROBOT_WORLD_HAND_EYE_SHAH = 0,
            CALIB_ROBOT_WORLD_HAND_EYE_LI = 1;


    //
    // C++:  Mat cv::initCameraMatrix2D(vector_vector_Point3f objectPoints, vector_vector_Point2f imagePoints, Size imageSize, double aspectRatio = 1.0)
    //

    /**
     * Finds an initial camera intrinsic matrix from 3D-2D point correspondences.
     *
     * @param objectPoints Vector of vectors of the calibration pattern points in the calibration pattern
     * coordinate space. In the old interface all the per-view vectors are concatenated. See
     * #calibrateCamera for details.
     * @param imagePoints Vector of vectors of the projections of the calibration pattern points. In the
     * old interface all the per-view vectors are concatenated.
     * @param imageSize Image size in pixels used to initialize the principal point.
     * @param aspectRatio If it is zero or negative, both \(f_x\) and \(f_y\) are estimated independently.
     * Otherwise, \(f_x = f_y \cdot \texttt{aspectRatio}\) .
     *
     * The function estimates and returns an initial camera intrinsic matrix for the camera calibration process.
     * Currently, the function only supports planar calibration patterns, which are patterns where each
     * object point has z-coordinate =0.
     * @return automatically generated
     */
    public static Mat initCameraMatrix2D(List<MatOfPoint3f> objectPoints, List<MatOfPoint2f> imagePoints, Size imageSize, double aspectRatio) {
        List<Mat> objectPoints_tmplm = new ArrayList<Mat>((objectPoints != null) ? objectPoints.size() : 0);
        Mat objectPoints_mat = Converters.vector_vector_Point3f_to_Mat(objectPoints, objectPoints_tmplm);
        List<Mat> imagePoints_tmplm = new ArrayList<Mat>((imagePoints != null) ? imagePoints.size() : 0);
        Mat imagePoints_mat = Converters.vector_vector_Point2f_to_Mat(imagePoints, imagePoints_tmplm);
        return new Mat(initCameraMatrix2D_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, aspectRatio));
    }

    /**
     * Finds an initial camera intrinsic matrix from 3D-2D point correspondences.
     *
     * @param objectPoints Vector of vectors of the calibration pattern points in the calibration pattern
     * coordinate space. In the old interface all the per-view vectors are concatenated. See
     * #calibrateCamera for details.
     * @param imagePoints Vector of vectors of the projections of the calibration pattern points. In the
     * old interface all the per-view vectors are concatenated.
     * @param imageSize Image size in pixels used to initialize the principal point.
     * Otherwise, \(f_x = f_y \cdot \texttt{aspectRatio}\) .
     *
     * The function estimates and returns an initial camera intrinsic matrix for the camera calibration process.
     * Currently, the function only supports planar calibration patterns, which are patterns where each
     * object point has z-coordinate =0.
     * @return automatically generated
     */
    public static Mat initCameraMatrix2D(List<MatOfPoint3f> objectPoints, List<MatOfPoint2f> imagePoints, Size imageSize) {
        List<Mat> objectPoints_tmplm = new ArrayList<Mat>((objectPoints != null) ? objectPoints.size() : 0);
        Mat objectPoints_mat = Converters.vector_vector_Point3f_to_Mat(objectPoints, objectPoints_tmplm);
        List<Mat> imagePoints_tmplm = new ArrayList<Mat>((imagePoints != null) ? imagePoints.size() : 0);
        Mat imagePoints_mat = Converters.vector_vector_Point2f_to_Mat(imagePoints, imagePoints_tmplm);
        return new Mat(initCameraMatrix2D_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height));
    }


    //
    // C++:  bool cv::findChessboardCorners(Mat image, Size patternSize, vector_Point2f& corners, int flags = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE)
    //

    /**
     * Finds the positions of internal corners of the chessboard.
     *
     * @param image Source chessboard view. It must be an 8-bit grayscale or color image.
     * @param patternSize Number of inner corners per a chessboard row and column
     * ( patternSize = cv::Size(points_per_row,points_per_colum) = cv::Size(columns,rows) ).
     * @param corners Output array of detected corners.
     * @param flags Various operation flags that can be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: CALIB_CB_ADAPTIVE_THRESH Use adaptive thresholding to convert the image to black
     * and white, rather than a fixed threshold level (computed from the average image brightness).
     *   </li>
     *   <li>
     *    REF: CALIB_CB_NORMALIZE_IMAGE Normalize the image gamma with equalizeHist before
     * applying fixed or adaptive thresholding.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_FILTER_QUADS Use additional criteria (like contour area, perimeter,
     * square-like shape) to filter out false quads extracted at the contour retrieval stage.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_FAST_CHECK Run a fast check on the image that looks for chessboard corners,
     * and shortcut the call if none is found. This can drastically speed up the call in the
     * degenerate condition when no chessboard is observed.
     *   </li>
     * </ul>
     *
     * The function attempts to determine whether the input image is a view of the chessboard pattern and
     * locate the internal chessboard corners. The function returns a non-zero value if all of the corners
     * are found and they are placed in a certain order (row by row, left to right in every row).
     * Otherwise, if the function fails to find all the corners or reorder them, it returns 0. For example,
     * a regular chessboard has 8 x 8 squares and 7 x 7 internal corners, that is, points where the black
     * squares touch each other. The detected coordinates are approximate, and to determine their positions
     * more accurately, the function calls #cornerSubPix. You also may use the function #cornerSubPix with
     * different parameters if returned coordinates are not accurate enough.
     *
     * Sample usage of detecting and drawing chessboard corners: :
     * <code>
     *     Size patternsize(8,6); //interior number of corners
     *     Mat gray = ....; //source image
     *     vector&lt;Point2f&gt; corners; //this will be filled by the detected corners
     *
     *     //CALIB_CB_FAST_CHECK saves a lot of time on images
     *     //that do not contain any chessboard corners
     *     bool patternfound = findChessboardCorners(gray, patternsize, corners,
     *             CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
     *             + CALIB_CB_FAST_CHECK);
     *
     *     if(patternfound)
     *       cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
     *         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
     *
     *     drawChessboardCorners(img, patternsize, Mat(corners), patternfound);
     * </code>
     * <b>Note:</b> The function requires white space (like a square-thick border, the wider the better) around
     * the board to make the detection more robust in various environments. Otherwise, if there is no
     * border and the background is dark, the outer black squares cannot be segmented properly and so the
     * square grouping and ordering algorithm fails.
     *
     * Use gen_pattern.py (REF: tutorial_camera_calibration_pattern) to create checkerboard.
     * @return automatically generated
     */
    public static boolean findChessboardCorners(Mat image, Size patternSize, MatOfPoint2f corners, int flags) {
        Mat corners_mat = corners;
        return findChessboardCorners_0(image.nativeObj, patternSize.width, patternSize.height, corners_mat.nativeObj, flags);
    }

    /**
     * Finds the positions of internal corners of the chessboard.
     *
     * @param image Source chessboard view. It must be an 8-bit grayscale or color image.
     * @param patternSize Number of inner corners per a chessboard row and column
     * ( patternSize = cv::Size(points_per_row,points_per_colum) = cv::Size(columns,rows) ).
     * @param corners Output array of detected corners.
     * <ul>
     *   <li>
     *    REF: CALIB_CB_ADAPTIVE_THRESH Use adaptive thresholding to convert the image to black
     * and white, rather than a fixed threshold level (computed from the average image brightness).
     *   </li>
     *   <li>
     *    REF: CALIB_CB_NORMALIZE_IMAGE Normalize the image gamma with equalizeHist before
     * applying fixed or adaptive thresholding.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_FILTER_QUADS Use additional criteria (like contour area, perimeter,
     * square-like shape) to filter out false quads extracted at the contour retrieval stage.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_FAST_CHECK Run a fast check on the image that looks for chessboard corners,
     * and shortcut the call if none is found. This can drastically speed up the call in the
     * degenerate condition when no chessboard is observed.
     *   </li>
     * </ul>
     *
     * The function attempts to determine whether the input image is a view of the chessboard pattern and
     * locate the internal chessboard corners. The function returns a non-zero value if all of the corners
     * are found and they are placed in a certain order (row by row, left to right in every row).
     * Otherwise, if the function fails to find all the corners or reorder them, it returns 0. For example,
     * a regular chessboard has 8 x 8 squares and 7 x 7 internal corners, that is, points where the black
     * squares touch each other. The detected coordinates are approximate, and to determine their positions
     * more accurately, the function calls #cornerSubPix. You also may use the function #cornerSubPix with
     * different parameters if returned coordinates are not accurate enough.
     *
     * Sample usage of detecting and drawing chessboard corners: :
     * <code>
     *     Size patternsize(8,6); //interior number of corners
     *     Mat gray = ....; //source image
     *     vector&lt;Point2f&gt; corners; //this will be filled by the detected corners
     *
     *     //CALIB_CB_FAST_CHECK saves a lot of time on images
     *     //that do not contain any chessboard corners
     *     bool patternfound = findChessboardCorners(gray, patternsize, corners,
     *             CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
     *             + CALIB_CB_FAST_CHECK);
     *
     *     if(patternfound)
     *       cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
     *         TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
     *
     *     drawChessboardCorners(img, patternsize, Mat(corners), patternfound);
     * </code>
     * <b>Note:</b> The function requires white space (like a square-thick border, the wider the better) around
     * the board to make the detection more robust in various environments. Otherwise, if there is no
     * border and the background is dark, the outer black squares cannot be segmented properly and so the
     * square grouping and ordering algorithm fails.
     *
     * Use gen_pattern.py (REF: tutorial_camera_calibration_pattern) to create checkerboard.
     * @return automatically generated
     */
    public static boolean findChessboardCorners(Mat image, Size patternSize, MatOfPoint2f corners) {
        Mat corners_mat = corners;
        return findChessboardCorners_1(image.nativeObj, patternSize.width, patternSize.height, corners_mat.nativeObj);
    }


    //
    // C++:  bool cv::checkChessboard(Mat img, Size size)
    //

    public static boolean checkChessboard(Mat img, Size size) {
        return checkChessboard_0(img.nativeObj, size.width, size.height);
    }


    //
    // C++:  bool cv::findChessboardCornersSB(Mat image, Size patternSize, Mat& corners, int flags, Mat& meta)
    //

    /**
     * Finds the positions of internal corners of the chessboard using a sector based approach.
     *
     * @param image Source chessboard view. It must be an 8-bit grayscale or color image.
     * @param patternSize Number of inner corners per a chessboard row and column
     * ( patternSize = cv::Size(points_per_row,points_per_colum) = cv::Size(columns,rows) ).
     * @param corners Output array of detected corners.
     * @param flags Various operation flags that can be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: CALIB_CB_NORMALIZE_IMAGE Normalize the image gamma with equalizeHist before detection.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_EXHAUSTIVE Run an exhaustive search to improve detection rate.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_ACCURACY Up sample input image to improve sub-pixel accuracy due to aliasing effects.
     *   </li>
     *   <li>
     *    REF: CALIB_CB_LARGER The detected pattern is allowed to be larger than patternSize (see description).
     *   </li>
     *   <li>
     *    REF: CALIB_CB_MARKER The detected pattern must have a marker (see description).
     * This should be used if an accurate camera calibration is required.
     * @param meta Optional output arrray of detected corners (CV_8UC1 and size = cv::Size(columns,rows)).
     * Each entry stands for one corner of the pattern and can have one of the following values:
     *   </li>
     *   <li>
     *    0 = no meta data attached
     *   </li>
     *   <li>
     *    1 = left-top corner of a black cell
     *   </li>
     *   <li>
     *    2 = left-top corner of a white cell
     *   </li>
     *   <li>
     *    3 = left-top corner of a black cell with a white marker dot
     *   </li>
     *   <li>
     *    4 = left-top corner of a white cell with a black marker dot (pattern origin in case of markers otherwise first corner)
     *   </li>
     * </ul>
     *
     * The function is analog to #findChessboardCorners but uses a localized radon
     * transformation approximated by box filters being more robust to all sort of
     * noise, faster on larger images and is able to directly return the sub-pixel
     * position of the internal chessboard corners. The Method is based on the paper
     * CITE: duda2018 "Accurate Detection and Localization of Checkerboard Corners for
     * Calibration" demonstrating that the returned sub-pixel positions are more
     * accurate than the one returned by cornerSubPix allowing a precise camera
     * calibration for demanding applications.
     *
     * In the case, the flags REF: CALIB_CB_LARGER or REF: CALIB_CB_MARKER are given,
     * the result can be recovered from the optional meta array. Both flags are
     * helpful to use calibration patterns exceeding the field of view of the camera.
     * These oversized patterns allow more accurate calibrations as corners can be
     * utilized, which are as close as possible to the image borders.  For a
     * consistent coordinate system across all images, the optional marker (see image
     * below) can be used to move the origin of the board to the location where the
     * black circle is located.
     *
     * <b>Note:</b> The function requires a white boarder with roughly the same width as one
     * of the checkerboard fields around the whole board to improve the detection in
     * various environments. In addition, because of the localized radon
     * transformation it is beneficial to use round corners for the field corners
     * which are located on the outside of the board. The following figure illustrates
     * a sample checkerboard optimized for the detection. However, any other checkerboard
     * can be used as well.
     *
     * Use gen_pattern.py (REF: tutorial_camera_calibration_pattern) to create checkerboard.
     * ![Checkerboard](pics/checkerboard_radon.png)
     * @return automatically generated
     */
    public static boolean findChessboardCornersSBWithMeta(Mat image, Size patternSize, Mat corners, int flags, Mat meta) {
        return findChessboardCornersSBWithMeta_0(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj, flags, meta.nativeObj);
    }


    //
    // C++:  bool cv::findChessboardCornersSB(Mat image, Size patternSize, Mat& corners, int flags = 0)
    //

    public static boolean findChessboardCornersSB(Mat image, Size patternSize, Mat corners, int flags) {
        return findChessboardCornersSB_0(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj, flags);
    }

    public static boolean findChessboardCornersSB(Mat image, Size patternSize, Mat corners) {
        return findChessboardCornersSB_1(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj);
    }


    //
    // C++:  Scalar cv::estimateChessboardSharpness(Mat image, Size patternSize, Mat corners, float rise_distance = 0.8F, bool vertical = false, Mat& sharpness = Mat())
    //

    /**
     * Estimates the sharpness of a detected chessboard.
     *
     * Image sharpness, as well as brightness, are a critical parameter for accuracte
     * camera calibration. For accessing these parameters for filtering out
     * problematic calibraiton images, this method calculates edge profiles by traveling from
     * black to white chessboard cell centers. Based on this, the number of pixels is
     * calculated required to transit from black to white. This width of the
     * transition area is a good indication of how sharp the chessboard is imaged
     * and should be below ~3.0 pixels.
     *
     * @param image Gray image used to find chessboard corners
     * @param patternSize Size of a found chessboard pattern
     * @param corners Corners found by #findChessboardCornersSB
     * @param rise_distance Rise distance 0.8 means 10% ... 90% of the final signal strength
     * @param vertical By default edge responses for horizontal lines are calculated
     * @param sharpness Optional output array with a sharpness value for calculated edge responses (see description)
     *
     * The optional sharpness array is of type CV_32FC1 and has for each calculated
     * profile one row with the following five entries:
     * 0 = x coordinate of the underlying edge in the image
     * 1 = y coordinate of the underlying edge in the image
     * 2 = width of the transition area (sharpness)
     * 3 = signal strength in the black cell (min brightness)
     * 4 = signal strength in the white cell (max brightness)
     *
     * @return Scalar(average sharpness, average min brightness, average max brightness,0)
     */
    public static Scalar estimateChessboardSharpness(Mat image, Size patternSize, Mat corners, float rise_distance, boolean vertical, Mat sharpness) {
        return new Scalar(estimateChessboardSharpness_0(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj, rise_distance, vertical, sharpness.nativeObj));
    }

    /**
     * Estimates the sharpness of a detected chessboard.
     *
     * Image sharpness, as well as brightness, are a critical parameter for accuracte
     * camera calibration. For accessing these parameters for filtering out
     * problematic calibraiton images, this method calculates edge profiles by traveling from
     * black to white chessboard cell centers. Based on this, the number of pixels is
     * calculated required to transit from black to white. This width of the
     * transition area is a good indication of how sharp the chessboard is imaged
     * and should be below ~3.0 pixels.
     *
     * @param image Gray image used to find chessboard corners
     * @param patternSize Size of a found chessboard pattern
     * @param corners Corners found by #findChessboardCornersSB
     * @param rise_distance Rise distance 0.8 means 10% ... 90% of the final signal strength
     * @param vertical By default edge responses for horizontal lines are calculated
     *
     * The optional sharpness array is of type CV_32FC1 and has for each calculated
     * profile one row with the following five entries:
     * 0 = x coordinate of the underlying edge in the image
     * 1 = y coordinate of the underlying edge in the image
     * 2 = width of the transition area (sharpness)
     * 3 = signal strength in the black cell (min brightness)
     * 4 = signal strength in the white cell (max brightness)
     *
     * @return Scalar(average sharpness, average min brightness, average max brightness,0)
     */
    public static Scalar estimateChessboardSharpness(Mat image, Size patternSize, Mat corners, float rise_distance, boolean vertical) {
        return new Scalar(estimateChessboardSharpness_1(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj, rise_distance, vertical));
    }

    /**
     * Estimates the sharpness of a detected chessboard.
     *
     * Image sharpness, as well as brightness, are a critical parameter for accuracte
     * camera calibration. For accessing these parameters for filtering out
     * problematic calibraiton images, this method calculates edge profiles by traveling from
     * black to white chessboard cell centers. Based on this, the number of pixels is
     * calculated required to transit from black to white. This width of the
     * transition area is a good indication of how sharp the chessboard is imaged
     * and should be below ~3.0 pixels.
     *
     * @param image Gray image used to find chessboard corners
     * @param patternSize Size of a found chessboard pattern
     * @param corners Corners found by #findChessboardCornersSB
     * @param rise_distance Rise distance 0.8 means 10% ... 90% of the final signal strength
     *
     * The optional sharpness array is of type CV_32FC1 and has for each calculated
     * profile one row with the following five entries:
     * 0 = x coordinate of the underlying edge in the image
     * 1 = y coordinate of the underlying edge in the image
     * 2 = width of the transition area (sharpness)
     * 3 = signal strength in the black cell (min brightness)
     * 4 = signal strength in the white cell (max brightness)
     *
     * @return Scalar(average sharpness, average min brightness, average max brightness,0)
     */
    public static Scalar estimateChessboardSharpness(Mat image, Size patternSize, Mat corners, float rise_distance) {
        return new Scalar(estimateChessboardSharpness_2(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj, rise_distance));
    }

    /**
     * Estimates the sharpness of a detected chessboard.
     *
     * Image sharpness, as well as brightness, are a critical parameter for accuracte
     * camera calibration. For accessing these parameters for filtering out
     * problematic calibraiton images, this method calculates edge profiles by traveling from
     * black to white chessboard cell centers. Based on this, the number of pixels is
     * calculated required to transit from black to white. This width of the
     * transition area is a good indication of how sharp the chessboard is imaged
     * and should be below ~3.0 pixels.
     *
     * @param image Gray image used to find chessboard corners
     * @param patternSize Size of a found chessboard pattern
     * @param corners Corners found by #findChessboardCornersSB
     *
     * The optional sharpness array is of type CV_32FC1 and has for each calculated
     * profile one row with the following five entries:
     * 0 = x coordinate of the underlying edge in the image
     * 1 = y coordinate of the underlying edge in the image
     * 2 = width of the transition area (sharpness)
     * 3 = signal strength in the black cell (min brightness)
     * 4 = signal strength in the white cell (max brightness)
     *
     * @return Scalar(average sharpness, average min brightness, average max brightness,0)
     */
    public static Scalar estimateChessboardSharpness(Mat image, Size patternSize, Mat corners) {
        return new Scalar(estimateChessboardSharpness_3(image.nativeObj, patternSize.width, patternSize.height, corners.nativeObj));
    }


    //
    // C++:  bool cv::find4QuadCornerSubpix(Mat img, Mat& corners, Size region_size)
    //

    public static boolean find4QuadCornerSubpix(Mat img, Mat corners, Size region_size) {
        return find4QuadCornerSubpix_0(img.nativeObj, corners.nativeObj, region_size.width, region_size.height);
    }


    //
    // C++:  void cv::drawChessboardCorners(Mat& image, Size patternSize, vector_Point2f corners, bool patternWasFound)
    //

    /**
     * Renders the detected chessboard corners.
     *
     * @param image Destination image. It must be an 8-bit color image.
     * @param patternSize Number of inner corners per a chessboard row and column
     * (patternSize = cv::Size(points_per_row,points_per_column)).
     * @param corners Array of detected corners, the output of #findChessboardCorners.
     * @param patternWasFound Parameter indicating whether the complete board was found or not. The
     * return value of #findChessboardCorners should be passed here.
     *
     * The function draws individual chessboard corners detected either as red circles if the board was not
     * found, or as colored corners connected with lines if the board was found.
     */
    public static void drawChessboardCorners(Mat image, Size patternSize, MatOfPoint2f corners, boolean patternWasFound) {
        Mat corners_mat = corners;
        drawChessboardCorners_0(image.nativeObj, patternSize.width, patternSize.height, corners_mat.nativeObj, patternWasFound);
    }


    //
    // C++:  bool cv::findCirclesGrid(Mat image, Size patternSize, Mat& centers, int flags, Ptr_FeatureDetector blobDetector, CirclesGridFinderParameters parameters)
    //

    // Unknown type 'Ptr_FeatureDetector' (I), skipping the function


    //
    // C++:  bool cv::findCirclesGrid(Mat image, Size patternSize, Mat& centers, int flags = CALIB_CB_SYMMETRIC_GRID, Ptr_FeatureDetector blobDetector = SimpleBlobDetector::create())
    //

    public static boolean findCirclesGrid(Mat image, Size patternSize, Mat centers, int flags) {
        return findCirclesGrid_0(image.nativeObj, patternSize.width, patternSize.height, centers.nativeObj, flags);
    }

    public static boolean findCirclesGrid(Mat image, Size patternSize, Mat centers) {
        return findCirclesGrid_2(image.nativeObj, patternSize.width, patternSize.height, centers.nativeObj);
    }


    //
    // C++:  double cv::calibrateCamera(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    /**
     * Finds the camera intrinsic and extrinsic parameters from several views of a calibration
     * pattern.
     *
     * @param objectPoints In the new interface it is a vector of vectors of calibration pattern points in
     * the calibration pattern coordinate space (e.g. std::vector&lt;std::vector&lt;cv::Vec3f&gt;&gt;). The outer
     * vector contains as many elements as the number of pattern views. If the same calibration pattern
     * is shown in each view and it is fully visible, all the vectors will be the same. Although, it is
     * possible to use partially occluded patterns or even different patterns in different views. Then,
     * the vectors will be different. Although the points are 3D, they all lie in the calibration pattern's
     * XY coordinate plane (thus 0 in the Z-coordinate), if the used calibration pattern is a planar rig.
     * In the old interface all the vectors of object points from different views are concatenated
     * together.
     * @param imagePoints In the new interface it is a vector of vectors of the projections of calibration
     * pattern points (e.g. std::vector&lt;std::vector&lt;cv::Vec2f&gt;&gt;). imagePoints.size() and
     * objectPoints.size(), and imagePoints[i].size() and objectPoints[i].size() for each i, must be equal,
     * respectively. In the old interface all the vectors of object points from different views are
     * concatenated together.
     * @param imageSize Size of the image used only to initialize the camera intrinsic matrix.
     * @param cameraMatrix Input/output 3x3 floating-point camera intrinsic matrix
     * \(\cameramatrix{A}\) . If REF: CALIB_USE_INTRINSIC_GUESS
     * and/or REF: CALIB_FIX_ASPECT_RATIO, REF: CALIB_FIX_PRINCIPAL_POINT or REF: CALIB_FIX_FOCAL_LENGTH
     * are specified, some or all of fx, fy, cx, cy must be initialized before calling the function.
     * @param distCoeffs Input/output vector of distortion coefficients
     * \(\distcoeffs\).
     * @param rvecs Output vector of rotation vectors (REF: Rodrigues ) estimated for each pattern view
     * (e.g. std::vector&lt;cv::Mat&gt;&gt;). That is, each i-th rotation vector together with the corresponding
     * i-th translation vector (see the next output parameter description) brings the calibration pattern
     * from the object coordinate space (in which object points are specified) to the camera coordinate
     * space. In more technical terms, the tuple of the i-th rotation and translation vector performs
     * a change of basis from object coordinate space to camera coordinate space. Due to its duality, this
     * tuple is equivalent to the position of the calibration pattern with respect to the camera coordinate
     * space.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter
     * describtion above.
     * @param stdDeviationsIntrinsics Output vector of standard deviations estimated for intrinsic
     * parameters. Order of deviations values:
     * \((f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6 , s_1, s_2, s_3,
     *  s_4, \tau_x, \tau_y)\) If one of parameters is not estimated, it's deviation is equals to zero.
     * @param stdDeviationsExtrinsics Output vector of standard deviations estimated for extrinsic
     * parameters. Order of deviations values: \((R_0, T_0, \dotsc , R_{M - 1}, T_{M - 1})\) where M is
     * the number of pattern views. \(R_i, T_i\) are concatenated 1x3 vectors.
     *  @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: CALIB_USE_INTRINSIC_GUESS cameraMatrix contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center ( imageSize is used), and focal distances are computed in a least-squares fashion.
     * Note, that if intrinsic parameters are known, there is no need to use this function just to
     * estimate extrinsic parameters. Use REF: solvePnP instead.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_PRINCIPAL_POINT The principal point is not changed during the global
     * optimization. It stays at the center or at a different location specified when
     *  REF: CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_ASPECT_RATIO The functions consider only fy as a free parameter. The
     * ratio fx/fy stays the same as in the input cameraMatrix . When
     *  REF: CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are
     * ignored, only their ratio is computed and used further.
     *   </li>
     *   <li>
     *    REF: CALIB_ZERO_TANGENT_DIST Tangential distortion coefficients \((p_1, p_2)\) are set
     * to zeros and stay zero.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_FOCAL_LENGTH The focal length is not changed during the global optimization if
     *  REF: CALIB_USE_INTRINSIC_GUESS is set.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_K1,..., REF: CALIB_FIX_K6 The corresponding radial distortion
     * coefficient is not changed during the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is
     * set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_RATIONAL_MODEL Coefficients k4, k5, and k6 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the rational model and return 8 coefficients or more.
     *   </li>
     *   <li>
     *    REF: CALIB_THIN_PRISM_MODEL Coefficients s1, s2, s3 and s4 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the thin prism model and return 12 coefficients or more.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_S1_S2_S3_S4 The thin prism distortion coefficients are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_TILTED_MODEL Coefficients tauX and tauY are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the tilted sensor model and return 14 coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_TAUX_TAUY The coefficients of the tilted sensor model are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     * @param criteria Termination criteria for the iterative optimization algorithm.
     *   </li>
     * </ul>
     *
     * @return the overall RMS re-projection error.
     *
     * The function estimates the intrinsic camera parameters and extrinsic parameters for each of the
     * views. The algorithm is based on CITE: Zhang2000 and CITE: BouguetMCT . The coordinates of 3D object
     * points and their corresponding 2D projections in each view must be specified. That may be achieved
     * by using an object with known geometry and easily detectable feature points. Such an object is
     * called a calibration rig or calibration pattern, and OpenCV has built-in support for a chessboard as
     * a calibration rig (see REF: findChessboardCorners). Currently, initialization of intrinsic
     * parameters (when REF: CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for planar calibration
     * patterns (where Z-coordinates of the object points must be all zeros). 3D calibration rigs can also
     * be used as long as initial cameraMatrix is provided.
     *
     * The algorithm performs the following steps:
     *
     * <ul>
     *   <li>
     *    Compute the initial intrinsic parameters (the option only available for planar calibration
     *     patterns) or read them from the input parameters. The distortion coefficients are all set to
     *     zeros initially unless some of CALIB_FIX_K? are specified.
     *   </li>
     * </ul>
     *
     * <ul>
     *   <li>
     *    Estimate the initial camera pose as if the intrinsic parameters have been already known. This is
     *     done using REF: solvePnP .
     *   </li>
     * </ul>
     *
     * <ul>
     *   <li>
     *    Run the global Levenberg-Marquardt optimization algorithm to minimize the reprojection error,
     *     that is, the total sum of squared distances between the observed feature points imagePoints and
     *     the projected (using the current estimates for camera parameters and the poses) object points
     *     objectPoints. See REF: projectPoints for details.
     *   </li>
     * </ul>
     *
     * <b>Note:</b>
     *     If you use a non-square (i.e. non-N-by-N) grid and REF: findChessboardCorners for calibration,
     *     and REF: calibrateCamera returns bad values (zero distortion coefficients, \(c_x\) and
     *     \(c_y\) very far from the image center, and/or large differences between \(f_x\) and
     *     \(f_y\) (ratios of 10:1 or more)), then you are probably using patternSize=cvSize(rows,cols)
     *     instead of using patternSize=cvSize(cols,rows) in REF: findChessboardCorners.
     *
     * SEE:
     *    calibrateCameraRO, findChessboardCorners, solvePnP, initCameraMatrix2D, stereoCalibrate,
     *    undistort
     */
    public static double calibrateCameraExtended(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraExtended_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds the camera intrinsic and extrinsic parameters from several views of a calibration
     * pattern.
     *
     * @param objectPoints In the new interface it is a vector of vectors of calibration pattern points in
     * the calibration pattern coordinate space (e.g. std::vector&lt;std::vector&lt;cv::Vec3f&gt;&gt;). The outer
     * vector contains as many elements as the number of pattern views. If the same calibration pattern
     * is shown in each view and it is fully visible, all the vectors will be the same. Although, it is
     * possible to use partially occluded patterns or even different patterns in different views. Then,
     * the vectors will be different. Although the points are 3D, they all lie in the calibration pattern's
     * XY coordinate plane (thus 0 in the Z-coordinate), if the used calibration pattern is a planar rig.
     * In the old interface all the vectors of object points from different views are concatenated
     * together.
     * @param imagePoints In the new interface it is a vector of vectors of the projections of calibration
     * pattern points (e.g. std::vector&lt;std::vector&lt;cv::Vec2f&gt;&gt;). imagePoints.size() and
     * objectPoints.size(), and imagePoints[i].size() and objectPoints[i].size() for each i, must be equal,
     * respectively. In the old interface all the vectors of object points from different views are
     * concatenated together.
     * @param imageSize Size of the image used only to initialize the camera intrinsic matrix.
     * @param cameraMatrix Input/output 3x3 floating-point camera intrinsic matrix
     * \(\cameramatrix{A}\) . If REF: CALIB_USE_INTRINSIC_GUESS
     * and/or REF: CALIB_FIX_ASPECT_RATIO, REF: CALIB_FIX_PRINCIPAL_POINT or REF: CALIB_FIX_FOCAL_LENGTH
     * are specified, some or all of fx, fy, cx, cy must be initialized before calling the function.
     * @param distCoeffs Input/output vector of distortion coefficients
     * \(\distcoeffs\).
     * @param rvecs Output vector of rotation vectors (REF: Rodrigues ) estimated for each pattern view
     * (e.g. std::vector&lt;cv::Mat&gt;&gt;). That is, each i-th rotation vector together with the corresponding
     * i-th translation vector (see the next output parameter description) brings the calibration pattern
     * from the object coordinate space (in which object points are specified) to the camera coordinate
     * space. In more technical terms, the tuple of the i-th rotation and translation vector performs
     * a change of basis from object coordinate space to camera coordinate space. Due to its duality, this
     * tuple is equivalent to the position of the calibration pattern with respect to the camera coordinate
     * space.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter
     * describtion above.
     * @param stdDeviationsIntrinsics Output vector of standard deviations estimated for intrinsic
     * parameters. Order of deviations values:
     * \((f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6 , s_1, s_2, s_3,
     *  s_4, \tau_x, \tau_y)\) If one of parameters is not estimated, it's deviation is equals to zero.
     * @param stdDeviationsExtrinsics Output vector of standard deviations estimated for extrinsic
     * parameters. Order of deviations values: \((R_0, T_0, \dotsc , R_{M - 1}, T_{M - 1})\) where M is
     * the number of pattern views. \(R_i, T_i\) are concatenated 1x3 vectors.
     *  @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: CALIB_USE_INTRINSIC_GUESS cameraMatrix contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center ( imageSize is used), and focal distances are computed in a least-squares fashion.
     * Note, that if intrinsic parameters are known, there is no need to use this function just to
     * estimate extrinsic parameters. Use REF: solvePnP instead.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_PRINCIPAL_POINT The principal point is not changed during the global
     * optimization. It stays at the center or at a different location specified when
     *  REF: CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_ASPECT_RATIO The functions consider only fy as a free parameter. The
     * ratio fx/fy stays the same as in the input cameraMatrix . When
     *  REF: CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are
     * ignored, only their ratio is computed and used further.
     *   </li>
     *   <li>
     *    REF: CALIB_ZERO_TANGENT_DIST Tangential distortion coefficients \((p_1, p_2)\) are set
     * to zeros and stay zero.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_FOCAL_LENGTH The focal length is not changed during the global optimization if
     *  REF: CALIB_USE_INTRINSIC_GUESS is set.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_K1,..., REF: CALIB_FIX_K6 The corresponding radial distortion
     * coefficient is not changed during the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is
     * set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_RATIONAL_MODEL Coefficients k4, k5, and k6 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the rational model and return 8 coefficients or more.
     *   </li>
     *   <li>
     *    REF: CALIB_THIN_PRISM_MODEL Coefficients s1, s2, s3 and s4 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the thin prism model and return 12 coefficients or more.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_S1_S2_S3_S4 The thin prism distortion coefficients are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_TILTED_MODEL Coefficients tauX and tauY are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the tilted sensor model and return 14 coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_TAUX_TAUY The coefficients of the tilted sensor model are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     * </ul>
     *
     * @return the overall RMS re-projection error.
     *
     * The function estimates the intrinsic camera parameters and extrinsic parameters for each of the
     * views. The algorithm is based on CITE: Zhang2000 and CITE: BouguetMCT . The coordinates of 3D object
     * points and their corresponding 2D projections in each view must be specified. That may be achieved
     * by using an object with known geometry and easily detectable feature points. Such an object is
     * called a calibration rig or calibration pattern, and OpenCV has built-in support for a chessboard as
     * a calibration rig (see REF: findChessboardCorners). Currently, initialization of intrinsic
     * parameters (when REF: CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for planar calibration
     * patterns (where Z-coordinates of the object points must be all zeros). 3D calibration rigs can also
     * be used as long as initial cameraMatrix is provided.
     *
     * The algorithm performs the following steps:
     *
     * <ul>
     *   <li>
     *    Compute the initial intrinsic parameters (the option only available for planar calibration
     *     patterns) or read them from the input parameters. The distortion coefficients are all set to
     *     zeros initially unless some of CALIB_FIX_K? are specified.
     *   </li>
     * </ul>
     *
     * <ul>
     *   <li>
     *    Estimate the initial camera pose as if the intrinsic parameters have been already known. This is
     *     done using REF: solvePnP .
     *   </li>
     * </ul>
     *
     * <ul>
     *   <li>
     *    Run the global Levenberg-Marquardt optimization algorithm to minimize the reprojection error,
     *     that is, the total sum of squared distances between the observed feature points imagePoints and
     *     the projected (using the current estimates for camera parameters and the poses) object points
     *     objectPoints. See REF: projectPoints for details.
     *   </li>
     * </ul>
     *
     * <b>Note:</b>
     *     If you use a non-square (i.e. non-N-by-N) grid and REF: findChessboardCorners for calibration,
     *     and REF: calibrateCamera returns bad values (zero distortion coefficients, \(c_x\) and
     *     \(c_y\) very far from the image center, and/or large differences between \(f_x\) and
     *     \(f_y\) (ratios of 10:1 or more)), then you are probably using patternSize=cvSize(rows,cols)
     *     instead of using patternSize=cvSize(cols,rows) in REF: findChessboardCorners.
     *
     * SEE:
     *    calibrateCameraRO, findChessboardCorners, solvePnP, initCameraMatrix2D, stereoCalibrate,
     *    undistort
     */
    public static double calibrateCameraExtended(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraExtended_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds the camera intrinsic and extrinsic parameters from several views of a calibration
     * pattern.
     *
     * @param objectPoints In the new interface it is a vector of vectors of calibration pattern points in
     * the calibration pattern coordinate space (e.g. std::vector&lt;std::vector&lt;cv::Vec3f&gt;&gt;). The outer
     * vector contains as many elements as the number of pattern views. If the same calibration pattern
     * is shown in each view and it is fully visible, all the vectors will be the same. Although, it is
     * possible to use partially occluded patterns or even different patterns in different views. Then,
     * the vectors will be different. Although the points are 3D, they all lie in the calibration pattern's
     * XY coordinate plane (thus 0 in the Z-coordinate), if the used calibration pattern is a planar rig.
     * In the old interface all the vectors of object points from different views are concatenated
     * together.
     * @param imagePoints In the new interface it is a vector of vectors of the projections of calibration
     * pattern points (e.g. std::vector&lt;std::vector&lt;cv::Vec2f&gt;&gt;). imagePoints.size() and
     * objectPoints.size(), and imagePoints[i].size() and objectPoints[i].size() for each i, must be equal,
     * respectively. In the old interface all the vectors of object points from different views are
     * concatenated together.
     * @param imageSize Size of the image used only to initialize the camera intrinsic matrix.
     * @param cameraMatrix Input/output 3x3 floating-point camera intrinsic matrix
     * \(\cameramatrix{A}\) . If REF: CALIB_USE_INTRINSIC_GUESS
     * and/or REF: CALIB_FIX_ASPECT_RATIO, REF: CALIB_FIX_PRINCIPAL_POINT or REF: CALIB_FIX_FOCAL_LENGTH
     * are specified, some or all of fx, fy, cx, cy must be initialized before calling the function.
     * @param distCoeffs Input/output vector of distortion coefficients
     * \(\distcoeffs\).
     * @param rvecs Output vector of rotation vectors (REF: Rodrigues ) estimated for each pattern view
     * (e.g. std::vector&lt;cv::Mat&gt;&gt;). That is, each i-th rotation vector together with the corresponding
     * i-th translation vector (see the next output parameter description) brings the calibration pattern
     * from the object coordinate space (in which object points are specified) to the camera coordinate
     * space. In more technical terms, the tuple of the i-th rotation and translation vector performs
     * a change of basis from object coordinate space to camera coordinate space. Due to its duality, this
     * tuple is equivalent to the position of the calibration pattern with respect to the camera coordinate
     * space.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter
     * describtion above.
     * @param stdDeviationsIntrinsics Output vector of standard deviations estimated for intrinsic
     * parameters. Order of deviations values:
     * \((f_x, f_y, c_x, c_y, k_1, k_2, p_1, p_2, k_3, k_4, k_5, k_6 , s_1, s_2, s_3,
     *  s_4, \tau_x, \tau_y)\) If one of parameters is not estimated, it's deviation is equals to zero.
     * @param stdDeviationsExtrinsics Output vector of standard deviations estimated for extrinsic
     * parameters. Order of deviations values: \((R_0, T_0, \dotsc , R_{M - 1}, T_{M - 1})\) where M is
     * the number of pattern views. \(R_i, T_i\) are concatenated 1x3 vectors.
     *  @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * <ul>
     *   <li>
     *    REF: CALIB_USE_INTRINSIC_GUESS cameraMatrix contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center ( imageSize is used), and focal distances are computed in a least-squares fashion.
     * Note, that if intrinsic parameters are known, there is no need to use this function just to
     * estimate extrinsic parameters. Use REF: solvePnP instead.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_PRINCIPAL_POINT The principal point is not changed during the global
     * optimization. It stays at the center or at a different location specified when
     *  REF: CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_ASPECT_RATIO The functions consider only fy as a free parameter. The
     * ratio fx/fy stays the same as in the input cameraMatrix . When
     *  REF: CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are
     * ignored, only their ratio is computed and used further.
     *   </li>
     *   <li>
     *    REF: CALIB_ZERO_TANGENT_DIST Tangential distortion coefficients \((p_1, p_2)\) are set
     * to zeros and stay zero.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_FOCAL_LENGTH The focal length is not changed during the global optimization if
     *  REF: CALIB_USE_INTRINSIC_GUESS is set.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_K1,..., REF: CALIB_FIX_K6 The corresponding radial distortion
     * coefficient is not changed during the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is
     * set, the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_RATIONAL_MODEL Coefficients k4, k5, and k6 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the rational model and return 8 coefficients or more.
     *   </li>
     *   <li>
     *    REF: CALIB_THIN_PRISM_MODEL Coefficients s1, s2, s3 and s4 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the thin prism model and return 12 coefficients or more.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_S1_S2_S3_S4 The thin prism distortion coefficients are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_TILTED_MODEL Coefficients tauX and tauY are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the tilted sensor model and return 14 coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_TAUX_TAUY The coefficients of the tilted sensor model are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     * </ul>
     *
     * @return the overall RMS re-projection error.
     *
     * The function estimates the intrinsic camera parameters and extrinsic parameters for each of the
     * views. The algorithm is based on CITE: Zhang2000 and CITE: BouguetMCT . The coordinates of 3D object
     * points and their corresponding 2D projections in each view must be specified. That may be achieved
     * by using an object with known geometry and easily detectable feature points. Such an object is
     * called a calibration rig or calibration pattern, and OpenCV has built-in support for a chessboard as
     * a calibration rig (see REF: findChessboardCorners). Currently, initialization of intrinsic
     * parameters (when REF: CALIB_USE_INTRINSIC_GUESS is not set) is only implemented for planar calibration
     * patterns (where Z-coordinates of the object points must be all zeros). 3D calibration rigs can also
     * be used as long as initial cameraMatrix is provided.
     *
     * The algorithm performs the following steps:
     *
     * <ul>
     *   <li>
     *    Compute the initial intrinsic parameters (the option only available for planar calibration
     *     patterns) or read them from the input parameters. The distortion coefficients are all set to
     *     zeros initially unless some of CALIB_FIX_K? are specified.
     *   </li>
     * </ul>
     *
     * <ul>
     *   <li>
     *    Estimate the initial camera pose as if the intrinsic parameters have been already known. This is
     *     done using REF: solvePnP .
     *   </li>
     * </ul>
     *
     * <ul>
     *   <li>
     *    Run the global Levenberg-Marquardt optimization algorithm to minimize the reprojection error,
     *     that is, the total sum of squared distances between the observed feature points imagePoints and
     *     the projected (using the current estimates for camera parameters and the poses) object points
     *     objectPoints. See REF: projectPoints for details.
     *   </li>
     * </ul>
     *
     * <b>Note:</b>
     *     If you use a non-square (i.e. non-N-by-N) grid and REF: findChessboardCorners for calibration,
     *     and REF: calibrateCamera returns bad values (zero distortion coefficients, \(c_x\) and
     *     \(c_y\) very far from the image center, and/or large differences between \(f_x\) and
     *     \(f_y\) (ratios of 10:1 or more)), then you are probably using patternSize=cvSize(rows,cols)
     *     instead of using patternSize=cvSize(cols,rows) in REF: findChessboardCorners.
     *
     * SEE:
     *    calibrateCameraRO, findChessboardCorners, solvePnP, initCameraMatrix2D, stereoCalibrate,
     *    undistort
     */
    public static double calibrateCameraExtended(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraExtended_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double cv::calibrateCamera(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    public static double calibrateCamera(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCamera_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCamera(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCamera_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCamera(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCamera_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double cv::calibrateCameraRO(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, int iFixedPoint, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& newObjPoints, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& stdDeviationsObjPoints, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    /**
     * Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern.
     *
     * This function is an extension of #calibrateCamera with the method of releasing object which was
     * proposed in CITE: strobl2011iccv. In many common cases with inaccurate, unmeasured, roughly planar
     * targets (calibration plates), this method can dramatically improve the precision of the estimated
     * camera parameters. Both the object-releasing method and standard method are supported by this
     * function. Use the parameter <b>iFixedPoint</b> for method selection. In the internal implementation,
     * #calibrateCamera is a wrapper for this function.
     *
     * @param objectPoints Vector of vectors of calibration pattern points in the calibration pattern
     * coordinate space. See #calibrateCamera for details. If the method of releasing object to be used,
     * the identical calibration board must be used in each view and it must be fully visible, and all
     * objectPoints[i] must be the same and all points should be roughly close to a plane. <b>The calibration
     * target has to be rigid, or at least static if the camera (rather than the calibration target) is
     * shifted for grabbing images.</b>
     * @param imagePoints Vector of vectors of the projections of calibration pattern points. See
     * #calibrateCamera for details.
     * @param imageSize Size of the image used only to initialize the intrinsic camera matrix.
     * @param iFixedPoint The index of the 3D object point in objectPoints[0] to be fixed. It also acts as
     * a switch for calibration method selection. If object-releasing method to be used, pass in the
     * parameter in the range of [1, objectPoints[0].size()-2], otherwise a value out of this range will
     * make standard calibration method selected. Usually the top-right corner point of the calibration
     * board grid is recommended to be fixed when object-releasing method being utilized. According to
     * \cite strobl2011iccv, two other points are also fixed. In this implementation, objectPoints[0].front
     * and objectPoints[0].back.z are used. With object-releasing method, accurate rvecs, tvecs and
     * newObjPoints are only possible if coordinates of these three fixed points are accurate enough.
     * @param cameraMatrix Output 3x3 floating-point camera matrix. See #calibrateCamera for details.
     * @param distCoeffs Output vector of distortion coefficients. See #calibrateCamera for details.
     * @param rvecs Output vector of rotation vectors estimated for each pattern view. See #calibrateCamera
     * for details.
     * @param tvecs Output vector of translation vectors estimated for each pattern view.
     * @param newObjPoints The updated output vector of calibration pattern points. The coordinates might
     * be scaled based on three fixed points. The returned coordinates are accurate only if the above
     * mentioned three fixed points are accurate. If not needed, noArray() can be passed in. This parameter
     * is ignored with standard calibration method.
     * @param stdDeviationsIntrinsics Output vector of standard deviations estimated for intrinsic parameters.
     * See #calibrateCamera for details.
     * @param stdDeviationsExtrinsics Output vector of standard deviations estimated for extrinsic parameters.
     * See #calibrateCamera for details.
     * @param stdDeviationsObjPoints Output vector of standard deviations estimated for refined coordinates
     * of calibration pattern points. It has the same size and order as objectPoints[0] vector. This
     * parameter is ignored with standard calibration method.
     *  @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of some predefined values. See
     * #calibrateCamera for details. If the method of releasing object is used, the calibration time may
     * be much longer. CALIB_USE_QR or CALIB_USE_LU could be used for faster calibration with potentially
     * less precise and less stable in some rare cases.
     * @param criteria Termination criteria for the iterative optimization algorithm.
     *
     * @return the overall RMS re-projection error.
     *
     * The function estimates the intrinsic camera parameters and extrinsic parameters for each of the
     * views. The algorithm is based on CITE: Zhang2000, CITE: BouguetMCT and CITE: strobl2011iccv. See
     * #calibrateCamera for other detailed explanations.
     * SEE:
     *    calibrateCamera, findChessboardCorners, solvePnP, initCameraMatrix2D, stereoCalibrate, undistort
     */
    public static double calibrateCameraROExtended(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, int iFixedPoint, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat newObjPoints, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat stdDeviationsObjPoints, Mat perViewErrors, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraROExtended_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, iFixedPoint, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, newObjPoints.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, stdDeviationsObjPoints.nativeObj, perViewErrors.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern.
     *
     * This function is an extension of #calibrateCamera with the method of releasing object which was
     * proposed in CITE: strobl2011iccv. In many common cases with inaccurate, unmeasured, roughly planar
     * targets (calibration plates), this method can dramatically improve the precision of the estimated
     * camera parameters. Both the object-releasing method and standard method are supported by this
     * function. Use the parameter <b>iFixedPoint</b> for method selection. In the internal implementation,
     * #calibrateCamera is a wrapper for this function.
     *
     * @param objectPoints Vector of vectors of calibration pattern points in the calibration pattern
     * coordinate space. See #calibrateCamera for details. If the method of releasing object to be used,
     * the identical calibration board must be used in each view and it must be fully visible, and all
     * objectPoints[i] must be the same and all points should be roughly close to a plane. <b>The calibration
     * target has to be rigid, or at least static if the camera (rather than the calibration target) is
     * shifted for grabbing images.</b>
     * @param imagePoints Vector of vectors of the projections of calibration pattern points. See
     * #calibrateCamera for details.
     * @param imageSize Size of the image used only to initialize the intrinsic camera matrix.
     * @param iFixedPoint The index of the 3D object point in objectPoints[0] to be fixed. It also acts as
     * a switch for calibration method selection. If object-releasing method to be used, pass in the
     * parameter in the range of [1, objectPoints[0].size()-2], otherwise a value out of this range will
     * make standard calibration method selected. Usually the top-right corner point of the calibration
     * board grid is recommended to be fixed when object-releasing method being utilized. According to
     * \cite strobl2011iccv, two other points are also fixed. In this implementation, objectPoints[0].front
     * and objectPoints[0].back.z are used. With object-releasing method, accurate rvecs, tvecs and
     * newObjPoints are only possible if coordinates of these three fixed points are accurate enough.
     * @param cameraMatrix Output 3x3 floating-point camera matrix. See #calibrateCamera for details.
     * @param distCoeffs Output vector of distortion coefficients. See #calibrateCamera for details.
     * @param rvecs Output vector of rotation vectors estimated for each pattern view. See #calibrateCamera
     * for details.
     * @param tvecs Output vector of translation vectors estimated for each pattern view.
     * @param newObjPoints The updated output vector of calibration pattern points. The coordinates might
     * be scaled based on three fixed points. The returned coordinates are accurate only if the above
     * mentioned three fixed points are accurate. If not needed, noArray() can be passed in. This parameter
     * is ignored with standard calibration method.
     * @param stdDeviationsIntrinsics Output vector of standard deviations estimated for intrinsic parameters.
     * See #calibrateCamera for details.
     * @param stdDeviationsExtrinsics Output vector of standard deviations estimated for extrinsic parameters.
     * See #calibrateCamera for details.
     * @param stdDeviationsObjPoints Output vector of standard deviations estimated for refined coordinates
     * of calibration pattern points. It has the same size and order as objectPoints[0] vector. This
     * parameter is ignored with standard calibration method.
     *  @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of some predefined values. See
     * #calibrateCamera for details. If the method of releasing object is used, the calibration time may
     * be much longer. CALIB_USE_QR or CALIB_USE_LU could be used for faster calibration with potentially
     * less precise and less stable in some rare cases.
     *
     * @return the overall RMS re-projection error.
     *
     * The function estimates the intrinsic camera parameters and extrinsic parameters for each of the
     * views. The algorithm is based on CITE: Zhang2000, CITE: BouguetMCT and CITE: strobl2011iccv. See
     * #calibrateCamera for other detailed explanations.
     * SEE:
     *    calibrateCamera, findChessboardCorners, solvePnP, initCameraMatrix2D, stereoCalibrate, undistort
     */
    public static double calibrateCameraROExtended(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, int iFixedPoint, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat newObjPoints, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat stdDeviationsObjPoints, Mat perViewErrors, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraROExtended_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, iFixedPoint, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, newObjPoints.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, stdDeviationsObjPoints.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds the camera intrinsic and extrinsic parameters from several views of a calibration pattern.
     *
     * This function is an extension of #calibrateCamera with the method of releasing object which was
     * proposed in CITE: strobl2011iccv. In many common cases with inaccurate, unmeasured, roughly planar
     * targets (calibration plates), this method can dramatically improve the precision of the estimated
     * camera parameters. Both the object-releasing method and standard method are supported by this
     * function. Use the parameter <b>iFixedPoint</b> for method selection. In the internal implementation,
     * #calibrateCamera is a wrapper for this function.
     *
     * @param objectPoints Vector of vectors of calibration pattern points in the calibration pattern
     * coordinate space. See #calibrateCamera for details. If the method of releasing object to be used,
     * the identical calibration board must be used in each view and it must be fully visible, and all
     * objectPoints[i] must be the same and all points should be roughly close to a plane. <b>The calibration
     * target has to be rigid, or at least static if the camera (rather than the calibration target) is
     * shifted for grabbing images.</b>
     * @param imagePoints Vector of vectors of the projections of calibration pattern points. See
     * #calibrateCamera for details.
     * @param imageSize Size of the image used only to initialize the intrinsic camera matrix.
     * @param iFixedPoint The index of the 3D object point in objectPoints[0] to be fixed. It also acts as
     * a switch for calibration method selection. If object-releasing method to be used, pass in the
     * parameter in the range of [1, objectPoints[0].size()-2], otherwise a value out of this range will
     * make standard calibration method selected. Usually the top-right corner point of the calibration
     * board grid is recommended to be fixed when object-releasing method being utilized. According to
     * \cite strobl2011iccv, two other points are also fixed. In this implementation, objectPoints[0].front
     * and objectPoints[0].back.z are used. With object-releasing method, accurate rvecs, tvecs and
     * newObjPoints are only possible if coordinates of these three fixed points are accurate enough.
     * @param cameraMatrix Output 3x3 floating-point camera matrix. See #calibrateCamera for details.
     * @param distCoeffs Output vector of distortion coefficients. See #calibrateCamera for details.
     * @param rvecs Output vector of rotation vectors estimated for each pattern view. See #calibrateCamera
     * for details.
     * @param tvecs Output vector of translation vectors estimated for each pattern view.
     * @param newObjPoints The updated output vector of calibration pattern points. The coordinates might
     * be scaled based on three fixed points. The returned coordinates are accurate only if the above
     * mentioned three fixed points are accurate. If not needed, noArray() can be passed in. This parameter
     * is ignored with standard calibration method.
     * @param stdDeviationsIntrinsics Output vector of standard deviations estimated for intrinsic parameters.
     * See #calibrateCamera for details.
     * @param stdDeviationsExtrinsics Output vector of standard deviations estimated for extrinsic parameters.
     * See #calibrateCamera for details.
     * @param stdDeviationsObjPoints Output vector of standard deviations estimated for refined coordinates
     * of calibration pattern points. It has the same size and order as objectPoints[0] vector. This
     * parameter is ignored with standard calibration method.
     *  @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * #calibrateCamera for details. If the method of releasing object is used, the calibration time may
     * be much longer. CALIB_USE_QR or CALIB_USE_LU could be used for faster calibration with potentially
     * less precise and less stable in some rare cases.
     *
     * @return the overall RMS re-projection error.
     *
     * The function estimates the intrinsic camera parameters and extrinsic parameters for each of the
     * views. The algorithm is based on CITE: Zhang2000, CITE: BouguetMCT and CITE: strobl2011iccv. See
     * #calibrateCamera for other detailed explanations.
     * SEE:
     *    calibrateCamera, findChessboardCorners, solvePnP, initCameraMatrix2D, stereoCalibrate, undistort
     */
    public static double calibrateCameraROExtended(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, int iFixedPoint, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat newObjPoints, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat stdDeviationsObjPoints, Mat perViewErrors) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraROExtended_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, iFixedPoint, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, newObjPoints.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, stdDeviationsObjPoints.nativeObj, perViewErrors.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double cv::calibrateCameraRO(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, int iFixedPoint, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& newObjPoints, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    public static double calibrateCameraRO(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, int iFixedPoint, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat newObjPoints, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraRO_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, iFixedPoint, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, newObjPoints.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraRO(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, int iFixedPoint, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat newObjPoints, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraRO_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, iFixedPoint, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, newObjPoints.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    public static double calibrateCameraRO(List<Mat> objectPoints, List<Mat> imagePoints, Size imageSize, int iFixedPoint, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat newObjPoints) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraRO_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, imageSize.width, imageSize.height, iFixedPoint, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, newObjPoints.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  void cv::calibrationMatrixValues(Mat cameraMatrix, Size imageSize, double apertureWidth, double apertureHeight, double& fovx, double& fovy, double& focalLength, Point2d& principalPoint, double& aspectRatio)
    //

    /**
     * Computes useful camera characteristics from the camera intrinsic matrix.
     *
     * @param cameraMatrix Input camera intrinsic matrix that can be estimated by #calibrateCamera or
     * #stereoCalibrate .
     * @param imageSize Input image size in pixels.
     * @param apertureWidth Physical width in mm of the sensor.
     * @param apertureHeight Physical height in mm of the sensor.
     * @param fovx Output field of view in degrees along the horizontal sensor axis.
     * @param fovy Output field of view in degrees along the vertical sensor axis.
     * @param focalLength Focal length of the lens in mm.
     * @param principalPoint Principal point in mm.
     * @param aspectRatio \(f_y/f_x\)
     *
     * The function computes various useful camera characteristics from the previously estimated camera
     * matrix.
     *
     * <b>Note:</b>
     *    Do keep in mind that the unity measure 'mm' stands for whatever unit of measure one chooses for
     *     the chessboard pitch (it can thus be any value).
     */
    public static void calibrationMatrixValues(Mat cameraMatrix, Size imageSize, double apertureWidth, double apertureHeight, double[] fovx, double[] fovy, double[] focalLength, Point principalPoint, double[] aspectRatio) {
        double[] fovx_out = new double[1];
        double[] fovy_out = new double[1];
        double[] focalLength_out = new double[1];
        double[] principalPoint_out = new double[2];
        double[] aspectRatio_out = new double[1];
        calibrationMatrixValues_0(cameraMatrix.nativeObj, imageSize.width, imageSize.height, apertureWidth, apertureHeight, fovx_out, fovy_out, focalLength_out, principalPoint_out, aspectRatio_out);
        if(fovx!=null) fovx[0] = (double)fovx_out[0];
        if(fovy!=null) fovy[0] = (double)fovy_out[0];
        if(focalLength!=null) focalLength[0] = (double)focalLength_out[0];
        if(principalPoint!=null){ principalPoint.x = principalPoint_out[0]; principalPoint.y = principalPoint_out[1]; } 
        if(aspectRatio!=null) aspectRatio[0] = (double)aspectRatio_out[0];
    }


    //
    // C++:  double cv::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size imageSize, Mat& R, Mat& T, Mat& E, Mat& F, vector_Mat& rvecs, vector_Mat& tvecs, Mat& perViewErrors, int flags = CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6))
    //

    /**
     * Calibrates a stereo camera set up. This function finds the intrinsic parameters
     * for each of the two cameras and the extrinsic parameters between the two cameras.
     *
     * @param objectPoints Vector of vectors of the calibration pattern points. The same structure as
     * in REF: calibrateCamera. For each pattern view, both cameras need to see the same object
     * points. Therefore, objectPoints.size(), imagePoints1.size(), and imagePoints2.size() need to be
     * equal as well as objectPoints[i].size(), imagePoints1[i].size(), and imagePoints2[i].size() need to
     * be equal for each i.
     * @param imagePoints1 Vector of vectors of the projections of the calibration pattern points,
     * observed by the first camera. The same structure as in REF: calibrateCamera.
     * @param imagePoints2 Vector of vectors of the projections of the calibration pattern points,
     * observed by the second camera. The same structure as in REF: calibrateCamera.
     * @param cameraMatrix1 Input/output camera intrinsic matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output second camera intrinsic matrix for the second camera. See description for
     * cameraMatrix1.
     * @param distCoeffs2 Input/output lens distortion coefficients for the second camera. See
     * description for distCoeffs1.
     * @param imageSize Size of the image used only to initialize the camera intrinsic matrices.
     * @param R Output rotation matrix. Together with the translation vector T, this matrix brings
     * points given in the first camera's coordinate system to points in the second camera's
     * coordinate system. In more technical terms, the tuple of R and T performs a change of basis
     * from the first camera's coordinate system to the second camera's coordinate system. Due to its
     * duality, this tuple is equivalent to the position of the first camera with respect to the
     * second camera coordinate system.
     * @param T Output translation vector, see description above.
     * @param E Output essential matrix.
     * @param F Output fundamental matrix.
     * @param rvecs Output vector of rotation vectors ( REF: Rodrigues ) estimated for each pattern view in the
     * coordinate system of the first camera of the stereo pair (e.g. std::vector&lt;cv::Mat&gt;). More in detail, each
     * i-th rotation vector together with the corresponding i-th translation vector (see the next output parameter
     * description) brings the calibration pattern from the object coordinate space (in which object points are
     * specified) to the camera coordinate space of the first camera of the stereo pair. In more technical terms,
     * the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space
     * to camera coordinate space of the first camera of the stereo pair.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter description
     * of previous output parameter ( rvecs ).
     * @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: CALIB_FIX_INTRINSIC Fix cameraMatrix? and distCoeffs? so that only R, T, E, and F
     * matrices are estimated.
     *   </li>
     *   <li>
     *    REF: CALIB_USE_INTRINSIC_GUESS Optimize some or all of the intrinsic parameters
     * according to the specified flags. Initial values are provided by the user.
     *   </li>
     *   <li>
     *    REF: CALIB_USE_EXTRINSIC_GUESS R and T contain valid initial values that are optimized further.
     * Otherwise R and T are initialized to the median value of the pattern views (each dimension separately).
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_PRINCIPAL_POINT Fix the principal points during the optimization.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_FOCAL_LENGTH Fix \(f^{(j)}_x\) and \(f^{(j)}_y\) .
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_ASPECT_RATIO Optimize \(f^{(j)}_y\) . Fix the ratio \(f^{(j)}_x/f^{(j)}_y\)
     * .
     *   </li>
     *   <li>
     *    REF: CALIB_SAME_FOCAL_LENGTH Enforce \(f^{(0)}_x=f^{(1)}_x\) and \(f^{(0)}_y=f^{(1)}_y\) .
     *   </li>
     *   <li>
     *    REF: CALIB_ZERO_TANGENT_DIST Set tangential distortion coefficients for each camera to
     * zeros and fix there.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_K1,..., REF: CALIB_FIX_K6 Do not change the corresponding radial
     * distortion coefficient during the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set,
     * the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_RATIONAL_MODEL Enable coefficients k4, k5, and k6. To provide the backward
     * compatibility, this extra flag should be explicitly specified to make the calibration
     * function use the rational model and return 8 coefficients. If the flag is not set, the
     * function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_THIN_PRISM_MODEL Coefficients s1, s2, s3 and s4 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the thin prism model and return 12 coefficients. If the flag is not
     * set, the function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_S1_S2_S3_S4 The thin prism distortion coefficients are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_TILTED_MODEL Coefficients tauX and tauY are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the tilted sensor model and return 14 coefficients. If the flag is not
     * set, the function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_TAUX_TAUY The coefficients of the tilted sensor model are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     * @param criteria Termination criteria for the iterative optimization algorithm.
     *   </li>
     * </ul>
     *
     * The function estimates the transformation between two cameras making a stereo pair. If one computes
     * the poses of an object relative to the first camera and to the second camera,
     * ( \(R_1\),\(T_1\) ) and (\(R_2\),\(T_2\)), respectively, for a stereo camera where the
     * relative position and orientation between the two cameras are fixed, then those poses definitely
     * relate to each other. This means, if the relative position and orientation (\(R\),\(T\)) of the
     * two cameras is known, it is possible to compute (\(R_2\),\(T_2\)) when (\(R_1\),\(T_1\)) is
     * given. This is what the described function does. It computes (\(R\),\(T\)) such that:
     *
     * \(R_2=R R_1\)
     * \(T_2=R T_1 + T.\)
     *
     * Therefore, one can compute the coordinate representation of a 3D point for the second camera's
     * coordinate system when given the point's coordinate representation in the first camera's coordinate
     * system:
     *
     * \(\begin{bmatrix}
     * X_2 \\
     * Y_2 \\
     * Z_2 \\
     * 1
     * \end{bmatrix} = \begin{bmatrix}
     * R &amp; T \\
     * 0 &amp; 1
     * \end{bmatrix} \begin{bmatrix}
     * X_1 \\
     * Y_1 \\
     * Z_1 \\
     * 1
     * \end{bmatrix}.\)
     *
     *
     * Optionally, it computes the essential matrix E:
     *
     * \(E= \vecthreethree{0}{-T_2}{T_1}{T_2}{0}{-T_0}{-T_1}{T_0}{0} R\)
     *
     * where \(T_i\) are components of the translation vector \(T\) : \(T=[T_0, T_1, T_2]^T\) .
     * And the function can also compute the fundamental matrix F:
     *
     * \(F = cameraMatrix2^{-T}\cdot E \cdot cameraMatrix1^{-1}\)
     *
     * Besides the stereo-related information, the function can also perform a full calibration of each of
     * the two cameras. However, due to the high dimensionality of the parameter space and noise in the
     * input data, the function can diverge from the correct solution. If the intrinsic parameters can be
     * estimated with high accuracy for each of the cameras individually (for example, using
     * #calibrateCamera ), you are recommended to do so and then pass REF: CALIB_FIX_INTRINSIC flag to the
     * function along with the computed intrinsic parameters. Otherwise, if all the parameters are
     * estimated at once, it makes sense to restrict some parameters, for example, pass
     *  REF: CALIB_SAME_FOCAL_LENGTH and REF: CALIB_ZERO_TANGENT_DIST flags, which is usually a
     * reasonable assumption.
     *
     * Similarly to #calibrateCamera, the function minimizes the total re-projection error for all the
     * points in all the available views from both cameras. The function returns the final value of the
     * re-projection error.
     * @return automatically generated
     */
    public static double stereoCalibrateExtended(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, List<Mat> rvecs, List<Mat> tvecs, Mat perViewErrors, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = stereoCalibrateExtended_0(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, perViewErrors.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Calibrates a stereo camera set up. This function finds the intrinsic parameters
     * for each of the two cameras and the extrinsic parameters between the two cameras.
     *
     * @param objectPoints Vector of vectors of the calibration pattern points. The same structure as
     * in REF: calibrateCamera. For each pattern view, both cameras need to see the same object
     * points. Therefore, objectPoints.size(), imagePoints1.size(), and imagePoints2.size() need to be
     * equal as well as objectPoints[i].size(), imagePoints1[i].size(), and imagePoints2[i].size() need to
     * be equal for each i.
     * @param imagePoints1 Vector of vectors of the projections of the calibration pattern points,
     * observed by the first camera. The same structure as in REF: calibrateCamera.
     * @param imagePoints2 Vector of vectors of the projections of the calibration pattern points,
     * observed by the second camera. The same structure as in REF: calibrateCamera.
     * @param cameraMatrix1 Input/output camera intrinsic matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output second camera intrinsic matrix for the second camera. See description for
     * cameraMatrix1.
     * @param distCoeffs2 Input/output lens distortion coefficients for the second camera. See
     * description for distCoeffs1.
     * @param imageSize Size of the image used only to initialize the camera intrinsic matrices.
     * @param R Output rotation matrix. Together with the translation vector T, this matrix brings
     * points given in the first camera's coordinate system to points in the second camera's
     * coordinate system. In more technical terms, the tuple of R and T performs a change of basis
     * from the first camera's coordinate system to the second camera's coordinate system. Due to its
     * duality, this tuple is equivalent to the position of the first camera with respect to the
     * second camera coordinate system.
     * @param T Output translation vector, see description above.
     * @param E Output essential matrix.
     * @param F Output fundamental matrix.
     * @param rvecs Output vector of rotation vectors ( REF: Rodrigues ) estimated for each pattern view in the
     * coordinate system of the first camera of the stereo pair (e.g. std::vector&lt;cv::Mat&gt;). More in detail, each
     * i-th rotation vector together with the corresponding i-th translation vector (see the next output parameter
     * description) brings the calibration pattern from the object coordinate space (in which object points are
     * specified) to the camera coordinate space of the first camera of the stereo pair. In more technical terms,
     * the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space
     * to camera coordinate space of the first camera of the stereo pair.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter description
     * of previous output parameter ( rvecs ).
     * @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: CALIB_FIX_INTRINSIC Fix cameraMatrix? and distCoeffs? so that only R, T, E, and F
     * matrices are estimated.
     *   </li>
     *   <li>
     *    REF: CALIB_USE_INTRINSIC_GUESS Optimize some or all of the intrinsic parameters
     * according to the specified flags. Initial values are provided by the user.
     *   </li>
     *   <li>
     *    REF: CALIB_USE_EXTRINSIC_GUESS R and T contain valid initial values that are optimized further.
     * Otherwise R and T are initialized to the median value of the pattern views (each dimension separately).
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_PRINCIPAL_POINT Fix the principal points during the optimization.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_FOCAL_LENGTH Fix \(f^{(j)}_x\) and \(f^{(j)}_y\) .
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_ASPECT_RATIO Optimize \(f^{(j)}_y\) . Fix the ratio \(f^{(j)}_x/f^{(j)}_y\)
     * .
     *   </li>
     *   <li>
     *    REF: CALIB_SAME_FOCAL_LENGTH Enforce \(f^{(0)}_x=f^{(1)}_x\) and \(f^{(0)}_y=f^{(1)}_y\) .
     *   </li>
     *   <li>
     *    REF: CALIB_ZERO_TANGENT_DIST Set tangential distortion coefficients for each camera to
     * zeros and fix there.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_K1,..., REF: CALIB_FIX_K6 Do not change the corresponding radial
     * distortion coefficient during the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set,
     * the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_RATIONAL_MODEL Enable coefficients k4, k5, and k6. To provide the backward
     * compatibility, this extra flag should be explicitly specified to make the calibration
     * function use the rational model and return 8 coefficients. If the flag is not set, the
     * function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_THIN_PRISM_MODEL Coefficients s1, s2, s3 and s4 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the thin prism model and return 12 coefficients. If the flag is not
     * set, the function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_S1_S2_S3_S4 The thin prism distortion coefficients are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_TILTED_MODEL Coefficients tauX and tauY are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the tilted sensor model and return 14 coefficients. If the flag is not
     * set, the function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_TAUX_TAUY The coefficients of the tilted sensor model are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     * </ul>
     *
     * The function estimates the transformation between two cameras making a stereo pair. If one computes
     * the poses of an object relative to the first camera and to the second camera,
     * ( \(R_1\),\(T_1\) ) and (\(R_2\),\(T_2\)), respectively, for a stereo camera where the
     * relative position and orientation between the two cameras are fixed, then those poses definitely
     * relate to each other. This means, if the relative position and orientation (\(R\),\(T\)) of the
     * two cameras is known, it is possible to compute (\(R_2\),\(T_2\)) when (\(R_1\),\(T_1\)) is
     * given. This is what the described function does. It computes (\(R\),\(T\)) such that:
     *
     * \(R_2=R R_1\)
     * \(T_2=R T_1 + T.\)
     *
     * Therefore, one can compute the coordinate representation of a 3D point for the second camera's
     * coordinate system when given the point's coordinate representation in the first camera's coordinate
     * system:
     *
     * \(\begin{bmatrix}
     * X_2 \\
     * Y_2 \\
     * Z_2 \\
     * 1
     * \end{bmatrix} = \begin{bmatrix}
     * R &amp; T \\
     * 0 &amp; 1
     * \end{bmatrix} \begin{bmatrix}
     * X_1 \\
     * Y_1 \\
     * Z_1 \\
     * 1
     * \end{bmatrix}.\)
     *
     *
     * Optionally, it computes the essential matrix E:
     *
     * \(E= \vecthreethree{0}{-T_2}{T_1}{T_2}{0}{-T_0}{-T_1}{T_0}{0} R\)
     *
     * where \(T_i\) are components of the translation vector \(T\) : \(T=[T_0, T_1, T_2]^T\) .
     * And the function can also compute the fundamental matrix F:
     *
     * \(F = cameraMatrix2^{-T}\cdot E \cdot cameraMatrix1^{-1}\)
     *
     * Besides the stereo-related information, the function can also perform a full calibration of each of
     * the two cameras. However, due to the high dimensionality of the parameter space and noise in the
     * input data, the function can diverge from the correct solution. If the intrinsic parameters can be
     * estimated with high accuracy for each of the cameras individually (for example, using
     * #calibrateCamera ), you are recommended to do so and then pass REF: CALIB_FIX_INTRINSIC flag to the
     * function along with the computed intrinsic parameters. Otherwise, if all the parameters are
     * estimated at once, it makes sense to restrict some parameters, for example, pass
     *  REF: CALIB_SAME_FOCAL_LENGTH and REF: CALIB_ZERO_TANGENT_DIST flags, which is usually a
     * reasonable assumption.
     *
     * Similarly to #calibrateCamera, the function minimizes the total re-projection error for all the
     * points in all the available views from both cameras. The function returns the final value of the
     * re-projection error.
     * @return automatically generated
     */
    public static double stereoCalibrateExtended(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, List<Mat> rvecs, List<Mat> tvecs, Mat perViewErrors, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = stereoCalibrateExtended_1(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Calibrates a stereo camera set up. This function finds the intrinsic parameters
     * for each of the two cameras and the extrinsic parameters between the two cameras.
     *
     * @param objectPoints Vector of vectors of the calibration pattern points. The same structure as
     * in REF: calibrateCamera. For each pattern view, both cameras need to see the same object
     * points. Therefore, objectPoints.size(), imagePoints1.size(), and imagePoints2.size() need to be
     * equal as well as objectPoints[i].size(), imagePoints1[i].size(), and imagePoints2[i].size() need to
     * be equal for each i.
     * @param imagePoints1 Vector of vectors of the projections of the calibration pattern points,
     * observed by the first camera. The same structure as in REF: calibrateCamera.
     * @param imagePoints2 Vector of vectors of the projections of the calibration pattern points,
     * observed by the second camera. The same structure as in REF: calibrateCamera.
     * @param cameraMatrix1 Input/output camera intrinsic matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output second camera intrinsic matrix for the second camera. See description for
     * cameraMatrix1.
     * @param distCoeffs2 Input/output lens distortion coefficients for the second camera. See
     * description for distCoeffs1.
     * @param imageSize Size of the image used only to initialize the camera intrinsic matrices.
     * @param R Output rotation matrix. Together with the translation vector T, this matrix brings
     * points given in the first camera's coordinate system to points in the second camera's
     * coordinate system. In more technical terms, the tuple of R and T performs a change of basis
     * from the first camera's coordinate system to the second camera's coordinate system. Due to its
     * duality, this tuple is equivalent to the position of the first camera with respect to the
     * second camera coordinate system.
     * @param T Output translation vector, see description above.
     * @param E Output essential matrix.
     * @param F Output fundamental matrix.
     * @param rvecs Output vector of rotation vectors ( REF: Rodrigues ) estimated for each pattern view in the
     * coordinate system of the first camera of the stereo pair (e.g. std::vector&lt;cv::Mat&gt;). More in detail, each
     * i-th rotation vector together with the corresponding i-th translation vector (see the next output parameter
     * description) brings the calibration pattern from the object coordinate space (in which object points are
     * specified) to the camera coordinate space of the first camera of the stereo pair. In more technical terms,
     * the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space
     * to camera coordinate space of the first camera of the stereo pair.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter description
     * of previous output parameter ( rvecs ).
     * @param perViewErrors Output vector of the RMS re-projection error estimated for each pattern view.
     * <ul>
     *   <li>
     *    REF: CALIB_FIX_INTRINSIC Fix cameraMatrix? and distCoeffs? so that only R, T, E, and F
     * matrices are estimated.
     *   </li>
     *   <li>
     *    REF: CALIB_USE_INTRINSIC_GUESS Optimize some or all of the intrinsic parameters
     * according to the specified flags. Initial values are provided by the user.
     *   </li>
     *   <li>
     *    REF: CALIB_USE_EXTRINSIC_GUESS R and T contain valid initial values that are optimized further.
     * Otherwise R and T are initialized to the median value of the pattern views (each dimension separately).
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_PRINCIPAL_POINT Fix the principal points during the optimization.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_FOCAL_LENGTH Fix \(f^{(j)}_x\) and \(f^{(j)}_y\) .
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_ASPECT_RATIO Optimize \(f^{(j)}_y\) . Fix the ratio \(f^{(j)}_x/f^{(j)}_y\)
     * .
     *   </li>
     *   <li>
     *    REF: CALIB_SAME_FOCAL_LENGTH Enforce \(f^{(0)}_x=f^{(1)}_x\) and \(f^{(0)}_y=f^{(1)}_y\) .
     *   </li>
     *   <li>
     *    REF: CALIB_ZERO_TANGENT_DIST Set tangential distortion coefficients for each camera to
     * zeros and fix there.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_K1,..., REF: CALIB_FIX_K6 Do not change the corresponding radial
     * distortion coefficient during the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set,
     * the coefficient from the supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_RATIONAL_MODEL Enable coefficients k4, k5, and k6. To provide the backward
     * compatibility, this extra flag should be explicitly specified to make the calibration
     * function use the rational model and return 8 coefficients. If the flag is not set, the
     * function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_THIN_PRISM_MODEL Coefficients s1, s2, s3 and s4 are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the thin prism model and return 12 coefficients. If the flag is not
     * set, the function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_S1_S2_S3_S4 The thin prism distortion coefficients are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     *   <li>
     *    REF: CALIB_TILTED_MODEL Coefficients tauX and tauY are enabled. To provide the
     * backward compatibility, this extra flag should be explicitly specified to make the
     * calibration function use the tilted sensor model and return 14 coefficients. If the flag is not
     * set, the function computes and returns only 5 distortion coefficients.
     *   </li>
     *   <li>
     *    REF: CALIB_FIX_TAUX_TAUY The coefficients of the tilted sensor model are not changed during
     * the optimization. If REF: CALIB_USE_INTRINSIC_GUESS is set, the coefficient from the
     * supplied distCoeffs matrix is used. Otherwise, it is set to 0.
     *   </li>
     * </ul>
     *
     * The function estimates the transformation between two cameras making a stereo pair. If one computes
     * the poses of an object relative to the first camera and to the second camera,
     * ( \(R_1\),\(T_1\) ) and (\(R_2\),\(T_2\)), respectively, for a stereo camera where the
     * relative position and orientation between the two cameras are fixed, then those poses definitely
     * relate to each other. This means, if the relative position and orientation (\(R\),\(T\)) of the
     * two cameras is known, it is possible to compute (\(R_2\),\(T_2\)) when (\(R_1\),\(T_1\)) is
     * given. This is what the described function does. It computes (\(R\),\(T\)) such that:
     *
     * \(R_2=R R_1\)
     * \(T_2=R T_1 + T.\)
     *
     * Therefore, one can compute the coordinate representation of a 3D point for the second camera's
     * coordinate system when given the point's coordinate representation in the first camera's coordinate
     * system:
     *
     * \(\begin{bmatrix}
     * X_2 \\
     * Y_2 \\
     * Z_2 \\
     * 1
     * \end{bmatrix} = \begin{bmatrix}
     * R &amp; T \\
     * 0 &amp; 1
     * \end{bmatrix} \begin{bmatrix}
     * X_1 \\
     * Y_1 \\
     * Z_1 \\
     * 1
     * \end{bmatrix}.\)
     *
     *
     * Optionally, it computes the essential matrix E:
     *
     * \(E= \vecthreethree{0}{-T_2}{T_1}{T_2}{0}{-T_0}{-T_1}{T_0}{0} R\)
     *
     * where \(T_i\) are components of the translation vector \(T\) : \(T=[T_0, T_1, T_2]^T\) .
     * And the function can also compute the fundamental matrix F:
     *
     * \(F = cameraMatrix2^{-T}\cdot E \cdot cameraMatrix1^{-1}\)
     *
     * Besides the stereo-related information, the function can also perform a full calibration of each of
     * the two cameras. However, due to the high dimensionality of the parameter space and noise in the
     * input data, the function can diverge from the correct solution. If the intrinsic parameters can be
     * estimated with high accuracy for each of the cameras individually (for example, using
     * #calibrateCamera ), you are recommended to do so and then pass REF: CALIB_FIX_INTRINSIC flag to the
     * function along with the computed intrinsic parameters. Otherwise, if all the parameters are
     * estimated at once, it makes sense to restrict some parameters, for example, pass
     *  REF: CALIB_SAME_FOCAL_LENGTH and REF: CALIB_ZERO_TANGENT_DIST flags, which is usually a
     * reasonable assumption.
     *
     * Similarly to #calibrateCamera, the function minimizes the total re-projection error for all the
     * points in all the available views from both cameras. The function returns the final value of the
     * re-projection error.
     * @return automatically generated
     */
    public static double stereoCalibrateExtended(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, List<Mat> rvecs, List<Mat> tvecs, Mat perViewErrors) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = stereoCalibrateExtended_2(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, perViewErrors.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double cv::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size imageSize, Mat& R, Mat& T, Mat& E, Mat& F, int flags = CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6))
    //

    public static double stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return stereoCalibrate_0(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    public static double stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return stereoCalibrate_1(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, flags);
    }

    public static double stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return stereoCalibrate_2(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj);
    }


    //
    // C++:  double cv::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size imageSize, Mat& R, Mat& T, Mat& E, Mat& F, Mat& perViewErrors, int flags = CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6))
    //

    public static double stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, Mat perViewErrors, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return stereoCalibrate_3(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, perViewErrors.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    public static double stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, Mat perViewErrors, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return stereoCalibrate_4(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, perViewErrors.nativeObj, flags);
    }

    public static double stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Size imageSize, Mat R, Mat T, Mat E, Mat F, Mat perViewErrors) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return stereoCalibrate_5(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, E.nativeObj, F.nativeObj, perViewErrors.nativeObj);
    }


    //
    // C++:  double cv::calibrateMultiview(vector_Mat objPoints, vector_vector_Mat imagePoints, vector_Size imageSize, Mat detectionMask, vector_Mat& Rs, vector_Mat& Ts, vector_Mat& Ks, vector_Mat& distortions, vector_Mat& rvecs0, vector_Mat& tvecs0, Mat isFisheye, Mat& perFrameErrors, Mat& initializationPairs, bool useIntrinsicsGuess = false, Mat flagsForIntrinsics = Mat())
    //

    // Unknown type 'vector_vector_Mat' (I), skipping the function


    //
    // C++:  void cv::fisheye::projectPoints(Mat objectPoints, Mat& imagePoints, Mat rvec, Mat tvec, Mat K, Mat D, double alpha = 0, Mat& jacobian = Mat())
    //

    public static void fisheye_projectPoints(Mat objectPoints, Mat imagePoints, Mat rvec, Mat tvec, Mat K, Mat D, double alpha, Mat jacobian) {
        fisheye_projectPoints_0(objectPoints.nativeObj, imagePoints.nativeObj, rvec.nativeObj, tvec.nativeObj, K.nativeObj, D.nativeObj, alpha, jacobian.nativeObj);
    }

    public static void fisheye_projectPoints(Mat objectPoints, Mat imagePoints, Mat rvec, Mat tvec, Mat K, Mat D, double alpha) {
        fisheye_projectPoints_1(objectPoints.nativeObj, imagePoints.nativeObj, rvec.nativeObj, tvec.nativeObj, K.nativeObj, D.nativeObj, alpha);
    }

    public static void fisheye_projectPoints(Mat objectPoints, Mat imagePoints, Mat rvec, Mat tvec, Mat K, Mat D) {
        fisheye_projectPoints_2(objectPoints.nativeObj, imagePoints.nativeObj, rvec.nativeObj, tvec.nativeObj, K.nativeObj, D.nativeObj);
    }


    //
    // C++:  void cv::fisheye::distortPoints(Mat undistorted, Mat& distorted, Mat K, Mat D, double alpha = 0)
    //

    /**
     * Distorts 2D points using fisheye model.
     *
     * @param undistorted Array of object points, 1xN/Nx1 2-channel (or vector&lt;Point2f&gt; ), where N is
     * the number of points in the view.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param alpha The skew coefficient.
     * @param distorted Output array of image points, 1xN/Nx1 2-channel, or vector&lt;Point2f&gt; .
     *
     * Note that the function assumes the camera intrinsic matrix of the undistorted points to be identity.
     * This means if you want to distort image points you have to multiply them with \(K^{-1}\).
     */
    public static void fisheye_distortPoints(Mat undistorted, Mat distorted, Mat K, Mat D, double alpha) {
        fisheye_distortPoints_0(undistorted.nativeObj, distorted.nativeObj, K.nativeObj, D.nativeObj, alpha);
    }

    /**
     * Distorts 2D points using fisheye model.
     *
     * @param undistorted Array of object points, 1xN/Nx1 2-channel (or vector&lt;Point2f&gt; ), where N is
     * the number of points in the view.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param distorted Output array of image points, 1xN/Nx1 2-channel, or vector&lt;Point2f&gt; .
     *
     * Note that the function assumes the camera intrinsic matrix of the undistorted points to be identity.
     * This means if you want to distort image points you have to multiply them with \(K^{-1}\).
     */
    public static void fisheye_distortPoints(Mat undistorted, Mat distorted, Mat K, Mat D) {
        fisheye_distortPoints_1(undistorted.nativeObj, distorted.nativeObj, K.nativeObj, D.nativeObj);
    }


    //
    // C++:  void cv::fisheye::undistortPoints(Mat distorted, Mat& undistorted, Mat K, Mat D, Mat R = Mat(), Mat P = Mat(), TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 10, 1e-8))
    //

    /**
     * Undistorts 2D points using fisheye model
     *
     * @param distorted Array of object points, 1xN/Nx1 2-channel (or vector&lt;Point2f&gt; ), where N is the
     * number of points in the view.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * @param criteria Termination criteria
     * @param undistorted Output array of image points, 1xN/Nx1 2-channel, or vector&lt;Point2f&gt; .
     */
    public static void fisheye_undistortPoints(Mat distorted, Mat undistorted, Mat K, Mat D, Mat R, Mat P, TermCriteria criteria) {
        fisheye_undistortPoints_0(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj, R.nativeObj, P.nativeObj, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    /**
     * Undistorts 2D points using fisheye model
     *
     * @param distorted Array of object points, 1xN/Nx1 2-channel (or vector&lt;Point2f&gt; ), where N is the
     * number of points in the view.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * @param undistorted Output array of image points, 1xN/Nx1 2-channel, or vector&lt;Point2f&gt; .
     */
    public static void fisheye_undistortPoints(Mat distorted, Mat undistorted, Mat K, Mat D, Mat R, Mat P) {
        fisheye_undistortPoints_1(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj, R.nativeObj, P.nativeObj);
    }

    /**
     * Undistorts 2D points using fisheye model
     *
     * @param distorted Array of object points, 1xN/Nx1 2-channel (or vector&lt;Point2f&gt; ), where N is the
     * number of points in the view.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param undistorted Output array of image points, 1xN/Nx1 2-channel, or vector&lt;Point2f&gt; .
     */
    public static void fisheye_undistortPoints(Mat distorted, Mat undistorted, Mat K, Mat D, Mat R) {
        fisheye_undistortPoints_2(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj, R.nativeObj);
    }

    /**
     * Undistorts 2D points using fisheye model
     *
     * @param distorted Array of object points, 1xN/Nx1 2-channel (or vector&lt;Point2f&gt; ), where N is the
     * number of points in the view.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * 1-channel or 1x1 3-channel
     * @param undistorted Output array of image points, 1xN/Nx1 2-channel, or vector&lt;Point2f&gt; .
     */
    public static void fisheye_undistortPoints(Mat distorted, Mat undistorted, Mat K, Mat D) {
        fisheye_undistortPoints_3(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj);
    }


    //
    // C++:  void cv::fisheye::initUndistortRectifyMap(Mat K, Mat D, Mat R, Mat P, Size size, int m1type, Mat& map1, Mat& map2)
    //

    /**
     * Computes undistortion and rectification maps for image transform by cv::remap(). If D is empty zero
     * distortion is used, if R or P is empty identity matrixes are used.
     *
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * @param size Undistorted image size.
     * @param m1type Type of the first output map that can be CV_32FC1 or CV_16SC2 . See convertMaps()
     * for details.
     * @param map1 The first output map.
     * @param map2 The second output map.
     */
    public static void fisheye_initUndistortRectifyMap(Mat K, Mat D, Mat R, Mat P, Size size, int m1type, Mat map1, Mat map2) {
        fisheye_initUndistortRectifyMap_0(K.nativeObj, D.nativeObj, R.nativeObj, P.nativeObj, size.width, size.height, m1type, map1.nativeObj, map2.nativeObj);
    }


    //
    // C++:  void cv::fisheye::undistortImage(Mat distorted, Mat& undistorted, Mat K, Mat D, Mat Knew = cv::Mat(), Size new_size = Size())
    //

    /**
     * Transforms an image to compensate for fisheye lens distortion.
     *
     * @param distorted image with fisheye lens distortion.
     * @param undistorted Output image with compensated fisheye lens distortion.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param Knew Camera intrinsic matrix of the distorted image. By default, it is the identity matrix but you
     * may additionally scale and shift the result by using a different matrix.
     * @param new_size the new size
     *
     * The function transforms an image to compensate radial and tangential lens distortion.
     *
     * The function is simply a combination of fisheye::initUndistortRectifyMap (with unity R ) and remap
     * (with bilinear interpolation). See the former function for details of the transformation being
     * performed.
     *
     * See below the results of undistortImage.
     * <ul>
     *   <li>
     *       a\) result of undistort of perspective camera model (all possible coefficients (k_1, k_2, k_3,
     *         k_4, k_5, k_6) of distortion were optimized under calibration)
     *   <ul>
     *     <li>
     *        b\) result of fisheye::undistortImage of fisheye camera model (all possible coefficients (k_1, k_2,
     *         k_3, k_4) of fisheye distortion were optimized under calibration)
     *     </li>
     *     <li>
     *        c\) original image was captured with fisheye lens
     *     </li>
     *   </ul>
     *
     * Pictures a) and b) almost the same. But if we consider points of image located far from the center
     * of image, we can notice that on image a) these points are distorted.
     *   </li>
     * </ul>
     *
     * ![image](pics/fisheye_undistorted.jpg)
     */
    public static void fisheye_undistortImage(Mat distorted, Mat undistorted, Mat K, Mat D, Mat Knew, Size new_size) {
        fisheye_undistortImage_0(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj, Knew.nativeObj, new_size.width, new_size.height);
    }

    /**
     * Transforms an image to compensate for fisheye lens distortion.
     *
     * @param distorted image with fisheye lens distortion.
     * @param undistorted Output image with compensated fisheye lens distortion.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param Knew Camera intrinsic matrix of the distorted image. By default, it is the identity matrix but you
     * may additionally scale and shift the result by using a different matrix.
     *
     * The function transforms an image to compensate radial and tangential lens distortion.
     *
     * The function is simply a combination of fisheye::initUndistortRectifyMap (with unity R ) and remap
     * (with bilinear interpolation). See the former function for details of the transformation being
     * performed.
     *
     * See below the results of undistortImage.
     * <ul>
     *   <li>
     *       a\) result of undistort of perspective camera model (all possible coefficients (k_1, k_2, k_3,
     *         k_4, k_5, k_6) of distortion were optimized under calibration)
     *   <ul>
     *     <li>
     *        b\) result of fisheye::undistortImage of fisheye camera model (all possible coefficients (k_1, k_2,
     *         k_3, k_4) of fisheye distortion were optimized under calibration)
     *     </li>
     *     <li>
     *        c\) original image was captured with fisheye lens
     *     </li>
     *   </ul>
     *
     * Pictures a) and b) almost the same. But if we consider points of image located far from the center
     * of image, we can notice that on image a) these points are distorted.
     *   </li>
     * </ul>
     *
     * ![image](pics/fisheye_undistorted.jpg)
     */
    public static void fisheye_undistortImage(Mat distorted, Mat undistorted, Mat K, Mat D, Mat Knew) {
        fisheye_undistortImage_1(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj, Knew.nativeObj);
    }

    /**
     * Transforms an image to compensate for fisheye lens distortion.
     *
     * @param distorted image with fisheye lens distortion.
     * @param undistorted Output image with compensated fisheye lens distortion.
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * may additionally scale and shift the result by using a different matrix.
     *
     * The function transforms an image to compensate radial and tangential lens distortion.
     *
     * The function is simply a combination of fisheye::initUndistortRectifyMap (with unity R ) and remap
     * (with bilinear interpolation). See the former function for details of the transformation being
     * performed.
     *
     * See below the results of undistortImage.
     * <ul>
     *   <li>
     *       a\) result of undistort of perspective camera model (all possible coefficients (k_1, k_2, k_3,
     *         k_4, k_5, k_6) of distortion were optimized under calibration)
     *   <ul>
     *     <li>
     *        b\) result of fisheye::undistortImage of fisheye camera model (all possible coefficients (k_1, k_2,
     *         k_3, k_4) of fisheye distortion were optimized under calibration)
     *     </li>
     *     <li>
     *        c\) original image was captured with fisheye lens
     *     </li>
     *   </ul>
     *
     * Pictures a) and b) almost the same. But if we consider points of image located far from the center
     * of image, we can notice that on image a) these points are distorted.
     *   </li>
     * </ul>
     *
     * ![image](pics/fisheye_undistorted.jpg)
     */
    public static void fisheye_undistortImage(Mat distorted, Mat undistorted, Mat K, Mat D) {
        fisheye_undistortImage_2(distorted.nativeObj, undistorted.nativeObj, K.nativeObj, D.nativeObj);
    }


    //
    // C++:  void cv::fisheye::estimateNewCameraMatrixForUndistortRectify(Mat K, Mat D, Size image_size, Mat R, Mat& P, double balance = 0.0, Size new_size = Size(), double fov_scale = 1.0)
    //

    /**
     * Estimates new camera intrinsic matrix for undistortion or rectification.
     *
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param image_size Size of the image
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * @param balance Sets the new focal length in range between the min focal length and the max focal
     * length. Balance is in range of [0, 1].
     * @param new_size the new size
     * @param fov_scale Divisor for new focal length.
     */
    public static void fisheye_estimateNewCameraMatrixForUndistortRectify(Mat K, Mat D, Size image_size, Mat R, Mat P, double balance, Size new_size, double fov_scale) {
        fisheye_estimateNewCameraMatrixForUndistortRectify_0(K.nativeObj, D.nativeObj, image_size.width, image_size.height, R.nativeObj, P.nativeObj, balance, new_size.width, new_size.height, fov_scale);
    }

    /**
     * Estimates new camera intrinsic matrix for undistortion or rectification.
     *
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param image_size Size of the image
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * @param balance Sets the new focal length in range between the min focal length and the max focal
     * length. Balance is in range of [0, 1].
     * @param new_size the new size
     */
    public static void fisheye_estimateNewCameraMatrixForUndistortRectify(Mat K, Mat D, Size image_size, Mat R, Mat P, double balance, Size new_size) {
        fisheye_estimateNewCameraMatrixForUndistortRectify_1(K.nativeObj, D.nativeObj, image_size.width, image_size.height, R.nativeObj, P.nativeObj, balance, new_size.width, new_size.height);
    }

    /**
     * Estimates new camera intrinsic matrix for undistortion or rectification.
     *
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param image_size Size of the image
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * @param balance Sets the new focal length in range between the min focal length and the max focal
     * length. Balance is in range of [0, 1].
     */
    public static void fisheye_estimateNewCameraMatrixForUndistortRectify(Mat K, Mat D, Size image_size, Mat R, Mat P, double balance) {
        fisheye_estimateNewCameraMatrixForUndistortRectify_2(K.nativeObj, D.nativeObj, image_size.width, image_size.height, R.nativeObj, P.nativeObj, balance);
    }

    /**
     * Estimates new camera intrinsic matrix for undistortion or rectification.
     *
     * @param K Camera intrinsic matrix \(cameramatrix{K}\).
     * @param image_size Size of the image
     * @param D Input vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param R Rectification transformation in the object space: 3x3 1-channel, or vector: 3x1/1x3
     * 1-channel or 1x1 3-channel
     * @param P New camera intrinsic matrix (3x3) or new projection matrix (3x4)
     * length. Balance is in range of [0, 1].
     */
    public static void fisheye_estimateNewCameraMatrixForUndistortRectify(Mat K, Mat D, Size image_size, Mat R, Mat P) {
        fisheye_estimateNewCameraMatrixForUndistortRectify_3(K.nativeObj, D.nativeObj, image_size.width, image_size.height, R.nativeObj, P.nativeObj);
    }


    //
    // C++:  double cv::fisheye::calibrate(vector_Mat objectPoints, vector_Mat imagePoints, Size image_size, Mat& K, Mat& D, vector_Mat& rvecs, vector_Mat& tvecs, int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    /**
     * Performs camera calibration
     *
     * @param objectPoints vector of vectors of calibration pattern points in the calibration pattern
     * coordinate space.
     * @param imagePoints vector of vectors of the projections of calibration pattern points.
     * imagePoints.size() and objectPoints.size() and imagePoints[i].size() must be equal to
     * objectPoints[i].size() for each i.
     * @param image_size Size of the image used only to initialize the camera intrinsic matrix.
     * @param K Output 3x3 floating-point camera intrinsic matrix
     * \(\cameramatrix{A}\) . If
     * REF: fisheye::CALIB_USE_INTRINSIC_GUESS is specified, some or all of fx, fy, cx, cy must be
     * initialized before calling the function.
     * @param D Output vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param rvecs Output vector of rotation vectors (see Rodrigues ) estimated for each pattern view.
     * That is, each k-th rotation vector together with the corresponding k-th translation vector (see
     * the next output parameter description) brings the calibration pattern from the model coordinate
     * space (in which object points are specified) to the world coordinate space, that is, a real
     * position of the calibration pattern in the k-th pattern view (k=0.. *M* -1).
     * @param tvecs Output vector of translation vectors estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: fisheye::CALIB_USE_INTRINSIC_GUESS  cameraMatrix contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center ( imageSize is used), and focal distances are computed in a least-squares fashion.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_RECOMPUTE_EXTRINSIC  Extrinsic will be recomputed after each iteration
     * of intrinsic optimization.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_CHECK_COND  The functions will check validity of condition number.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_SKEW  Skew coefficient (alpha) is set to zero and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_K1,..., REF: fisheye::CALIB_FIX_K4 Selected distortion coefficients
     * are set to zeros and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_PRINCIPAL_POINT  The principal point is not changed during the global
     * optimization. It stays at the center or at a different location specified when REF: fisheye::CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_FOCAL_LENGTH The focal length is not changed during the global
     * optimization. It is the \(max(width,height)/\pi\) or the provided \(f_x\), \(f_y\) when REF: fisheye::CALIB_USE_INTRINSIC_GUESS is set too.
     * @param criteria Termination criteria for the iterative optimization algorithm.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static double fisheye_calibrate(List<Mat> objectPoints, List<Mat> imagePoints, Size image_size, Mat K, Mat D, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = fisheye_calibrate_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, image_size.width, image_size.height, K.nativeObj, D.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Performs camera calibration
     *
     * @param objectPoints vector of vectors of calibration pattern points in the calibration pattern
     * coordinate space.
     * @param imagePoints vector of vectors of the projections of calibration pattern points.
     * imagePoints.size() and objectPoints.size() and imagePoints[i].size() must be equal to
     * objectPoints[i].size() for each i.
     * @param image_size Size of the image used only to initialize the camera intrinsic matrix.
     * @param K Output 3x3 floating-point camera intrinsic matrix
     * \(\cameramatrix{A}\) . If
     * REF: fisheye::CALIB_USE_INTRINSIC_GUESS is specified, some or all of fx, fy, cx, cy must be
     * initialized before calling the function.
     * @param D Output vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param rvecs Output vector of rotation vectors (see Rodrigues ) estimated for each pattern view.
     * That is, each k-th rotation vector together with the corresponding k-th translation vector (see
     * the next output parameter description) brings the calibration pattern from the model coordinate
     * space (in which object points are specified) to the world coordinate space, that is, a real
     * position of the calibration pattern in the k-th pattern view (k=0.. *M* -1).
     * @param tvecs Output vector of translation vectors estimated for each pattern view.
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: fisheye::CALIB_USE_INTRINSIC_GUESS  cameraMatrix contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center ( imageSize is used), and focal distances are computed in a least-squares fashion.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_RECOMPUTE_EXTRINSIC  Extrinsic will be recomputed after each iteration
     * of intrinsic optimization.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_CHECK_COND  The functions will check validity of condition number.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_SKEW  Skew coefficient (alpha) is set to zero and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_K1,..., REF: fisheye::CALIB_FIX_K4 Selected distortion coefficients
     * are set to zeros and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_PRINCIPAL_POINT  The principal point is not changed during the global
     * optimization. It stays at the center or at a different location specified when REF: fisheye::CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_FOCAL_LENGTH The focal length is not changed during the global
     * optimization. It is the \(max(width,height)/\pi\) or the provided \(f_x\), \(f_y\) when REF: fisheye::CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static double fisheye_calibrate(List<Mat> objectPoints, List<Mat> imagePoints, Size image_size, Mat K, Mat D, List<Mat> rvecs, List<Mat> tvecs, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = fisheye_calibrate_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, image_size.width, image_size.height, K.nativeObj, D.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Performs camera calibration
     *
     * @param objectPoints vector of vectors of calibration pattern points in the calibration pattern
     * coordinate space.
     * @param imagePoints vector of vectors of the projections of calibration pattern points.
     * imagePoints.size() and objectPoints.size() and imagePoints[i].size() must be equal to
     * objectPoints[i].size() for each i.
     * @param image_size Size of the image used only to initialize the camera intrinsic matrix.
     * @param K Output 3x3 floating-point camera intrinsic matrix
     * \(\cameramatrix{A}\) . If
     * REF: fisheye::CALIB_USE_INTRINSIC_GUESS is specified, some or all of fx, fy, cx, cy must be
     * initialized before calling the function.
     * @param D Output vector of distortion coefficients \(\distcoeffsfisheye\).
     * @param rvecs Output vector of rotation vectors (see Rodrigues ) estimated for each pattern view.
     * That is, each k-th rotation vector together with the corresponding k-th translation vector (see
     * the next output parameter description) brings the calibration pattern from the model coordinate
     * space (in which object points are specified) to the world coordinate space, that is, a real
     * position of the calibration pattern in the k-th pattern view (k=0.. *M* -1).
     * @param tvecs Output vector of translation vectors estimated for each pattern view.
     * <ul>
     *   <li>
     *    REF: fisheye::CALIB_USE_INTRINSIC_GUESS  cameraMatrix contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center ( imageSize is used), and focal distances are computed in a least-squares fashion.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_RECOMPUTE_EXTRINSIC  Extrinsic will be recomputed after each iteration
     * of intrinsic optimization.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_CHECK_COND  The functions will check validity of condition number.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_SKEW  Skew coefficient (alpha) is set to zero and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_K1,..., REF: fisheye::CALIB_FIX_K4 Selected distortion coefficients
     * are set to zeros and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_PRINCIPAL_POINT  The principal point is not changed during the global
     * optimization. It stays at the center or at a different location specified when REF: fisheye::CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_FOCAL_LENGTH The focal length is not changed during the global
     * optimization. It is the \(max(width,height)/\pi\) or the provided \(f_x\), \(f_y\) when REF: fisheye::CALIB_USE_INTRINSIC_GUESS is set too.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static double fisheye_calibrate(List<Mat> objectPoints, List<Mat> imagePoints, Size image_size, Mat K, Mat D, List<Mat> rvecs, List<Mat> tvecs) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints_mat = Converters.vector_Mat_to_Mat(imagePoints);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = fisheye_calibrate_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, image_size.width, image_size.height, K.nativeObj, D.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  void cv::fisheye::stereoRectify(Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat tvec, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, int flags, Size newImageSize = Size(), double balance = 0.0, double fov_scale = 1.0)
    //

    /**
     * Stereo rectification for fisheye camera model
     *
     * @param K1 First camera intrinsic matrix.
     * @param D1 First camera distortion parameters.
     * @param K2 Second camera intrinsic matrix.
     * @param D2 Second camera distortion parameters.
     * @param imageSize Size of the image used for stereo calibration.
     * @param R Rotation matrix between the coordinate systems of the first and the second
     * cameras.
     * @param tvec Translation vector between coordinate systems of the cameras.
     * @param R1 Output 3x3 rectification transform (rotation matrix) for the first camera.
     * @param R2 Output 3x3 rectification transform (rotation matrix) for the second camera.
     * @param P1 Output 3x4 projection matrix in the new (rectified) coordinate systems for the first
     * camera.
     * @param P2 Output 3x4 projection matrix in the new (rectified) coordinate systems for the second
     * camera.
     * @param Q Output \(4 \times 4\) disparity-to-depth mapping matrix (see reprojectImageTo3D ).
     * @param flags Operation flags that may be zero or REF: fisheye::CALIB_ZERO_DISPARITY . If the flag is set,
     * the function makes the principal points of each camera have the same pixel coordinates in the
     * rectified views. And if the flag is not set, the function may still shift the images in the
     * horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the
     * useful image area.
     * @param newImageSize New image resolution after rectification. The same size should be passed to
     * #initUndistortRectifyMap (see the stereo_calib.cpp sample in OpenCV samples directory). When (0,0)
     * is passed (default), it is set to the original imageSize . Setting it to larger value can help you
     * preserve details in the original image, especially when there is a big radial distortion.
     * @param balance Sets the new focal length in range between the min focal length and the max focal
     * length. Balance is in range of [0, 1].
     * @param fov_scale Divisor for new focal length.
     */
    public static void fisheye_stereoRectify(Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat tvec, Mat R1, Mat R2, Mat P1, Mat P2, Mat Q, int flags, Size newImageSize, double balance, double fov_scale) {
        fisheye_stereoRectify_0(K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, tvec.nativeObj, R1.nativeObj, R2.nativeObj, P1.nativeObj, P2.nativeObj, Q.nativeObj, flags, newImageSize.width, newImageSize.height, balance, fov_scale);
    }

    /**
     * Stereo rectification for fisheye camera model
     *
     * @param K1 First camera intrinsic matrix.
     * @param D1 First camera distortion parameters.
     * @param K2 Second camera intrinsic matrix.
     * @param D2 Second camera distortion parameters.
     * @param imageSize Size of the image used for stereo calibration.
     * @param R Rotation matrix between the coordinate systems of the first and the second
     * cameras.
     * @param tvec Translation vector between coordinate systems of the cameras.
     * @param R1 Output 3x3 rectification transform (rotation matrix) for the first camera.
     * @param R2 Output 3x3 rectification transform (rotation matrix) for the second camera.
     * @param P1 Output 3x4 projection matrix in the new (rectified) coordinate systems for the first
     * camera.
     * @param P2 Output 3x4 projection matrix in the new (rectified) coordinate systems for the second
     * camera.
     * @param Q Output \(4 \times 4\) disparity-to-depth mapping matrix (see reprojectImageTo3D ).
     * @param flags Operation flags that may be zero or REF: fisheye::CALIB_ZERO_DISPARITY . If the flag is set,
     * the function makes the principal points of each camera have the same pixel coordinates in the
     * rectified views. And if the flag is not set, the function may still shift the images in the
     * horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the
     * useful image area.
     * @param newImageSize New image resolution after rectification. The same size should be passed to
     * #initUndistortRectifyMap (see the stereo_calib.cpp sample in OpenCV samples directory). When (0,0)
     * is passed (default), it is set to the original imageSize . Setting it to larger value can help you
     * preserve details in the original image, especially when there is a big radial distortion.
     * @param balance Sets the new focal length in range between the min focal length and the max focal
     * length. Balance is in range of [0, 1].
     */
    public static void fisheye_stereoRectify(Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat tvec, Mat R1, Mat R2, Mat P1, Mat P2, Mat Q, int flags, Size newImageSize, double balance) {
        fisheye_stereoRectify_1(K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, tvec.nativeObj, R1.nativeObj, R2.nativeObj, P1.nativeObj, P2.nativeObj, Q.nativeObj, flags, newImageSize.width, newImageSize.height, balance);
    }

    /**
     * Stereo rectification for fisheye camera model
     *
     * @param K1 First camera intrinsic matrix.
     * @param D1 First camera distortion parameters.
     * @param K2 Second camera intrinsic matrix.
     * @param D2 Second camera distortion parameters.
     * @param imageSize Size of the image used for stereo calibration.
     * @param R Rotation matrix between the coordinate systems of the first and the second
     * cameras.
     * @param tvec Translation vector between coordinate systems of the cameras.
     * @param R1 Output 3x3 rectification transform (rotation matrix) for the first camera.
     * @param R2 Output 3x3 rectification transform (rotation matrix) for the second camera.
     * @param P1 Output 3x4 projection matrix in the new (rectified) coordinate systems for the first
     * camera.
     * @param P2 Output 3x4 projection matrix in the new (rectified) coordinate systems for the second
     * camera.
     * @param Q Output \(4 \times 4\) disparity-to-depth mapping matrix (see reprojectImageTo3D ).
     * @param flags Operation flags that may be zero or REF: fisheye::CALIB_ZERO_DISPARITY . If the flag is set,
     * the function makes the principal points of each camera have the same pixel coordinates in the
     * rectified views. And if the flag is not set, the function may still shift the images in the
     * horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the
     * useful image area.
     * @param newImageSize New image resolution after rectification. The same size should be passed to
     * #initUndistortRectifyMap (see the stereo_calib.cpp sample in OpenCV samples directory). When (0,0)
     * is passed (default), it is set to the original imageSize . Setting it to larger value can help you
     * preserve details in the original image, especially when there is a big radial distortion.
     * length. Balance is in range of [0, 1].
     */
    public static void fisheye_stereoRectify(Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat tvec, Mat R1, Mat R2, Mat P1, Mat P2, Mat Q, int flags, Size newImageSize) {
        fisheye_stereoRectify_2(K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, tvec.nativeObj, R1.nativeObj, R2.nativeObj, P1.nativeObj, P2.nativeObj, Q.nativeObj, flags, newImageSize.width, newImageSize.height);
    }

    /**
     * Stereo rectification for fisheye camera model
     *
     * @param K1 First camera intrinsic matrix.
     * @param D1 First camera distortion parameters.
     * @param K2 Second camera intrinsic matrix.
     * @param D2 Second camera distortion parameters.
     * @param imageSize Size of the image used for stereo calibration.
     * @param R Rotation matrix between the coordinate systems of the first and the second
     * cameras.
     * @param tvec Translation vector between coordinate systems of the cameras.
     * @param R1 Output 3x3 rectification transform (rotation matrix) for the first camera.
     * @param R2 Output 3x3 rectification transform (rotation matrix) for the second camera.
     * @param P1 Output 3x4 projection matrix in the new (rectified) coordinate systems for the first
     * camera.
     * @param P2 Output 3x4 projection matrix in the new (rectified) coordinate systems for the second
     * camera.
     * @param Q Output \(4 \times 4\) disparity-to-depth mapping matrix (see reprojectImageTo3D ).
     * @param flags Operation flags that may be zero or REF: fisheye::CALIB_ZERO_DISPARITY . If the flag is set,
     * the function makes the principal points of each camera have the same pixel coordinates in the
     * rectified views. And if the flag is not set, the function may still shift the images in the
     * horizontal or vertical direction (depending on the orientation of epipolar lines) to maximize the
     * useful image area.
     * #initUndistortRectifyMap (see the stereo_calib.cpp sample in OpenCV samples directory). When (0,0)
     * is passed (default), it is set to the original imageSize . Setting it to larger value can help you
     * preserve details in the original image, especially when there is a big radial distortion.
     * length. Balance is in range of [0, 1].
     */
    public static void fisheye_stereoRectify(Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat tvec, Mat R1, Mat R2, Mat P1, Mat P2, Mat Q, int flags) {
        fisheye_stereoRectify_3(K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, tvec.nativeObj, R1.nativeObj, R2.nativeObj, P1.nativeObj, P2.nativeObj, Q.nativeObj, flags);
    }


    //
    // C++:  double cv::fisheye::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& K1, Mat& D1, Mat& K2, Mat& D2, Size imageSize, Mat& R, Mat& T, vector_Mat& rvecs, vector_Mat& tvecs, int flags = fisheye::CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    /**
     * Performs stereo calibration
     *
     * @param objectPoints Vector of vectors of the calibration pattern points.
     * @param imagePoints1 Vector of vectors of the projections of the calibration pattern points,
     * observed by the first camera.
     * @param imagePoints2 Vector of vectors of the projections of the calibration pattern points,
     * observed by the second camera.
     * @param K1 Input/output first camera intrinsic matrix:
     * \(\vecthreethree{f_x^{(j)}}{0}{c_x^{(j)}}{0}{f_y^{(j)}}{c_y^{(j)}}{0}{0}{1}\) , \(j = 0,\, 1\) . If
     * any of REF: fisheye::CALIB_USE_INTRINSIC_GUESS , REF: fisheye::CALIB_FIX_INTRINSIC are specified,
     * some or all of the matrix components must be initialized.
     * @param D1 Input/output vector of distortion coefficients \(\distcoeffsfisheye\) of 4 elements.
     * @param K2 Input/output second camera intrinsic matrix. The parameter is similar to K1 .
     * @param D2 Input/output lens distortion coefficients for the second camera. The parameter is
     * similar to D1 .
     * @param imageSize Size of the image used only to initialize camera intrinsic matrix.
     * @param R Output rotation matrix between the 1st and the 2nd camera coordinate systems.
     * @param T Output translation vector between the coordinate systems of the cameras.
     * @param rvecs Output vector of rotation vectors ( REF: Rodrigues ) estimated for each pattern view in the
     * coordinate system of the first camera of the stereo pair (e.g. std::vector&lt;cv::Mat&gt;). More in detail, each
     * i-th rotation vector together with the corresponding i-th translation vector (see the next output parameter
     * description) brings the calibration pattern from the object coordinate space (in which object points are
     * specified) to the camera coordinate space of the first camera of the stereo pair. In more technical terms,
     * the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space
     * to camera coordinate space of the first camera of the stereo pair.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter description
     * of previous output parameter ( rvecs ).
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: fisheye::CALIB_FIX_INTRINSIC  Fix K1, K2? and D1, D2? so that only R, T matrices
     * are estimated.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_USE_INTRINSIC_GUESS  K1, K2 contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center (imageSize is used), and focal distances are computed in a least-squares fashion.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_RECOMPUTE_EXTRINSIC  Extrinsic will be recomputed after each iteration
     * of intrinsic optimization.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_CHECK_COND  The functions will check validity of condition number.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_SKEW  Skew coefficient (alpha) is set to zero and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_K1,..., REF: fisheye::CALIB_FIX_K4 Selected distortion coefficients are set to zeros and stay
     * zero.
     * @param criteria Termination criteria for the iterative optimization algorithm.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static double fisheye_stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat T, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = fisheye_stereoCalibrate_0(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Performs stereo calibration
     *
     * @param objectPoints Vector of vectors of the calibration pattern points.
     * @param imagePoints1 Vector of vectors of the projections of the calibration pattern points,
     * observed by the first camera.
     * @param imagePoints2 Vector of vectors of the projections of the calibration pattern points,
     * observed by the second camera.
     * @param K1 Input/output first camera intrinsic matrix:
     * \(\vecthreethree{f_x^{(j)}}{0}{c_x^{(j)}}{0}{f_y^{(j)}}{c_y^{(j)}}{0}{0}{1}\) , \(j = 0,\, 1\) . If
     * any of REF: fisheye::CALIB_USE_INTRINSIC_GUESS , REF: fisheye::CALIB_FIX_INTRINSIC are specified,
     * some or all of the matrix components must be initialized.
     * @param D1 Input/output vector of distortion coefficients \(\distcoeffsfisheye\) of 4 elements.
     * @param K2 Input/output second camera intrinsic matrix. The parameter is similar to K1 .
     * @param D2 Input/output lens distortion coefficients for the second camera. The parameter is
     * similar to D1 .
     * @param imageSize Size of the image used only to initialize camera intrinsic matrix.
     * @param R Output rotation matrix between the 1st and the 2nd camera coordinate systems.
     * @param T Output translation vector between the coordinate systems of the cameras.
     * @param rvecs Output vector of rotation vectors ( REF: Rodrigues ) estimated for each pattern view in the
     * coordinate system of the first camera of the stereo pair (e.g. std::vector&lt;cv::Mat&gt;). More in detail, each
     * i-th rotation vector together with the corresponding i-th translation vector (see the next output parameter
     * description) brings the calibration pattern from the object coordinate space (in which object points are
     * specified) to the camera coordinate space of the first camera of the stereo pair. In more technical terms,
     * the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space
     * to camera coordinate space of the first camera of the stereo pair.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter description
     * of previous output parameter ( rvecs ).
     * @param flags Different flags that may be zero or a combination of the following values:
     * <ul>
     *   <li>
     *    REF: fisheye::CALIB_FIX_INTRINSIC  Fix K1, K2? and D1, D2? so that only R, T matrices
     * are estimated.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_USE_INTRINSIC_GUESS  K1, K2 contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center (imageSize is used), and focal distances are computed in a least-squares fashion.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_RECOMPUTE_EXTRINSIC  Extrinsic will be recomputed after each iteration
     * of intrinsic optimization.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_CHECK_COND  The functions will check validity of condition number.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_SKEW  Skew coefficient (alpha) is set to zero and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_K1,..., REF: fisheye::CALIB_FIX_K4 Selected distortion coefficients are set to zeros and stay
     * zero.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static double fisheye_stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat T, List<Mat> rvecs, List<Mat> tvecs, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = fisheye_stereoCalibrate_1(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Performs stereo calibration
     *
     * @param objectPoints Vector of vectors of the calibration pattern points.
     * @param imagePoints1 Vector of vectors of the projections of the calibration pattern points,
     * observed by the first camera.
     * @param imagePoints2 Vector of vectors of the projections of the calibration pattern points,
     * observed by the second camera.
     * @param K1 Input/output first camera intrinsic matrix:
     * \(\vecthreethree{f_x^{(j)}}{0}{c_x^{(j)}}{0}{f_y^{(j)}}{c_y^{(j)}}{0}{0}{1}\) , \(j = 0,\, 1\) . If
     * any of REF: fisheye::CALIB_USE_INTRINSIC_GUESS , REF: fisheye::CALIB_FIX_INTRINSIC are specified,
     * some or all of the matrix components must be initialized.
     * @param D1 Input/output vector of distortion coefficients \(\distcoeffsfisheye\) of 4 elements.
     * @param K2 Input/output second camera intrinsic matrix. The parameter is similar to K1 .
     * @param D2 Input/output lens distortion coefficients for the second camera. The parameter is
     * similar to D1 .
     * @param imageSize Size of the image used only to initialize camera intrinsic matrix.
     * @param R Output rotation matrix between the 1st and the 2nd camera coordinate systems.
     * @param T Output translation vector between the coordinate systems of the cameras.
     * @param rvecs Output vector of rotation vectors ( REF: Rodrigues ) estimated for each pattern view in the
     * coordinate system of the first camera of the stereo pair (e.g. std::vector&lt;cv::Mat&gt;). More in detail, each
     * i-th rotation vector together with the corresponding i-th translation vector (see the next output parameter
     * description) brings the calibration pattern from the object coordinate space (in which object points are
     * specified) to the camera coordinate space of the first camera of the stereo pair. In more technical terms,
     * the tuple of the i-th rotation and translation vector performs a change of basis from object coordinate space
     * to camera coordinate space of the first camera of the stereo pair.
     * @param tvecs Output vector of translation vectors estimated for each pattern view, see parameter description
     * of previous output parameter ( rvecs ).
     * <ul>
     *   <li>
     *    REF: fisheye::CALIB_FIX_INTRINSIC  Fix K1, K2? and D1, D2? so that only R, T matrices
     * are estimated.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_USE_INTRINSIC_GUESS  K1, K2 contains valid initial values of
     * fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image
     * center (imageSize is used), and focal distances are computed in a least-squares fashion.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_RECOMPUTE_EXTRINSIC  Extrinsic will be recomputed after each iteration
     * of intrinsic optimization.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_CHECK_COND  The functions will check validity of condition number.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_SKEW  Skew coefficient (alpha) is set to zero and stay zero.
     *   </li>
     *   <li>
     *    REF: fisheye::CALIB_FIX_K1,..., REF: fisheye::CALIB_FIX_K4 Selected distortion coefficients are set to zeros and stay
     * zero.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static double fisheye_stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat T, List<Mat> rvecs, List<Mat> tvecs) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = fisheye_stereoCalibrate_2(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double cv::fisheye::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& K1, Mat& D1, Mat& K2, Mat& D2, Size imageSize, Mat& R, Mat& T, int flags = fisheye::CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    //

    public static double fisheye_stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat T, int flags, TermCriteria criteria) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return fisheye_stereoCalibrate_3(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    public static double fisheye_stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat T, int flags) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return fisheye_stereoCalibrate_4(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj, flags);
    }

    public static double fisheye_stereoCalibrate(List<Mat> objectPoints, List<Mat> imagePoints1, List<Mat> imagePoints2, Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat T) {
        Mat objectPoints_mat = Converters.vector_Mat_to_Mat(objectPoints);
        Mat imagePoints1_mat = Converters.vector_Mat_to_Mat(imagePoints1);
        Mat imagePoints2_mat = Converters.vector_Mat_to_Mat(imagePoints2);
        return fisheye_stereoCalibrate_5(objectPoints_mat.nativeObj, imagePoints1_mat.nativeObj, imagePoints2_mat.nativeObj, K1.nativeObj, D1.nativeObj, K2.nativeObj, D2.nativeObj, imageSize.width, imageSize.height, R.nativeObj, T.nativeObj);
    }




    // C++:  Mat cv::initCameraMatrix2D(vector_vector_Point3f objectPoints, vector_vector_Point2f imagePoints, Size imageSize, double aspectRatio = 1.0)
    private static native long initCameraMatrix2D_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, double aspectRatio);
    private static native long initCameraMatrix2D_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height);

    // C++:  bool cv::findChessboardCorners(Mat image, Size patternSize, vector_Point2f& corners, int flags = CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE)
    private static native boolean findChessboardCorners_0(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_mat_nativeObj, int flags);
    private static native boolean findChessboardCorners_1(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_mat_nativeObj);

    // C++:  bool cv::checkChessboard(Mat img, Size size)
    private static native boolean checkChessboard_0(long img_nativeObj, double size_width, double size_height);

    // C++:  bool cv::findChessboardCornersSB(Mat image, Size patternSize, Mat& corners, int flags, Mat& meta)
    private static native boolean findChessboardCornersSBWithMeta_0(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj, int flags, long meta_nativeObj);

    // C++:  bool cv::findChessboardCornersSB(Mat image, Size patternSize, Mat& corners, int flags = 0)
    private static native boolean findChessboardCornersSB_0(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj, int flags);
    private static native boolean findChessboardCornersSB_1(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj);

    // C++:  Scalar cv::estimateChessboardSharpness(Mat image, Size patternSize, Mat corners, float rise_distance = 0.8F, bool vertical = false, Mat& sharpness = Mat())
    private static native double[] estimateChessboardSharpness_0(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj, float rise_distance, boolean vertical, long sharpness_nativeObj);
    private static native double[] estimateChessboardSharpness_1(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj, float rise_distance, boolean vertical);
    private static native double[] estimateChessboardSharpness_2(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj, float rise_distance);
    private static native double[] estimateChessboardSharpness_3(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_nativeObj);

    // C++:  bool cv::find4QuadCornerSubpix(Mat img, Mat& corners, Size region_size)
    private static native boolean find4QuadCornerSubpix_0(long img_nativeObj, long corners_nativeObj, double region_size_width, double region_size_height);

    // C++:  void cv::drawChessboardCorners(Mat& image, Size patternSize, vector_Point2f corners, bool patternWasFound)
    private static native void drawChessboardCorners_0(long image_nativeObj, double patternSize_width, double patternSize_height, long corners_mat_nativeObj, boolean patternWasFound);

    // C++:  bool cv::findCirclesGrid(Mat image, Size patternSize, Mat& centers, int flags = CALIB_CB_SYMMETRIC_GRID, Ptr_FeatureDetector blobDetector = SimpleBlobDetector::create())
    private static native boolean findCirclesGrid_0(long image_nativeObj, double patternSize_width, double patternSize_height, long centers_nativeObj, int flags);
    private static native boolean findCirclesGrid_2(long image_nativeObj, double patternSize_width, double patternSize_height, long centers_nativeObj);

    // C++:  double cv::calibrateCamera(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double calibrateCameraExtended_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraExtended_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj, int flags);
    private static native double calibrateCameraExtended_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj);

    // C++:  double cv::calibrateCamera(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double calibrateCamera_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCamera_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags);
    private static native double calibrateCamera_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj);

    // C++:  double cv::calibrateCameraRO(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, int iFixedPoint, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& newObjPoints, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& stdDeviationsObjPoints, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double calibrateCameraROExtended_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, int iFixedPoint, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long newObjPoints_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long stdDeviationsObjPoints_nativeObj, long perViewErrors_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraROExtended_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, int iFixedPoint, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long newObjPoints_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long stdDeviationsObjPoints_nativeObj, long perViewErrors_nativeObj, int flags);
    private static native double calibrateCameraROExtended_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, int iFixedPoint, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long newObjPoints_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long stdDeviationsObjPoints_nativeObj, long perViewErrors_nativeObj);

    // C++:  double cv::calibrateCameraRO(vector_Mat objectPoints, vector_Mat imagePoints, Size imageSize, int iFixedPoint, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& newObjPoints, int flags = 0, TermCriteria criteria = TermCriteria( TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double calibrateCameraRO_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, int iFixedPoint, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long newObjPoints_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraRO_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, int iFixedPoint, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long newObjPoints_nativeObj, int flags);
    private static native double calibrateCameraRO_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double imageSize_width, double imageSize_height, int iFixedPoint, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long newObjPoints_nativeObj);

    // C++:  void cv::calibrationMatrixValues(Mat cameraMatrix, Size imageSize, double apertureWidth, double apertureHeight, double& fovx, double& fovy, double& focalLength, Point2d& principalPoint, double& aspectRatio)
    private static native void calibrationMatrixValues_0(long cameraMatrix_nativeObj, double imageSize_width, double imageSize_height, double apertureWidth, double apertureHeight, double[] fovx_out, double[] fovy_out, double[] focalLength_out, double[] principalPoint_out, double[] aspectRatio_out);

    // C++:  double cv::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size imageSize, Mat& R, Mat& T, Mat& E, Mat& F, vector_Mat& rvecs, vector_Mat& tvecs, Mat& perViewErrors, int flags = CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6))
    private static native double stereoCalibrateExtended_0(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long perViewErrors_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double stereoCalibrateExtended_1(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long perViewErrors_nativeObj, int flags);
    private static native double stereoCalibrateExtended_2(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long perViewErrors_nativeObj);

    // C++:  double cv::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size imageSize, Mat& R, Mat& T, Mat& E, Mat& F, int flags = CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-6))
    private static native double stereoCalibrate_0(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double stereoCalibrate_1(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, int flags);
    private static native double stereoCalibrate_2(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj);

    // C++:  double cv::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& cameraMatrix1, Mat& distCoeffs1, Mat& cameraMatrix2, Mat& distCoeffs2, Size imageSize, Mat& R, Mat& T, Mat& E, Mat& F, Mat& perViewErrors, int flags = CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 1e-6))
    private static native double stereoCalibrate_3(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, long perViewErrors_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double stereoCalibrate_4(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, long perViewErrors_nativeObj, int flags);
    private static native double stereoCalibrate_5(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long E_nativeObj, long F_nativeObj, long perViewErrors_nativeObj);

    // C++:  void cv::fisheye::projectPoints(Mat objectPoints, Mat& imagePoints, Mat rvec, Mat tvec, Mat K, Mat D, double alpha = 0, Mat& jacobian = Mat())
    private static native void fisheye_projectPoints_0(long objectPoints_nativeObj, long imagePoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long K_nativeObj, long D_nativeObj, double alpha, long jacobian_nativeObj);
    private static native void fisheye_projectPoints_1(long objectPoints_nativeObj, long imagePoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long K_nativeObj, long D_nativeObj, double alpha);
    private static native void fisheye_projectPoints_2(long objectPoints_nativeObj, long imagePoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long K_nativeObj, long D_nativeObj);

    // C++:  void cv::fisheye::distortPoints(Mat undistorted, Mat& distorted, Mat K, Mat D, double alpha = 0)
    private static native void fisheye_distortPoints_0(long undistorted_nativeObj, long distorted_nativeObj, long K_nativeObj, long D_nativeObj, double alpha);
    private static native void fisheye_distortPoints_1(long undistorted_nativeObj, long distorted_nativeObj, long K_nativeObj, long D_nativeObj);

    // C++:  void cv::fisheye::undistortPoints(Mat distorted, Mat& undistorted, Mat K, Mat D, Mat R = Mat(), Mat P = Mat(), TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 10, 1e-8))
    private static native void fisheye_undistortPoints_0(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj, long R_nativeObj, long P_nativeObj, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native void fisheye_undistortPoints_1(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj, long R_nativeObj, long P_nativeObj);
    private static native void fisheye_undistortPoints_2(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj, long R_nativeObj);
    private static native void fisheye_undistortPoints_3(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj);

    // C++:  void cv::fisheye::initUndistortRectifyMap(Mat K, Mat D, Mat R, Mat P, Size size, int m1type, Mat& map1, Mat& map2)
    private static native void fisheye_initUndistortRectifyMap_0(long K_nativeObj, long D_nativeObj, long R_nativeObj, long P_nativeObj, double size_width, double size_height, int m1type, long map1_nativeObj, long map2_nativeObj);

    // C++:  void cv::fisheye::undistortImage(Mat distorted, Mat& undistorted, Mat K, Mat D, Mat Knew = cv::Mat(), Size new_size = Size())
    private static native void fisheye_undistortImage_0(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj, long Knew_nativeObj, double new_size_width, double new_size_height);
    private static native void fisheye_undistortImage_1(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj, long Knew_nativeObj);
    private static native void fisheye_undistortImage_2(long distorted_nativeObj, long undistorted_nativeObj, long K_nativeObj, long D_nativeObj);

    // C++:  void cv::fisheye::estimateNewCameraMatrixForUndistortRectify(Mat K, Mat D, Size image_size, Mat R, Mat& P, double balance = 0.0, Size new_size = Size(), double fov_scale = 1.0)
    private static native void fisheye_estimateNewCameraMatrixForUndistortRectify_0(long K_nativeObj, long D_nativeObj, double image_size_width, double image_size_height, long R_nativeObj, long P_nativeObj, double balance, double new_size_width, double new_size_height, double fov_scale);
    private static native void fisheye_estimateNewCameraMatrixForUndistortRectify_1(long K_nativeObj, long D_nativeObj, double image_size_width, double image_size_height, long R_nativeObj, long P_nativeObj, double balance, double new_size_width, double new_size_height);
    private static native void fisheye_estimateNewCameraMatrixForUndistortRectify_2(long K_nativeObj, long D_nativeObj, double image_size_width, double image_size_height, long R_nativeObj, long P_nativeObj, double balance);
    private static native void fisheye_estimateNewCameraMatrixForUndistortRectify_3(long K_nativeObj, long D_nativeObj, double image_size_width, double image_size_height, long R_nativeObj, long P_nativeObj);

    // C++:  double cv::fisheye::calibrate(vector_Mat objectPoints, vector_Mat imagePoints, Size image_size, Mat& K, Mat& D, vector_Mat& rvecs, vector_Mat& tvecs, int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double fisheye_calibrate_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double image_size_width, double image_size_height, long K_nativeObj, long D_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double fisheye_calibrate_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double image_size_width, double image_size_height, long K_nativeObj, long D_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags);
    private static native double fisheye_calibrate_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, double image_size_width, double image_size_height, long K_nativeObj, long D_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj);

    // C++:  void cv::fisheye::stereoRectify(Mat K1, Mat D1, Mat K2, Mat D2, Size imageSize, Mat R, Mat tvec, Mat& R1, Mat& R2, Mat& P1, Mat& P2, Mat& Q, int flags, Size newImageSize = Size(), double balance = 0.0, double fov_scale = 1.0)
    private static native void fisheye_stereoRectify_0(long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long tvec_nativeObj, long R1_nativeObj, long R2_nativeObj, long P1_nativeObj, long P2_nativeObj, long Q_nativeObj, int flags, double newImageSize_width, double newImageSize_height, double balance, double fov_scale);
    private static native void fisheye_stereoRectify_1(long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long tvec_nativeObj, long R1_nativeObj, long R2_nativeObj, long P1_nativeObj, long P2_nativeObj, long Q_nativeObj, int flags, double newImageSize_width, double newImageSize_height, double balance);
    private static native void fisheye_stereoRectify_2(long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long tvec_nativeObj, long R1_nativeObj, long R2_nativeObj, long P1_nativeObj, long P2_nativeObj, long Q_nativeObj, int flags, double newImageSize_width, double newImageSize_height);
    private static native void fisheye_stereoRectify_3(long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long tvec_nativeObj, long R1_nativeObj, long R2_nativeObj, long P1_nativeObj, long P2_nativeObj, long Q_nativeObj, int flags);

    // C++:  double cv::fisheye::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& K1, Mat& D1, Mat& K2, Mat& D2, Size imageSize, Mat& R, Mat& T, vector_Mat& rvecs, vector_Mat& tvecs, int flags = fisheye::CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double fisheye_stereoCalibrate_0(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double fisheye_stereoCalibrate_1(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags);
    private static native double fisheye_stereoCalibrate_2(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj);

    // C++:  double cv::fisheye::stereoCalibrate(vector_Mat objectPoints, vector_Mat imagePoints1, vector_Mat imagePoints2, Mat& K1, Mat& D1, Mat& K2, Mat& D2, Size imageSize, Mat& R, Mat& T, int flags = fisheye::CALIB_FIX_INTRINSIC, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON))
    private static native double fisheye_stereoCalibrate_3(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double fisheye_stereoCalibrate_4(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj, int flags);
    private static native double fisheye_stereoCalibrate_5(long objectPoints_mat_nativeObj, long imagePoints1_mat_nativeObj, long imagePoints2_mat_nativeObj, long K1_nativeObj, long D1_nativeObj, long K2_nativeObj, long D2_nativeObj, double imageSize_width, double imageSize_height, long R_nativeObj, long T_nativeObj);

}
