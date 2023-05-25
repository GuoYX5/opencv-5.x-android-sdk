//
// This file is auto-generated. Please don't modify it!
//
package org.opencv.cv3d;

import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.utils.Converters;

// C++: class 3d

public class Cv3d {

    // C++: enum <unnamed>
    public static final int
            LMEDS = 4,
            RANSAC = 8,
            RHO = 16,
            USAC_DEFAULT = 32,
            USAC_PARALLEL = 33,
            USAC_FM_8PTS = 34,
            USAC_FAST = 35,
            USAC_ACCURATE = 36,
            USAC_PROSAC = 37,
            USAC_MAGSAC = 38,
            FM_7POINT = 1,
            FM_8POINT = 2,
            FM_LMEDS = 4,
            FM_RANSAC = 8;


    // C++: enum LocalOptimMethod (cv.LocalOptimMethod)
    public static final int
            LOCAL_OPTIM_NULL = 0,
            LOCAL_OPTIM_INNER_LO = 1,
            LOCAL_OPTIM_INNER_AND_ITER_LO = 2,
            LOCAL_OPTIM_GC = 3,
            LOCAL_OPTIM_SIGMA = 4;


    // C++: enum MatrixType (cv.MatrixType)
    public static final int
            MatrixType_AUTO = 0,
            MatrixType_DENSE = 1,
            MatrixType_SPARSE = 2;


    // C++: enum NeighborSearchMethod (cv.NeighborSearchMethod)
    public static final int
            NEIGH_FLANN_KNN = 0,
            NEIGH_GRID = 1,
            NEIGH_FLANN_RADIUS = 2;


    // C++: enum OdometryAlgoType (cv.OdometryAlgoType)
    public static final int
            OdometryAlgoType_COMMON = 0,
            OdometryAlgoType_FAST = 1;


    // C++: enum OdometryFramePyramidType (cv.OdometryFramePyramidType)
    public static final int
            PYR_IMAGE = 0,
            PYR_DEPTH = 1,
            PYR_MASK = 2,
            PYR_CLOUD = 3,
            PYR_DIX = 4,
            PYR_DIY = 5,
            PYR_TEXMASK = 6,
            PYR_NORM = 7,
            PYR_NORMMASK = 8,
            N_PYRAMIDS = 8+1;


    // C++: enum OdometryType (cv.OdometryType)
    public static final int
            OdometryType_DEPTH = 0,
            OdometryType_RGB = 1,
            OdometryType_RGB_DEPTH = 2;


    // C++: enum RgbdPlaneMethod (cv.RgbdPlaneMethod)
    public static final int
            RGBD_PLANE_METHOD_DEFAULT = 0;


    // C++: enum SacMethod (cv.SacMethod)
    public static final int
            SAC_METHOD_RANSAC = 0;


    // C++: enum SacModelType (cv.SacModelType)
    public static final int
            SAC_MODEL_PLANE = 0,
            SAC_MODEL_SPHERE = 1;


    // C++: enum SamplingMethod (cv.SamplingMethod)
    public static final int
            SAMPLING_UNIFORM = 0,
            SAMPLING_PROGRESSIVE_NAPSAC = 1,
            SAMPLING_NAPSAC = 2,
            SAMPLING_PROSAC = 3;


    // C++: enum ScoreMethod (cv.ScoreMethod)
    public static final int
            SCORE_METHOD_RANSAC = 0,
            SCORE_METHOD_MSAC = 1,
            SCORE_METHOD_MAGSAC = 2,
            SCORE_METHOD_LMEDS = 3;


    // C++: enum SolvePnPMethod (cv.SolvePnPMethod)
    public static final int
            SOLVEPNP_ITERATIVE = 0,
            SOLVEPNP_EPNP = 1,
            SOLVEPNP_P3P = 2,
            SOLVEPNP_DLS = 3,
            SOLVEPNP_UPNP = 4,
            SOLVEPNP_AP3P = 5,
            SOLVEPNP_IPPE = 6,
            SOLVEPNP_IPPE_SQUARE = 7,
            SOLVEPNP_SQPNP = 8,
            SOLVEPNP_MAX_COUNT = 8+1;


    // C++: enum UndistortTypes (cv.UndistortTypes)
    public static final int
            PROJ_SPHERICAL_ORTHO = 0,
            PROJ_SPHERICAL_EQRECT = 1;


    // C++: enum VariableType (cv.VariableType)
    public static final int
            VariableType_LINEAR = 0,
            VariableType_SO3 = 1,
            VariableType_SE3 = 2;


    // C++: enum VolumeType (cv.VolumeType)
    public static final int
            VolumeType_TSDF = 0,
            VolumeType_HashTSDF = 1,
            VolumeType_ColorTSDF = 2;


    //
    // C++:  void cv::Rodrigues(Mat src, Mat& dst, Mat& jacobian = Mat())
    //

    /**
     * Converts a rotation matrix to a rotation vector or vice versa.
     *
     * @param src Input rotation vector (3x1 or 1x3) or rotation matrix (3x3).
     * @param dst Output rotation matrix (3x3) or rotation vector (3x1 or 1x3), respectively.
     * @param jacobian Optional output Jacobian matrix, 3x9 or 9x3, which is a matrix of partial
     * derivatives of the output array components with respect to the input array components.
     *
     * \(\begin{array}{l} \theta \leftarrow norm(r) \\ r  \leftarrow r/ \theta \\ R =  \cos(\theta) I + (1- \cos{\theta} ) r r^T +  \sin(\theta) \vecthreethree{0}{-r_z}{r_y}{r_z}{0}{-r_x}{-r_y}{r_x}{0} \end{array}\)
     *
     * Inverse transformation can be also done easily, since
     *
     * \(\sin ( \theta ) \vecthreethree{0}{-r_z}{r_y}{r_z}{0}{-r_x}{-r_y}{r_x}{0} = \frac{R - R^T}{2}\)
     *
     * A rotation vector is a convenient and most compact representation of a rotation matrix (since any
     * rotation matrix has just 3 degrees of freedom). The representation is used in the global 3D geometry
     * optimization procedures like REF: calibrateCamera, REF: stereoCalibrate, or REF: solvePnP .
     *
     * <b>Note:</b> More information about the computation of the derivative of a 3D rotation matrix with respect to its exponential coordinate
     * can be found in:
     * <ul>
     *   <li>
     *      A Compact Formula for the Derivative of a 3-D Rotation in Exponential Coordinates, Guillermo Gallego, Anthony J. Yezzi CITE: Gallego2014ACF
     *   </li>
     * </ul>
     *
     * <b>Note:</b> Useful information on SE(3) and Lie Groups can be found in:
     * <ul>
     *   <li>
     *      A tutorial on SE(3) transformation parameterizations and on-manifold optimization, Jose-Luis Blanco CITE: blanco2010tutorial
     *   </li>
     *   <li>
     *      Lie Groups for 2D and 3D Transformation, Ethan Eade CITE: Eade17
     *   </li>
     *   <li>
     *      A micro Lie theory for state estimation in robotics, Joan Solà, Jérémie Deray, Dinesh Atchuthan CITE: Sol2018AML
     *   </li>
     * </ul>
     */
    public static void Rodrigues(Mat src, Mat dst, Mat jacobian) {
        Rodrigues_0(src.nativeObj, dst.nativeObj, jacobian.nativeObj);
    }

    /**
     * Converts a rotation matrix to a rotation vector or vice versa.
     *
     * @param src Input rotation vector (3x1 or 1x3) or rotation matrix (3x3).
     * @param dst Output rotation matrix (3x3) or rotation vector (3x1 or 1x3), respectively.
     * derivatives of the output array components with respect to the input array components.
     *
     * \(\begin{array}{l} \theta \leftarrow norm(r) \\ r  \leftarrow r/ \theta \\ R =  \cos(\theta) I + (1- \cos{\theta} ) r r^T +  \sin(\theta) \vecthreethree{0}{-r_z}{r_y}{r_z}{0}{-r_x}{-r_y}{r_x}{0} \end{array}\)
     *
     * Inverse transformation can be also done easily, since
     *
     * \(\sin ( \theta ) \vecthreethree{0}{-r_z}{r_y}{r_z}{0}{-r_x}{-r_y}{r_x}{0} = \frac{R - R^T}{2}\)
     *
     * A rotation vector is a convenient and most compact representation of a rotation matrix (since any
     * rotation matrix has just 3 degrees of freedom). The representation is used in the global 3D geometry
     * optimization procedures like REF: calibrateCamera, REF: stereoCalibrate, or REF: solvePnP .
     *
     * <b>Note:</b> More information about the computation of the derivative of a 3D rotation matrix with respect to its exponential coordinate
     * can be found in:
     * <ul>
     *   <li>
     *      A Compact Formula for the Derivative of a 3-D Rotation in Exponential Coordinates, Guillermo Gallego, Anthony J. Yezzi CITE: Gallego2014ACF
     *   </li>
     * </ul>
     *
     * <b>Note:</b> Useful information on SE(3) and Lie Groups can be found in:
     * <ul>
     *   <li>
     *      A tutorial on SE(3) transformation parameterizations and on-manifold optimization, Jose-Luis Blanco CITE: blanco2010tutorial
     *   </li>
     *   <li>
     *      Lie Groups for 2D and 3D Transformation, Ethan Eade CITE: Eade17
     *   </li>
     *   <li>
     *      A micro Lie theory for state estimation in robotics, Joan Solà, Jérémie Deray, Dinesh Atchuthan CITE: Sol2018AML
     *   </li>
     * </ul>
     */
    public static void Rodrigues(Mat src, Mat dst) {
        Rodrigues_1(src.nativeObj, dst.nativeObj);
    }


    //
    // C++:  Mat cv::findHomography(vector_Point2f srcPoints, vector_Point2f dstPoints, int method = 0, double ransacReprojThreshold = 3, Mat& mask = Mat(), int maxIters = 2000, double confidence = 0.995)
    //

    /**
     * Finds a perspective transformation between two planes.
     *
     * @param srcPoints Coordinates of the points in the original plane, a matrix of the type CV_32FC2
     * or vector&lt;Point2f&gt; .
     * @param dstPoints Coordinates of the points in the target plane, a matrix of the type CV_32FC2 or
     * a vector&lt;Point2f&gt; .
     * @param method Method used to compute a homography matrix. The following methods are possible:
     * <ul>
     *   <li>
     *    <b>0</b> - a regular method using all the points, i.e., the least squares method
     *   </li>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     *   </li>
     *   <li>
     *    REF: RHO - PROSAC-based robust method
     * @param ransacReprojThreshold Maximum allowed reprojection error to treat a point pair as an inlier
     * (used in the RANSAC and RHO methods only). That is, if
     * \(\| \texttt{dstPoints} _i -  \texttt{convertPointsHomogeneous} ( \texttt{H} \cdot \texttt{srcPoints} _i) \|_2  &gt;  \texttt{ransacReprojThreshold}\)
     * then the point \(i\) is considered as an outlier. If srcPoints and dstPoints are measured in pixels,
     * it usually makes sense to set this parameter somewhere in the range of 1 to 10.
     * @param mask Optional output mask set by a robust method ( RANSAC or LMeDS ). Note that the input
     * mask values are ignored.
     * @param maxIters The maximum number of RANSAC iterations.
     * @param confidence Confidence level, between 0 and 1.
     *   </li>
     * </ul>
     *
     * The function finds and returns the perspective transformation \(H\) between the source and the
     * destination planes:
     *
     * \(s_i  \vecthree{x'_i}{y'_i}{1} \sim H  \vecthree{x_i}{y_i}{1}\)
     *
     * so that the back-projection error
     *
     * \(\sum _i \left ( x'_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2+ \left ( y'_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2\)
     *
     * is minimized. If the parameter method is set to the default value 0, the function uses all the point
     * pairs to compute an initial homography estimate with a simple least-squares scheme.
     *
     * However, if not all of the point pairs ( \(srcPoints_i\), \(dstPoints_i\) ) fit the rigid perspective
     * transformation (that is, there are some outliers), this initial estimate will be poor. In this case,
     * you can use one of the three robust methods. The methods RANSAC, LMeDS and RHO try many different
     * random subsets of the corresponding point pairs (of four pairs each, collinear pairs are discarded), estimate the homography matrix
     * using this subset and a simple least-squares algorithm, and then compute the quality/goodness of the
     * computed homography (which is the number of inliers for RANSAC or the least median re-projection error for
     * LMeDS). The best subset is then used to produce the initial estimate of the homography matrix and
     * the mask of inliers/outliers.
     *
     * Regardless of the method, robust or not, the computed homography matrix is refined further (using
     * inliers only in case of a robust method) with the Levenberg-Marquardt method to reduce the
     * re-projection error even more.
     *
     * The methods RANSAC and RHO can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers. Finally, if there are no outliers and the
     * noise is rather small, use the default method (method=0).
     *
     * The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is
     * determined up to a scale. Thus, it is normalized so that \(h_{33}=1\). Note that whenever an \(H\) matrix
     * cannot be estimated, an empty one will be returned.
     *
     * SEE:
     * getAffineTransform, estimateAffine2D, estimateAffinePartial2D, getPerspectiveTransform, warpPerspective,
     * perspectiveTransform
     * @return automatically generated
     */
    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints, int method, double ransacReprojThreshold, Mat mask, int maxIters, double confidence) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_0(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj, method, ransacReprojThreshold, mask.nativeObj, maxIters, confidence));
    }

    /**
     * Finds a perspective transformation between two planes.
     *
     * @param srcPoints Coordinates of the points in the original plane, a matrix of the type CV_32FC2
     * or vector&lt;Point2f&gt; .
     * @param dstPoints Coordinates of the points in the target plane, a matrix of the type CV_32FC2 or
     * a vector&lt;Point2f&gt; .
     * @param method Method used to compute a homography matrix. The following methods are possible:
     * <ul>
     *   <li>
     *    <b>0</b> - a regular method using all the points, i.e., the least squares method
     *   </li>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     *   </li>
     *   <li>
     *    REF: RHO - PROSAC-based robust method
     * @param ransacReprojThreshold Maximum allowed reprojection error to treat a point pair as an inlier
     * (used in the RANSAC and RHO methods only). That is, if
     * \(\| \texttt{dstPoints} _i -  \texttt{convertPointsHomogeneous} ( \texttt{H} \cdot \texttt{srcPoints} _i) \|_2  &gt;  \texttt{ransacReprojThreshold}\)
     * then the point \(i\) is considered as an outlier. If srcPoints and dstPoints are measured in pixels,
     * it usually makes sense to set this parameter somewhere in the range of 1 to 10.
     * @param mask Optional output mask set by a robust method ( RANSAC or LMeDS ). Note that the input
     * mask values are ignored.
     * @param maxIters The maximum number of RANSAC iterations.
     *   </li>
     * </ul>
     *
     * The function finds and returns the perspective transformation \(H\) between the source and the
     * destination planes:
     *
     * \(s_i  \vecthree{x'_i}{y'_i}{1} \sim H  \vecthree{x_i}{y_i}{1}\)
     *
     * so that the back-projection error
     *
     * \(\sum _i \left ( x'_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2+ \left ( y'_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2\)
     *
     * is minimized. If the parameter method is set to the default value 0, the function uses all the point
     * pairs to compute an initial homography estimate with a simple least-squares scheme.
     *
     * However, if not all of the point pairs ( \(srcPoints_i\), \(dstPoints_i\) ) fit the rigid perspective
     * transformation (that is, there are some outliers), this initial estimate will be poor. In this case,
     * you can use one of the three robust methods. The methods RANSAC, LMeDS and RHO try many different
     * random subsets of the corresponding point pairs (of four pairs each, collinear pairs are discarded), estimate the homography matrix
     * using this subset and a simple least-squares algorithm, and then compute the quality/goodness of the
     * computed homography (which is the number of inliers for RANSAC or the least median re-projection error for
     * LMeDS). The best subset is then used to produce the initial estimate of the homography matrix and
     * the mask of inliers/outliers.
     *
     * Regardless of the method, robust or not, the computed homography matrix is refined further (using
     * inliers only in case of a robust method) with the Levenberg-Marquardt method to reduce the
     * re-projection error even more.
     *
     * The methods RANSAC and RHO can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers. Finally, if there are no outliers and the
     * noise is rather small, use the default method (method=0).
     *
     * The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is
     * determined up to a scale. Thus, it is normalized so that \(h_{33}=1\). Note that whenever an \(H\) matrix
     * cannot be estimated, an empty one will be returned.
     *
     * SEE:
     * getAffineTransform, estimateAffine2D, estimateAffinePartial2D, getPerspectiveTransform, warpPerspective,
     * perspectiveTransform
     * @return automatically generated
     */
    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints, int method, double ransacReprojThreshold, Mat mask, int maxIters) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_1(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj, method, ransacReprojThreshold, mask.nativeObj, maxIters));
    }

    /**
     * Finds a perspective transformation between two planes.
     *
     * @param srcPoints Coordinates of the points in the original plane, a matrix of the type CV_32FC2
     * or vector&lt;Point2f&gt; .
     * @param dstPoints Coordinates of the points in the target plane, a matrix of the type CV_32FC2 or
     * a vector&lt;Point2f&gt; .
     * @param method Method used to compute a homography matrix. The following methods are possible:
     * <ul>
     *   <li>
     *    <b>0</b> - a regular method using all the points, i.e., the least squares method
     *   </li>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     *   </li>
     *   <li>
     *    REF: RHO - PROSAC-based robust method
     * @param ransacReprojThreshold Maximum allowed reprojection error to treat a point pair as an inlier
     * (used in the RANSAC and RHO methods only). That is, if
     * \(\| \texttt{dstPoints} _i -  \texttt{convertPointsHomogeneous} ( \texttt{H} \cdot \texttt{srcPoints} _i) \|_2  &gt;  \texttt{ransacReprojThreshold}\)
     * then the point \(i\) is considered as an outlier. If srcPoints and dstPoints are measured in pixels,
     * it usually makes sense to set this parameter somewhere in the range of 1 to 10.
     * @param mask Optional output mask set by a robust method ( RANSAC or LMeDS ). Note that the input
     * mask values are ignored.
     *   </li>
     * </ul>
     *
     * The function finds and returns the perspective transformation \(H\) between the source and the
     * destination planes:
     *
     * \(s_i  \vecthree{x'_i}{y'_i}{1} \sim H  \vecthree{x_i}{y_i}{1}\)
     *
     * so that the back-projection error
     *
     * \(\sum _i \left ( x'_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2+ \left ( y'_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2\)
     *
     * is minimized. If the parameter method is set to the default value 0, the function uses all the point
     * pairs to compute an initial homography estimate with a simple least-squares scheme.
     *
     * However, if not all of the point pairs ( \(srcPoints_i\), \(dstPoints_i\) ) fit the rigid perspective
     * transformation (that is, there are some outliers), this initial estimate will be poor. In this case,
     * you can use one of the three robust methods. The methods RANSAC, LMeDS and RHO try many different
     * random subsets of the corresponding point pairs (of four pairs each, collinear pairs are discarded), estimate the homography matrix
     * using this subset and a simple least-squares algorithm, and then compute the quality/goodness of the
     * computed homography (which is the number of inliers for RANSAC or the least median re-projection error for
     * LMeDS). The best subset is then used to produce the initial estimate of the homography matrix and
     * the mask of inliers/outliers.
     *
     * Regardless of the method, robust or not, the computed homography matrix is refined further (using
     * inliers only in case of a robust method) with the Levenberg-Marquardt method to reduce the
     * re-projection error even more.
     *
     * The methods RANSAC and RHO can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers. Finally, if there are no outliers and the
     * noise is rather small, use the default method (method=0).
     *
     * The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is
     * determined up to a scale. Thus, it is normalized so that \(h_{33}=1\). Note that whenever an \(H\) matrix
     * cannot be estimated, an empty one will be returned.
     *
     * SEE:
     * getAffineTransform, estimateAffine2D, estimateAffinePartial2D, getPerspectiveTransform, warpPerspective,
     * perspectiveTransform
     * @return automatically generated
     */
    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints, int method, double ransacReprojThreshold, Mat mask) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_2(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj, method, ransacReprojThreshold, mask.nativeObj));
    }

    /**
     * Finds a perspective transformation between two planes.
     *
     * @param srcPoints Coordinates of the points in the original plane, a matrix of the type CV_32FC2
     * or vector&lt;Point2f&gt; .
     * @param dstPoints Coordinates of the points in the target plane, a matrix of the type CV_32FC2 or
     * a vector&lt;Point2f&gt; .
     * @param method Method used to compute a homography matrix. The following methods are possible:
     * <ul>
     *   <li>
     *    <b>0</b> - a regular method using all the points, i.e., the least squares method
     *   </li>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     *   </li>
     *   <li>
     *    REF: RHO - PROSAC-based robust method
     * @param ransacReprojThreshold Maximum allowed reprojection error to treat a point pair as an inlier
     * (used in the RANSAC and RHO methods only). That is, if
     * \(\| \texttt{dstPoints} _i -  \texttt{convertPointsHomogeneous} ( \texttt{H} \cdot \texttt{srcPoints} _i) \|_2  &gt;  \texttt{ransacReprojThreshold}\)
     * then the point \(i\) is considered as an outlier. If srcPoints and dstPoints are measured in pixels,
     * it usually makes sense to set this parameter somewhere in the range of 1 to 10.
     * mask values are ignored.
     *   </li>
     * </ul>
     *
     * The function finds and returns the perspective transformation \(H\) between the source and the
     * destination planes:
     *
     * \(s_i  \vecthree{x'_i}{y'_i}{1} \sim H  \vecthree{x_i}{y_i}{1}\)
     *
     * so that the back-projection error
     *
     * \(\sum _i \left ( x'_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2+ \left ( y'_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2\)
     *
     * is minimized. If the parameter method is set to the default value 0, the function uses all the point
     * pairs to compute an initial homography estimate with a simple least-squares scheme.
     *
     * However, if not all of the point pairs ( \(srcPoints_i\), \(dstPoints_i\) ) fit the rigid perspective
     * transformation (that is, there are some outliers), this initial estimate will be poor. In this case,
     * you can use one of the three robust methods. The methods RANSAC, LMeDS and RHO try many different
     * random subsets of the corresponding point pairs (of four pairs each, collinear pairs are discarded), estimate the homography matrix
     * using this subset and a simple least-squares algorithm, and then compute the quality/goodness of the
     * computed homography (which is the number of inliers for RANSAC or the least median re-projection error for
     * LMeDS). The best subset is then used to produce the initial estimate of the homography matrix and
     * the mask of inliers/outliers.
     *
     * Regardless of the method, robust or not, the computed homography matrix is refined further (using
     * inliers only in case of a robust method) with the Levenberg-Marquardt method to reduce the
     * re-projection error even more.
     *
     * The methods RANSAC and RHO can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers. Finally, if there are no outliers and the
     * noise is rather small, use the default method (method=0).
     *
     * The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is
     * determined up to a scale. Thus, it is normalized so that \(h_{33}=1\). Note that whenever an \(H\) matrix
     * cannot be estimated, an empty one will be returned.
     *
     * SEE:
     * getAffineTransform, estimateAffine2D, estimateAffinePartial2D, getPerspectiveTransform, warpPerspective,
     * perspectiveTransform
     * @return automatically generated
     */
    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints, int method, double ransacReprojThreshold) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_3(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj, method, ransacReprojThreshold));
    }

    /**
     * Finds a perspective transformation between two planes.
     *
     * @param srcPoints Coordinates of the points in the original plane, a matrix of the type CV_32FC2
     * or vector&lt;Point2f&gt; .
     * @param dstPoints Coordinates of the points in the target plane, a matrix of the type CV_32FC2 or
     * a vector&lt;Point2f&gt; .
     * @param method Method used to compute a homography matrix. The following methods are possible:
     * <ul>
     *   <li>
     *    <b>0</b> - a regular method using all the points, i.e., the least squares method
     *   </li>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     *   </li>
     *   <li>
     *    REF: RHO - PROSAC-based robust method
     * (used in the RANSAC and RHO methods only). That is, if
     * \(\| \texttt{dstPoints} _i -  \texttt{convertPointsHomogeneous} ( \texttt{H} \cdot \texttt{srcPoints} _i) \|_2  &gt;  \texttt{ransacReprojThreshold}\)
     * then the point \(i\) is considered as an outlier. If srcPoints and dstPoints are measured in pixels,
     * it usually makes sense to set this parameter somewhere in the range of 1 to 10.
     * mask values are ignored.
     *   </li>
     * </ul>
     *
     * The function finds and returns the perspective transformation \(H\) between the source and the
     * destination planes:
     *
     * \(s_i  \vecthree{x'_i}{y'_i}{1} \sim H  \vecthree{x_i}{y_i}{1}\)
     *
     * so that the back-projection error
     *
     * \(\sum _i \left ( x'_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2+ \left ( y'_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2\)
     *
     * is minimized. If the parameter method is set to the default value 0, the function uses all the point
     * pairs to compute an initial homography estimate with a simple least-squares scheme.
     *
     * However, if not all of the point pairs ( \(srcPoints_i\), \(dstPoints_i\) ) fit the rigid perspective
     * transformation (that is, there are some outliers), this initial estimate will be poor. In this case,
     * you can use one of the three robust methods. The methods RANSAC, LMeDS and RHO try many different
     * random subsets of the corresponding point pairs (of four pairs each, collinear pairs are discarded), estimate the homography matrix
     * using this subset and a simple least-squares algorithm, and then compute the quality/goodness of the
     * computed homography (which is the number of inliers for RANSAC or the least median re-projection error for
     * LMeDS). The best subset is then used to produce the initial estimate of the homography matrix and
     * the mask of inliers/outliers.
     *
     * Regardless of the method, robust or not, the computed homography matrix is refined further (using
     * inliers only in case of a robust method) with the Levenberg-Marquardt method to reduce the
     * re-projection error even more.
     *
     * The methods RANSAC and RHO can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers. Finally, if there are no outliers and the
     * noise is rather small, use the default method (method=0).
     *
     * The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is
     * determined up to a scale. Thus, it is normalized so that \(h_{33}=1\). Note that whenever an \(H\) matrix
     * cannot be estimated, an empty one will be returned.
     *
     * SEE:
     * getAffineTransform, estimateAffine2D, estimateAffinePartial2D, getPerspectiveTransform, warpPerspective,
     * perspectiveTransform
     * @return automatically generated
     */
    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints, int method) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_4(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj, method));
    }

    /**
     * Finds a perspective transformation between two planes.
     *
     * @param srcPoints Coordinates of the points in the original plane, a matrix of the type CV_32FC2
     * or vector&lt;Point2f&gt; .
     * @param dstPoints Coordinates of the points in the target plane, a matrix of the type CV_32FC2 or
     * a vector&lt;Point2f&gt; .
     * <ul>
     *   <li>
     *    <b>0</b> - a regular method using all the points, i.e., the least squares method
     *   </li>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     *   </li>
     *   <li>
     *    REF: RHO - PROSAC-based robust method
     * (used in the RANSAC and RHO methods only). That is, if
     * \(\| \texttt{dstPoints} _i -  \texttt{convertPointsHomogeneous} ( \texttt{H} \cdot \texttt{srcPoints} _i) \|_2  &gt;  \texttt{ransacReprojThreshold}\)
     * then the point \(i\) is considered as an outlier. If srcPoints and dstPoints are measured in pixels,
     * it usually makes sense to set this parameter somewhere in the range of 1 to 10.
     * mask values are ignored.
     *   </li>
     * </ul>
     *
     * The function finds and returns the perspective transformation \(H\) between the source and the
     * destination planes:
     *
     * \(s_i  \vecthree{x'_i}{y'_i}{1} \sim H  \vecthree{x_i}{y_i}{1}\)
     *
     * so that the back-projection error
     *
     * \(\sum _i \left ( x'_i- \frac{h_{11} x_i + h_{12} y_i + h_{13}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2+ \left ( y'_i- \frac{h_{21} x_i + h_{22} y_i + h_{23}}{h_{31} x_i + h_{32} y_i + h_{33}} \right )^2\)
     *
     * is minimized. If the parameter method is set to the default value 0, the function uses all the point
     * pairs to compute an initial homography estimate with a simple least-squares scheme.
     *
     * However, if not all of the point pairs ( \(srcPoints_i\), \(dstPoints_i\) ) fit the rigid perspective
     * transformation (that is, there are some outliers), this initial estimate will be poor. In this case,
     * you can use one of the three robust methods. The methods RANSAC, LMeDS and RHO try many different
     * random subsets of the corresponding point pairs (of four pairs each, collinear pairs are discarded), estimate the homography matrix
     * using this subset and a simple least-squares algorithm, and then compute the quality/goodness of the
     * computed homography (which is the number of inliers for RANSAC or the least median re-projection error for
     * LMeDS). The best subset is then used to produce the initial estimate of the homography matrix and
     * the mask of inliers/outliers.
     *
     * Regardless of the method, robust or not, the computed homography matrix is refined further (using
     * inliers only in case of a robust method) with the Levenberg-Marquardt method to reduce the
     * re-projection error even more.
     *
     * The methods RANSAC and RHO can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers. Finally, if there are no outliers and the
     * noise is rather small, use the default method (method=0).
     *
     * The function is used to find initial intrinsic and extrinsic matrices. Homography matrix is
     * determined up to a scale. Thus, it is normalized so that \(h_{33}=1\). Note that whenever an \(H\) matrix
     * cannot be estimated, an empty one will be returned.
     *
     * SEE:
     * getAffineTransform, estimateAffine2D, estimateAffinePartial2D, getPerspectiveTransform, warpPerspective,
     * perspectiveTransform
     * @return automatically generated
     */
    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_5(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj));
    }


    //
    // C++:  Mat cv::findHomography(vector_Point2f srcPoints, vector_Point2f dstPoints, Mat& mask, UsacParams params)
    //

    public static Mat findHomography(MatOfPoint2f srcPoints, MatOfPoint2f dstPoints, Mat mask, UsacParams params) {
        Mat srcPoints_mat = srcPoints;
        Mat dstPoints_mat = dstPoints;
        return new Mat(findHomography_6(srcPoints_mat.nativeObj, dstPoints_mat.nativeObj, mask.nativeObj, params.nativeObj));
    }


    //
    // C++:  Vec3d cv::RQDecomp3x3(Mat src, Mat& mtxR, Mat& mtxQ, Mat& Qx = Mat(), Mat& Qy = Mat(), Mat& Qz = Mat())
    //

    /**
     * Computes an RQ decomposition of 3x3 matrices.
     *
     * @param src 3x3 input matrix.
     * @param mtxR Output 3x3 upper-triangular matrix.
     * @param mtxQ Output 3x3 orthogonal matrix.
     * @param Qx Optional output 3x3 rotation matrix around x-axis.
     * @param Qy Optional output 3x3 rotation matrix around y-axis.
     * @param Qz Optional output 3x3 rotation matrix around z-axis.
     *
     * The function computes a RQ decomposition using the given rotations. This function is used in
     * #decomposeProjectionMatrix to decompose the left 3x3 submatrix of a projection matrix into a camera
     * and a rotation matrix.
     *
     * It optionally returns three rotation matrices, one for each axis, and the three Euler angles in
     * degrees (as the return value) that could be used in OpenGL. Note, there is always more than one
     * sequence of rotations about the three principal axes that results in the same orientation of an
     * object, e.g. see CITE: Slabaugh . Returned tree rotation matrices and corresponding three Euler angles
     * are only one of the possible solutions.
     * @return automatically generated
     */
    public static double[] RQDecomp3x3(Mat src, Mat mtxR, Mat mtxQ, Mat Qx, Mat Qy, Mat Qz) {
        return RQDecomp3x3_0(src.nativeObj, mtxR.nativeObj, mtxQ.nativeObj, Qx.nativeObj, Qy.nativeObj, Qz.nativeObj);
    }

    /**
     * Computes an RQ decomposition of 3x3 matrices.
     *
     * @param src 3x3 input matrix.
     * @param mtxR Output 3x3 upper-triangular matrix.
     * @param mtxQ Output 3x3 orthogonal matrix.
     * @param Qx Optional output 3x3 rotation matrix around x-axis.
     * @param Qy Optional output 3x3 rotation matrix around y-axis.
     *
     * The function computes a RQ decomposition using the given rotations. This function is used in
     * #decomposeProjectionMatrix to decompose the left 3x3 submatrix of a projection matrix into a camera
     * and a rotation matrix.
     *
     * It optionally returns three rotation matrices, one for each axis, and the three Euler angles in
     * degrees (as the return value) that could be used in OpenGL. Note, there is always more than one
     * sequence of rotations about the three principal axes that results in the same orientation of an
     * object, e.g. see CITE: Slabaugh . Returned tree rotation matrices and corresponding three Euler angles
     * are only one of the possible solutions.
     * @return automatically generated
     */
    public static double[] RQDecomp3x3(Mat src, Mat mtxR, Mat mtxQ, Mat Qx, Mat Qy) {
        return RQDecomp3x3_1(src.nativeObj, mtxR.nativeObj, mtxQ.nativeObj, Qx.nativeObj, Qy.nativeObj);
    }

    /**
     * Computes an RQ decomposition of 3x3 matrices.
     *
     * @param src 3x3 input matrix.
     * @param mtxR Output 3x3 upper-triangular matrix.
     * @param mtxQ Output 3x3 orthogonal matrix.
     * @param Qx Optional output 3x3 rotation matrix around x-axis.
     *
     * The function computes a RQ decomposition using the given rotations. This function is used in
     * #decomposeProjectionMatrix to decompose the left 3x3 submatrix of a projection matrix into a camera
     * and a rotation matrix.
     *
     * It optionally returns three rotation matrices, one for each axis, and the three Euler angles in
     * degrees (as the return value) that could be used in OpenGL. Note, there is always more than one
     * sequence of rotations about the three principal axes that results in the same orientation of an
     * object, e.g. see CITE: Slabaugh . Returned tree rotation matrices and corresponding three Euler angles
     * are only one of the possible solutions.
     * @return automatically generated
     */
    public static double[] RQDecomp3x3(Mat src, Mat mtxR, Mat mtxQ, Mat Qx) {
        return RQDecomp3x3_2(src.nativeObj, mtxR.nativeObj, mtxQ.nativeObj, Qx.nativeObj);
    }

    /**
     * Computes an RQ decomposition of 3x3 matrices.
     *
     * @param src 3x3 input matrix.
     * @param mtxR Output 3x3 upper-triangular matrix.
     * @param mtxQ Output 3x3 orthogonal matrix.
     *
     * The function computes a RQ decomposition using the given rotations. This function is used in
     * #decomposeProjectionMatrix to decompose the left 3x3 submatrix of a projection matrix into a camera
     * and a rotation matrix.
     *
     * It optionally returns three rotation matrices, one for each axis, and the three Euler angles in
     * degrees (as the return value) that could be used in OpenGL. Note, there is always more than one
     * sequence of rotations about the three principal axes that results in the same orientation of an
     * object, e.g. see CITE: Slabaugh . Returned tree rotation matrices and corresponding three Euler angles
     * are only one of the possible solutions.
     * @return automatically generated
     */
    public static double[] RQDecomp3x3(Mat src, Mat mtxR, Mat mtxQ) {
        return RQDecomp3x3_3(src.nativeObj, mtxR.nativeObj, mtxQ.nativeObj);
    }


    //
    // C++:  void cv::decomposeProjectionMatrix(Mat projMatrix, Mat& cameraMatrix, Mat& rotMatrix, Mat& transVect, Mat& rotMatrixX = Mat(), Mat& rotMatrixY = Mat(), Mat& rotMatrixZ = Mat(), Mat& eulerAngles = Mat())
    //

    /**
     * Decomposes a projection matrix into a rotation matrix and a camera intrinsic matrix.
     *
     * @param projMatrix 3x4 input projection matrix P.
     * @param cameraMatrix Output 3x3 camera intrinsic matrix \(\cameramatrix{A}\).
     * @param rotMatrix Output 3x3 external rotation matrix R.
     * @param transVect Output 4x1 translation vector T.
     * @param rotMatrixX Optional 3x3 rotation matrix around x-axis.
     * @param rotMatrixY Optional 3x3 rotation matrix around y-axis.
     * @param rotMatrixZ Optional 3x3 rotation matrix around z-axis.
     * @param eulerAngles Optional three-element vector containing three Euler angles of rotation in
     * degrees.
     *
     * The function computes a decomposition of a projection matrix into a calibration and a rotation
     * matrix and the position of a camera.
     *
     * It optionally returns three rotation matrices, one for each axis, and three Euler angles that could
     * be used in OpenGL. Note, there is always more than one sequence of rotations about the three
     * principal axes that results in the same orientation of an object, e.g. see CITE: Slabaugh . Returned
     * tree rotation matrices and corresponding three Euler angles are only one of the possible solutions.
     *
     * The function is based on #RQDecomp3x3 .
     */
    public static void decomposeProjectionMatrix(Mat projMatrix, Mat cameraMatrix, Mat rotMatrix, Mat transVect, Mat rotMatrixX, Mat rotMatrixY, Mat rotMatrixZ, Mat eulerAngles) {
        decomposeProjectionMatrix_0(projMatrix.nativeObj, cameraMatrix.nativeObj, rotMatrix.nativeObj, transVect.nativeObj, rotMatrixX.nativeObj, rotMatrixY.nativeObj, rotMatrixZ.nativeObj, eulerAngles.nativeObj);
    }

    /**
     * Decomposes a projection matrix into a rotation matrix and a camera intrinsic matrix.
     *
     * @param projMatrix 3x4 input projection matrix P.
     * @param cameraMatrix Output 3x3 camera intrinsic matrix \(\cameramatrix{A}\).
     * @param rotMatrix Output 3x3 external rotation matrix R.
     * @param transVect Output 4x1 translation vector T.
     * @param rotMatrixX Optional 3x3 rotation matrix around x-axis.
     * @param rotMatrixY Optional 3x3 rotation matrix around y-axis.
     * @param rotMatrixZ Optional 3x3 rotation matrix around z-axis.
     * degrees.
     *
     * The function computes a decomposition of a projection matrix into a calibration and a rotation
     * matrix and the position of a camera.
     *
     * It optionally returns three rotation matrices, one for each axis, and three Euler angles that could
     * be used in OpenGL. Note, there is always more than one sequence of rotations about the three
     * principal axes that results in the same orientation of an object, e.g. see CITE: Slabaugh . Returned
     * tree rotation matrices and corresponding three Euler angles are only one of the possible solutions.
     *
     * The function is based on #RQDecomp3x3 .
     */
    public static void decomposeProjectionMatrix(Mat projMatrix, Mat cameraMatrix, Mat rotMatrix, Mat transVect, Mat rotMatrixX, Mat rotMatrixY, Mat rotMatrixZ) {
        decomposeProjectionMatrix_1(projMatrix.nativeObj, cameraMatrix.nativeObj, rotMatrix.nativeObj, transVect.nativeObj, rotMatrixX.nativeObj, rotMatrixY.nativeObj, rotMatrixZ.nativeObj);
    }

    /**
     * Decomposes a projection matrix into a rotation matrix and a camera intrinsic matrix.
     *
     * @param projMatrix 3x4 input projection matrix P.
     * @param cameraMatrix Output 3x3 camera intrinsic matrix \(\cameramatrix{A}\).
     * @param rotMatrix Output 3x3 external rotation matrix R.
     * @param transVect Output 4x1 translation vector T.
     * @param rotMatrixX Optional 3x3 rotation matrix around x-axis.
     * @param rotMatrixY Optional 3x3 rotation matrix around y-axis.
     * degrees.
     *
     * The function computes a decomposition of a projection matrix into a calibration and a rotation
     * matrix and the position of a camera.
     *
     * It optionally returns three rotation matrices, one for each axis, and three Euler angles that could
     * be used in OpenGL. Note, there is always more than one sequence of rotations about the three
     * principal axes that results in the same orientation of an object, e.g. see CITE: Slabaugh . Returned
     * tree rotation matrices and corresponding three Euler angles are only one of the possible solutions.
     *
     * The function is based on #RQDecomp3x3 .
     */
    public static void decomposeProjectionMatrix(Mat projMatrix, Mat cameraMatrix, Mat rotMatrix, Mat transVect, Mat rotMatrixX, Mat rotMatrixY) {
        decomposeProjectionMatrix_2(projMatrix.nativeObj, cameraMatrix.nativeObj, rotMatrix.nativeObj, transVect.nativeObj, rotMatrixX.nativeObj, rotMatrixY.nativeObj);
    }

    /**
     * Decomposes a projection matrix into a rotation matrix and a camera intrinsic matrix.
     *
     * @param projMatrix 3x4 input projection matrix P.
     * @param cameraMatrix Output 3x3 camera intrinsic matrix \(\cameramatrix{A}\).
     * @param rotMatrix Output 3x3 external rotation matrix R.
     * @param transVect Output 4x1 translation vector T.
     * @param rotMatrixX Optional 3x3 rotation matrix around x-axis.
     * degrees.
     *
     * The function computes a decomposition of a projection matrix into a calibration and a rotation
     * matrix and the position of a camera.
     *
     * It optionally returns three rotation matrices, one for each axis, and three Euler angles that could
     * be used in OpenGL. Note, there is always more than one sequence of rotations about the three
     * principal axes that results in the same orientation of an object, e.g. see CITE: Slabaugh . Returned
     * tree rotation matrices and corresponding three Euler angles are only one of the possible solutions.
     *
     * The function is based on #RQDecomp3x3 .
     */
    public static void decomposeProjectionMatrix(Mat projMatrix, Mat cameraMatrix, Mat rotMatrix, Mat transVect, Mat rotMatrixX) {
        decomposeProjectionMatrix_3(projMatrix.nativeObj, cameraMatrix.nativeObj, rotMatrix.nativeObj, transVect.nativeObj, rotMatrixX.nativeObj);
    }

    /**
     * Decomposes a projection matrix into a rotation matrix and a camera intrinsic matrix.
     *
     * @param projMatrix 3x4 input projection matrix P.
     * @param cameraMatrix Output 3x3 camera intrinsic matrix \(\cameramatrix{A}\).
     * @param rotMatrix Output 3x3 external rotation matrix R.
     * @param transVect Output 4x1 translation vector T.
     * degrees.
     *
     * The function computes a decomposition of a projection matrix into a calibration and a rotation
     * matrix and the position of a camera.
     *
     * It optionally returns three rotation matrices, one for each axis, and three Euler angles that could
     * be used in OpenGL. Note, there is always more than one sequence of rotations about the three
     * principal axes that results in the same orientation of an object, e.g. see CITE: Slabaugh . Returned
     * tree rotation matrices and corresponding three Euler angles are only one of the possible solutions.
     *
     * The function is based on #RQDecomp3x3 .
     */
    public static void decomposeProjectionMatrix(Mat projMatrix, Mat cameraMatrix, Mat rotMatrix, Mat transVect) {
        decomposeProjectionMatrix_4(projMatrix.nativeObj, cameraMatrix.nativeObj, rotMatrix.nativeObj, transVect.nativeObj);
    }


    //
    // C++:  void cv::matMulDeriv(Mat A, Mat B, Mat& dABdA, Mat& dABdB)
    //

    /**
     * Computes partial derivatives of the matrix product for each multiplied matrix.
     *
     * @param A First multiplied matrix.
     * @param B Second multiplied matrix.
     * @param dABdA First output derivative matrix d(A\*B)/dA of size
     * \(\texttt{A.rows*B.cols} \times {A.rows*A.cols}\) .
     * @param dABdB Second output derivative matrix d(A\*B)/dB of size
     * \(\texttt{A.rows*B.cols} \times {B.rows*B.cols}\) .
     *
     * The function computes partial derivatives of the elements of the matrix product \(A*B\) with regard to
     * the elements of each of the two input matrices. The function is used to compute the Jacobian
     * matrices in #stereoCalibrate but can also be used in any other similar optimization function.
     */
    public static void matMulDeriv(Mat A, Mat B, Mat dABdA, Mat dABdB) {
        matMulDeriv_0(A.nativeObj, B.nativeObj, dABdA.nativeObj, dABdB.nativeObj);
    }


    //
    // C++:  void cv::composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat& rvec3, Mat& tvec3, Mat& dr3dr1 = Mat(), Mat& dr3dt1 = Mat(), Mat& dr3dr2 = Mat(), Mat& dr3dt2 = Mat(), Mat& dt3dr1 = Mat(), Mat& dt3dt1 = Mat(), Mat& dt3dr2 = Mat(), Mat& dt3dt2 = Mat())
    //

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     * @param dr3dr2 Optional output derivative of rvec3 with regard to rvec2
     * @param dr3dt2 Optional output derivative of rvec3 with regard to tvec2
     * @param dt3dr1 Optional output derivative of tvec3 with regard to rvec1
     * @param dt3dt1 Optional output derivative of tvec3 with regard to tvec1
     * @param dt3dr2 Optional output derivative of tvec3 with regard to rvec2
     * @param dt3dt2 Optional output derivative of tvec3 with regard to tvec2
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1, Mat dr3dr2, Mat dr3dt2, Mat dt3dr1, Mat dt3dt1, Mat dt3dr2, Mat dt3dt2) {
        composeRT_0(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj, dr3dr2.nativeObj, dr3dt2.nativeObj, dt3dr1.nativeObj, dt3dt1.nativeObj, dt3dr2.nativeObj, dt3dt2.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     * @param dr3dr2 Optional output derivative of rvec3 with regard to rvec2
     * @param dr3dt2 Optional output derivative of rvec3 with regard to tvec2
     * @param dt3dr1 Optional output derivative of tvec3 with regard to rvec1
     * @param dt3dt1 Optional output derivative of tvec3 with regard to tvec1
     * @param dt3dr2 Optional output derivative of tvec3 with regard to rvec2
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1, Mat dr3dr2, Mat dr3dt2, Mat dt3dr1, Mat dt3dt1, Mat dt3dr2) {
        composeRT_1(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj, dr3dr2.nativeObj, dr3dt2.nativeObj, dt3dr1.nativeObj, dt3dt1.nativeObj, dt3dr2.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     * @param dr3dr2 Optional output derivative of rvec3 with regard to rvec2
     * @param dr3dt2 Optional output derivative of rvec3 with regard to tvec2
     * @param dt3dr1 Optional output derivative of tvec3 with regard to rvec1
     * @param dt3dt1 Optional output derivative of tvec3 with regard to tvec1
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1, Mat dr3dr2, Mat dr3dt2, Mat dt3dr1, Mat dt3dt1) {
        composeRT_2(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj, dr3dr2.nativeObj, dr3dt2.nativeObj, dt3dr1.nativeObj, dt3dt1.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     * @param dr3dr2 Optional output derivative of rvec3 with regard to rvec2
     * @param dr3dt2 Optional output derivative of rvec3 with regard to tvec2
     * @param dt3dr1 Optional output derivative of tvec3 with regard to rvec1
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1, Mat dr3dr2, Mat dr3dt2, Mat dt3dr1) {
        composeRT_3(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj, dr3dr2.nativeObj, dr3dt2.nativeObj, dt3dr1.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     * @param dr3dr2 Optional output derivative of rvec3 with regard to rvec2
     * @param dr3dt2 Optional output derivative of rvec3 with regard to tvec2
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1, Mat dr3dr2, Mat dr3dt2) {
        composeRT_4(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj, dr3dr2.nativeObj, dr3dt2.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     * @param dr3dr2 Optional output derivative of rvec3 with regard to rvec2
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1, Mat dr3dr2) {
        composeRT_5(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj, dr3dr2.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     * @param dr3dt1 Optional output derivative of rvec3 with regard to tvec1
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1, Mat dr3dt1) {
        composeRT_6(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj, dr3dt1.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     * @param dr3dr1 Optional output derivative of rvec3 with regard to rvec1
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3, Mat dr3dr1) {
        composeRT_7(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj, dr3dr1.nativeObj);
    }

    /**
     * Combines two rotation-and-shift transformations.
     *
     * @param rvec1 First rotation vector.
     * @param tvec1 First translation vector.
     * @param rvec2 Second rotation vector.
     * @param tvec2 Second translation vector.
     * @param rvec3 Output rotation vector of the superposition.
     * @param tvec3 Output translation vector of the superposition.
     *
     * The functions compute:
     *
     * \(\begin{array}{l} \texttt{rvec3} =  \mathrm{rodrigues} ^{-1} \left ( \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \mathrm{rodrigues} ( \texttt{rvec1} ) \right )  \\ \texttt{tvec3} =  \mathrm{rodrigues} ( \texttt{rvec2} )  \cdot \texttt{tvec1} +  \texttt{tvec2} \end{array} ,\)
     *
     * where \(\mathrm{rodrigues}\) denotes a rotation vector to a rotation matrix transformation, and
     * \(\mathrm{rodrigues}^{-1}\) denotes the inverse transformation. See #Rodrigues for details.
     *
     * Also, the functions can compute the derivatives of the output vectors with regards to the input
     * vectors (see #matMulDeriv ). The functions are used inside #stereoCalibrate but can also be used in
     * your own code where Levenberg-Marquardt or another gradient-based solver is used to optimize a
     * function that contains a matrix multiplication.
     */
    public static void composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat rvec3, Mat tvec3) {
        composeRT_8(rvec1.nativeObj, tvec1.nativeObj, rvec2.nativeObj, tvec2.nativeObj, rvec3.nativeObj, tvec3.nativeObj);
    }


    //
    // C++:  void cv::projectPoints(vector_Point3f objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, vector_double distCoeffs, vector_Point2f& imagePoints, Mat& jacobian = Mat(), double aspectRatio = 0)
    //

    /**
     * Projects 3D points to an image plane.
     *
     * @param objectPoints Array of object points expressed wrt. the world coordinate frame. A 3xN/Nx3
     * 1-channel or 1xN/Nx1 3-channel (or vector&lt;Point3f&gt; ), where N is the number of points in the view.
     * @param rvec The rotation vector (REF: Rodrigues) that, together with tvec, performs a change of
     * basis from world to camera coordinate system, see REF: calibrateCamera for details.
     * @param tvec The translation vector, see parameter description above.
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\) . If the vector is empty, the zero distortion coefficients are assumed.
     * @param imagePoints Output array of image points, 1xN/Nx1 2-channel, or
     * vector&lt;Point2f&gt; .
     * @param jacobian Optional output 2Nx(10+&lt;numDistCoeffs&gt;) jacobian matrix of derivatives of image
     * points with respect to components of the rotation vector, translation vector, focal lengths,
     * coordinates of the principal point and the distortion coefficients. In the old interface different
     * components of the jacobian are returned via different output parameters.
     * @param aspectRatio Optional "fixed aspect ratio" parameter. If the parameter is not 0, the
     * function assumes that the aspect ratio (\(f_x / f_y\)) is fixed and correspondingly adjusts the
     * jacobian matrix.
     *
     * The function computes the 2D projections of 3D points to the image plane, given intrinsic and
     * extrinsic camera parameters. Optionally, the function computes Jacobians -matrices of partial
     * derivatives of image points coordinates (as functions of all the input parameters) with respect to
     * the particular parameters, intrinsic and/or extrinsic. The Jacobians are used during the global
     * optimization in REF: calibrateCamera, REF: solvePnP, and REF: stereoCalibrate. The function itself
     * can also be used to compute a re-projection error, given the current intrinsic and extrinsic
     * parameters.
     *
     * <b>Note:</b> By setting rvec = tvec = \([0, 0, 0]\), or by setting cameraMatrix to a 3x3 identity matrix,
     * or by passing zero distortion coefficients, one can get various useful partial cases of the
     * function. This means, one can compute the distorted coordinates for a sparse set of points or apply
     * a perspective transformation (and also compute the derivatives) in the ideal zero-distortion setup.
     */
    public static void projectPoints(MatOfPoint3f objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs, MatOfPoint2f imagePoints, Mat jacobian, double aspectRatio) {
        Mat objectPoints_mat = objectPoints;
        Mat distCoeffs_mat = distCoeffs;
        Mat imagePoints_mat = imagePoints;
        projectPoints_0(objectPoints_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, imagePoints_mat.nativeObj, jacobian.nativeObj, aspectRatio);
    }

    /**
     * Projects 3D points to an image plane.
     *
     * @param objectPoints Array of object points expressed wrt. the world coordinate frame. A 3xN/Nx3
     * 1-channel or 1xN/Nx1 3-channel (or vector&lt;Point3f&gt; ), where N is the number of points in the view.
     * @param rvec The rotation vector (REF: Rodrigues) that, together with tvec, performs a change of
     * basis from world to camera coordinate system, see REF: calibrateCamera for details.
     * @param tvec The translation vector, see parameter description above.
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\) . If the vector is empty, the zero distortion coefficients are assumed.
     * @param imagePoints Output array of image points, 1xN/Nx1 2-channel, or
     * vector&lt;Point2f&gt; .
     * @param jacobian Optional output 2Nx(10+&lt;numDistCoeffs&gt;) jacobian matrix of derivatives of image
     * points with respect to components of the rotation vector, translation vector, focal lengths,
     * coordinates of the principal point and the distortion coefficients. In the old interface different
     * components of the jacobian are returned via different output parameters.
     * function assumes that the aspect ratio (\(f_x / f_y\)) is fixed and correspondingly adjusts the
     * jacobian matrix.
     *
     * The function computes the 2D projections of 3D points to the image plane, given intrinsic and
     * extrinsic camera parameters. Optionally, the function computes Jacobians -matrices of partial
     * derivatives of image points coordinates (as functions of all the input parameters) with respect to
     * the particular parameters, intrinsic and/or extrinsic. The Jacobians are used during the global
     * optimization in REF: calibrateCamera, REF: solvePnP, and REF: stereoCalibrate. The function itself
     * can also be used to compute a re-projection error, given the current intrinsic and extrinsic
     * parameters.
     *
     * <b>Note:</b> By setting rvec = tvec = \([0, 0, 0]\), or by setting cameraMatrix to a 3x3 identity matrix,
     * or by passing zero distortion coefficients, one can get various useful partial cases of the
     * function. This means, one can compute the distorted coordinates for a sparse set of points or apply
     * a perspective transformation (and also compute the derivatives) in the ideal zero-distortion setup.
     */
    public static void projectPoints(MatOfPoint3f objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs, MatOfPoint2f imagePoints, Mat jacobian) {
        Mat objectPoints_mat = objectPoints;
        Mat distCoeffs_mat = distCoeffs;
        Mat imagePoints_mat = imagePoints;
        projectPoints_1(objectPoints_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, imagePoints_mat.nativeObj, jacobian.nativeObj);
    }

    /**
     * Projects 3D points to an image plane.
     *
     * @param objectPoints Array of object points expressed wrt. the world coordinate frame. A 3xN/Nx3
     * 1-channel or 1xN/Nx1 3-channel (or vector&lt;Point3f&gt; ), where N is the number of points in the view.
     * @param rvec The rotation vector (REF: Rodrigues) that, together with tvec, performs a change of
     * basis from world to camera coordinate system, see REF: calibrateCamera for details.
     * @param tvec The translation vector, see parameter description above.
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\) . If the vector is empty, the zero distortion coefficients are assumed.
     * @param imagePoints Output array of image points, 1xN/Nx1 2-channel, or
     * vector&lt;Point2f&gt; .
     * points with respect to components of the rotation vector, translation vector, focal lengths,
     * coordinates of the principal point and the distortion coefficients. In the old interface different
     * components of the jacobian are returned via different output parameters.
     * function assumes that the aspect ratio (\(f_x / f_y\)) is fixed and correspondingly adjusts the
     * jacobian matrix.
     *
     * The function computes the 2D projections of 3D points to the image plane, given intrinsic and
     * extrinsic camera parameters. Optionally, the function computes Jacobians -matrices of partial
     * derivatives of image points coordinates (as functions of all the input parameters) with respect to
     * the particular parameters, intrinsic and/or extrinsic. The Jacobians are used during the global
     * optimization in REF: calibrateCamera, REF: solvePnP, and REF: stereoCalibrate. The function itself
     * can also be used to compute a re-projection error, given the current intrinsic and extrinsic
     * parameters.
     *
     * <b>Note:</b> By setting rvec = tvec = \([0, 0, 0]\), or by setting cameraMatrix to a 3x3 identity matrix,
     * or by passing zero distortion coefficients, one can get various useful partial cases of the
     * function. This means, one can compute the distorted coordinates for a sparse set of points or apply
     * a perspective transformation (and also compute the derivatives) in the ideal zero-distortion setup.
     */
    public static void projectPoints(MatOfPoint3f objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, MatOfDouble distCoeffs, MatOfPoint2f imagePoints) {
        Mat objectPoints_mat = objectPoints;
        Mat distCoeffs_mat = distCoeffs;
        Mat imagePoints_mat = imagePoints;
        projectPoints_2(objectPoints_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, imagePoints_mat.nativeObj);
    }


    //
    // C++:  void cv::projectPoints(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat& imagePoints, Mat& dpdr, Mat& dpdt, Mat& dpdf = Mat(), Mat& dpdc = Mat(), Mat& dpdk = Mat(), Mat& dpdo = Mat(), double aspectRatio = 0.)
    //

    public static void projectPointsSepJ(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints, Mat dpdr, Mat dpdt, Mat dpdf, Mat dpdc, Mat dpdk, Mat dpdo, double aspectRatio) {
        projectPointsSepJ_0(objectPoints.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, imagePoints.nativeObj, dpdr.nativeObj, dpdt.nativeObj, dpdf.nativeObj, dpdc.nativeObj, dpdk.nativeObj, dpdo.nativeObj, aspectRatio);
    }

    public static void projectPointsSepJ(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints, Mat dpdr, Mat dpdt, Mat dpdf, Mat dpdc, Mat dpdk, Mat dpdo) {
        projectPointsSepJ_1(objectPoints.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, imagePoints.nativeObj, dpdr.nativeObj, dpdt.nativeObj, dpdf.nativeObj, dpdc.nativeObj, dpdk.nativeObj, dpdo.nativeObj);
    }

    public static void projectPointsSepJ(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints, Mat dpdr, Mat dpdt, Mat dpdf, Mat dpdc, Mat dpdk) {
        projectPointsSepJ_2(objectPoints.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, imagePoints.nativeObj, dpdr.nativeObj, dpdt.nativeObj, dpdf.nativeObj, dpdc.nativeObj, dpdk.nativeObj);
    }

    public static void projectPointsSepJ(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints, Mat dpdr, Mat dpdt, Mat dpdf, Mat dpdc) {
        projectPointsSepJ_3(objectPoints.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, imagePoints.nativeObj, dpdr.nativeObj, dpdt.nativeObj, dpdf.nativeObj, dpdc.nativeObj);
    }

    public static void projectPointsSepJ(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints, Mat dpdr, Mat dpdt, Mat dpdf) {
        projectPointsSepJ_4(objectPoints.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, imagePoints.nativeObj, dpdr.nativeObj, dpdt.nativeObj, dpdf.nativeObj);
    }

    public static void projectPointsSepJ(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat imagePoints, Mat dpdr, Mat dpdt) {
        projectPointsSepJ_5(objectPoints.nativeObj, rvec.nativeObj, tvec.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, imagePoints.nativeObj, dpdr.nativeObj, dpdt.nativeObj);
    }


    //
    // C++:  bool cv::solvePnP(vector_Point3f objectPoints, vector_Point2f imagePoints, Mat cameraMatrix, vector_double distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE)
    //

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns the rotation and the translation vectors that transform a 3D point expressed in the object
     * coordinate frame to the camera coordinate frame, using different methods:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): need 4 input points to return a unique solution.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param flags Method for solving a PnP problem: see REF: calib3d_solvePnP_flags
     *
     * More information about Perspective-n-Points is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <ul>
     *     <li>
     *       With REF: SOLVEPNP_SQPNP input points must be &gt;= 3
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnP(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess, int flags) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnP_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess, flags);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns the rotation and the translation vectors that transform a 3D point expressed in the object
     * coordinate frame to the camera coordinate frame, using different methods:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): need 4 input points to return a unique solution.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     *
     * More information about Perspective-n-Points is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <ul>
     *     <li>
     *       With REF: SOLVEPNP_SQPNP input points must be &gt;= 3
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnP(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnP_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns the rotation and the translation vectors that transform a 3D point expressed in the object
     * coordinate frame to the camera coordinate frame, using different methods:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): need 4 input points to return a unique solution.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     *
     * More information about Perspective-n-Points is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <ul>
     *     <li>
     *       With REF: SOLVEPNP_SQPNP input points must be &gt;= 3
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnP(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnP_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj);
    }


    //
    // C++:  bool cv::solvePnPRansac(vector_Point3f objectPoints, vector_Point2f imagePoints, Mat cameraMatrix, vector_double distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false, int iterationsCount = 100, float reprojectionError = 8.0, double confidence = 0.99, Mat& inliers = Mat(), int flags = SOLVEPNP_ITERATIVE)
    //

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for REF: SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param iterationsCount Number of iterations.
     * @param reprojectionError Inlier threshold value used by the RANSAC procedure. The parameter value
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     * @param confidence The probability that the algorithm produces a useful result.
     * @param inliers Output vector that contains indices of inliers in objectPoints and imagePoints .
     * @param flags Method for solving a PnP problem (see REF: solvePnP ).
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence, Mat inliers, int flags) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_0(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers.nativeObj, flags);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for REF: SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param iterationsCount Number of iterations.
     * @param reprojectionError Inlier threshold value used by the RANSAC procedure. The parameter value
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     * @param confidence The probability that the algorithm produces a useful result.
     * @param inliers Output vector that contains indices of inliers in objectPoints and imagePoints .
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence, Mat inliers) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_1(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess, iterationsCount, reprojectionError, confidence, inliers.nativeObj);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for REF: SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param iterationsCount Number of iterations.
     * @param reprojectionError Inlier threshold value used by the RANSAC procedure. The parameter value
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     * @param confidence The probability that the algorithm produces a useful result.
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_2(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess, iterationsCount, reprojectionError, confidence);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for REF: SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param iterationsCount Number of iterations.
     * @param reprojectionError Inlier threshold value used by the RANSAC procedure. The parameter value
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_3(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess, iterationsCount, reprojectionError);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for REF: SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param iterationsCount Number of iterations.
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess, int iterationsCount) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_4(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess, iterationsCount);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * @param useExtrinsicGuess Parameter used for REF: SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_5(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess);
    }

    /**
     * Finds an object pose from 3D-2D point correspondences using the RANSAC scheme.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Output translation vector.
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * is the maximum allowed distance between the observed and computed point projections to consider it
     * an inlier.
     *
     * The function estimates an object pose given a set of object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients. This function finds such
     * a pose that minimizes reprojection error, that is, the sum of squared distances between the observed
     * projections imagePoints and the projected (using REF: projectPoints ) objectPoints. The use of RANSAC
     * makes the function resistant to outliers.
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePNPRansac for object detection can be found at
     *         opencv_source_code/samples/cpp/tutorial_code/3d/real_time_pose_estimation/
     *   </li>
     *   <li>
     *       The default method used to estimate the camera pose for the Minimal Sample Sets step
     *        is #SOLVEPNP_EPNP. Exceptions are:
     *   <ul>
     *     <li>
     *           if you choose #SOLVEPNP_P3P or #SOLVEPNP_AP3P, these methods will be used.
     *     </li>
     *     <li>
     *           if the number of input points is equal to 4, #SOLVEPNP_P3P is used.
     *     </li>
     *   </ul>
     *   <li>
     *       The method used to estimate the camera pose using all the inliers is defined by the
     *        flags parameters unless it is equal to #SOLVEPNP_P3P or #SOLVEPNP_AP3P. In this case,
     *        the method #SOLVEPNP_EPNP will be used instead.
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_6(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj);
    }


    //
    // C++:  bool cv::solvePnPRansac(vector_Point3f objectPoints, vector_Point2f imagePoints, Mat& cameraMatrix, vector_double distCoeffs, Mat& rvec, Mat& tvec, Mat& inliers, UsacParams params = UsacParams())
    //

    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, Mat inliers, UsacParams params) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_7(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, inliers.nativeObj, params.nativeObj);
    }

    public static boolean solvePnPRansac(MatOfPoint3f objectPoints, MatOfPoint2f imagePoints, Mat cameraMatrix, MatOfDouble distCoeffs, Mat rvec, Mat tvec, Mat inliers) {
        Mat objectPoints_mat = objectPoints;
        Mat imagePoints_mat = imagePoints;
        Mat distCoeffs_mat = distCoeffs;
        return solvePnPRansac_8(objectPoints_mat.nativeObj, imagePoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs_mat.nativeObj, rvec.nativeObj, tvec.nativeObj, inliers.nativeObj);
    }


    //
    // C++:  int cv::solveP3P(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, int flags)
    //

    /**
     * Finds an object pose from 3 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, 3x3 1-channel or
     * 1x3/3x1 3-channel. vector&lt;Point3f&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, 3x2 1-channel or 1x3/3x1 2-channel.
     *  vector&lt;Point2f&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system. A P3P problem has up to 4 solutions.
     * @param tvecs Output translation vectors.
     * @param flags Method for solving a P3P problem:
     * <ul>
     *   <li>
     *    REF: SOLVEPNP_P3P Method is based on the paper of X.S. Gao, X.-R. Hou, J. Tang, H.-F. Chang
     * "Complete Solution Classification for the Perspective-Three-Point Problem" (CITE: gao2003complete).
     *   </li>
     *   <li>
     *    REF: SOLVEPNP_AP3P Method is based on the paper of T. Ke and S. Roumeliotis.
     * "An Efficient Algebraic Solution to the Perspective-Three-Point Problem" (CITE: Ke17).
     *   </li>
     * </ul>
     *
     * The function estimates the object pose given 3 object points, their corresponding image
     * projections, as well as the camera intrinsic matrix and the distortion coefficients.
     *
     * <b>Note:</b>
     * The solutions are sorted by reprojection errors (lowest to highest).
     * @return automatically generated
     */
    public static int solveP3P(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solveP3P_0(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  void cv::solvePnPRefineLM(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, FLT_EPSILON))
    //

    /**
     * Refine a pose (the translation and the rotation that transform a 3D point expressed in the object coordinate frame
     * to the camera coordinate frame) from a 3D-2D point correspondences and starting from an initial solution.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel,
     * where N is the number of points. vector&lt;Point3d&gt; can also be passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can also be passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Input/Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system. Input values are used as an initial solution.
     * @param tvec Input/Output translation vector. Input values are used as an initial solution.
     * @param criteria Criteria when to stop the Levenberg-Marquard iterative algorithm.
     *
     * The function refines the object pose given at least 3 object points, their corresponding image
     * projections, an initial solution for the rotation and translation vector,
     * as well as the camera intrinsic matrix and the distortion coefficients.
     * The function minimizes the projection error with respect to the rotation and the translation vectors, according
     * to a Levenberg-Marquardt iterative minimization CITE: Madsen04 CITE: Eade13 process.
     */
    public static void solvePnPRefineLM(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, TermCriteria criteria) {
        solvePnPRefineLM_0(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    /**
     * Refine a pose (the translation and the rotation that transform a 3D point expressed in the object coordinate frame
     * to the camera coordinate frame) from a 3D-2D point correspondences and starting from an initial solution.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel,
     * where N is the number of points. vector&lt;Point3d&gt; can also be passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can also be passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Input/Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system. Input values are used as an initial solution.
     * @param tvec Input/Output translation vector. Input values are used as an initial solution.
     *
     * The function refines the object pose given at least 3 object points, their corresponding image
     * projections, an initial solution for the rotation and translation vector,
     * as well as the camera intrinsic matrix and the distortion coefficients.
     * The function minimizes the projection error with respect to the rotation and the translation vectors, according
     * to a Levenberg-Marquardt iterative minimization CITE: Madsen04 CITE: Eade13 process.
     */
    public static void solvePnPRefineLM(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec) {
        solvePnPRefineLM_1(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj);
    }


    //
    // C++:  void cv::solvePnPRefineVVS(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, FLT_EPSILON), double VVSlambda = 1)
    //

    /**
     * Refine a pose (the translation and the rotation that transform a 3D point expressed in the object coordinate frame
     * to the camera coordinate frame) from a 3D-2D point correspondences and starting from an initial solution.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel,
     * where N is the number of points. vector&lt;Point3d&gt; can also be passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can also be passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Input/Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system. Input values are used as an initial solution.
     * @param tvec Input/Output translation vector. Input values are used as an initial solution.
     * @param criteria Criteria when to stop the Levenberg-Marquard iterative algorithm.
     * @param VVSlambda Gain for the virtual visual servoing control law, equivalent to the \(\alpha\)
     * gain in the Damped Gauss-Newton formulation.
     *
     * The function refines the object pose given at least 3 object points, their corresponding image
     * projections, an initial solution for the rotation and translation vector,
     * as well as the camera intrinsic matrix and the distortion coefficients.
     * The function minimizes the projection error with respect to the rotation and the translation vectors, using a
     * virtual visual servoing (VVS) CITE: Chaumette06 CITE: Marchand16 scheme.
     */
    public static void solvePnPRefineVVS(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, TermCriteria criteria, double VVSlambda) {
        solvePnPRefineVVS_0(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, criteria.type, criteria.maxCount, criteria.epsilon, VVSlambda);
    }

    /**
     * Refine a pose (the translation and the rotation that transform a 3D point expressed in the object coordinate frame
     * to the camera coordinate frame) from a 3D-2D point correspondences and starting from an initial solution.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel,
     * where N is the number of points. vector&lt;Point3d&gt; can also be passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can also be passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Input/Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system. Input values are used as an initial solution.
     * @param tvec Input/Output translation vector. Input values are used as an initial solution.
     * @param criteria Criteria when to stop the Levenberg-Marquard iterative algorithm.
     * gain in the Damped Gauss-Newton formulation.
     *
     * The function refines the object pose given at least 3 object points, their corresponding image
     * projections, an initial solution for the rotation and translation vector,
     * as well as the camera intrinsic matrix and the distortion coefficients.
     * The function minimizes the projection error with respect to the rotation and the translation vectors, using a
     * virtual visual servoing (VVS) CITE: Chaumette06 CITE: Marchand16 scheme.
     */
    public static void solvePnPRefineVVS(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, TermCriteria criteria) {
        solvePnPRefineVVS_1(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    /**
     * Refine a pose (the translation and the rotation that transform a 3D point expressed in the object coordinate frame
     * to the camera coordinate frame) from a 3D-2D point correspondences and starting from an initial solution.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel,
     * where N is the number of points. vector&lt;Point3d&gt; can also be passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can also be passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvec Input/Output rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system. Input values are used as an initial solution.
     * @param tvec Input/Output translation vector. Input values are used as an initial solution.
     * gain in the Damped Gauss-Newton formulation.
     *
     * The function refines the object pose given at least 3 object points, their corresponding image
     * projections, an initial solution for the rotation and translation vector,
     * as well as the camera intrinsic matrix and the distortion coefficients.
     * The function minimizes the projection error with respect to the rotation and the translation vectors, using a
     * virtual visual servoing (VVS) CITE: Chaumette06 CITE: Marchand16 scheme.
     */
    public static void solvePnPRefineVVS(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec) {
        solvePnPRefineVVS_2(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj);
    }


    //
    // C++:  int cv::solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE, Mat rvec = Mat(), Mat tvec = Mat(), Mat& reprojectionError = Mat())
    //

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns a list of all the possible solutions (a solution is a &lt;rotation vector, translation vector&gt;
     * couple), depending on the number of input points and the chosen method:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): 3 or 4 input points. Number of returned solutions can be between 0 and 4 with 3 input points.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar. Returns 2 solutions.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4 and 2 solutions are returned. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     * Only 1 solution is returned.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Vector of output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvecs Vector of output translation vectors.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param flags Method for solving a PnP problem: see REF: calib3d_solvePnP_flags
     * @param rvec Rotation vector used to initialize an iterative PnP refinement algorithm, when flag is REF: SOLVEPNP_ITERATIVE
     * and useExtrinsicGuess is set to true.
     * @param tvec Translation vector used to initialize an iterative PnP refinement algorithm, when flag is REF: SOLVEPNP_ITERATIVE
     * and useExtrinsicGuess is set to true.
     * @param reprojectionError Optional vector of reprojection error, that is the RMS error
     * (\( \text{RMSE} = \sqrt{\frac{\sum_{i}^{N} \left ( \hat{y_i} - y_i \right )^2}{N}} \)) between the input image points
     * and the 3D object points projected with the estimated pose.
     *
     * More information is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static int solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, boolean useExtrinsicGuess, int flags, Mat rvec, Mat tvec, Mat reprojectionError) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solvePnPGeneric_0(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, useExtrinsicGuess, flags, rvec.nativeObj, tvec.nativeObj, reprojectionError.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns a list of all the possible solutions (a solution is a &lt;rotation vector, translation vector&gt;
     * couple), depending on the number of input points and the chosen method:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): 3 or 4 input points. Number of returned solutions can be between 0 and 4 with 3 input points.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar. Returns 2 solutions.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4 and 2 solutions are returned. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     * Only 1 solution is returned.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Vector of output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvecs Vector of output translation vectors.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param flags Method for solving a PnP problem: see REF: calib3d_solvePnP_flags
     * @param rvec Rotation vector used to initialize an iterative PnP refinement algorithm, when flag is REF: SOLVEPNP_ITERATIVE
     * and useExtrinsicGuess is set to true.
     * @param tvec Translation vector used to initialize an iterative PnP refinement algorithm, when flag is REF: SOLVEPNP_ITERATIVE
     * and useExtrinsicGuess is set to true.
     * (\( \text{RMSE} = \sqrt{\frac{\sum_{i}^{N} \left ( \hat{y_i} - y_i \right )^2}{N}} \)) between the input image points
     * and the 3D object points projected with the estimated pose.
     *
     * More information is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static int solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, boolean useExtrinsicGuess, int flags, Mat rvec, Mat tvec) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solvePnPGeneric_1(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, useExtrinsicGuess, flags, rvec.nativeObj, tvec.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns a list of all the possible solutions (a solution is a &lt;rotation vector, translation vector&gt;
     * couple), depending on the number of input points and the chosen method:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): 3 or 4 input points. Number of returned solutions can be between 0 and 4 with 3 input points.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar. Returns 2 solutions.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4 and 2 solutions are returned. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     * Only 1 solution is returned.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Vector of output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvecs Vector of output translation vectors.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param flags Method for solving a PnP problem: see REF: calib3d_solvePnP_flags
     * @param rvec Rotation vector used to initialize an iterative PnP refinement algorithm, when flag is REF: SOLVEPNP_ITERATIVE
     * and useExtrinsicGuess is set to true.
     * and useExtrinsicGuess is set to true.
     * (\( \text{RMSE} = \sqrt{\frac{\sum_{i}^{N} \left ( \hat{y_i} - y_i \right )^2}{N}} \)) between the input image points
     * and the 3D object points projected with the estimated pose.
     *
     * More information is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static int solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, boolean useExtrinsicGuess, int flags, Mat rvec) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solvePnPGeneric_2(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, useExtrinsicGuess, flags, rvec.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns a list of all the possible solutions (a solution is a &lt;rotation vector, translation vector&gt;
     * couple), depending on the number of input points and the chosen method:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): 3 or 4 input points. Number of returned solutions can be between 0 and 4 with 3 input points.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar. Returns 2 solutions.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4 and 2 solutions are returned. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     * Only 1 solution is returned.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Vector of output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvecs Vector of output translation vectors.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * @param flags Method for solving a PnP problem: see REF: calib3d_solvePnP_flags
     * and useExtrinsicGuess is set to true.
     * and useExtrinsicGuess is set to true.
     * (\( \text{RMSE} = \sqrt{\frac{\sum_{i}^{N} \left ( \hat{y_i} - y_i \right )^2}{N}} \)) between the input image points
     * and the 3D object points projected with the estimated pose.
     *
     * More information is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static int solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, boolean useExtrinsicGuess, int flags) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solvePnPGeneric_3(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, useExtrinsicGuess, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns a list of all the possible solutions (a solution is a &lt;rotation vector, translation vector&gt;
     * couple), depending on the number of input points and the chosen method:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): 3 or 4 input points. Number of returned solutions can be between 0 and 4 with 3 input points.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar. Returns 2 solutions.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4 and 2 solutions are returned. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     * Only 1 solution is returned.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Vector of output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvecs Vector of output translation vectors.
     * @param useExtrinsicGuess Parameter used for #SOLVEPNP_ITERATIVE. If true (1), the function uses
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * and useExtrinsicGuess is set to true.
     * and useExtrinsicGuess is set to true.
     * (\( \text{RMSE} = \sqrt{\frac{\sum_{i}^{N} \left ( \hat{y_i} - y_i \right )^2}{N}} \)) between the input image points
     * and the 3D object points projected with the estimated pose.
     *
     * More information is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static int solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, boolean useExtrinsicGuess) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solvePnPGeneric_4(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, useExtrinsicGuess);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    /**
     * Finds an object pose from 3D-2D point correspondences.
     *
     * SEE: REF: calib3d_solvePnP
     *
     * This function returns a list of all the possible solutions (a solution is a &lt;rotation vector, translation vector&gt;
     * couple), depending on the number of input points and the chosen method:
     * <ul>
     *   <li>
     *  P3P methods (REF: SOLVEPNP_P3P, REF: SOLVEPNP_AP3P): 3 or 4 input points. Number of returned solutions can be between 0 and 4 with 3 input points.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE Input points must be &gt;= 4 and object points must be coplanar. Returns 2 solutions.
     *   </li>
     *   <li>
     *  REF: SOLVEPNP_IPPE_SQUARE Special case suitable for marker pose estimation.
     * Number of input points must be 4 and 2 solutions are returned. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *    point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *    point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   <li>
     *  for all the other flags, number of input points must be &gt;= 4 and object points can be in any configuration.
     * Only 1 solution is returned.
     *   </li>
     * </ul>
     *
     * @param objectPoints Array of object points in the object coordinate space, Nx3 1-channel or
     * 1xN/Nx1 3-channel, where N is the number of points. vector&lt;Point3d&gt; can be also passed here.
     * @param imagePoints Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel,
     * where N is the number of points. vector&lt;Point2d&gt; can be also passed here.
     * @param cameraMatrix Input camera intrinsic matrix \(\cameramatrix{A}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param rvecs Vector of output rotation vectors (see REF: Rodrigues ) that, together with tvecs, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvecs Vector of output translation vectors.
     * the provided rvec and tvec values as initial approximations of the rotation and translation
     * vectors, respectively, and further optimizes them.
     * and useExtrinsicGuess is set to true.
     * and useExtrinsicGuess is set to true.
     * (\( \text{RMSE} = \sqrt{\frac{\sum_{i}^{N} \left ( \hat{y_i} - y_i \right )^2}{N}} \)) between the input image points
     * and the 3D object points projected with the estimated pose.
     *
     * More information is described in REF: calib3d_solvePnP
     *
     * <b>Note:</b>
     * <ul>
     *   <li>
     *       An example of how to use solvePnP for planar augmented reality can be found at
     *         opencv_source_code/samples/python/plane_ar.py
     *   </li>
     *   <li>
     *       If you are using Python:
     *   <ul>
     *     <li>
     *          Numpy array slices won't work as input because solvePnP requires contiguous
     *         arrays (enforced by the assertion using cv::Mat::checkVector() around line 55 of
     *         modules/3d/src/solvepnp.cpp version 2.4.9)
     *     </li>
     *     <li>
     *          The P3P algorithm requires image points to be in an array of shape (N,1,2) due
     *         to its calling of #undistortPoints (around line 75 of modules/3d/src/solvepnp.cpp version 2.4.9)
     *         which requires 2-channel information.
     *     </li>
     *     <li>
     *          Thus, given some data D = np.array(...) where D.shape = (N,M), in order to use a subset of
     *         it as, e.g., imagePoints, one must effectively copy it into a new array: imagePoints =
     *         np.ascontiguousarray(D[:,:2]).reshape((N,1,2))
     *     </li>
     *   </ul>
     *   <li>
     *       The methods REF: SOLVEPNP_DLS and REF: SOLVEPNP_UPNP cannot be used as the current implementations are
     *        unstable and sometimes give completely wrong results. If you pass one of these two
     *        flags, REF: SOLVEPNP_EPNP method will be used instead.
     *   </li>
     *   <li>
     *       The minimum number of points is 4 in the general case. In the case of REF: SOLVEPNP_P3P and REF: SOLVEPNP_AP3P
     *        methods, it is required to use exactly 4 points (the first 3 points are used to estimate all the solutions
     *        of the P3P problem, the last one is used to retain the best solution that minimizes the reprojection error).
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_ITERATIVE method and {@code useExtrinsicGuess=true}, the minimum number of points is 3 (3 points
     *        are sufficient to compute a pose but there are up to 4 solutions). The initial solution should be close to the
     *        global solution to converge.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE input points must be &gt;= 4 and object points must be coplanar.
     *   </li>
     *   <li>
     *       With REF: SOLVEPNP_IPPE_SQUARE this is a special case suitable for marker pose estimation.
     *        Number of input points must be 4. Object points must be defined in the following order:
     *   <ul>
     *     <li>
     *           point 0: [-squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 1: [ squareLength / 2,  squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 2: [ squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *     <li>
     *           point 3: [-squareLength / 2, -squareLength / 2, 0]
     *     </li>
     *   </ul>
     *   </li>
     * </ul>
     * @return automatically generated
     */
    public static int solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs) {
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        int retVal = solvePnPGeneric_5(objectPoints.nativeObj, imagePoints.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  void cv::drawFrameAxes(Mat& image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length, int thickness = 3)
    //

    /**
     * Draw axes of the world/object coordinate system from pose estimation. SEE: solvePnP
     *
     * @param image Input/output image. It must have 1 or 3 channels. The number of channels is not altered.
     * @param cameraMatrix Input 3x3 floating-point matrix of camera intrinsic parameters.
     * \(\cameramatrix{A}\)
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is empty, the zero distortion coefficients are assumed.
     * @param rvec Rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Translation vector.
     * @param length Length of the painted axes in the same unit than tvec (usually in meters).
     * @param thickness Line thickness of the painted axes.
     *
     * This function draws the axes of the world/object coordinate system w.r.t. to the camera frame.
     * OX is drawn in red, OY in green and OZ in blue.
     */
    public static void drawFrameAxes(Mat image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length, int thickness) {
        drawFrameAxes_0(image.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, length, thickness);
    }

    /**
     * Draw axes of the world/object coordinate system from pose estimation. SEE: solvePnP
     *
     * @param image Input/output image. It must have 1 or 3 channels. The number of channels is not altered.
     * @param cameraMatrix Input 3x3 floating-point matrix of camera intrinsic parameters.
     * \(\cameramatrix{A}\)
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is empty, the zero distortion coefficients are assumed.
     * @param rvec Rotation vector (see REF: Rodrigues ) that, together with tvec, brings points from
     * the model coordinate system to the camera coordinate system.
     * @param tvec Translation vector.
     * @param length Length of the painted axes in the same unit than tvec (usually in meters).
     *
     * This function draws the axes of the world/object coordinate system w.r.t. to the camera frame.
     * OX is drawn in red, OY in green and OZ in blue.
     */
    public static void drawFrameAxes(Mat image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length) {
        drawFrameAxes_1(image.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, length);
    }


    //
    // C++:  void cv::convertPointsToHomogeneous(Mat src, Mat& dst, int dtype = -1)
    //

    /**
     * Converts points from Euclidean to homogeneous space.
     *
     * @param src Input vector of N-dimensional points.
     * @param dst Output vector of N+1-dimensional points.
     * @param dtype The desired output array depth (either CV_32F or CV_64F are currently supported).
     *     If it's -1, then it's set automatically to CV_32F or CV_64F, depending on the input depth.
     *
     * The function converts points from Euclidean to homogeneous space by appending 1's to the tuple of
     * point coordinates. That is, each point (x1, x2, ..., xn) is converted to (x1, x2, ..., xn, 1).
     */
    public static void convertPointsToHomogeneous(Mat src, Mat dst, int dtype) {
        convertPointsToHomogeneous_0(src.nativeObj, dst.nativeObj, dtype);
    }

    /**
     * Converts points from Euclidean to homogeneous space.
     *
     * @param src Input vector of N-dimensional points.
     * @param dst Output vector of N+1-dimensional points.
     *     If it's -1, then it's set automatically to CV_32F or CV_64F, depending on the input depth.
     *
     * The function converts points from Euclidean to homogeneous space by appending 1's to the tuple of
     * point coordinates. That is, each point (x1, x2, ..., xn) is converted to (x1, x2, ..., xn, 1).
     */
    public static void convertPointsToHomogeneous(Mat src, Mat dst) {
        convertPointsToHomogeneous_1(src.nativeObj, dst.nativeObj);
    }


    //
    // C++:  void cv::convertPointsFromHomogeneous(Mat src, Mat& dst, int dtype = -1)
    //

    /**
     * Converts points from homogeneous to Euclidean space.
     *
     * @param src Input vector of N-dimensional points.
     * @param dst Output vector of N-1-dimensional points.
     * @param dtype The desired output array depth (either CV_32F or CV_64F are currently supported).
     *     If it's -1, then it's set automatically to CV_32F or CV_64F, depending on the input depth.
     *
     * The function converts points homogeneous to Euclidean space using perspective projection. That is,
     * each point (x1, x2, ... x(n-1), xn) is converted to (x1/xn, x2/xn, ..., x(n-1)/xn). When xn=0, the
     * output point coordinates will be (0,0,0,...).
     */
    public static void convertPointsFromHomogeneous(Mat src, Mat dst, int dtype) {
        convertPointsFromHomogeneous_0(src.nativeObj, dst.nativeObj, dtype);
    }

    /**
     * Converts points from homogeneous to Euclidean space.
     *
     * @param src Input vector of N-dimensional points.
     * @param dst Output vector of N-1-dimensional points.
     *     If it's -1, then it's set automatically to CV_32F or CV_64F, depending on the input depth.
     *
     * The function converts points homogeneous to Euclidean space using perspective projection. That is,
     * each point (x1, x2, ... x(n-1), xn) is converted to (x1/xn, x2/xn, ..., x(n-1)/xn). When xn=0, the
     * output point coordinates will be (0,0,0,...).
     */
    public static void convertPointsFromHomogeneous(Mat src, Mat dst) {
        convertPointsFromHomogeneous_1(src.nativeObj, dst.nativeObj);
    }


    //
    // C++:  Mat cv::findFundamentalMat(vector_Point2f points1, vector_Point2f points2, int method, double ransacReprojThreshold, double confidence, int maxIters, Mat& mask = Mat())
    //

    /**
     * Calculates a fundamental matrix from the corresponding points in two images.
     *
     * @param points1 Array of N points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: FM_7POINT for a 7-point algorithm. \(N = 7\)
     *   </li>
     *   <li>
     *    REF: FM_8POINT for an 8-point algorithm. \(N \ge 8\)
     *   </li>
     *   <li>
     *    REF: FM_RANSAC for the RANSAC algorithm. \(N \ge 8\)
     *   </li>
     *   <li>
     *    REF: FM_LMEDS for the LMedS algorithm. \(N \ge 8\)
     * @param ransacReprojThreshold Parameter used only for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param confidence Parameter used for the RANSAC and LMedS methods only. It specifies a desirable level
     * of confidence (probability) that the estimated matrix is correct.
     * @param mask optional output mask
     * @param maxIters The maximum number of robust method iterations.
     *   </li>
     * </ul>
     *
     * The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T F [p_1; 1] = 0\)
     *
     * where \(F\) is a fundamental matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively.
     *
     * The function calculates the fundamental matrix using one of four methods listed above and returns
     * the found fundamental matrix. Normally just one matrix is found. But in case of the 7-point
     * algorithm, the function may return up to 3 solutions ( \(9 \times 3\) matrix that stores all 3
     * matrices sequentially).
     *
     * The calculated fundamental matrix may be passed further to #computeCorrespondEpilines that finds the
     * epipolar lines corresponding to the specified points. It can also be passed to
     * #stereoRectifyUncalibrated to compute the rectification transformation. :
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     Mat fundamental_matrix =
     *      findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
     * </code>
     * @return automatically generated
     */
    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, int method, double ransacReprojThreshold, double confidence, int maxIters, Mat mask) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_0(points1_mat.nativeObj, points2_mat.nativeObj, method, ransacReprojThreshold, confidence, maxIters, mask.nativeObj));
    }

    /**
     * Calculates a fundamental matrix from the corresponding points in two images.
     *
     * @param points1 Array of N points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: FM_7POINT for a 7-point algorithm. \(N = 7\)
     *   </li>
     *   <li>
     *    REF: FM_8POINT for an 8-point algorithm. \(N \ge 8\)
     *   </li>
     *   <li>
     *    REF: FM_RANSAC for the RANSAC algorithm. \(N \ge 8\)
     *   </li>
     *   <li>
     *    REF: FM_LMEDS for the LMedS algorithm. \(N \ge 8\)
     * @param ransacReprojThreshold Parameter used only for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param confidence Parameter used for the RANSAC and LMedS methods only. It specifies a desirable level
     * of confidence (probability) that the estimated matrix is correct.
     * @param maxIters The maximum number of robust method iterations.
     *   </li>
     * </ul>
     *
     * The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T F [p_1; 1] = 0\)
     *
     * where \(F\) is a fundamental matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively.
     *
     * The function calculates the fundamental matrix using one of four methods listed above and returns
     * the found fundamental matrix. Normally just one matrix is found. But in case of the 7-point
     * algorithm, the function may return up to 3 solutions ( \(9 \times 3\) matrix that stores all 3
     * matrices sequentially).
     *
     * The calculated fundamental matrix may be passed further to #computeCorrespondEpilines that finds the
     * epipolar lines corresponding to the specified points. It can also be passed to
     * #stereoRectifyUncalibrated to compute the rectification transformation. :
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     Mat fundamental_matrix =
     *      findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
     * </code>
     * @return automatically generated
     */
    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, int method, double ransacReprojThreshold, double confidence, int maxIters) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_1(points1_mat.nativeObj, points2_mat.nativeObj, method, ransacReprojThreshold, confidence, maxIters));
    }


    //
    // C++:  Mat cv::findFundamentalMat(vector_Point2f points1, vector_Point2f points2, int method = FM_RANSAC, double ransacReprojThreshold = 3., double confidence = 0.99, Mat& mask = Mat())
    //

    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, int method, double ransacReprojThreshold, double confidence, Mat mask) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_2(points1_mat.nativeObj, points2_mat.nativeObj, method, ransacReprojThreshold, confidence, mask.nativeObj));
    }

    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, int method, double ransacReprojThreshold, double confidence) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_3(points1_mat.nativeObj, points2_mat.nativeObj, method, ransacReprojThreshold, confidence));
    }

    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, int method, double ransacReprojThreshold) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_4(points1_mat.nativeObj, points2_mat.nativeObj, method, ransacReprojThreshold));
    }

    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, int method) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_5(points1_mat.nativeObj, points2_mat.nativeObj, method));
    }

    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_6(points1_mat.nativeObj, points2_mat.nativeObj));
    }


    //
    // C++:  Mat cv::findFundamentalMat(vector_Point2f points1, vector_Point2f points2, Mat& mask, UsacParams params)
    //

    public static Mat findFundamentalMat(MatOfPoint2f points1, MatOfPoint2f points2, Mat mask, UsacParams params) {
        Mat points1_mat = points1;
        Mat points2_mat = points2;
        return new Mat(findFundamentalMat_7(points1_mat.nativeObj, points2_mat.nativeObj, mask.nativeObj, params.nativeObj));
    }


    //
    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method = RANSAC, double prob = 0.999, double threshold = 1.0, int maxIters = 1000, Mat& mask = Mat())
    //

    /**
     * Calculates an essential matrix from the corresponding points in two images.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera intrinsic matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param mask Output array of N elements, every element of which is set to 0 for outliers and to 1
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     * @param maxIters The maximum number of robust method iterations.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method, double prob, double threshold, int maxIters, Mat mask) {
        return new Mat(findEssentialMat_0(points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, method, prob, threshold, maxIters, mask.nativeObj));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera intrinsic matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     * @param maxIters The maximum number of robust method iterations.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method, double prob, double threshold, int maxIters) {
        return new Mat(findEssentialMat_1(points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, method, prob, threshold, maxIters));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera intrinsic matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method, double prob, double threshold) {
        return new Mat(findEssentialMat_2(points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, method, prob, threshold));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera intrinsic matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method, double prob) {
        return new Mat(findEssentialMat_3(points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, method, prob));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera intrinsic matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method) {
        return new Mat(findEssentialMat_4(points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, method));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera intrinsic matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix) {
        return new Mat(findEssentialMat_5(points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj));
    }


    //
    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, double focal = 1.0, Point2d pp = Point2d(0, 0), int method = RANSAC, double prob = 0.999, double threshold = 1.0, int maxIters = 1000, Mat& mask = Mat())
    //

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param mask Output array of N elements, every element of which is set to 0 for outliers and to 1
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     * @param maxIters The maximum number of robust method iterations.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal, Point pp, int method, double prob, double threshold, int maxIters, Mat mask) {
        return new Mat(findEssentialMat_6(points1.nativeObj, points2.nativeObj, focal, pp.x, pp.y, method, prob, threshold, maxIters, mask.nativeObj));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     * @param maxIters The maximum number of robust method iterations.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal, Point pp, int method, double prob, double threshold, int maxIters) {
        return new Mat(findEssentialMat_7(points1.nativeObj, points2.nativeObj, focal, pp.x, pp.y, method, prob, threshold, maxIters));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal, Point pp, int method, double prob, double threshold) {
        return new Mat(findEssentialMat_8(points1.nativeObj, points2.nativeObj, focal, pp.x, pp.y, method, prob, threshold));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal, Point pp, int method, double prob) {
        return new Mat(findEssentialMat_9(points1.nativeObj, points2.nativeObj, focal, pp.x, pp.y, method, prob));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * @param method Method for computing a fundamental matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal, Point pp, int method) {
        return new Mat(findEssentialMat_10(points1.nativeObj, points2.nativeObj, focal, pp.x, pp.y, method));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal, Point pp) {
        return new Mat(findEssentialMat_11(points1.nativeObj, points2.nativeObj, focal, pp.x, pp.y));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param focal focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, double focal) {
        return new Mat(findEssentialMat_12(points1.nativeObj, points2.nativeObj, focal));
    }

    /**
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * are feature points from cameras with same focal length and principal point.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * confidence (probability) that the estimated matrix is correct.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2) {
        return new Mat(findEssentialMat_13(points1.nativeObj, points2.nativeObj));
    }


    //
    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, int method = RANSAC, double prob = 0.999, double threshold = 1.0, Mat& mask = Mat())
    //

    /**
     * Calculates an essential matrix from the corresponding points in two images from potentially two different cameras.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param cameraMatrix2 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param distCoeffs1 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param distCoeffs2 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param mask Output array of N elements, every element of which is set to 0 for outliers and to 1
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or  #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, int method, double prob, double threshold, Mat mask) {
        return new Mat(findEssentialMat_14(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, method, prob, threshold, mask.nativeObj));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images from potentially two different cameras.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param cameraMatrix2 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param distCoeffs1 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param distCoeffs2 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or  #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, int method, double prob, double threshold) {
        return new Mat(findEssentialMat_15(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, method, prob, threshold));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images from potentially two different cameras.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param cameraMatrix2 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param distCoeffs1 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param distCoeffs2 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or  #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, int method, double prob) {
        return new Mat(findEssentialMat_16(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, method, prob));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images from potentially two different cameras.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param cameraMatrix2 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param distCoeffs1 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param distCoeffs2 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or  #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, int method) {
        return new Mat(findEssentialMat_17(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, method));
    }

    /**
     * Calculates an essential matrix from the corresponding points in two images from potentially two different cameras.
     *
     * @param points1 Array of N (N &gt;= 5) 2D points from the first image. The point coordinates should
     * be floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param cameraMatrix2 Camera matrix \(K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera matrix. If this assumption does not hold for your use case, use
     * #undistortPoints with {@code P = cv::NoArray()} for both cameras to transform image points
     * to normalized image coordinates, which are valid for the identity camera matrix. When
     * passing these coordinates, pass the identity matrix for this parameter.
     * @param distCoeffs1 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param distCoeffs2 Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * for the other points. The array is computed only in the RANSAC and LMedS methods.
     *   </li>
     * </ul>
     *
     * This function estimates essential matrix based on the five-point algorithm solver in CITE: Nister03 .
     * CITE: SteweniusCFS is also a related. The epipolar geometry is described by the following equation:
     *
     * \([p_2; 1]^T K^{-T} E K^{-1} [p_1; 1] = 0\)
     *
     * where \(E\) is an essential matrix, \(p_1\) and \(p_2\) are corresponding points in the first and the
     * second images, respectively. The result of this function may be passed further to
     * #decomposeEssentialMat or  #recoverPose to recover the relative pose between cameras.
     * @return automatically generated
     */
    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2) {
        return new Mat(findEssentialMat_18(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj));
    }


    //
    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat cameraMatrix2, Mat dist_coeff1, Mat dist_coeff2, Mat& mask, UsacParams params)
    //

    public static Mat findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat cameraMatrix2, Mat dist_coeff1, Mat dist_coeff2, Mat mask, UsacParams params) {
        return new Mat(findEssentialMat_19(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, cameraMatrix2.nativeObj, dist_coeff1.nativeObj, dist_coeff2.nativeObj, mask.nativeObj, params.nativeObj));
    }


    //
    // C++:  void cv::decomposeEssentialMat(Mat E, Mat& R1, Mat& R2, Mat& t)
    //

    /**
     * Decompose an essential matrix to possible rotations and translation.
     *
     * @param E The input essential matrix.
     * @param R1 One possible rotation matrix.
     * @param R2 Another possible rotation matrix.
     * @param t One possible translation.
     *
     * This function decomposes the essential matrix E using svd decomposition CITE: HartleyZ00. In
     * general, four possible poses exist for the decomposition of E. They are \([R_1, t]\),
     * \([R_1, -t]\), \([R_2, t]\), \([R_2, -t]\).
     *
     * If E gives the epipolar constraint \([p_2; 1]^T A^{-T} E A^{-1} [p_1; 1] = 0\) between the image
     * points \(p_1\) in the first image and \(p_2\) in second image, then any of the tuples
     * \([R_1, t]\), \([R_1, -t]\), \([R_2, t]\), \([R_2, -t]\) is a change of basis from the first
     * camera's coordinate system to the second camera's coordinate system. However, by decomposing E, one
     * can only get the direction of the translation. For this reason, the translation t is returned with
     * unit length.
     */
    public static void decomposeEssentialMat(Mat E, Mat R1, Mat R2, Mat t) {
        decomposeEssentialMat_0(E.nativeObj, R1.nativeObj, R2.nativeObj, t.nativeObj);
    }


    //
    // C++:  int cv::recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat& E, Mat& R, Mat& t, int method = cv::RANSAC, double prob = 0.999, double threshold = 1.0, Mat& mask = Mat())
    //

    /**
     * Recovers the relative camera rotation and the translation from corresponding points in two images from two different cameras, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs2 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param E The output essential matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * @param mask Input/output mask for inliers in points1 and points2. If it is not empty, then it marks
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *   </li>
     * </ul>
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for findEssentialMat.:
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // Input: camera calibration of both cameras, for example using intrinsic chessboard calibration.
     *     Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
     *
     *     // Output: Essential matrix, relative rotation and relative translation.
     *     Mat E, R, t, mask;
     *
     *     recoverPose(points1, points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, E, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat E, Mat R, Mat t, int method, double prob, double threshold, Mat mask) {
        return recoverPose_0(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, E.nativeObj, R.nativeObj, t.nativeObj, method, prob, threshold, mask.nativeObj);
    }

    /**
     * Recovers the relative camera rotation and the translation from corresponding points in two images from two different cameras, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs2 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param E The output essential matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * @param threshold Parameter used for RANSAC. It is the maximum distance from a point to an epipolar
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *   </li>
     * </ul>
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for findEssentialMat.:
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // Input: camera calibration of both cameras, for example using intrinsic chessboard calibration.
     *     Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
     *
     *     // Output: Essential matrix, relative rotation and relative translation.
     *     Mat E, R, t, mask;
     *
     *     recoverPose(points1, points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, E, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat E, Mat R, Mat t, int method, double prob, double threshold) {
        return recoverPose_1(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, E.nativeObj, R.nativeObj, t.nativeObj, method, prob, threshold);
    }

    /**
     * Recovers the relative camera rotation and the translation from corresponding points in two images from two different cameras, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs2 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param E The output essential matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * @param prob Parameter used for the RANSAC or LMedS methods only. It specifies a desirable level of
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *   </li>
     * </ul>
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for findEssentialMat.:
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // Input: camera calibration of both cameras, for example using intrinsic chessboard calibration.
     *     Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
     *
     *     // Output: Essential matrix, relative rotation and relative translation.
     *     Mat E, R, t, mask;
     *
     *     recoverPose(points1, points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, E, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat E, Mat R, Mat t, int method, double prob) {
        return recoverPose_2(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, E.nativeObj, R.nativeObj, t.nativeObj, method, prob);
    }

    /**
     * Recovers the relative camera rotation and the translation from corresponding points in two images from two different cameras, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs2 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param E The output essential matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param method Method for computing an essential matrix.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *   </li>
     * </ul>
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for findEssentialMat.:
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // Input: camera calibration of both cameras, for example using intrinsic chessboard calibration.
     *     Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
     *
     *     // Output: Essential matrix, relative rotation and relative translation.
     *     Mat E, R, t, mask;
     *
     *     recoverPose(points1, points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, E, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat E, Mat R, Mat t, int method) {
        return recoverPose_3(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, E.nativeObj, R.nativeObj, t.nativeObj, method);
    }

    /**
     * Recovers the relative camera rotation and the translation from corresponding points in two images from two different cameras, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix1 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs1 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param cameraMatrix2 Input/output camera matrix for the first camera, the same as in
     * REF: calibrateCamera. Furthermore, for the stereo case, additional flags may be used, see below.
     * @param distCoeffs2 Input/output vector of distortion coefficients, the same as in
     * REF: calibrateCamera.
     * @param E The output essential matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * <ul>
     *   <li>
     *    REF: RANSAC for the RANSAC algorithm.
     *   </li>
     *   <li>
     *    REF: LMEDS for the LMedS algorithm.
     * confidence (probability) that the estimated matrix is correct.
     * line in pixels, beyond which the point is considered an outlier and is not used for computing the
     * final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the
     * point localization, image resolution, and the image noise.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *   </li>
     * </ul>
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for findEssentialMat.:
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // Input: camera calibration of both cameras, for example using intrinsic chessboard calibration.
     *     Mat cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2;
     *
     *     // Output: Essential matrix, relative rotation and relative translation.
     *     Mat E, R, t, mask;
     *
     *     recoverPose(points1, points2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, E, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat E, Mat R, Mat t) {
        return recoverPose_4(points1.nativeObj, points2.nativeObj, cameraMatrix1.nativeObj, distCoeffs1.nativeObj, cameraMatrix2.nativeObj, distCoeffs2.nativeObj, E.nativeObj, R.nativeObj, t.nativeObj);
    }


    //
    // C++:  int cv::recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat& R, Mat& t, Mat& mask = Mat())
    //

    /**
     * Recovers the relative camera rotation and the translation from an estimated essential
     * matrix and the corresponding points in two images, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param mask Input/output mask for inliers in points1 and points2. If it is not empty, then it marks
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for #findEssentialMat :
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // cametra matrix with both focal lengths = 1, and principal point = (0, 0)
     *     Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
     *
     *     Mat E, R, t, mask;
     *
     *     E = findEssentialMat(points1, points2, cameraMatrix, RANSAC, 0.999, 1.0, mask);
     *     recoverPose(E, points1, points2, cameraMatrix, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat R, Mat t, Mat mask) {
        return recoverPose_5(E.nativeObj, points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, R.nativeObj, t.nativeObj, mask.nativeObj);
    }

    /**
     * Recovers the relative camera rotation and the translation from an estimated essential
     * matrix and the corresponding points in two images, using chirality check. Returns the number of
     * inliers that pass the check.
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * described below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function decomposes an essential matrix using REF: decomposeEssentialMat and then verifies
     * possible pose hypotheses by doing chirality check. The chirality check means that the
     * triangulated 3D points should have positive depth. Some details can be found in CITE: Nister03.
     *
     * This function can be used to process the output E and mask from REF: findEssentialMat. In this
     * scenario, points1 and points2 are the same input for #findEssentialMat :
     * <code>
     *     // Example. Estimation of fundamental matrix using the RANSAC algorithm
     *     int point_count = 100;
     *     vector&lt;Point2f&gt; points1(point_count);
     *     vector&lt;Point2f&gt; points2(point_count);
     *
     *     // initialize the points here ...
     *     for( int i = 0; i &lt; point_count; i++ )
     *     {
     *         points1[i] = ...;
     *         points2[i] = ...;
     *     }
     *
     *     // cametra matrix with both focal lengths = 1, and principal point = (0, 0)
     *     Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
     *
     *     Mat E, R, t, mask;
     *
     *     E = findEssentialMat(points1, points2, cameraMatrix, RANSAC, 0.999, 1.0, mask);
     *     recoverPose(E, points1, points2, cameraMatrix, R, t, mask);
     * </code>
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat R, Mat t) {
        return recoverPose_6(E.nativeObj, points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, R.nativeObj, t.nativeObj);
    }


    //
    // C++:  int cv::recoverPose(Mat E, Mat points1, Mat points2, Mat& R, Mat& t, double focal = 1.0, Point2d pp = Point2d(0, 0), Mat& mask = Mat())
    //

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param focal Focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * @param mask Input/output mask for inliers in points1 and points2. If it is not empty, then it marks
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat R, Mat t, double focal, Point pp, Mat mask) {
        return recoverPose_7(E.nativeObj, points1.nativeObj, points2.nativeObj, R.nativeObj, t.nativeObj, focal, pp.x, pp.y, mask.nativeObj);
    }

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param focal Focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * @param pp principal point of the camera.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat R, Mat t, double focal, Point pp) {
        return recoverPose_8(E.nativeObj, points1.nativeObj, points2.nativeObj, R.nativeObj, t.nativeObj, focal, pp.x, pp.y);
    }

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param focal Focal length of the camera. Note that this function assumes that points1 and points2
     * are feature points from cameras with same focal length and principal point.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat R, Mat t, double focal) {
        return recoverPose_9(E.nativeObj, points1.nativeObj, points2.nativeObj, R.nativeObj, t.nativeObj, focal);
    }

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1 .
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * are feature points from cameras with same focal length and principal point.
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function differs from the one above that it computes camera intrinsic matrix from focal length and
     * principal point:
     *
     * \(A =
     * \begin{bmatrix}
     * f &amp; 0 &amp; x_{pp}  \\
     * 0 &amp; f &amp; y_{pp}  \\
     * 0 &amp; 0 &amp; 1
     * \end{bmatrix}\)
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat R, Mat t) {
        return recoverPose_10(E.nativeObj, points1.nativeObj, points2.nativeObj, R.nativeObj, t.nativeObj);
    }


    //
    // C++:  int cv::recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat& R, Mat& t, double distanceThresh, Mat& mask = Mat(), Mat& triangulatedPoints = Mat())
    //

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1.
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param distanceThresh threshold distance which is used to filter out far away points (i.e. infinite
     * points).
     * @param mask Input/output mask for inliers in points1 and points2. If it is not empty, then it marks
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     * @param triangulatedPoints 3D points which were reconstructed by triangulation.
     *
     * This function differs from the one above that it outputs the triangulated 3D point that are used for
     * the chirality check.
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat R, Mat t, double distanceThresh, Mat mask, Mat triangulatedPoints) {
        return recoverPose_11(E.nativeObj, points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, R.nativeObj, t.nativeObj, distanceThresh, mask.nativeObj, triangulatedPoints.nativeObj);
    }

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1.
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param distanceThresh threshold distance which is used to filter out far away points (i.e. infinite
     * points).
     * @param mask Input/output mask for inliers in points1 and points2. If it is not empty, then it marks
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function differs from the one above that it outputs the triangulated 3D point that are used for
     * the chirality check.
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat R, Mat t, double distanceThresh, Mat mask) {
        return recoverPose_12(E.nativeObj, points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, R.nativeObj, t.nativeObj, distanceThresh, mask.nativeObj);
    }

    /**
     *
     * @param E The input essential matrix.
     * @param points1 Array of N 2D points from the first image. The point coordinates should be
     * floating-point (single or double precision).
     * @param points2 Array of the second image points of the same size and format as points1.
     * @param cameraMatrix Camera intrinsic matrix \(\cameramatrix{A}\) .
     * Note that this function assumes that points1 and points2 are feature points from cameras with the
     * same camera intrinsic matrix.
     * @param R Output rotation matrix. Together with the translation vector, this matrix makes up a tuple
     * that performs a change of basis from the first camera's coordinate system to the second camera's
     * coordinate system. Note that, in general, t can not be used for this tuple, see the parameter
     * description below.
     * @param t Output translation vector. This vector is obtained by REF: decomposeEssentialMat and
     * therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit
     * length.
     * @param distanceThresh threshold distance which is used to filter out far away points (i.e. infinite
     * points).
     * inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to
     * recover pose. In the output mask only inliers which pass the chirality check.
     *
     * This function differs from the one above that it outputs the triangulated 3D point that are used for
     * the chirality check.
     * @return automatically generated
     */
    public static int recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat R, Mat t, double distanceThresh) {
        return recoverPose_13(E.nativeObj, points1.nativeObj, points2.nativeObj, cameraMatrix.nativeObj, R.nativeObj, t.nativeObj, distanceThresh);
    }


    //
    // C++:  void cv::computeCorrespondEpilines(Mat points, int whichImage, Mat F, Mat& lines)
    //

    /**
     * For points in an image of a stereo pair, computes the corresponding epilines in the other image.
     *
     * @param points Input points. \(N \times 1\) or \(1 \times N\) matrix of type CV_32FC2 or
     * vector&lt;Point2f&gt; .
     * @param whichImage Index of the image (1 or 2) that contains the points .
     * @param F Fundamental matrix that can be estimated using #findFundamentalMat or #stereoRectify .
     * @param lines Output vector of the epipolar lines corresponding to the points in the other image.
     * Each line \(ax + by + c=0\) is encoded by 3 numbers \((a, b, c)\) .
     *
     * For every point in one of the two images of a stereo pair, the function finds the equation of the
     * corresponding epipolar line in the other image.
     *
     * From the fundamental matrix definition (see #findFundamentalMat ), line \(l^{(2)}_i\) in the second
     * image for the point \(p^{(1)}_i\) in the first image (when whichImage=1 ) is computed as:
     *
     * \(l^{(2)}_i = F p^{(1)}_i\)
     *
     * And vice versa, when whichImage=2, \(l^{(1)}_i\) is computed from \(p^{(2)}_i\) as:
     *
     * \(l^{(1)}_i = F^T p^{(2)}_i\)
     *
     * Line coefficients are defined up to a scale. They are normalized so that \(a_i^2+b_i^2=1\) .
     */
    public static void computeCorrespondEpilines(Mat points, int whichImage, Mat F, Mat lines) {
        computeCorrespondEpilines_0(points.nativeObj, whichImage, F.nativeObj, lines.nativeObj);
    }


    //
    // C++:  void cv::triangulatePoints(Mat projMatr1, Mat projMatr2, Mat projPoints1, Mat projPoints2, Mat& points4D)
    //

    /**
     * This function reconstructs 3-dimensional points (in homogeneous coordinates) by using
     * their observations with a stereo camera.
     *
     * @param projMatr1 3x4 projection matrix of the first camera, i.e. this matrix projects 3D points
     * given in the world's coordinate system into the first image.
     * @param projMatr2 3x4 projection matrix of the second camera, i.e. this matrix projects 3D points
     * given in the world's coordinate system into the second image.
     * @param projPoints1 2xN array of feature points in the first image. In the case of the c++ version,
     * it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1.
     * @param projPoints2 2xN array of corresponding points in the second image. In the case of the c++
     * version, it can be also a vector of feature points or two-channel matrix of size 1xN or Nx1.
     * @param points4D 4xN array of reconstructed points in homogeneous coordinates. These points are
     * returned in the world's coordinate system.
     *
     * <b>Note:</b>
     *    Keep in mind that all input data should be of float type in order for this function to work.
     *
     * <b>Note:</b>
     *    If the projection matrices from REF: stereoRectify are used, then the returned points are
     *    represented in the first camera's rectified coordinate system.
     *
     * SEE:
     *    reprojectImageTo3D
     */
    public static void triangulatePoints(Mat projMatr1, Mat projMatr2, Mat projPoints1, Mat projPoints2, Mat points4D) {
        triangulatePoints_0(projMatr1.nativeObj, projMatr2.nativeObj, projPoints1.nativeObj, projPoints2.nativeObj, points4D.nativeObj);
    }


    //
    // C++:  void cv::correctMatches(Mat F, Mat points1, Mat points2, Mat& newPoints1, Mat& newPoints2)
    //

    /**
     * Refines coordinates of corresponding points.
     *
     * @param F 3x3 fundamental matrix.
     * @param points1 1xN array containing the first set of points.
     * @param points2 1xN array containing the second set of points.
     * @param newPoints1 The optimized points1.
     * @param newPoints2 The optimized points2.
     *
     * The function implements the Optimal Triangulation Method (see Multiple View Geometry CITE: HartleyZ00 for details).
     * For each given point correspondence points1[i] &lt;-&gt; points2[i], and a fundamental matrix F, it
     * computes the corrected correspondences newPoints1[i] &lt;-&gt; newPoints2[i] that minimize the geometric
     * error \(d(points1[i], newPoints1[i])^2 + d(points2[i],newPoints2[i])^2\) (where \(d(a,b)\) is the
     * geometric distance between points \(a\) and \(b\) ) subject to the epipolar constraint
     * \(newPoints2^T \cdot F \cdot newPoints1 = 0\) .
     */
    public static void correctMatches(Mat F, Mat points1, Mat points2, Mat newPoints1, Mat newPoints2) {
        correctMatches_0(F.nativeObj, points1.nativeObj, points2.nativeObj, newPoints1.nativeObj, newPoints2.nativeObj);
    }


    //
    // C++:  double cv::sampsonDistance(Mat pt1, Mat pt2, Mat F)
    //

    /**
     * Calculates the Sampson Distance between two points.
     *
     * The function cv::sampsonDistance calculates and returns the first order approximation of the geometric error as:
     * \(
     * sd( \texttt{pt1} , \texttt{pt2} )=
     * \frac{(\texttt{pt2}^t \cdot \texttt{F} \cdot \texttt{pt1})^2}
     * {((\texttt{F} \cdot \texttt{pt1})(0))^2 +
     * ((\texttt{F} \cdot \texttt{pt1})(1))^2 +
     * ((\texttt{F}^t \cdot \texttt{pt2})(0))^2 +
     * ((\texttt{F}^t \cdot \texttt{pt2})(1))^2}
     * \)
     * The fundamental matrix may be calculated using the #findFundamentalMat function. See CITE: HartleyZ00 11.4.3 for details.
     * @param pt1 first homogeneous 2d point
     * @param pt2 second homogeneous 2d point
     * @param F fundamental matrix
     * @return The computed Sampson distance.
     */
    public static double sampsonDistance(Mat pt1, Mat pt2, Mat F) {
        return sampsonDistance_0(pt1.nativeObj, pt2.nativeObj, F.nativeObj);
    }


    //
    // C++:  int cv::estimateAffine3D(Mat src, Mat dst, Mat& out, Mat& inliers, double ransacThreshold = 3, double confidence = 0.99)
    //

    /**
     * Computes an optimal affine transformation between two 3D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * z\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; a_{13}\\
     * a_{21} &amp; a_{22} &amp; a_{23}\\
     * a_{31} &amp; a_{32} &amp; a_{33}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * Z\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * b_3\\
     * \end{bmatrix}
     * \)
     *
     * @param src First input 3D point set containing \((X,Y,Z)\).
     * @param dst Second input 3D point set containing \((x,y,z)\).
     * @param out Output 3D affine transformation matrix \(3 \times 4\) of the form
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; a_{13} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; a_{23} &amp; b_2\\
     * a_{31} &amp; a_{32} &amp; a_{33} &amp; b_3\\
     * \end{bmatrix}
     * \)
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param ransacThreshold Maximum reprojection error in the RANSAC algorithm to consider a point as
     * an inlier.
     * @param confidence Confidence level, between 0 and 1, for the estimated transformation. Anything
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     *
     * The function estimates an optimal 3D affine transformation between two 3D point sets using the
     * RANSAC algorithm.
     * @return automatically generated
     */
    public static int estimateAffine3D(Mat src, Mat dst, Mat out, Mat inliers, double ransacThreshold, double confidence) {
        return estimateAffine3D_0(src.nativeObj, dst.nativeObj, out.nativeObj, inliers.nativeObj, ransacThreshold, confidence);
    }

    /**
     * Computes an optimal affine transformation between two 3D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * z\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; a_{13}\\
     * a_{21} &amp; a_{22} &amp; a_{23}\\
     * a_{31} &amp; a_{32} &amp; a_{33}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * Z\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * b_3\\
     * \end{bmatrix}
     * \)
     *
     * @param src First input 3D point set containing \((X,Y,Z)\).
     * @param dst Second input 3D point set containing \((x,y,z)\).
     * @param out Output 3D affine transformation matrix \(3 \times 4\) of the form
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; a_{13} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; a_{23} &amp; b_2\\
     * a_{31} &amp; a_{32} &amp; a_{33} &amp; b_3\\
     * \end{bmatrix}
     * \)
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param ransacThreshold Maximum reprojection error in the RANSAC algorithm to consider a point as
     * an inlier.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     *
     * The function estimates an optimal 3D affine transformation between two 3D point sets using the
     * RANSAC algorithm.
     * @return automatically generated
     */
    public static int estimateAffine3D(Mat src, Mat dst, Mat out, Mat inliers, double ransacThreshold) {
        return estimateAffine3D_1(src.nativeObj, dst.nativeObj, out.nativeObj, inliers.nativeObj, ransacThreshold);
    }

    /**
     * Computes an optimal affine transformation between two 3D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * z\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; a_{13}\\
     * a_{21} &amp; a_{22} &amp; a_{23}\\
     * a_{31} &amp; a_{32} &amp; a_{33}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * Z\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * b_3\\
     * \end{bmatrix}
     * \)
     *
     * @param src First input 3D point set containing \((X,Y,Z)\).
     * @param dst Second input 3D point set containing \((x,y,z)\).
     * @param out Output 3D affine transformation matrix \(3 \times 4\) of the form
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; a_{13} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; a_{23} &amp; b_2\\
     * a_{31} &amp; a_{32} &amp; a_{33} &amp; b_3\\
     * \end{bmatrix}
     * \)
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * an inlier.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     *
     * The function estimates an optimal 3D affine transformation between two 3D point sets using the
     * RANSAC algorithm.
     * @return automatically generated
     */
    public static int estimateAffine3D(Mat src, Mat dst, Mat out, Mat inliers) {
        return estimateAffine3D_2(src.nativeObj, dst.nativeObj, out.nativeObj, inliers.nativeObj);
    }


    //
    // C++:  Mat cv::estimateAffine3D(Mat src, Mat dst, double* scale = nullptr, bool force_rotation = true)
    //

    /**
     * Computes an optimal affine transformation between two 3D point sets.
     *
     * It computes \(R,s,t\) minimizing \(\sum{i} dst_i - c \cdot R \cdot src_i \)
     * where \(R\) is a 3x3 rotation matrix, \(t\) is a 3x1 translation vector and \(s\) is a
     * scalar size value. This is an implementation of the algorithm by Umeyama \cite umeyama1991least .
     * The estimated affine transform has a homogeneous scale which is a subclass of affine
     * transformations with 7 degrees of freedom. The paired point sets need to comprise at least 3
     * points each.
     *
     * @param src First input 3D point set.
     * @param dst Second input 3D point set.
     * @param scale If null is passed, the scale parameter c will be assumed to be 1.0.
     * Else the pointed-to variable will be set to the optimal scale.
     * @param force_rotation If true, the returned rotation will never be a reflection.
     * This might be unwanted, e.g. when optimizing a transform between a right- and a
     * left-handed coordinate system.
     * @return 3D affine transformation matrix \(3 \times 4\) of the form
     * \(T =
     * \begin{bmatrix}
     * R &amp; t\\
     * \end{bmatrix}
     * \)
     */
    public static Mat estimateAffine3D(Mat src, Mat dst, double[] scale, boolean force_rotation) {
        double[] scale_out = new double[1];
        Mat retVal = new Mat(estimateAffine3D_3(src.nativeObj, dst.nativeObj, scale_out, force_rotation));
        if(scale!=null) scale[0] = (double)scale_out[0];
        return retVal;
    }

    /**
     * Computes an optimal affine transformation between two 3D point sets.
     *
     * It computes \(R,s,t\) minimizing \(\sum{i} dst_i - c \cdot R \cdot src_i \)
     * where \(R\) is a 3x3 rotation matrix, \(t\) is a 3x1 translation vector and \(s\) is a
     * scalar size value. This is an implementation of the algorithm by Umeyama \cite umeyama1991least .
     * The estimated affine transform has a homogeneous scale which is a subclass of affine
     * transformations with 7 degrees of freedom. The paired point sets need to comprise at least 3
     * points each.
     *
     * @param src First input 3D point set.
     * @param dst Second input 3D point set.
     * @param scale If null is passed, the scale parameter c will be assumed to be 1.0.
     * Else the pointed-to variable will be set to the optimal scale.
     * This might be unwanted, e.g. when optimizing a transform between a right- and a
     * left-handed coordinate system.
     * @return 3D affine transformation matrix \(3 \times 4\) of the form
     * \(T =
     * \begin{bmatrix}
     * R &amp; t\\
     * \end{bmatrix}
     * \)
     */
    public static Mat estimateAffine3D(Mat src, Mat dst, double[] scale) {
        double[] scale_out = new double[1];
        Mat retVal = new Mat(estimateAffine3D_4(src.nativeObj, dst.nativeObj, scale_out));
        if(scale!=null) scale[0] = (double)scale_out[0];
        return retVal;
    }

    /**
     * Computes an optimal affine transformation between two 3D point sets.
     *
     * It computes \(R,s,t\) minimizing \(\sum{i} dst_i - c \cdot R \cdot src_i \)
     * where \(R\) is a 3x3 rotation matrix, \(t\) is a 3x1 translation vector and \(s\) is a
     * scalar size value. This is an implementation of the algorithm by Umeyama \cite umeyama1991least .
     * The estimated affine transform has a homogeneous scale which is a subclass of affine
     * transformations with 7 degrees of freedom. The paired point sets need to comprise at least 3
     * points each.
     *
     * @param src First input 3D point set.
     * @param dst Second input 3D point set.
     * Else the pointed-to variable will be set to the optimal scale.
     * This might be unwanted, e.g. when optimizing a transform between a right- and a
     * left-handed coordinate system.
     * @return 3D affine transformation matrix \(3 \times 4\) of the form
     * \(T =
     * \begin{bmatrix}
     * R &amp; t\\
     * \end{bmatrix}
     * \)
     */
    public static Mat estimateAffine3D(Mat src, Mat dst) {
        return new Mat(estimateAffine3D_5(src.nativeObj, dst.nativeObj));
    }


    //
    // C++:  int cv::estimateTranslation3D(Mat src, Mat dst, Mat& out, Mat& inliers, double ransacThreshold = 3, double confidence = 0.99)
    //

    /**
     * Computes an optimal translation between two 3D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * z\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * X\\
     * Y\\
     * Z\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * b_3\\
     * \end{bmatrix}
     * \)
     *
     * @param src First input 3D point set containing \((X,Y,Z)\).
     * @param dst Second input 3D point set containing \((x,y,z)\).
     * @param out Output 3D translation vector \(3 \times 1\) of the form
     * \(
     * \begin{bmatrix}
     * b_1 \\
     * b_2 \\
     * b_3 \\
     * \end{bmatrix}
     * \)
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param ransacThreshold Maximum reprojection error in the RANSAC algorithm to consider a point as
     * an inlier.
     * @param confidence Confidence level, between 0 and 1, for the estimated transformation. Anything
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     *
     * The function estimates an optimal 3D translation between two 3D point sets using the
     * RANSAC algorithm.
     *
     * @return automatically generated
     */
    public static int estimateTranslation3D(Mat src, Mat dst, Mat out, Mat inliers, double ransacThreshold, double confidence) {
        return estimateTranslation3D_0(src.nativeObj, dst.nativeObj, out.nativeObj, inliers.nativeObj, ransacThreshold, confidence);
    }

    /**
     * Computes an optimal translation between two 3D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * z\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * X\\
     * Y\\
     * Z\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * b_3\\
     * \end{bmatrix}
     * \)
     *
     * @param src First input 3D point set containing \((X,Y,Z)\).
     * @param dst Second input 3D point set containing \((x,y,z)\).
     * @param out Output 3D translation vector \(3 \times 1\) of the form
     * \(
     * \begin{bmatrix}
     * b_1 \\
     * b_2 \\
     * b_3 \\
     * \end{bmatrix}
     * \)
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param ransacThreshold Maximum reprojection error in the RANSAC algorithm to consider a point as
     * an inlier.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     *
     * The function estimates an optimal 3D translation between two 3D point sets using the
     * RANSAC algorithm.
     *
     * @return automatically generated
     */
    public static int estimateTranslation3D(Mat src, Mat dst, Mat out, Mat inliers, double ransacThreshold) {
        return estimateTranslation3D_1(src.nativeObj, dst.nativeObj, out.nativeObj, inliers.nativeObj, ransacThreshold);
    }

    /**
     * Computes an optimal translation between two 3D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * z\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * X\\
     * Y\\
     * Z\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * b_3\\
     * \end{bmatrix}
     * \)
     *
     * @param src First input 3D point set containing \((X,Y,Z)\).
     * @param dst Second input 3D point set containing \((x,y,z)\).
     * @param out Output 3D translation vector \(3 \times 1\) of the form
     * \(
     * \begin{bmatrix}
     * b_1 \\
     * b_2 \\
     * b_3 \\
     * \end{bmatrix}
     * \)
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * an inlier.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     *
     * The function estimates an optimal 3D translation between two 3D point sets using the
     * RANSAC algorithm.
     *
     * @return automatically generated
     */
    public static int estimateTranslation3D(Mat src, Mat dst, Mat out, Mat inliers) {
        return estimateTranslation3D_2(src.nativeObj, dst.nativeObj, out.nativeObj, inliers.nativeObj);
    }


    //
    // C++:  Mat cv::estimateAffine2D(Mat from, Mat to, Mat& inliers = Mat(), int method = RANSAC, double ransacReprojThreshold = 3, size_t maxIters = 2000, double confidence = 0.99, size_t refineIters = 10)
    //

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * @param maxIters The maximum number of robust method iterations.
     * @param confidence Confidence level, between 0 and 1, for the estimated transformation. Anything
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * @param refineIters Maximum number of iterations of refining algorithm (Levenberg-Marquardt).
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold, long maxIters, double confidence, long refineIters) {
        return new Mat(estimateAffine2D_0(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold, maxIters, confidence, refineIters));
    }

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * @param maxIters The maximum number of robust method iterations.
     * @param confidence Confidence level, between 0 and 1, for the estimated transformation. Anything
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold, long maxIters, double confidence) {
        return new Mat(estimateAffine2D_1(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold, maxIters, confidence));
    }

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * @param maxIters The maximum number of robust method iterations.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold, long maxIters) {
        return new Mat(estimateAffine2D_2(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold, maxIters));
    }

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold) {
        return new Mat(estimateAffine2D_3(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold));
    }

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to, Mat inliers, int method) {
        return new Mat(estimateAffine2D_4(from.nativeObj, to.nativeObj, inliers.nativeObj, method));
    }

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * @param inliers Output vector indicating which points are inliers (1-inlier, 0-outlier).
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to, Mat inliers) {
        return new Mat(estimateAffine2D_5(from.nativeObj, to.nativeObj, inliers.nativeObj));
    }

    /**
     * Computes an optimal affine transformation between two 2D point sets.
     *
     * It computes
     * \(
     * \begin{bmatrix}
     * x\\
     * y\\
     * \end{bmatrix}
     * =
     * \begin{bmatrix}
     * a_{11} &amp; a_{12}\\
     * a_{21} &amp; a_{22}\\
     * \end{bmatrix}
     * \begin{bmatrix}
     * X\\
     * Y\\
     * \end{bmatrix}
     * +
     * \begin{bmatrix}
     * b_1\\
     * b_2\\
     * \end{bmatrix}
     * \)
     *
     * @param from First input 2D point set containing \((X,Y)\).
     * @param to Second input 2D point set containing \((x,y)\).
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation matrix \(2 \times 3\) or empty matrix if transformation
     * could not be estimated. The returned matrix has the following form:
     * \(
     * \begin{bmatrix}
     * a_{11} &amp; a_{12} &amp; b_1\\
     * a_{21} &amp; a_{22} &amp; b_2\\
     * \end{bmatrix}
     * \)
     *
     * The function estimates an optimal 2D affine transformation between two 2D point sets using the
     * selected robust algorithm.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but needs a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffinePartial2D, getAffineTransform
     */
    public static Mat estimateAffine2D(Mat from, Mat to) {
        return new Mat(estimateAffine2D_6(from.nativeObj, to.nativeObj));
    }


    //
    // C++:  Mat cv::estimateAffine2D(Mat pts1, Mat pts2, Mat& inliers, UsacParams params)
    //

    public static Mat estimateAffine2D(Mat pts1, Mat pts2, Mat inliers, UsacParams params) {
        return new Mat(estimateAffine2D_7(pts1.nativeObj, pts2.nativeObj, inliers.nativeObj, params.nativeObj));
    }


    //
    // C++:  Mat cv::estimateAffinePartial2D(Mat from, Mat to, Mat& inliers = Mat(), int method = RANSAC, double ransacReprojThreshold = 3, size_t maxIters = 2000, double confidence = 0.99, size_t refineIters = 10)
    //

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * @param inliers Output vector indicating which points are inliers.
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * @param maxIters The maximum number of robust method iterations.
     * @param confidence Confidence level, between 0 and 1, for the estimated transformation. Anything
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * @param refineIters Maximum number of iterations of refining algorithm (Levenberg-Marquardt).
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold, long maxIters, double confidence, long refineIters) {
        return new Mat(estimateAffinePartial2D_0(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold, maxIters, confidence, refineIters));
    }

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * @param inliers Output vector indicating which points are inliers.
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * @param maxIters The maximum number of robust method iterations.
     * @param confidence Confidence level, between 0 and 1, for the estimated transformation. Anything
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold, long maxIters, double confidence) {
        return new Mat(estimateAffinePartial2D_1(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold, maxIters, confidence));
    }

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * @param inliers Output vector indicating which points are inliers.
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * @param maxIters The maximum number of robust method iterations.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold, long maxIters) {
        return new Mat(estimateAffinePartial2D_2(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold, maxIters));
    }

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * @param inliers Output vector indicating which points are inliers.
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * @param ransacReprojThreshold Maximum reprojection error in the RANSAC algorithm to consider
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to, Mat inliers, int method, double ransacReprojThreshold) {
        return new Mat(estimateAffinePartial2D_3(from.nativeObj, to.nativeObj, inliers.nativeObj, method, ransacReprojThreshold));
    }

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * @param inliers Output vector indicating which points are inliers.
     * @param method Robust method used to compute transformation. The following methods are possible:
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to, Mat inliers, int method) {
        return new Mat(estimateAffinePartial2D_4(from.nativeObj, to.nativeObj, inliers.nativeObj, method));
    }

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * @param inliers Output vector indicating which points are inliers.
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to, Mat inliers) {
        return new Mat(estimateAffinePartial2D_5(from.nativeObj, to.nativeObj, inliers.nativeObj));
    }

    /**
     * Computes an optimal limited affine transformation with 4 degrees of freedom between
     * two 2D point sets.
     *
     * @param from First input 2D point set.
     * @param to Second input 2D point set.
     * <ul>
     *   <li>
     *    REF: RANSAC - RANSAC-based robust method
     *   </li>
     *   <li>
     *    REF: LMEDS - Least-Median robust method
     * RANSAC is the default method.
     * a point as an inlier. Applies only to RANSAC.
     * between 0.95 and 0.99 is usually good enough. Values too close to 1 can slow down the estimation
     * significantly. Values lower than 0.8-0.9 can result in an incorrectly estimated transformation.
     * Passing 0 will disable refining, so the output matrix will be output of robust method.
     *   </li>
     * </ul>
     *
     * @return Output 2D affine transformation (4 degrees of freedom) matrix \(2 \times 3\) or
     * empty matrix if transformation could not be estimated.
     *
     * The function estimates an optimal 2D affine transformation with 4 degrees of freedom limited to
     * combinations of translation, rotation, and uniform scaling. Uses the selected algorithm for robust
     * estimation.
     *
     * The computed transformation is then refined further (using only inliers) with the
     * Levenberg-Marquardt method to reduce the re-projection error even more.
     *
     * Estimated transformation matrix is:
     * \( \begin{bmatrix} \cos(\theta) \cdot s &amp; -\sin(\theta) \cdot s &amp; t_x \\
     *                 \sin(\theta) \cdot s &amp; \cos(\theta) \cdot s &amp; t_y
     * \end{bmatrix} \)
     * Where \( \theta \) is the rotation angle, \( s \) the scaling factor and \( t_x, t_y \) are
     * translations in \( x, y \) axes respectively.
     *
     * <b>Note:</b>
     * The RANSAC method can handle practically any ratio of outliers but need a threshold to
     * distinguish inliers from outliers. The method LMeDS does not need any threshold but it works
     * correctly only when there are more than 50% of inliers.
     *
     * SEE: estimateAffine2D, getAffineTransform
     */
    public static Mat estimateAffinePartial2D(Mat from, Mat to) {
        return new Mat(estimateAffinePartial2D_6(from.nativeObj, to.nativeObj));
    }


    //
    // C++:  int cv::decomposeHomographyMat(Mat H, Mat K, vector_Mat& rotations, vector_Mat& translations, vector_Mat& normals)
    //

    /**
     * Decompose a homography matrix to rotation(s), translation(s) and plane normal(s).
     *
     * @param H The input homography matrix between two images.
     * @param K The input camera intrinsic matrix.
     * @param rotations Array of rotation matrices.
     * @param translations Array of translation matrices.
     * @param normals Array of plane normal matrices.
     *
     * This function extracts relative camera motion between two views of a planar object and returns up to
     * four mathematical solution tuples of rotation, translation, and plane normal. The decomposition of
     * the homography matrix H is described in detail in CITE: Malis2007.
     *
     * If the homography H, induced by the plane, gives the constraint
     * \(s_i \vecthree{x'_i}{y'_i}{1} \sim H \vecthree{x_i}{y_i}{1}\) on the source image points
     * \(p_i\) and the destination image points \(p'_i\), then the tuple of rotations[k] and
     * translations[k] is a change of basis from the source camera's coordinate system to the destination
     * camera's coordinate system. However, by decomposing H, one can only get the translation normalized
     * by the (typically unknown) depth of the scene, i.e. its direction but with normalized length.
     *
     * If point correspondences are available, at least two solutions may further be invalidated, by
     * applying positive depth constraint, i.e. all points must be in front of the camera.
     * @return automatically generated
     */
    public static int decomposeHomographyMat(Mat H, Mat K, List<Mat> rotations, List<Mat> translations, List<Mat> normals) {
        Mat rotations_mat = new Mat();
        Mat translations_mat = new Mat();
        Mat normals_mat = new Mat();
        int retVal = decomposeHomographyMat_0(H.nativeObj, K.nativeObj, rotations_mat.nativeObj, translations_mat.nativeObj, normals_mat.nativeObj);
        Converters.Mat_to_vector_Mat(rotations_mat, rotations);
        rotations_mat.release();
        Converters.Mat_to_vector_Mat(translations_mat, translations);
        translations_mat.release();
        Converters.Mat_to_vector_Mat(normals_mat, normals);
        normals_mat.release();
        return retVal;
    }


    //
    // C++:  void cv::filterHomographyDecompByVisibleRefpoints(vector_Mat rotations, vector_Mat normals, Mat beforePoints, Mat afterPoints, Mat& possibleSolutions, Mat pointsMask = Mat())
    //

    /**
     * Filters homography decompositions based on additional information.
     *
     * @param rotations Vector of rotation matrices.
     * @param normals Vector of plane normal matrices.
     * @param beforePoints Vector of (rectified) visible reference points before the homography is applied
     * @param afterPoints Vector of (rectified) visible reference points after the homography is applied
     * @param possibleSolutions Vector of int indices representing the viable solution set after filtering
     * @param pointsMask optional Mat/Vector of 8u type representing the mask for the inliers as given by the #findHomography function
     *
     * This function is intended to filter the output of the #decomposeHomographyMat based on additional
     * information as described in CITE: Malis2007 . The summary of the method: the #decomposeHomographyMat function
     * returns 2 unique solutions and their "opposites" for a total of 4 solutions. If we have access to the
     * sets of points visible in the camera frame before and after the homography transformation is applied,
     * we can determine which are the true potential solutions and which are the opposites by verifying which
     * homographies are consistent with all visible reference points being in front of the camera. The inputs
     * are left unchanged; the filtered solution set is returned as indices into the existing one.
     */
    public static void filterHomographyDecompByVisibleRefpoints(List<Mat> rotations, List<Mat> normals, Mat beforePoints, Mat afterPoints, Mat possibleSolutions, Mat pointsMask) {
        Mat rotations_mat = Converters.vector_Mat_to_Mat(rotations);
        Mat normals_mat = Converters.vector_Mat_to_Mat(normals);
        filterHomographyDecompByVisibleRefpoints_0(rotations_mat.nativeObj, normals_mat.nativeObj, beforePoints.nativeObj, afterPoints.nativeObj, possibleSolutions.nativeObj, pointsMask.nativeObj);
    }

    /**
     * Filters homography decompositions based on additional information.
     *
     * @param rotations Vector of rotation matrices.
     * @param normals Vector of plane normal matrices.
     * @param beforePoints Vector of (rectified) visible reference points before the homography is applied
     * @param afterPoints Vector of (rectified) visible reference points after the homography is applied
     * @param possibleSolutions Vector of int indices representing the viable solution set after filtering
     *
     * This function is intended to filter the output of the #decomposeHomographyMat based on additional
     * information as described in CITE: Malis2007 . The summary of the method: the #decomposeHomographyMat function
     * returns 2 unique solutions and their "opposites" for a total of 4 solutions. If we have access to the
     * sets of points visible in the camera frame before and after the homography transformation is applied,
     * we can determine which are the true potential solutions and which are the opposites by verifying which
     * homographies are consistent with all visible reference points being in front of the camera. The inputs
     * are left unchanged; the filtered solution set is returned as indices into the existing one.
     */
    public static void filterHomographyDecompByVisibleRefpoints(List<Mat> rotations, List<Mat> normals, Mat beforePoints, Mat afterPoints, Mat possibleSolutions) {
        Mat rotations_mat = Converters.vector_Mat_to_Mat(rotations);
        Mat normals_mat = Converters.vector_Mat_to_Mat(normals);
        filterHomographyDecompByVisibleRefpoints_1(rotations_mat.nativeObj, normals_mat.nativeObj, beforePoints.nativeObj, afterPoints.nativeObj, possibleSolutions.nativeObj);
    }


    //
    // C++:  void cv::undistort(Mat src, Mat& dst, Mat cameraMatrix, Mat distCoeffs, Mat newCameraMatrix = Mat())
    //

    /**
     * Transforms an image to compensate for lens distortion.
     *
     * The function transforms an image to compensate radial and tangential lens distortion.
     *
     * The function is simply a combination of #initUndistortRectifyMap (with unity R ) and #remap
     * (with bilinear interpolation). See the former function for details of the transformation being
     * performed.
     *
     * Those pixels in the destination image, for which there is no correspondent pixels in the source
     * image, are filled with zeros (black color).
     *
     * A particular subset of the source image that will be visible in the corrected image can be regulated
     * by newCameraMatrix. You can use #getOptimalNewCameraMatrix to compute the appropriate
     * newCameraMatrix depending on your requirements.
     *
     * The camera matrix and the distortion parameters can be determined using #calibrateCamera. If
     * the resolution of images is different from the resolution used at the calibration stage, \(f_x,
     * f_y, c_x\) and \(c_y\) need to be scaled accordingly, while the distortion coefficients remain
     * the same.
     *
     * @param src Input (distorted) image.
     * @param dst Output (corrected) image that has the same size and type as src .
     * @param cameraMatrix Input camera matrix \(A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param newCameraMatrix Camera matrix of the distorted image. By default, it is the same as
     * cameraMatrix but you may additionally scale and shift the result by using a different matrix.
     */
    public static void undistort(Mat src, Mat dst, Mat cameraMatrix, Mat distCoeffs, Mat newCameraMatrix) {
        undistort_0(src.nativeObj, dst.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, newCameraMatrix.nativeObj);
    }

    /**
     * Transforms an image to compensate for lens distortion.
     *
     * The function transforms an image to compensate radial and tangential lens distortion.
     *
     * The function is simply a combination of #initUndistortRectifyMap (with unity R ) and #remap
     * (with bilinear interpolation). See the former function for details of the transformation being
     * performed.
     *
     * Those pixels in the destination image, for which there is no correspondent pixels in the source
     * image, are filled with zeros (black color).
     *
     * A particular subset of the source image that will be visible in the corrected image can be regulated
     * by newCameraMatrix. You can use #getOptimalNewCameraMatrix to compute the appropriate
     * newCameraMatrix depending on your requirements.
     *
     * The camera matrix and the distortion parameters can be determined using #calibrateCamera. If
     * the resolution of images is different from the resolution used at the calibration stage, \(f_x,
     * f_y, c_x\) and \(c_y\) need to be scaled accordingly, while the distortion coefficients remain
     * the same.
     *
     * @param src Input (distorted) image.
     * @param dst Output (corrected) image that has the same size and type as src .
     * @param cameraMatrix Input camera matrix \(A = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * cameraMatrix but you may additionally scale and shift the result by using a different matrix.
     */
    public static void undistort(Mat src, Mat dst, Mat cameraMatrix, Mat distCoeffs) {
        undistort_1(src.nativeObj, dst.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
    }


    //
    // C++:  void cv::initUndistortRectifyMap(Mat cameraMatrix, Mat distCoeffs, Mat R, Mat newCameraMatrix, Size size, int m1type, Mat& map1, Mat& map2)
    //

    /**
     * Computes the undistortion and rectification transformation map.
     *
     * The function computes the joint undistortion and rectification transformation and represents the
     * result in the form of maps for #remap. The undistorted image looks like original, as if it is
     * captured with a camera using the camera matrix =newCameraMatrix and zero distortion. In case of a
     * monocular camera, newCameraMatrix is usually equal to cameraMatrix, or it can be computed by
     * #getOptimalNewCameraMatrix for a better control over scaling. In case of a stereo camera,
     * newCameraMatrix is normally set to P1 or P2 computed by #stereoRectify .
     *
     * Also, this new camera is oriented differently in the coordinate space, according to R. That, for
     * example, helps to align two heads of a stereo camera so that the epipolar lines on both images
     * become horizontal and have the same y- coordinate (in case of a horizontally aligned stereo camera).
     *
     * The function actually builds the maps for the inverse mapping algorithm that is used by #remap. That
     * is, for each pixel \((u, v)\) in the destination (corrected and rectified) image, the function
     * computes the corresponding coordinates in the source image (that is, in the original image from
     * camera). The following process is applied:
     * \(
     * \begin{array}{l}
     * x  \leftarrow (u - {c'}_x)/{f'}_x  \\
     * y  \leftarrow (v - {c'}_y)/{f'}_y  \\
     * {[X\,Y\,W]} ^T  \leftarrow R^{-1}*[x \, y \, 1]^T  \\
     * x'  \leftarrow X/W  \\
     * y'  \leftarrow Y/W  \\
     * r^2  \leftarrow x'^2 + y'^2 \\
     * x''  \leftarrow x' \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6}
     * + 2p_1 x' y' + p_2(r^2 + 2 x'^2)  + s_1 r^2 + s_2 r^4\\
     * y''  \leftarrow y' \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6}
     * + p_1 (r^2 + 2 y'^2) + 2 p_2 x' y' + s_3 r^2 + s_4 r^4 \\
     * s\vecthree{x'''}{y'''}{1} =
     * \vecthreethree{R_{33}(\tau_x, \tau_y)}{0}{-R_{13}((\tau_x, \tau_y)}
     * {0}{R_{33}(\tau_x, \tau_y)}{-R_{23}(\tau_x, \tau_y)}
     * {0}{0}{1} R(\tau_x, \tau_y) \vecthree{x''}{y''}{1}\\
     * map_x(u,v)  \leftarrow x''' f_x + c_x  \\
     * map_y(u,v)  \leftarrow y''' f_y + c_y
     * \end{array}
     * \)
     * where \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * are the distortion coefficients.
     *
     * In case of a stereo camera, this function is called twice: once for each camera head, after
     * #stereoRectify, which in its turn is called after #stereoCalibrate. But if the stereo camera
     * was not calibrated, it is still possible to compute the rectification transformations directly from
     * the fundamental matrix using #stereoRectifyUncalibrated. For each camera, the function computes
     * homography H as the rectification transformation in a pixel domain, not a rotation matrix R in 3D
     * space. R can be computed from H as
     * \(\texttt{R} = \texttt{cameraMatrix} ^{-1} \cdot \texttt{H} \cdot \texttt{cameraMatrix}\)
     * where cameraMatrix can be chosen arbitrarily.
     *
     * @param cameraMatrix Input camera matrix \(A=\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param R Optional rectification transformation in the object space (3x3 matrix). R1 or R2 ,
     * computed by #stereoRectify can be passed here. If the matrix is empty, the identity transformation
     * is assumed. In #initUndistortRectifyMap R assumed to be an identity matrix.
     * @param newCameraMatrix New camera matrix \(A'=\vecthreethree{f_x'}{0}{c_x'}{0}{f_y'}{c_y'}{0}{0}{1}\).
     * @param size Undistorted image size.
     * @param m1type Type of the first output map that can be CV_32FC1, CV_32FC2 or CV_16SC2, see #convertMaps
     * @param map1 The first output map.
     * @param map2 The second output map.
     */
    public static void initUndistortRectifyMap(Mat cameraMatrix, Mat distCoeffs, Mat R, Mat newCameraMatrix, Size size, int m1type, Mat map1, Mat map2) {
        initUndistortRectifyMap_0(cameraMatrix.nativeObj, distCoeffs.nativeObj, R.nativeObj, newCameraMatrix.nativeObj, size.width, size.height, m1type, map1.nativeObj, map2.nativeObj);
    }


    //
    // C++:  void cv::initInverseRectificationMap(Mat cameraMatrix, Mat distCoeffs, Mat R, Mat newCameraMatrix, Size size, int m1type, Mat& map1, Mat& map2)
    //

    /**
     * Computes the projection and inverse-rectification transformation map. In essense, this is the inverse of
     * #initUndistortRectifyMap to accomodate stereo-rectification of projectors ('inverse-cameras') in projector-camera pairs.
     *
     * The function computes the joint projection and inverse rectification transformation and represents the
     * result in the form of maps for #remap. The projected image looks like a distorted version of the original which,
     * once projected by a projector, should visually match the original. In case of a monocular camera, newCameraMatrix
     * is usually equal to cameraMatrix, or it can be computed by
     * #getOptimalNewCameraMatrix for a better control over scaling. In case of a projector-camera pair,
     * newCameraMatrix is normally set to P1 or P2 computed by #stereoRectify .
     *
     * The projector is oriented differently in the coordinate space, according to R. In case of projector-camera pairs,
     * this helps align the projector (in the same manner as #initUndistortRectifyMap for the camera) to create a stereo-rectified pair. This
     * allows epipolar lines on both images to become horizontal and have the same y-coordinate (in case of a horizontally aligned projector-camera pair).
     *
     * The function builds the maps for the inverse mapping algorithm that is used by #remap. That
     * is, for each pixel \((u, v)\) in the destination (projected and inverse-rectified) image, the function
     * computes the corresponding coordinates in the source image (that is, in the original digital image). The following process is applied:
     *
     * \(
     * \begin{array}{l}
     * \text{newCameraMatrix}\\
     * x  \leftarrow (u - {c'}_x)/{f'}_x  \\
     * y  \leftarrow (v - {c'}_y)/{f'}_y  \\
     *
     * \\\text{Undistortion}
     * \\\scriptsize{\textit{though equation shown is for radial undistortion, function implements cv::undistortPoints()}}\\
     * r^2  \leftarrow x^2 + y^2 \\
     * \theta \leftarrow \frac{1 + k_1 r^2 + k_2 r^4 + k_3 r^6}{1 + k_4 r^2 + k_5 r^4 + k_6 r^6}\\
     * x' \leftarrow \frac{x}{\theta} \\
     * y'  \leftarrow \frac{y}{\theta} \\
     *
     * \\\text{Rectification}\\
     * {[X\,Y\,W]} ^T  \leftarrow R*[x' \, y' \, 1]^T  \\
     * x''  \leftarrow X/W  \\
     * y''  \leftarrow Y/W  \\
     *
     * \\\text{cameraMatrix}\\
     * map_x(u,v)  \leftarrow x'' f_x + c_x  \\
     * map_y(u,v)  \leftarrow y'' f_y + c_y
     * \end{array}
     * \)
     * where \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * are the distortion coefficients vector distCoeffs.
     *
     * In case of a stereo-rectified projector-camera pair, this function is called for the projector while #initUndistortRectifyMap is called for the camera head.
     * This is done after #stereoRectify, which in turn is called after #stereoCalibrate. If the projector-camera pair
     * is not calibrated, it is still possible to compute the rectification transformations directly from
     * the fundamental matrix using #stereoRectifyUncalibrated. For the projector and camera, the function computes
     * homography H as the rectification transformation in a pixel domain, not a rotation matrix R in 3D
     * space. R can be computed from H as
     * \(\texttt{R} = \texttt{cameraMatrix} ^{-1} \cdot \texttt{H} \cdot \texttt{cameraMatrix}\)
     * where cameraMatrix can be chosen arbitrarily.
     *
     * @param cameraMatrix Input camera matrix \(A=\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param R Optional rectification transformation in the object space (3x3 matrix). R1 or R2,
     * computed by #stereoRectify can be passed here. If the matrix is empty, the identity transformation
     * is assumed.
     * @param newCameraMatrix New camera matrix \(A'=\vecthreethree{f_x'}{0}{c_x'}{0}{f_y'}{c_y'}{0}{0}{1}\).
     * @param size Distorted image size.
     * @param m1type Type of the first output map. Can be CV_32FC1, CV_32FC2 or CV_16SC2, see #convertMaps
     * @param map1 The first output map for #remap.
     * @param map2 The second output map for #remap.
     */
    public static void initInverseRectificationMap(Mat cameraMatrix, Mat distCoeffs, Mat R, Mat newCameraMatrix, Size size, int m1type, Mat map1, Mat map2) {
        initInverseRectificationMap_0(cameraMatrix.nativeObj, distCoeffs.nativeObj, R.nativeObj, newCameraMatrix.nativeObj, size.width, size.height, m1type, map1.nativeObj, map2.nativeObj);
    }


    //
    // C++:  Mat cv::getDefaultNewCameraMatrix(Mat cameraMatrix, Size imgsize = Size(), bool centerPrincipalPoint = false)
    //

    /**
     * Returns the default new camera matrix.
     *
     * The function returns the camera matrix that is either an exact copy of the input cameraMatrix (when
     * centerPrinicipalPoint=false ), or the modified one (when centerPrincipalPoint=true).
     *
     * In the latter case, the new camera matrix will be:
     *
     * \(\begin{bmatrix} f_x &amp;&amp; 0 &amp;&amp; ( \texttt{imgSize.width} -1)*0.5  \\ 0 &amp;&amp; f_y &amp;&amp; ( \texttt{imgSize.height} -1)*0.5  \\ 0 &amp;&amp; 0 &amp;&amp; 1 \end{bmatrix} ,\)
     *
     * where \(f_x\) and \(f_y\) are \((0,0)\) and \((1,1)\) elements of cameraMatrix, respectively.
     *
     * By default, the undistortion functions in OpenCV (see #initUndistortRectifyMap, #undistort) do not
     * move the principal point. However, when you work with stereo, it is important to move the principal
     * points in both views to the same y-coordinate (which is required by most of stereo correspondence
     * algorithms), and may be to the same x-coordinate too. So, you can form the new camera matrix for
     * each view where the principal points are located at the center.
     *
     * @param cameraMatrix Input camera matrix.
     * @param imgsize Camera view image size in pixels.
     * @param centerPrincipalPoint Location of the principal point in the new camera matrix. The
     * parameter indicates whether this location should be at the image center or not.
     * @return automatically generated
     */
    public static Mat getDefaultNewCameraMatrix(Mat cameraMatrix, Size imgsize, boolean centerPrincipalPoint) {
        return new Mat(getDefaultNewCameraMatrix_0(cameraMatrix.nativeObj, imgsize.width, imgsize.height, centerPrincipalPoint));
    }

    /**
     * Returns the default new camera matrix.
     *
     * The function returns the camera matrix that is either an exact copy of the input cameraMatrix (when
     * centerPrinicipalPoint=false ), or the modified one (when centerPrincipalPoint=true).
     *
     * In the latter case, the new camera matrix will be:
     *
     * \(\begin{bmatrix} f_x &amp;&amp; 0 &amp;&amp; ( \texttt{imgSize.width} -1)*0.5  \\ 0 &amp;&amp; f_y &amp;&amp; ( \texttt{imgSize.height} -1)*0.5  \\ 0 &amp;&amp; 0 &amp;&amp; 1 \end{bmatrix} ,\)
     *
     * where \(f_x\) and \(f_y\) are \((0,0)\) and \((1,1)\) elements of cameraMatrix, respectively.
     *
     * By default, the undistortion functions in OpenCV (see #initUndistortRectifyMap, #undistort) do not
     * move the principal point. However, when you work with stereo, it is important to move the principal
     * points in both views to the same y-coordinate (which is required by most of stereo correspondence
     * algorithms), and may be to the same x-coordinate too. So, you can form the new camera matrix for
     * each view where the principal points are located at the center.
     *
     * @param cameraMatrix Input camera matrix.
     * @param imgsize Camera view image size in pixels.
     * parameter indicates whether this location should be at the image center or not.
     * @return automatically generated
     */
    public static Mat getDefaultNewCameraMatrix(Mat cameraMatrix, Size imgsize) {
        return new Mat(getDefaultNewCameraMatrix_1(cameraMatrix.nativeObj, imgsize.width, imgsize.height));
    }

    /**
     * Returns the default new camera matrix.
     *
     * The function returns the camera matrix that is either an exact copy of the input cameraMatrix (when
     * centerPrinicipalPoint=false ), or the modified one (when centerPrincipalPoint=true).
     *
     * In the latter case, the new camera matrix will be:
     *
     * \(\begin{bmatrix} f_x &amp;&amp; 0 &amp;&amp; ( \texttt{imgSize.width} -1)*0.5  \\ 0 &amp;&amp; f_y &amp;&amp; ( \texttt{imgSize.height} -1)*0.5  \\ 0 &amp;&amp; 0 &amp;&amp; 1 \end{bmatrix} ,\)
     *
     * where \(f_x\) and \(f_y\) are \((0,0)\) and \((1,1)\) elements of cameraMatrix, respectively.
     *
     * By default, the undistortion functions in OpenCV (see #initUndistortRectifyMap, #undistort) do not
     * move the principal point. However, when you work with stereo, it is important to move the principal
     * points in both views to the same y-coordinate (which is required by most of stereo correspondence
     * algorithms), and may be to the same x-coordinate too. So, you can form the new camera matrix for
     * each view where the principal points are located at the center.
     *
     * @param cameraMatrix Input camera matrix.
     * parameter indicates whether this location should be at the image center or not.
     * @return automatically generated
     */
    public static Mat getDefaultNewCameraMatrix(Mat cameraMatrix) {
        return new Mat(getDefaultNewCameraMatrix_2(cameraMatrix.nativeObj));
    }


    //
    // C++:  Mat cv::getOptimalNewCameraMatrix(Mat cameraMatrix, Mat distCoeffs, Size imageSize, double alpha, Size newImgSize = Size(), Rect* validPixROI = 0, bool centerPrincipalPoint = false)
    //

    /**
     * Returns the new camera intrinsic matrix based on the free scaling parameter.
     *
     * @param cameraMatrix Input camera intrinsic matrix.
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param imageSize Original image size.
     * @param alpha Free scaling parameter between 0 (when all the pixels in the undistorted image are
     * valid) and 1 (when all the source image pixels are retained in the undistorted image). See
     * #stereoRectify for details.
     * @param newImgSize Image size after rectification. By default, it is set to imageSize .
     * @param validPixROI Optional output rectangle that outlines all-good-pixels region in the
     * undistorted image. See roi1, roi2 description in #stereoRectify .
     * @param centerPrincipalPoint Optional flag that indicates whether in the new camera intrinsic matrix the
     * principal point should be at the image center or not. By default, the principal point is chosen to
     * best fit a subset of the source image (determined by alpha) to the corrected image.
     * @return new_camera_matrix Output new camera intrinsic matrix.
     *
     * The function computes and returns the optimal new camera intrinsic matrix based on the free scaling parameter.
     * By varying this parameter, you may retrieve only sensible pixels alpha=0 , keep all the original
     * image pixels if there is valuable information in the corners alpha=1 , or get something in between.
     * When alpha&gt;0 , the undistorted result is likely to have some black pixels corresponding to
     * "virtual" pixels outside of the captured distorted image. The original camera intrinsic matrix, distortion
     * coefficients, the computed new camera intrinsic matrix, and newImageSize should be passed to
     * #initUndistortRectifyMap to produce the maps for #remap .
     */
    public static Mat getOptimalNewCameraMatrix(Mat cameraMatrix, Mat distCoeffs, Size imageSize, double alpha, Size newImgSize, Rect validPixROI, boolean centerPrincipalPoint) {
        double[] validPixROI_out = new double[4];
        Mat retVal = new Mat(getOptimalNewCameraMatrix_0(cameraMatrix.nativeObj, distCoeffs.nativeObj, imageSize.width, imageSize.height, alpha, newImgSize.width, newImgSize.height, validPixROI_out, centerPrincipalPoint));
        if(validPixROI!=null){ validPixROI.x = (int)validPixROI_out[0]; validPixROI.y = (int)validPixROI_out[1]; validPixROI.width = (int)validPixROI_out[2]; validPixROI.height = (int)validPixROI_out[3]; } 
        return retVal;
    }

    /**
     * Returns the new camera intrinsic matrix based on the free scaling parameter.
     *
     * @param cameraMatrix Input camera intrinsic matrix.
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param imageSize Original image size.
     * @param alpha Free scaling parameter between 0 (when all the pixels in the undistorted image are
     * valid) and 1 (when all the source image pixels are retained in the undistorted image). See
     * #stereoRectify for details.
     * @param newImgSize Image size after rectification. By default, it is set to imageSize .
     * @param validPixROI Optional output rectangle that outlines all-good-pixels region in the
     * undistorted image. See roi1, roi2 description in #stereoRectify .
     * principal point should be at the image center or not. By default, the principal point is chosen to
     * best fit a subset of the source image (determined by alpha) to the corrected image.
     * @return new_camera_matrix Output new camera intrinsic matrix.
     *
     * The function computes and returns the optimal new camera intrinsic matrix based on the free scaling parameter.
     * By varying this parameter, you may retrieve only sensible pixels alpha=0 , keep all the original
     * image pixels if there is valuable information in the corners alpha=1 , or get something in between.
     * When alpha&gt;0 , the undistorted result is likely to have some black pixels corresponding to
     * "virtual" pixels outside of the captured distorted image. The original camera intrinsic matrix, distortion
     * coefficients, the computed new camera intrinsic matrix, and newImageSize should be passed to
     * #initUndistortRectifyMap to produce the maps for #remap .
     */
    public static Mat getOptimalNewCameraMatrix(Mat cameraMatrix, Mat distCoeffs, Size imageSize, double alpha, Size newImgSize, Rect validPixROI) {
        double[] validPixROI_out = new double[4];
        Mat retVal = new Mat(getOptimalNewCameraMatrix_1(cameraMatrix.nativeObj, distCoeffs.nativeObj, imageSize.width, imageSize.height, alpha, newImgSize.width, newImgSize.height, validPixROI_out));
        if(validPixROI!=null){ validPixROI.x = (int)validPixROI_out[0]; validPixROI.y = (int)validPixROI_out[1]; validPixROI.width = (int)validPixROI_out[2]; validPixROI.height = (int)validPixROI_out[3]; } 
        return retVal;
    }

    /**
     * Returns the new camera intrinsic matrix based on the free scaling parameter.
     *
     * @param cameraMatrix Input camera intrinsic matrix.
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param imageSize Original image size.
     * @param alpha Free scaling parameter between 0 (when all the pixels in the undistorted image are
     * valid) and 1 (when all the source image pixels are retained in the undistorted image). See
     * #stereoRectify for details.
     * @param newImgSize Image size after rectification. By default, it is set to imageSize .
     * undistorted image. See roi1, roi2 description in #stereoRectify .
     * principal point should be at the image center or not. By default, the principal point is chosen to
     * best fit a subset of the source image (determined by alpha) to the corrected image.
     * @return new_camera_matrix Output new camera intrinsic matrix.
     *
     * The function computes and returns the optimal new camera intrinsic matrix based on the free scaling parameter.
     * By varying this parameter, you may retrieve only sensible pixels alpha=0 , keep all the original
     * image pixels if there is valuable information in the corners alpha=1 , or get something in between.
     * When alpha&gt;0 , the undistorted result is likely to have some black pixels corresponding to
     * "virtual" pixels outside of the captured distorted image. The original camera intrinsic matrix, distortion
     * coefficients, the computed new camera intrinsic matrix, and newImageSize should be passed to
     * #initUndistortRectifyMap to produce the maps for #remap .
     */
    public static Mat getOptimalNewCameraMatrix(Mat cameraMatrix, Mat distCoeffs, Size imageSize, double alpha, Size newImgSize) {
        return new Mat(getOptimalNewCameraMatrix_2(cameraMatrix.nativeObj, distCoeffs.nativeObj, imageSize.width, imageSize.height, alpha, newImgSize.width, newImgSize.height));
    }

    /**
     * Returns the new camera intrinsic matrix based on the free scaling parameter.
     *
     * @param cameraMatrix Input camera intrinsic matrix.
     * @param distCoeffs Input vector of distortion coefficients
     * \(\distcoeffs\). If the vector is NULL/empty, the zero distortion coefficients are
     * assumed.
     * @param imageSize Original image size.
     * @param alpha Free scaling parameter between 0 (when all the pixels in the undistorted image are
     * valid) and 1 (when all the source image pixels are retained in the undistorted image). See
     * #stereoRectify for details.
     * undistorted image. See roi1, roi2 description in #stereoRectify .
     * principal point should be at the image center or not. By default, the principal point is chosen to
     * best fit a subset of the source image (determined by alpha) to the corrected image.
     * @return new_camera_matrix Output new camera intrinsic matrix.
     *
     * The function computes and returns the optimal new camera intrinsic matrix based on the free scaling parameter.
     * By varying this parameter, you may retrieve only sensible pixels alpha=0 , keep all the original
     * image pixels if there is valuable information in the corners alpha=1 , or get something in between.
     * When alpha&gt;0 , the undistorted result is likely to have some black pixels corresponding to
     * "virtual" pixels outside of the captured distorted image. The original camera intrinsic matrix, distortion
     * coefficients, the computed new camera intrinsic matrix, and newImageSize should be passed to
     * #initUndistortRectifyMap to produce the maps for #remap .
     */
    public static Mat getOptimalNewCameraMatrix(Mat cameraMatrix, Mat distCoeffs, Size imageSize, double alpha) {
        return new Mat(getOptimalNewCameraMatrix_3(cameraMatrix.nativeObj, distCoeffs.nativeObj, imageSize.width, imageSize.height, alpha));
    }


    //
    // C++:  void cv::undistortPoints(vector_Point2f src, vector_Point2f& dst, Mat cameraMatrix, Mat distCoeffs, Mat R = Mat(), Mat P = Mat(), TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER, 5, 0.01))
    //

    /**
     * Computes the ideal point coordinates from the observed point coordinates.
     *
     * The function is similar to #undistort and #initUndistortRectifyMap but it operates on a
     * sparse set of points instead of a raster image. Also the function performs a reverse transformation
     * to  #projectPoints. In case of a 3D object, it does not reconstruct its 3D coordinates, but for a
     * planar object, it does, up to a translation vector, if the proper R is specified.
     *
     * For each observed point coordinate \((u, v)\) the function computes:
     * \(
     * \begin{array}{l}
     * x^{"}  \leftarrow (u - c_x)/f_x  \\
     * y^{"}  \leftarrow (v - c_y)/f_y  \\
     * (x',y') = undistort(x^{"},y^{"}, \texttt{distCoeffs}) \\
     * {[X\,Y\,W]} ^T  \leftarrow R*[x' \, y' \, 1]^T  \\
     * x  \leftarrow X/W  \\
     * y  \leftarrow Y/W  \\
     * \text{only performed if P is specified:} \\
     * u'  \leftarrow x {f'}_x + {c'}_x  \\
     * v'  \leftarrow y {f'}_y + {c'}_y
     * \end{array}
     * \)
     *
     * where *undistort* is an approximate iterative algorithm that estimates the normalized original
     * point coordinates out of the normalized distorted point coordinates ("normalized" means that the
     * coordinates do not depend on the camera matrix).
     *
     * The function can be used for both a stereo camera head or a monocular camera (when R is empty).
     * @param src Observed point coordinates, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or
     * vector&lt;Point2f&gt; ).
     * @param dst Output ideal point coordinates (1xN/Nx1 2-channel or vector&lt;Point2f&gt; ) after undistortion and reverse perspective
     * transformation. If matrix P is identity or omitted, dst will contain normalized point coordinates.
     * @param cameraMatrix Camera matrix \(\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param R Rectification transformation in the object space (3x3 matrix). R1 or R2 computed by
     * #stereoRectify can be passed here. If the matrix is empty, the identity transformation is used.
     * @param P New camera matrix (3x3) or new projection matrix (3x4) \(\begin{bmatrix} {f'}_x &amp; 0 &amp; {c'}_x &amp; t_x \\ 0 &amp; {f'}_y &amp; {c'}_y &amp; t_y \\ 0 &amp; 0 &amp; 1 &amp; t_z \end{bmatrix}\). P1 or P2 computed by
     * #stereoRectify can be passed here. If the matrix is empty, the identity new camera matrix is used.
     * @param criteria termination criteria for the iterative point undistortion algorithm
     */
    public static void undistortPoints(MatOfPoint2f src, MatOfPoint2f dst, Mat cameraMatrix, Mat distCoeffs, Mat R, Mat P, TermCriteria criteria) {
        Mat src_mat = src;
        Mat dst_mat = dst;
        undistortPoints_0(src_mat.nativeObj, dst_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, R.nativeObj, P.nativeObj, criteria.type, criteria.maxCount, criteria.epsilon);
    }

    /**
     * Computes the ideal point coordinates from the observed point coordinates.
     *
     * The function is similar to #undistort and #initUndistortRectifyMap but it operates on a
     * sparse set of points instead of a raster image. Also the function performs a reverse transformation
     * to  #projectPoints. In case of a 3D object, it does not reconstruct its 3D coordinates, but for a
     * planar object, it does, up to a translation vector, if the proper R is specified.
     *
     * For each observed point coordinate \((u, v)\) the function computes:
     * \(
     * \begin{array}{l}
     * x^{"}  \leftarrow (u - c_x)/f_x  \\
     * y^{"}  \leftarrow (v - c_y)/f_y  \\
     * (x',y') = undistort(x^{"},y^{"}, \texttt{distCoeffs}) \\
     * {[X\,Y\,W]} ^T  \leftarrow R*[x' \, y' \, 1]^T  \\
     * x  \leftarrow X/W  \\
     * y  \leftarrow Y/W  \\
     * \text{only performed if P is specified:} \\
     * u'  \leftarrow x {f'}_x + {c'}_x  \\
     * v'  \leftarrow y {f'}_y + {c'}_y
     * \end{array}
     * \)
     *
     * where *undistort* is an approximate iterative algorithm that estimates the normalized original
     * point coordinates out of the normalized distorted point coordinates ("normalized" means that the
     * coordinates do not depend on the camera matrix).
     *
     * The function can be used for both a stereo camera head or a monocular camera (when R is empty).
     * @param src Observed point coordinates, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or
     * vector&lt;Point2f&gt; ).
     * @param dst Output ideal point coordinates (1xN/Nx1 2-channel or vector&lt;Point2f&gt; ) after undistortion and reverse perspective
     * transformation. If matrix P is identity or omitted, dst will contain normalized point coordinates.
     * @param cameraMatrix Camera matrix \(\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param R Rectification transformation in the object space (3x3 matrix). R1 or R2 computed by
     * #stereoRectify can be passed here. If the matrix is empty, the identity transformation is used.
     * @param P New camera matrix (3x3) or new projection matrix (3x4) \(\begin{bmatrix} {f'}_x &amp; 0 &amp; {c'}_x &amp; t_x \\ 0 &amp; {f'}_y &amp; {c'}_y &amp; t_y \\ 0 &amp; 0 &amp; 1 &amp; t_z \end{bmatrix}\). P1 or P2 computed by
     * #stereoRectify can be passed here. If the matrix is empty, the identity new camera matrix is used.
     */
    public static void undistortPoints(MatOfPoint2f src, MatOfPoint2f dst, Mat cameraMatrix, Mat distCoeffs, Mat R, Mat P) {
        Mat src_mat = src;
        Mat dst_mat = dst;
        undistortPoints_1(src_mat.nativeObj, dst_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, R.nativeObj, P.nativeObj);
    }

    /**
     * Computes the ideal point coordinates from the observed point coordinates.
     *
     * The function is similar to #undistort and #initUndistortRectifyMap but it operates on a
     * sparse set of points instead of a raster image. Also the function performs a reverse transformation
     * to  #projectPoints. In case of a 3D object, it does not reconstruct its 3D coordinates, but for a
     * planar object, it does, up to a translation vector, if the proper R is specified.
     *
     * For each observed point coordinate \((u, v)\) the function computes:
     * \(
     * \begin{array}{l}
     * x^{"}  \leftarrow (u - c_x)/f_x  \\
     * y^{"}  \leftarrow (v - c_y)/f_y  \\
     * (x',y') = undistort(x^{"},y^{"}, \texttt{distCoeffs}) \\
     * {[X\,Y\,W]} ^T  \leftarrow R*[x' \, y' \, 1]^T  \\
     * x  \leftarrow X/W  \\
     * y  \leftarrow Y/W  \\
     * \text{only performed if P is specified:} \\
     * u'  \leftarrow x {f'}_x + {c'}_x  \\
     * v'  \leftarrow y {f'}_y + {c'}_y
     * \end{array}
     * \)
     *
     * where *undistort* is an approximate iterative algorithm that estimates the normalized original
     * point coordinates out of the normalized distorted point coordinates ("normalized" means that the
     * coordinates do not depend on the camera matrix).
     *
     * The function can be used for both a stereo camera head or a monocular camera (when R is empty).
     * @param src Observed point coordinates, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or
     * vector&lt;Point2f&gt; ).
     * @param dst Output ideal point coordinates (1xN/Nx1 2-channel or vector&lt;Point2f&gt; ) after undistortion and reverse perspective
     * transformation. If matrix P is identity or omitted, dst will contain normalized point coordinates.
     * @param cameraMatrix Camera matrix \(\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * @param R Rectification transformation in the object space (3x3 matrix). R1 or R2 computed by
     * #stereoRectify can be passed here. If the matrix is empty, the identity transformation is used.
     * #stereoRectify can be passed here. If the matrix is empty, the identity new camera matrix is used.
     */
    public static void undistortPoints(MatOfPoint2f src, MatOfPoint2f dst, Mat cameraMatrix, Mat distCoeffs, Mat R) {
        Mat src_mat = src;
        Mat dst_mat = dst;
        undistortPoints_2(src_mat.nativeObj, dst_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, R.nativeObj);
    }

    /**
     * Computes the ideal point coordinates from the observed point coordinates.
     *
     * The function is similar to #undistort and #initUndistortRectifyMap but it operates on a
     * sparse set of points instead of a raster image. Also the function performs a reverse transformation
     * to  #projectPoints. In case of a 3D object, it does not reconstruct its 3D coordinates, but for a
     * planar object, it does, up to a translation vector, if the proper R is specified.
     *
     * For each observed point coordinate \((u, v)\) the function computes:
     * \(
     * \begin{array}{l}
     * x^{"}  \leftarrow (u - c_x)/f_x  \\
     * y^{"}  \leftarrow (v - c_y)/f_y  \\
     * (x',y') = undistort(x^{"},y^{"}, \texttt{distCoeffs}) \\
     * {[X\,Y\,W]} ^T  \leftarrow R*[x' \, y' \, 1]^T  \\
     * x  \leftarrow X/W  \\
     * y  \leftarrow Y/W  \\
     * \text{only performed if P is specified:} \\
     * u'  \leftarrow x {f'}_x + {c'}_x  \\
     * v'  \leftarrow y {f'}_y + {c'}_y
     * \end{array}
     * \)
     *
     * where *undistort* is an approximate iterative algorithm that estimates the normalized original
     * point coordinates out of the normalized distorted point coordinates ("normalized" means that the
     * coordinates do not depend on the camera matrix).
     *
     * The function can be used for both a stereo camera head or a monocular camera (when R is empty).
     * @param src Observed point coordinates, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or
     * vector&lt;Point2f&gt; ).
     * @param dst Output ideal point coordinates (1xN/Nx1 2-channel or vector&lt;Point2f&gt; ) after undistortion and reverse perspective
     * transformation. If matrix P is identity or omitted, dst will contain normalized point coordinates.
     * @param cameraMatrix Camera matrix \(\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Input vector of distortion coefficients
     * \((k_1, k_2, p_1, p_2[, k_3[, k_4, k_5, k_6[, s_1, s_2, s_3, s_4[, \tau_x, \tau_y]]]])\)
     * of 4, 5, 8, 12 or 14 elements. If the vector is NULL/empty, the zero distortion coefficients are assumed.
     * #stereoRectify can be passed here. If the matrix is empty, the identity transformation is used.
     * #stereoRectify can be passed here. If the matrix is empty, the identity new camera matrix is used.
     */
    public static void undistortPoints(MatOfPoint2f src, MatOfPoint2f dst, Mat cameraMatrix, Mat distCoeffs) {
        Mat src_mat = src;
        Mat dst_mat = dst;
        undistortPoints_3(src_mat.nativeObj, dst_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
    }


    //
    // C++:  void cv::undistortImagePoints(Mat src, Mat& dst, Mat cameraMatrix, Mat distCoeffs, TermCriteria arg1 = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 5, 0.01))
    //

    /**
     * Compute undistorted image points position
     *
     * @param src Observed points position, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or vector&lt;Point2f&gt; ).
     * @param dst Output undistorted points position (1xN/Nx1 2-channel or vector&lt;Point2f&gt; ).
     * @param cameraMatrix Camera matrix \(\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Distortion coefficients
     * @param arg1 automatically generated
     */
    public static void undistortImagePoints(Mat src, Mat dst, Mat cameraMatrix, Mat distCoeffs, TermCriteria arg1) {
        undistortImagePoints_0(src.nativeObj, dst.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, arg1.type, arg1.maxCount, arg1.epsilon);
    }

    /**
     * Compute undistorted image points position
     *
     * @param src Observed points position, 2xN/Nx2 1-channel or 1xN/Nx1 2-channel (CV_32FC2 or CV_64FC2) (or vector&lt;Point2f&gt; ).
     * @param dst Output undistorted points position (1xN/Nx1 2-channel or vector&lt;Point2f&gt; ).
     * @param cameraMatrix Camera matrix \(\vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\) .
     * @param distCoeffs Distortion coefficients
     */
    public static void undistortImagePoints(Mat src, Mat dst, Mat cameraMatrix, Mat distCoeffs) {
        undistortImagePoints_1(src.nativeObj, dst.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
    }


    //
    // C++:  void cv::loadPointCloud(String filename, Mat& vertices, Mat& normals = Mat())
    //

    /**
     * Loads a point cloud from a file.
     *
     * The function loads point cloud from the specified file and returns it.
     * If the cloud cannot be read, throws an error
     *
     * Currently, the following file formats are supported:
     * -  [Wavefront obj file *.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file)
     * -  [Polygon File Format *.ply](https://en.wikipedia.org/wiki/PLY_(file_format))
     *
     * @param filename Name of the file.
     * @param vertices (vector of Point3f) Point coordinates of a point cloud
     * @param normals (vector of Point3f) Point normals of a point cloud
     */
    public static void loadPointCloud(String filename, Mat vertices, Mat normals) {
        loadPointCloud_0(filename, vertices.nativeObj, normals.nativeObj);
    }

    /**
     * Loads a point cloud from a file.
     *
     * The function loads point cloud from the specified file and returns it.
     * If the cloud cannot be read, throws an error
     *
     * Currently, the following file formats are supported:
     * -  [Wavefront obj file *.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file)
     * -  [Polygon File Format *.ply](https://en.wikipedia.org/wiki/PLY_(file_format))
     *
     * @param filename Name of the file.
     * @param vertices (vector of Point3f) Point coordinates of a point cloud
     */
    public static void loadPointCloud(String filename, Mat vertices) {
        loadPointCloud_1(filename, vertices.nativeObj);
    }


    //
    // C++:  void cv::savePointCloud(String filename, Mat vertices, Mat normals = Mat())
    //

    /**
     * Saves a point cloud to a specified file.
     *
     * The function saves point cloud to the specified file.
     * File format is chosen based on the filename extension.
     *
     * @param filename Name of the file.
     * @param vertices (vector of Point3f) Point coordinates of a point cloud
     * @param normals (vector of Point3f) Point normals of a point cloud
     */
    public static void savePointCloud(String filename, Mat vertices, Mat normals) {
        savePointCloud_0(filename, vertices.nativeObj, normals.nativeObj);
    }

    /**
     * Saves a point cloud to a specified file.
     *
     * The function saves point cloud to the specified file.
     * File format is chosen based on the filename extension.
     *
     * @param filename Name of the file.
     * @param vertices (vector of Point3f) Point coordinates of a point cloud
     */
    public static void savePointCloud(String filename, Mat vertices) {
        savePointCloud_1(filename, vertices.nativeObj);
    }


    //
    // C++:  void cv::loadMesh(String filename, Mat& vertices, Mat& normals, vector_Mat& indices)
    //

    /**
     * Loads a mesh from a file.
     *
     * The function loads mesh from the specified file and returns it.
     * If the mesh cannot be read, throws an error
     *
     * Currently, the following file formats are supported:
     * -  [Wavefront obj file *.obj](https://en.wikipedia.org/wiki/Wavefront_.obj_file) (ONLY TRIANGULATED FACES)
     * @param filename Name of the file.
     * @param vertices (vector of Point3f) vertex coordinates of a mesh
     * @param normals (vector of Point3f) vertex normals of a mesh
     * @param indices (vector of vectors of int) vertex normals of a mesh
     */
    public static void loadMesh(String filename, Mat vertices, Mat normals, List<Mat> indices) {
        Mat indices_mat = new Mat();
        loadMesh_0(filename, vertices.nativeObj, normals.nativeObj, indices_mat.nativeObj);
        Converters.Mat_to_vector_Mat(indices_mat, indices);
        indices_mat.release();
    }


    //
    // C++:  void cv::saveMesh(String filename, Mat vertices, Mat normals, vector_Mat indices)
    //

    /**
     * Saves a mesh to a specified file.
     *
     * The function saves mesh to the specified file.
     * File format is chosen based on the filename extension.
     *
     * @param filename Name of the file.
     * @param vertices (vector of Point3f) vertex coordinates of a mesh
     * @param normals (vector of Point3f) vertex normals of a mesh
     * @param indices (vector of vectors of int) vertex normals of a mesh
     */
    public static void saveMesh(String filename, Mat vertices, Mat normals, List<Mat> indices) {
        Mat indices_mat = Converters.vector_Mat_to_Mat(indices);
        saveMesh_0(filename, vertices.nativeObj, normals.nativeObj, indices_mat.nativeObj);
    }


    //
    // C++:  void cv::registerDepth(Mat unregisteredCameraMatrix, Mat registeredCameraMatrix, Mat registeredDistCoeffs, Mat Rt, Mat unregisteredDepth, Size outputImagePlaneSize, Mat& registeredDepth, bool depthDilation = false)
    //

    /**
     * Registers depth data to an external camera
     * Registration is performed by creating a depth cloud, transforming the cloud by
     * the rigid body transformation between the cameras, and then projecting the
     * transformed points into the RGB camera.
     *
     * uv_rgb = K_rgb * [R | t] * z * inv(K_ir) * uv_ir
     *
     * Currently does not check for negative depth values.
     *
     * @param unregisteredCameraMatrix the camera matrix of the depth camera
     * @param registeredCameraMatrix the camera matrix of the external camera
     * @param registeredDistCoeffs the distortion coefficients of the external camera
     * @param Rt the rigid body transform between the cameras. Transforms points from depth camera frame to external camera frame.
     * @param unregisteredDepth the input depth data
     * @param outputImagePlaneSize the image plane dimensions of the external camera (width, height)
     * @param registeredDepth the result of transforming the depth into the external camera
     * @param depthDilation whether or not the depth is dilated to avoid holes and occlusion errors (optional)
     */
    public static void registerDepth(Mat unregisteredCameraMatrix, Mat registeredCameraMatrix, Mat registeredDistCoeffs, Mat Rt, Mat unregisteredDepth, Size outputImagePlaneSize, Mat registeredDepth, boolean depthDilation) {
        registerDepth_0(unregisteredCameraMatrix.nativeObj, registeredCameraMatrix.nativeObj, registeredDistCoeffs.nativeObj, Rt.nativeObj, unregisteredDepth.nativeObj, outputImagePlaneSize.width, outputImagePlaneSize.height, registeredDepth.nativeObj, depthDilation);
    }

    /**
     * Registers depth data to an external camera
     * Registration is performed by creating a depth cloud, transforming the cloud by
     * the rigid body transformation between the cameras, and then projecting the
     * transformed points into the RGB camera.
     *
     * uv_rgb = K_rgb * [R | t] * z * inv(K_ir) * uv_ir
     *
     * Currently does not check for negative depth values.
     *
     * @param unregisteredCameraMatrix the camera matrix of the depth camera
     * @param registeredCameraMatrix the camera matrix of the external camera
     * @param registeredDistCoeffs the distortion coefficients of the external camera
     * @param Rt the rigid body transform between the cameras. Transforms points from depth camera frame to external camera frame.
     * @param unregisteredDepth the input depth data
     * @param outputImagePlaneSize the image plane dimensions of the external camera (width, height)
     * @param registeredDepth the result of transforming the depth into the external camera
     */
    public static void registerDepth(Mat unregisteredCameraMatrix, Mat registeredCameraMatrix, Mat registeredDistCoeffs, Mat Rt, Mat unregisteredDepth, Size outputImagePlaneSize, Mat registeredDepth) {
        registerDepth_1(unregisteredCameraMatrix.nativeObj, registeredCameraMatrix.nativeObj, registeredDistCoeffs.nativeObj, Rt.nativeObj, unregisteredDepth.nativeObj, outputImagePlaneSize.width, outputImagePlaneSize.height, registeredDepth.nativeObj);
    }


    //
    // C++:  void cv::depthTo3dSparse(Mat depth, Mat in_K, Mat in_points, Mat& points3d)
    //

    /**
     * @param depth the depth image
     * @param in_K
     * @param in_points the list of xy coordinates
     * @param points3d the resulting 3d points (point is represented by 4 chanels value [x, y, z, 0])
     */
    public static void depthTo3dSparse(Mat depth, Mat in_K, Mat in_points, Mat points3d) {
        depthTo3dSparse_0(depth.nativeObj, in_K.nativeObj, in_points.nativeObj, points3d.nativeObj);
    }


    //
    // C++:  void cv::depthTo3d(Mat depth, Mat K, Mat& points3d, Mat mask = Mat())
    //

    /**
     * Converts a depth image to 3d points. If the mask is empty then the resulting array has the same dimensions as {@code depth},
     * otherwise it is 1d vector containing mask-enabled values only.
     * The coordinate system is x pointing left, y down and z away from the camera
     * @param depth the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
     * (as done with the Microsoft Kinect), otherwise, if given as CV_32F or CV_64F, it is assumed in meters)
     * @param K The calibration matrix
     * @param points3d the resulting 3d points (point is represented by 4 channels value [x, y, z, 0]). They are of the same depth as {@code depth} if it is CV_32F or CV_64F, and the
     * depth of {@code K} if {@code depth} is of depth CV_16U or CV_16S
     * @param mask the mask of the points to consider (can be empty)
     */
    public static void depthTo3d(Mat depth, Mat K, Mat points3d, Mat mask) {
        depthTo3d_0(depth.nativeObj, K.nativeObj, points3d.nativeObj, mask.nativeObj);
    }

    /**
     * Converts a depth image to 3d points. If the mask is empty then the resulting array has the same dimensions as {@code depth},
     * otherwise it is 1d vector containing mask-enabled values only.
     * The coordinate system is x pointing left, y down and z away from the camera
     * @param depth the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
     * (as done with the Microsoft Kinect), otherwise, if given as CV_32F or CV_64F, it is assumed in meters)
     * @param K The calibration matrix
     * @param points3d the resulting 3d points (point is represented by 4 channels value [x, y, z, 0]). They are of the same depth as {@code depth} if it is CV_32F or CV_64F, and the
     * depth of {@code K} if {@code depth} is of depth CV_16U or CV_16S
     */
    public static void depthTo3d(Mat depth, Mat K, Mat points3d) {
        depthTo3d_1(depth.nativeObj, K.nativeObj, points3d.nativeObj);
    }


    //
    // C++:  void cv::rescaleDepth(Mat in, int type, Mat& out, double depth_factor = 1000.0)
    //

    /**
     * If the input image is of type CV_16UC1 (like the Kinect one), the image is converted to floats, divided
     * by depth_factor to get a depth in meters, and the values 0 are converted to std::numeric_limits&lt;float&gt;::quiet_NaN()
     * Otherwise, the image is simply converted to floats
     * @param in the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
     * (as done with the Microsoft Kinect), it is assumed in meters)
     * @param type the desired output depth (CV_32F or CV_64F)
     * @param out The rescaled float depth image
     * @param depth_factor (optional) factor by which depth is converted to distance (by default = 1000.0 for Kinect sensor)
     */
    public static void rescaleDepth(Mat in, int type, Mat out, double depth_factor) {
        rescaleDepth_0(in.nativeObj, type, out.nativeObj, depth_factor);
    }

    /**
     * If the input image is of type CV_16UC1 (like the Kinect one), the image is converted to floats, divided
     * by depth_factor to get a depth in meters, and the values 0 are converted to std::numeric_limits&lt;float&gt;::quiet_NaN()
     * Otherwise, the image is simply converted to floats
     * @param in the depth image (if given as short int CV_U, it is assumed to be the depth in millimeters
     * (as done with the Microsoft Kinect), it is assumed in meters)
     * @param type the desired output depth (CV_32F or CV_64F)
     * @param out The rescaled float depth image
     */
    public static void rescaleDepth(Mat in, int type, Mat out) {
        rescaleDepth_1(in.nativeObj, type, out.nativeObj);
    }


    //
    // C++:  void cv::warpFrame(Mat depth, Mat image, Mat mask, Mat Rt, Mat cameraMatrix, Mat& warpedDepth = Mat(), Mat& warpedImage = Mat(), Mat& warpedMask = Mat())
    //

    /**
     * Warps depth or RGB-D image by reprojecting it in 3d, applying Rt transformation
     * and then projecting it back onto the image plane.
     * This function can be used to visualize the results of the Odometry algorithm.
     * @param depth Depth data, should be 1-channel CV_16U, CV_16S, CV_32F or CV_64F
     * @param image RGB image (optional), should be 1-, 3- or 4-channel CV_8U
     * @param mask Mask of used pixels (optional), should be CV_8UC1
     * @param Rt Rotation+translation matrix (3x4 or 4x4) to be applied to depth points
     * @param cameraMatrix Camera intrinsics matrix (3x3)
     * @param warpedDepth The warped depth data (optional)
     * @param warpedImage The warped RGB image (optional)
     * @param warpedMask The mask of valid pixels in warped image (optional)
     */
    public static void warpFrame(Mat depth, Mat image, Mat mask, Mat Rt, Mat cameraMatrix, Mat warpedDepth, Mat warpedImage, Mat warpedMask) {
        warpFrame_0(depth.nativeObj, image.nativeObj, mask.nativeObj, Rt.nativeObj, cameraMatrix.nativeObj, warpedDepth.nativeObj, warpedImage.nativeObj, warpedMask.nativeObj);
    }

    /**
     * Warps depth or RGB-D image by reprojecting it in 3d, applying Rt transformation
     * and then projecting it back onto the image plane.
     * This function can be used to visualize the results of the Odometry algorithm.
     * @param depth Depth data, should be 1-channel CV_16U, CV_16S, CV_32F or CV_64F
     * @param image RGB image (optional), should be 1-, 3- or 4-channel CV_8U
     * @param mask Mask of used pixels (optional), should be CV_8UC1
     * @param Rt Rotation+translation matrix (3x4 or 4x4) to be applied to depth points
     * @param cameraMatrix Camera intrinsics matrix (3x3)
     * @param warpedDepth The warped depth data (optional)
     * @param warpedImage The warped RGB image (optional)
     */
    public static void warpFrame(Mat depth, Mat image, Mat mask, Mat Rt, Mat cameraMatrix, Mat warpedDepth, Mat warpedImage) {
        warpFrame_1(depth.nativeObj, image.nativeObj, mask.nativeObj, Rt.nativeObj, cameraMatrix.nativeObj, warpedDepth.nativeObj, warpedImage.nativeObj);
    }

    /**
     * Warps depth or RGB-D image by reprojecting it in 3d, applying Rt transformation
     * and then projecting it back onto the image plane.
     * This function can be used to visualize the results of the Odometry algorithm.
     * @param depth Depth data, should be 1-channel CV_16U, CV_16S, CV_32F or CV_64F
     * @param image RGB image (optional), should be 1-, 3- or 4-channel CV_8U
     * @param mask Mask of used pixels (optional), should be CV_8UC1
     * @param Rt Rotation+translation matrix (3x4 or 4x4) to be applied to depth points
     * @param cameraMatrix Camera intrinsics matrix (3x3)
     * @param warpedDepth The warped depth data (optional)
     */
    public static void warpFrame(Mat depth, Mat image, Mat mask, Mat Rt, Mat cameraMatrix, Mat warpedDepth) {
        warpFrame_2(depth.nativeObj, image.nativeObj, mask.nativeObj, Rt.nativeObj, cameraMatrix.nativeObj, warpedDepth.nativeObj);
    }

    /**
     * Warps depth or RGB-D image by reprojecting it in 3d, applying Rt transformation
     * and then projecting it back onto the image plane.
     * This function can be used to visualize the results of the Odometry algorithm.
     * @param depth Depth data, should be 1-channel CV_16U, CV_16S, CV_32F or CV_64F
     * @param image RGB image (optional), should be 1-, 3- or 4-channel CV_8U
     * @param mask Mask of used pixels (optional), should be CV_8UC1
     * @param Rt Rotation+translation matrix (3x4 or 4x4) to be applied to depth points
     * @param cameraMatrix Camera intrinsics matrix (3x3)
     */
    public static void warpFrame(Mat depth, Mat image, Mat mask, Mat Rt, Mat cameraMatrix) {
        warpFrame_3(depth.nativeObj, image.nativeObj, mask.nativeObj, Rt.nativeObj, cameraMatrix.nativeObj);
    }


    //
    // C++:  void cv::findPlanes(Mat points3d, Mat normals, Mat& mask, Mat& plane_coefficients, int block_size = 40, int min_size = 40*40, double threshold = 0.01, double sensor_error_a = 0, double sensor_error_b = 0, double sensor_error_c = 0, RgbdPlaneMethod method = RGBD_PLANE_METHOD_DEFAULT)
    //

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     * @param min_size The minimum size of a cluster to be considered a plane
     * @param threshold The maximum distance of a point from a plane to belong to it (in meters)
     * @param sensor_error_a coefficient of the sensor error. 0 by default, use 0.0075 for a Kinect
     * @param sensor_error_b coefficient of the sensor error. 0 by default
     * @param sensor_error_c coefficient of the sensor error. 0 by default
     * @param method The method to use to compute the planes.
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size, int min_size, double threshold, double sensor_error_a, double sensor_error_b, double sensor_error_c, int method) {
        findPlanes_0(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size, min_size, threshold, sensor_error_a, sensor_error_b, sensor_error_c, method);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     * @param min_size The minimum size of a cluster to be considered a plane
     * @param threshold The maximum distance of a point from a plane to belong to it (in meters)
     * @param sensor_error_a coefficient of the sensor error. 0 by default, use 0.0075 for a Kinect
     * @param sensor_error_b coefficient of the sensor error. 0 by default
     * @param sensor_error_c coefficient of the sensor error. 0 by default
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size, int min_size, double threshold, double sensor_error_a, double sensor_error_b, double sensor_error_c) {
        findPlanes_1(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size, min_size, threshold, sensor_error_a, sensor_error_b, sensor_error_c);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     * @param min_size The minimum size of a cluster to be considered a plane
     * @param threshold The maximum distance of a point from a plane to belong to it (in meters)
     * @param sensor_error_a coefficient of the sensor error. 0 by default, use 0.0075 for a Kinect
     * @param sensor_error_b coefficient of the sensor error. 0 by default
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size, int min_size, double threshold, double sensor_error_a, double sensor_error_b) {
        findPlanes_2(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size, min_size, threshold, sensor_error_a, sensor_error_b);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     * @param min_size The minimum size of a cluster to be considered a plane
     * @param threshold The maximum distance of a point from a plane to belong to it (in meters)
     * @param sensor_error_a coefficient of the sensor error. 0 by default, use 0.0075 for a Kinect
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size, int min_size, double threshold, double sensor_error_a) {
        findPlanes_3(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size, min_size, threshold, sensor_error_a);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     * @param min_size The minimum size of a cluster to be considered a plane
     * @param threshold The maximum distance of a point from a plane to belong to it (in meters)
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size, int min_size, double threshold) {
        findPlanes_4(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size, min_size, threshold);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     * @param min_size The minimum size of a cluster to be considered a plane
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size, int min_size) {
        findPlanes_5(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size, min_size);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     * @param block_size The size of the blocks to look at for a stable MSE
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients, int block_size) {
        findPlanes_6(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj, block_size);
    }

    /**
     * Find the planes in a depth image
     * @param points3d the 3d points organized like the depth image: rows x cols with 3 channels
     * @param normals the normals for every point in the depth image; optional, can be empty
     * @param mask An image where each pixel is labeled with the plane it belongs to
     * and 255 if it does not belong to any plane
     * @param plane_coefficients the coefficients of the corresponding planes (a,b,c,d) such that ax+by+cz+d=0, norm(a,b,c)=1
     * and c &lt; 0 (so that the normal points towards the camera)
     */
    public static void findPlanes(Mat points3d, Mat normals, Mat mask, Mat plane_coefficients) {
        findPlanes_7(points3d.nativeObj, normals.nativeObj, mask.nativeObj, plane_coefficients.nativeObj);
    }




    // C++:  void cv::Rodrigues(Mat src, Mat& dst, Mat& jacobian = Mat())
    private static native void Rodrigues_0(long src_nativeObj, long dst_nativeObj, long jacobian_nativeObj);
    private static native void Rodrigues_1(long src_nativeObj, long dst_nativeObj);

    // C++:  Mat cv::findHomography(vector_Point2f srcPoints, vector_Point2f dstPoints, int method = 0, double ransacReprojThreshold = 3, Mat& mask = Mat(), int maxIters = 2000, double confidence = 0.995)
    private static native long findHomography_0(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj, int method, double ransacReprojThreshold, long mask_nativeObj, int maxIters, double confidence);
    private static native long findHomography_1(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj, int method, double ransacReprojThreshold, long mask_nativeObj, int maxIters);
    private static native long findHomography_2(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj, int method, double ransacReprojThreshold, long mask_nativeObj);
    private static native long findHomography_3(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj, int method, double ransacReprojThreshold);
    private static native long findHomography_4(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj, int method);
    private static native long findHomography_5(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj);

    // C++:  Mat cv::findHomography(vector_Point2f srcPoints, vector_Point2f dstPoints, Mat& mask, UsacParams params)
    private static native long findHomography_6(long srcPoints_mat_nativeObj, long dstPoints_mat_nativeObj, long mask_nativeObj, long params_nativeObj);

    // C++:  Vec3d cv::RQDecomp3x3(Mat src, Mat& mtxR, Mat& mtxQ, Mat& Qx = Mat(), Mat& Qy = Mat(), Mat& Qz = Mat())
    private static native double[] RQDecomp3x3_0(long src_nativeObj, long mtxR_nativeObj, long mtxQ_nativeObj, long Qx_nativeObj, long Qy_nativeObj, long Qz_nativeObj);
    private static native double[] RQDecomp3x3_1(long src_nativeObj, long mtxR_nativeObj, long mtxQ_nativeObj, long Qx_nativeObj, long Qy_nativeObj);
    private static native double[] RQDecomp3x3_2(long src_nativeObj, long mtxR_nativeObj, long mtxQ_nativeObj, long Qx_nativeObj);
    private static native double[] RQDecomp3x3_3(long src_nativeObj, long mtxR_nativeObj, long mtxQ_nativeObj);

    // C++:  void cv::decomposeProjectionMatrix(Mat projMatrix, Mat& cameraMatrix, Mat& rotMatrix, Mat& transVect, Mat& rotMatrixX = Mat(), Mat& rotMatrixY = Mat(), Mat& rotMatrixZ = Mat(), Mat& eulerAngles = Mat())
    private static native void decomposeProjectionMatrix_0(long projMatrix_nativeObj, long cameraMatrix_nativeObj, long rotMatrix_nativeObj, long transVect_nativeObj, long rotMatrixX_nativeObj, long rotMatrixY_nativeObj, long rotMatrixZ_nativeObj, long eulerAngles_nativeObj);
    private static native void decomposeProjectionMatrix_1(long projMatrix_nativeObj, long cameraMatrix_nativeObj, long rotMatrix_nativeObj, long transVect_nativeObj, long rotMatrixX_nativeObj, long rotMatrixY_nativeObj, long rotMatrixZ_nativeObj);
    private static native void decomposeProjectionMatrix_2(long projMatrix_nativeObj, long cameraMatrix_nativeObj, long rotMatrix_nativeObj, long transVect_nativeObj, long rotMatrixX_nativeObj, long rotMatrixY_nativeObj);
    private static native void decomposeProjectionMatrix_3(long projMatrix_nativeObj, long cameraMatrix_nativeObj, long rotMatrix_nativeObj, long transVect_nativeObj, long rotMatrixX_nativeObj);
    private static native void decomposeProjectionMatrix_4(long projMatrix_nativeObj, long cameraMatrix_nativeObj, long rotMatrix_nativeObj, long transVect_nativeObj);

    // C++:  void cv::matMulDeriv(Mat A, Mat B, Mat& dABdA, Mat& dABdB)
    private static native void matMulDeriv_0(long A_nativeObj, long B_nativeObj, long dABdA_nativeObj, long dABdB_nativeObj);

    // C++:  void cv::composeRT(Mat rvec1, Mat tvec1, Mat rvec2, Mat tvec2, Mat& rvec3, Mat& tvec3, Mat& dr3dr1 = Mat(), Mat& dr3dt1 = Mat(), Mat& dr3dr2 = Mat(), Mat& dr3dt2 = Mat(), Mat& dt3dr1 = Mat(), Mat& dt3dt1 = Mat(), Mat& dt3dr2 = Mat(), Mat& dt3dt2 = Mat())
    private static native void composeRT_0(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj, long dr3dr2_nativeObj, long dr3dt2_nativeObj, long dt3dr1_nativeObj, long dt3dt1_nativeObj, long dt3dr2_nativeObj, long dt3dt2_nativeObj);
    private static native void composeRT_1(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj, long dr3dr2_nativeObj, long dr3dt2_nativeObj, long dt3dr1_nativeObj, long dt3dt1_nativeObj, long dt3dr2_nativeObj);
    private static native void composeRT_2(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj, long dr3dr2_nativeObj, long dr3dt2_nativeObj, long dt3dr1_nativeObj, long dt3dt1_nativeObj);
    private static native void composeRT_3(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj, long dr3dr2_nativeObj, long dr3dt2_nativeObj, long dt3dr1_nativeObj);
    private static native void composeRT_4(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj, long dr3dr2_nativeObj, long dr3dt2_nativeObj);
    private static native void composeRT_5(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj, long dr3dr2_nativeObj);
    private static native void composeRT_6(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj, long dr3dt1_nativeObj);
    private static native void composeRT_7(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj, long dr3dr1_nativeObj);
    private static native void composeRT_8(long rvec1_nativeObj, long tvec1_nativeObj, long rvec2_nativeObj, long tvec2_nativeObj, long rvec3_nativeObj, long tvec3_nativeObj);

    // C++:  void cv::projectPoints(vector_Point3f objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, vector_double distCoeffs, vector_Point2f& imagePoints, Mat& jacobian = Mat(), double aspectRatio = 0)
    private static native void projectPoints_0(long objectPoints_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long imagePoints_mat_nativeObj, long jacobian_nativeObj, double aspectRatio);
    private static native void projectPoints_1(long objectPoints_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long imagePoints_mat_nativeObj, long jacobian_nativeObj);
    private static native void projectPoints_2(long objectPoints_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long imagePoints_mat_nativeObj);

    // C++:  void cv::projectPoints(Mat objectPoints, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs, Mat& imagePoints, Mat& dpdr, Mat& dpdt, Mat& dpdf = Mat(), Mat& dpdc = Mat(), Mat& dpdk = Mat(), Mat& dpdo = Mat(), double aspectRatio = 0.)
    private static native void projectPointsSepJ_0(long objectPoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long imagePoints_nativeObj, long dpdr_nativeObj, long dpdt_nativeObj, long dpdf_nativeObj, long dpdc_nativeObj, long dpdk_nativeObj, long dpdo_nativeObj, double aspectRatio);
    private static native void projectPointsSepJ_1(long objectPoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long imagePoints_nativeObj, long dpdr_nativeObj, long dpdt_nativeObj, long dpdf_nativeObj, long dpdc_nativeObj, long dpdk_nativeObj, long dpdo_nativeObj);
    private static native void projectPointsSepJ_2(long objectPoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long imagePoints_nativeObj, long dpdr_nativeObj, long dpdt_nativeObj, long dpdf_nativeObj, long dpdc_nativeObj, long dpdk_nativeObj);
    private static native void projectPointsSepJ_3(long objectPoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long imagePoints_nativeObj, long dpdr_nativeObj, long dpdt_nativeObj, long dpdf_nativeObj, long dpdc_nativeObj);
    private static native void projectPointsSepJ_4(long objectPoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long imagePoints_nativeObj, long dpdr_nativeObj, long dpdt_nativeObj, long dpdf_nativeObj);
    private static native void projectPointsSepJ_5(long objectPoints_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long imagePoints_nativeObj, long dpdr_nativeObj, long dpdt_nativeObj);

    // C++:  bool cv::solvePnP(vector_Point3f objectPoints, vector_Point2f imagePoints, Mat cameraMatrix, vector_double distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE)
    private static native boolean solvePnP_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess, int flags);
    private static native boolean solvePnP_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess);
    private static native boolean solvePnP_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj);

    // C++:  bool cv::solvePnPRansac(vector_Point3f objectPoints, vector_Point2f imagePoints, Mat cameraMatrix, vector_double distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false, int iterationsCount = 100, float reprojectionError = 8.0, double confidence = 0.99, Mat& inliers = Mat(), int flags = SOLVEPNP_ITERATIVE)
    private static native boolean solvePnPRansac_0(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence, long inliers_nativeObj, int flags);
    private static native boolean solvePnPRansac_1(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence, long inliers_nativeObj);
    private static native boolean solvePnPRansac_2(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence);
    private static native boolean solvePnPRansac_3(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess, int iterationsCount, float reprojectionError);
    private static native boolean solvePnPRansac_4(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess, int iterationsCount);
    private static native boolean solvePnPRansac_5(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess);
    private static native boolean solvePnPRansac_6(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj);

    // C++:  bool cv::solvePnPRansac(vector_Point3f objectPoints, vector_Point2f imagePoints, Mat& cameraMatrix, vector_double distCoeffs, Mat& rvec, Mat& tvec, Mat& inliers, UsacParams params = UsacParams())
    private static native boolean solvePnPRansac_7(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long inliers_nativeObj, long params_nativeObj);
    private static native boolean solvePnPRansac_8(long objectPoints_mat_nativeObj, long imagePoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_mat_nativeObj, long rvec_nativeObj, long tvec_nativeObj, long inliers_nativeObj);

    // C++:  int cv::solveP3P(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, int flags)
    private static native int solveP3P_0(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags);

    // C++:  void cv::solvePnPRefineLM(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, FLT_EPSILON))
    private static native void solvePnPRefineLM_0(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native void solvePnPRefineLM_1(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj);

    // C++:  void cv::solvePnPRefineVVS(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, TermCriteria criteria = TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 20, FLT_EPSILON), double VVSlambda = 1)
    private static native void solvePnPRefineVVS_0(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, int criteria_type, int criteria_maxCount, double criteria_epsilon, double VVSlambda);
    private static native void solvePnPRefineVVS_1(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native void solvePnPRefineVVS_2(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj);

    // C++:  int cv::solvePnPGeneric(Mat objectPoints, Mat imagePoints, Mat cameraMatrix, Mat distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, bool useExtrinsicGuess = false, int flags = SOLVEPNP_ITERATIVE, Mat rvec = Mat(), Mat tvec = Mat(), Mat& reprojectionError = Mat())
    private static native int solvePnPGeneric_0(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, boolean useExtrinsicGuess, int flags, long rvec_nativeObj, long tvec_nativeObj, long reprojectionError_nativeObj);
    private static native int solvePnPGeneric_1(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, boolean useExtrinsicGuess, int flags, long rvec_nativeObj, long tvec_nativeObj);
    private static native int solvePnPGeneric_2(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, boolean useExtrinsicGuess, int flags, long rvec_nativeObj);
    private static native int solvePnPGeneric_3(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, boolean useExtrinsicGuess, int flags);
    private static native int solvePnPGeneric_4(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, boolean useExtrinsicGuess);
    private static native int solvePnPGeneric_5(long objectPoints_nativeObj, long imagePoints_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj);

    // C++:  void cv::drawFrameAxes(Mat& image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length, int thickness = 3)
    private static native void drawFrameAxes_0(long image_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, float length, int thickness);
    private static native void drawFrameAxes_1(long image_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, float length);

    // C++:  void cv::convertPointsToHomogeneous(Mat src, Mat& dst, int dtype = -1)
    private static native void convertPointsToHomogeneous_0(long src_nativeObj, long dst_nativeObj, int dtype);
    private static native void convertPointsToHomogeneous_1(long src_nativeObj, long dst_nativeObj);

    // C++:  void cv::convertPointsFromHomogeneous(Mat src, Mat& dst, int dtype = -1)
    private static native void convertPointsFromHomogeneous_0(long src_nativeObj, long dst_nativeObj, int dtype);
    private static native void convertPointsFromHomogeneous_1(long src_nativeObj, long dst_nativeObj);

    // C++:  Mat cv::findFundamentalMat(vector_Point2f points1, vector_Point2f points2, int method, double ransacReprojThreshold, double confidence, int maxIters, Mat& mask = Mat())
    private static native long findFundamentalMat_0(long points1_mat_nativeObj, long points2_mat_nativeObj, int method, double ransacReprojThreshold, double confidence, int maxIters, long mask_nativeObj);
    private static native long findFundamentalMat_1(long points1_mat_nativeObj, long points2_mat_nativeObj, int method, double ransacReprojThreshold, double confidence, int maxIters);

    // C++:  Mat cv::findFundamentalMat(vector_Point2f points1, vector_Point2f points2, int method = FM_RANSAC, double ransacReprojThreshold = 3., double confidence = 0.99, Mat& mask = Mat())
    private static native long findFundamentalMat_2(long points1_mat_nativeObj, long points2_mat_nativeObj, int method, double ransacReprojThreshold, double confidence, long mask_nativeObj);
    private static native long findFundamentalMat_3(long points1_mat_nativeObj, long points2_mat_nativeObj, int method, double ransacReprojThreshold, double confidence);
    private static native long findFundamentalMat_4(long points1_mat_nativeObj, long points2_mat_nativeObj, int method, double ransacReprojThreshold);
    private static native long findFundamentalMat_5(long points1_mat_nativeObj, long points2_mat_nativeObj, int method);
    private static native long findFundamentalMat_6(long points1_mat_nativeObj, long points2_mat_nativeObj);

    // C++:  Mat cv::findFundamentalMat(vector_Point2f points1, vector_Point2f points2, Mat& mask, UsacParams params)
    private static native long findFundamentalMat_7(long points1_mat_nativeObj, long points2_mat_nativeObj, long mask_nativeObj, long params_nativeObj);

    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix, int method = RANSAC, double prob = 0.999, double threshold = 1.0, int maxIters = 1000, Mat& mask = Mat())
    private static native long findEssentialMat_0(long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, int method, double prob, double threshold, int maxIters, long mask_nativeObj);
    private static native long findEssentialMat_1(long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, int method, double prob, double threshold, int maxIters);
    private static native long findEssentialMat_2(long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, int method, double prob, double threshold);
    private static native long findEssentialMat_3(long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, int method, double prob);
    private static native long findEssentialMat_4(long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, int method);
    private static native long findEssentialMat_5(long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj);

    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, double focal = 1.0, Point2d pp = Point2d(0, 0), int method = RANSAC, double prob = 0.999, double threshold = 1.0, int maxIters = 1000, Mat& mask = Mat())
    private static native long findEssentialMat_6(long points1_nativeObj, long points2_nativeObj, double focal, double pp_x, double pp_y, int method, double prob, double threshold, int maxIters, long mask_nativeObj);
    private static native long findEssentialMat_7(long points1_nativeObj, long points2_nativeObj, double focal, double pp_x, double pp_y, int method, double prob, double threshold, int maxIters);
    private static native long findEssentialMat_8(long points1_nativeObj, long points2_nativeObj, double focal, double pp_x, double pp_y, int method, double prob, double threshold);
    private static native long findEssentialMat_9(long points1_nativeObj, long points2_nativeObj, double focal, double pp_x, double pp_y, int method, double prob);
    private static native long findEssentialMat_10(long points1_nativeObj, long points2_nativeObj, double focal, double pp_x, double pp_y, int method);
    private static native long findEssentialMat_11(long points1_nativeObj, long points2_nativeObj, double focal, double pp_x, double pp_y);
    private static native long findEssentialMat_12(long points1_nativeObj, long points2_nativeObj, double focal);
    private static native long findEssentialMat_13(long points1_nativeObj, long points2_nativeObj);

    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, int method = RANSAC, double prob = 0.999, double threshold = 1.0, Mat& mask = Mat())
    private static native long findEssentialMat_14(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, int method, double prob, double threshold, long mask_nativeObj);
    private static native long findEssentialMat_15(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, int method, double prob, double threshold);
    private static native long findEssentialMat_16(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, int method, double prob);
    private static native long findEssentialMat_17(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, int method);
    private static native long findEssentialMat_18(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj);

    // C++:  Mat cv::findEssentialMat(Mat points1, Mat points2, Mat cameraMatrix1, Mat cameraMatrix2, Mat dist_coeff1, Mat dist_coeff2, Mat& mask, UsacParams params)
    private static native long findEssentialMat_19(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long cameraMatrix2_nativeObj, long dist_coeff1_nativeObj, long dist_coeff2_nativeObj, long mask_nativeObj, long params_nativeObj);

    // C++:  void cv::decomposeEssentialMat(Mat E, Mat& R1, Mat& R2, Mat& t)
    private static native void decomposeEssentialMat_0(long E_nativeObj, long R1_nativeObj, long R2_nativeObj, long t_nativeObj);

    // C++:  int cv::recoverPose(Mat points1, Mat points2, Mat cameraMatrix1, Mat distCoeffs1, Mat cameraMatrix2, Mat distCoeffs2, Mat& E, Mat& R, Mat& t, int method = cv::RANSAC, double prob = 0.999, double threshold = 1.0, Mat& mask = Mat())
    private static native int recoverPose_0(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, long E_nativeObj, long R_nativeObj, long t_nativeObj, int method, double prob, double threshold, long mask_nativeObj);
    private static native int recoverPose_1(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, long E_nativeObj, long R_nativeObj, long t_nativeObj, int method, double prob, double threshold);
    private static native int recoverPose_2(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, long E_nativeObj, long R_nativeObj, long t_nativeObj, int method, double prob);
    private static native int recoverPose_3(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, long E_nativeObj, long R_nativeObj, long t_nativeObj, int method);
    private static native int recoverPose_4(long points1_nativeObj, long points2_nativeObj, long cameraMatrix1_nativeObj, long distCoeffs1_nativeObj, long cameraMatrix2_nativeObj, long distCoeffs2_nativeObj, long E_nativeObj, long R_nativeObj, long t_nativeObj);

    // C++:  int cv::recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat& R, Mat& t, Mat& mask = Mat())
    private static native int recoverPose_5(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, long R_nativeObj, long t_nativeObj, long mask_nativeObj);
    private static native int recoverPose_6(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, long R_nativeObj, long t_nativeObj);

    // C++:  int cv::recoverPose(Mat E, Mat points1, Mat points2, Mat& R, Mat& t, double focal = 1.0, Point2d pp = Point2d(0, 0), Mat& mask = Mat())
    private static native int recoverPose_7(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long R_nativeObj, long t_nativeObj, double focal, double pp_x, double pp_y, long mask_nativeObj);
    private static native int recoverPose_8(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long R_nativeObj, long t_nativeObj, double focal, double pp_x, double pp_y);
    private static native int recoverPose_9(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long R_nativeObj, long t_nativeObj, double focal);
    private static native int recoverPose_10(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long R_nativeObj, long t_nativeObj);

    // C++:  int cv::recoverPose(Mat E, Mat points1, Mat points2, Mat cameraMatrix, Mat& R, Mat& t, double distanceThresh, Mat& mask = Mat(), Mat& triangulatedPoints = Mat())
    private static native int recoverPose_11(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, long R_nativeObj, long t_nativeObj, double distanceThresh, long mask_nativeObj, long triangulatedPoints_nativeObj);
    private static native int recoverPose_12(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, long R_nativeObj, long t_nativeObj, double distanceThresh, long mask_nativeObj);
    private static native int recoverPose_13(long E_nativeObj, long points1_nativeObj, long points2_nativeObj, long cameraMatrix_nativeObj, long R_nativeObj, long t_nativeObj, double distanceThresh);

    // C++:  void cv::computeCorrespondEpilines(Mat points, int whichImage, Mat F, Mat& lines)
    private static native void computeCorrespondEpilines_0(long points_nativeObj, int whichImage, long F_nativeObj, long lines_nativeObj);

    // C++:  void cv::triangulatePoints(Mat projMatr1, Mat projMatr2, Mat projPoints1, Mat projPoints2, Mat& points4D)
    private static native void triangulatePoints_0(long projMatr1_nativeObj, long projMatr2_nativeObj, long projPoints1_nativeObj, long projPoints2_nativeObj, long points4D_nativeObj);

    // C++:  void cv::correctMatches(Mat F, Mat points1, Mat points2, Mat& newPoints1, Mat& newPoints2)
    private static native void correctMatches_0(long F_nativeObj, long points1_nativeObj, long points2_nativeObj, long newPoints1_nativeObj, long newPoints2_nativeObj);

    // C++:  double cv::sampsonDistance(Mat pt1, Mat pt2, Mat F)
    private static native double sampsonDistance_0(long pt1_nativeObj, long pt2_nativeObj, long F_nativeObj);

    // C++:  int cv::estimateAffine3D(Mat src, Mat dst, Mat& out, Mat& inliers, double ransacThreshold = 3, double confidence = 0.99)
    private static native int estimateAffine3D_0(long src_nativeObj, long dst_nativeObj, long out_nativeObj, long inliers_nativeObj, double ransacThreshold, double confidence);
    private static native int estimateAffine3D_1(long src_nativeObj, long dst_nativeObj, long out_nativeObj, long inliers_nativeObj, double ransacThreshold);
    private static native int estimateAffine3D_2(long src_nativeObj, long dst_nativeObj, long out_nativeObj, long inliers_nativeObj);

    // C++:  Mat cv::estimateAffine3D(Mat src, Mat dst, double* scale = nullptr, bool force_rotation = true)
    private static native long estimateAffine3D_3(long src_nativeObj, long dst_nativeObj, double[] scale_out, boolean force_rotation);
    private static native long estimateAffine3D_4(long src_nativeObj, long dst_nativeObj, double[] scale_out);
    private static native long estimateAffine3D_5(long src_nativeObj, long dst_nativeObj);

    // C++:  int cv::estimateTranslation3D(Mat src, Mat dst, Mat& out, Mat& inliers, double ransacThreshold = 3, double confidence = 0.99)
    private static native int estimateTranslation3D_0(long src_nativeObj, long dst_nativeObj, long out_nativeObj, long inliers_nativeObj, double ransacThreshold, double confidence);
    private static native int estimateTranslation3D_1(long src_nativeObj, long dst_nativeObj, long out_nativeObj, long inliers_nativeObj, double ransacThreshold);
    private static native int estimateTranslation3D_2(long src_nativeObj, long dst_nativeObj, long out_nativeObj, long inliers_nativeObj);

    // C++:  Mat cv::estimateAffine2D(Mat from, Mat to, Mat& inliers = Mat(), int method = RANSAC, double ransacReprojThreshold = 3, size_t maxIters = 2000, double confidence = 0.99, size_t refineIters = 10)
    private static native long estimateAffine2D_0(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold, long maxIters, double confidence, long refineIters);
    private static native long estimateAffine2D_1(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold, long maxIters, double confidence);
    private static native long estimateAffine2D_2(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold, long maxIters);
    private static native long estimateAffine2D_3(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold);
    private static native long estimateAffine2D_4(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method);
    private static native long estimateAffine2D_5(long from_nativeObj, long to_nativeObj, long inliers_nativeObj);
    private static native long estimateAffine2D_6(long from_nativeObj, long to_nativeObj);

    // C++:  Mat cv::estimateAffine2D(Mat pts1, Mat pts2, Mat& inliers, UsacParams params)
    private static native long estimateAffine2D_7(long pts1_nativeObj, long pts2_nativeObj, long inliers_nativeObj, long params_nativeObj);

    // C++:  Mat cv::estimateAffinePartial2D(Mat from, Mat to, Mat& inliers = Mat(), int method = RANSAC, double ransacReprojThreshold = 3, size_t maxIters = 2000, double confidence = 0.99, size_t refineIters = 10)
    private static native long estimateAffinePartial2D_0(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold, long maxIters, double confidence, long refineIters);
    private static native long estimateAffinePartial2D_1(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold, long maxIters, double confidence);
    private static native long estimateAffinePartial2D_2(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold, long maxIters);
    private static native long estimateAffinePartial2D_3(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method, double ransacReprojThreshold);
    private static native long estimateAffinePartial2D_4(long from_nativeObj, long to_nativeObj, long inliers_nativeObj, int method);
    private static native long estimateAffinePartial2D_5(long from_nativeObj, long to_nativeObj, long inliers_nativeObj);
    private static native long estimateAffinePartial2D_6(long from_nativeObj, long to_nativeObj);

    // C++:  int cv::decomposeHomographyMat(Mat H, Mat K, vector_Mat& rotations, vector_Mat& translations, vector_Mat& normals)
    private static native int decomposeHomographyMat_0(long H_nativeObj, long K_nativeObj, long rotations_mat_nativeObj, long translations_mat_nativeObj, long normals_mat_nativeObj);

    // C++:  void cv::filterHomographyDecompByVisibleRefpoints(vector_Mat rotations, vector_Mat normals, Mat beforePoints, Mat afterPoints, Mat& possibleSolutions, Mat pointsMask = Mat())
    private static native void filterHomographyDecompByVisibleRefpoints_0(long rotations_mat_nativeObj, long normals_mat_nativeObj, long beforePoints_nativeObj, long afterPoints_nativeObj, long possibleSolutions_nativeObj, long pointsMask_nativeObj);
    private static native void filterHomographyDecompByVisibleRefpoints_1(long rotations_mat_nativeObj, long normals_mat_nativeObj, long beforePoints_nativeObj, long afterPoints_nativeObj, long possibleSolutions_nativeObj);

    // C++:  void cv::undistort(Mat src, Mat& dst, Mat cameraMatrix, Mat distCoeffs, Mat newCameraMatrix = Mat())
    private static native void undistort_0(long src_nativeObj, long dst_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long newCameraMatrix_nativeObj);
    private static native void undistort_1(long src_nativeObj, long dst_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj);

    // C++:  void cv::initUndistortRectifyMap(Mat cameraMatrix, Mat distCoeffs, Mat R, Mat newCameraMatrix, Size size, int m1type, Mat& map1, Mat& map2)
    private static native void initUndistortRectifyMap_0(long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long R_nativeObj, long newCameraMatrix_nativeObj, double size_width, double size_height, int m1type, long map1_nativeObj, long map2_nativeObj);

    // C++:  void cv::initInverseRectificationMap(Mat cameraMatrix, Mat distCoeffs, Mat R, Mat newCameraMatrix, Size size, int m1type, Mat& map1, Mat& map2)
    private static native void initInverseRectificationMap_0(long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long R_nativeObj, long newCameraMatrix_nativeObj, double size_width, double size_height, int m1type, long map1_nativeObj, long map2_nativeObj);

    // C++:  Mat cv::getDefaultNewCameraMatrix(Mat cameraMatrix, Size imgsize = Size(), bool centerPrincipalPoint = false)
    private static native long getDefaultNewCameraMatrix_0(long cameraMatrix_nativeObj, double imgsize_width, double imgsize_height, boolean centerPrincipalPoint);
    private static native long getDefaultNewCameraMatrix_1(long cameraMatrix_nativeObj, double imgsize_width, double imgsize_height);
    private static native long getDefaultNewCameraMatrix_2(long cameraMatrix_nativeObj);

    // C++:  Mat cv::getOptimalNewCameraMatrix(Mat cameraMatrix, Mat distCoeffs, Size imageSize, double alpha, Size newImgSize = Size(), Rect* validPixROI = 0, bool centerPrincipalPoint = false)
    private static native long getOptimalNewCameraMatrix_0(long cameraMatrix_nativeObj, long distCoeffs_nativeObj, double imageSize_width, double imageSize_height, double alpha, double newImgSize_width, double newImgSize_height, double[] validPixROI_out, boolean centerPrincipalPoint);
    private static native long getOptimalNewCameraMatrix_1(long cameraMatrix_nativeObj, long distCoeffs_nativeObj, double imageSize_width, double imageSize_height, double alpha, double newImgSize_width, double newImgSize_height, double[] validPixROI_out);
    private static native long getOptimalNewCameraMatrix_2(long cameraMatrix_nativeObj, long distCoeffs_nativeObj, double imageSize_width, double imageSize_height, double alpha, double newImgSize_width, double newImgSize_height);
    private static native long getOptimalNewCameraMatrix_3(long cameraMatrix_nativeObj, long distCoeffs_nativeObj, double imageSize_width, double imageSize_height, double alpha);

    // C++:  void cv::undistortPoints(vector_Point2f src, vector_Point2f& dst, Mat cameraMatrix, Mat distCoeffs, Mat R = Mat(), Mat P = Mat(), TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER, 5, 0.01))
    private static native void undistortPoints_0(long src_mat_nativeObj, long dst_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long R_nativeObj, long P_nativeObj, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native void undistortPoints_1(long src_mat_nativeObj, long dst_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long R_nativeObj, long P_nativeObj);
    private static native void undistortPoints_2(long src_mat_nativeObj, long dst_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long R_nativeObj);
    private static native void undistortPoints_3(long src_mat_nativeObj, long dst_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj);

    // C++:  void cv::undistortImagePoints(Mat src, Mat& dst, Mat cameraMatrix, Mat distCoeffs, TermCriteria arg1 = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 5, 0.01))
    private static native void undistortImagePoints_0(long src_nativeObj, long dst_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, int arg1_type, int arg1_maxCount, double arg1_epsilon);
    private static native void undistortImagePoints_1(long src_nativeObj, long dst_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj);

    // C++:  void cv::loadPointCloud(String filename, Mat& vertices, Mat& normals = Mat())
    private static native void loadPointCloud_0(String filename, long vertices_nativeObj, long normals_nativeObj);
    private static native void loadPointCloud_1(String filename, long vertices_nativeObj);

    // C++:  void cv::savePointCloud(String filename, Mat vertices, Mat normals = Mat())
    private static native void savePointCloud_0(String filename, long vertices_nativeObj, long normals_nativeObj);
    private static native void savePointCloud_1(String filename, long vertices_nativeObj);

    // C++:  void cv::loadMesh(String filename, Mat& vertices, Mat& normals, vector_Mat& indices)
    private static native void loadMesh_0(String filename, long vertices_nativeObj, long normals_nativeObj, long indices_mat_nativeObj);

    // C++:  void cv::saveMesh(String filename, Mat vertices, Mat normals, vector_Mat indices)
    private static native void saveMesh_0(String filename, long vertices_nativeObj, long normals_nativeObj, long indices_mat_nativeObj);

    // C++:  void cv::registerDepth(Mat unregisteredCameraMatrix, Mat registeredCameraMatrix, Mat registeredDistCoeffs, Mat Rt, Mat unregisteredDepth, Size outputImagePlaneSize, Mat& registeredDepth, bool depthDilation = false)
    private static native void registerDepth_0(long unregisteredCameraMatrix_nativeObj, long registeredCameraMatrix_nativeObj, long registeredDistCoeffs_nativeObj, long Rt_nativeObj, long unregisteredDepth_nativeObj, double outputImagePlaneSize_width, double outputImagePlaneSize_height, long registeredDepth_nativeObj, boolean depthDilation);
    private static native void registerDepth_1(long unregisteredCameraMatrix_nativeObj, long registeredCameraMatrix_nativeObj, long registeredDistCoeffs_nativeObj, long Rt_nativeObj, long unregisteredDepth_nativeObj, double outputImagePlaneSize_width, double outputImagePlaneSize_height, long registeredDepth_nativeObj);

    // C++:  void cv::depthTo3dSparse(Mat depth, Mat in_K, Mat in_points, Mat& points3d)
    private static native void depthTo3dSparse_0(long depth_nativeObj, long in_K_nativeObj, long in_points_nativeObj, long points3d_nativeObj);

    // C++:  void cv::depthTo3d(Mat depth, Mat K, Mat& points3d, Mat mask = Mat())
    private static native void depthTo3d_0(long depth_nativeObj, long K_nativeObj, long points3d_nativeObj, long mask_nativeObj);
    private static native void depthTo3d_1(long depth_nativeObj, long K_nativeObj, long points3d_nativeObj);

    // C++:  void cv::rescaleDepth(Mat in, int type, Mat& out, double depth_factor = 1000.0)
    private static native void rescaleDepth_0(long in_nativeObj, int type, long out_nativeObj, double depth_factor);
    private static native void rescaleDepth_1(long in_nativeObj, int type, long out_nativeObj);

    // C++:  void cv::warpFrame(Mat depth, Mat image, Mat mask, Mat Rt, Mat cameraMatrix, Mat& warpedDepth = Mat(), Mat& warpedImage = Mat(), Mat& warpedMask = Mat())
    private static native void warpFrame_0(long depth_nativeObj, long image_nativeObj, long mask_nativeObj, long Rt_nativeObj, long cameraMatrix_nativeObj, long warpedDepth_nativeObj, long warpedImage_nativeObj, long warpedMask_nativeObj);
    private static native void warpFrame_1(long depth_nativeObj, long image_nativeObj, long mask_nativeObj, long Rt_nativeObj, long cameraMatrix_nativeObj, long warpedDepth_nativeObj, long warpedImage_nativeObj);
    private static native void warpFrame_2(long depth_nativeObj, long image_nativeObj, long mask_nativeObj, long Rt_nativeObj, long cameraMatrix_nativeObj, long warpedDepth_nativeObj);
    private static native void warpFrame_3(long depth_nativeObj, long image_nativeObj, long mask_nativeObj, long Rt_nativeObj, long cameraMatrix_nativeObj);

    // C++:  void cv::findPlanes(Mat points3d, Mat normals, Mat& mask, Mat& plane_coefficients, int block_size = 40, int min_size = 40*40, double threshold = 0.01, double sensor_error_a = 0, double sensor_error_b = 0, double sensor_error_c = 0, RgbdPlaneMethod method = RGBD_PLANE_METHOD_DEFAULT)
    private static native void findPlanes_0(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size, int min_size, double threshold, double sensor_error_a, double sensor_error_b, double sensor_error_c, int method);
    private static native void findPlanes_1(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size, int min_size, double threshold, double sensor_error_a, double sensor_error_b, double sensor_error_c);
    private static native void findPlanes_2(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size, int min_size, double threshold, double sensor_error_a, double sensor_error_b);
    private static native void findPlanes_3(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size, int min_size, double threshold, double sensor_error_a);
    private static native void findPlanes_4(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size, int min_size, double threshold);
    private static native void findPlanes_5(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size, int min_size);
    private static native void findPlanes_6(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj, int block_size);
    private static native void findPlanes_7(long points3d_nativeObj, long normals_nativeObj, long mask_nativeObj, long plane_coefficients_nativeObj);

}
