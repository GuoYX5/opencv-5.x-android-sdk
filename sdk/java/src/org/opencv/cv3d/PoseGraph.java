//
// This file is auto-generated. Please don't modify it!
//
package org.opencv.cv3d;



// C++: class PoseGraph
/**
 * Base class for Levenberg-Marquadt solvers.
 *
 * This class can be used for general local optimization using sparse linear solvers, exponential param update or fixed variables
 * implemented in child classes.
 * This base class does not depend on a type, layout or a group structure of a param vector or an objective function jacobian.
 * A child class should provide a storage for that data and implement all virtual member functions that process it.
 * This class does not support fixed/masked variables, this should also be implemented in child classes.
 */
public class PoseGraph {

    protected final long nativeObj;
    protected PoseGraph(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    // internal usage only
    public static PoseGraph __fromPtr__(long addr) { return new PoseGraph(addr); }

    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // native support for java finalize()
    private static native void delete(long nativeObj);

}
