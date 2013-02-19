/*
 * Class for measuring time.
 * 
 * Written by Blai Bonet (bonet@ldc.usb.ve)
 * 
 * Last modified 1/18/13 
 * 
 */

import java.lang.management.*;
 
class Timing {
    private ThreadMXBean bean;

    public Timing() {
        bean = ManagementFactory.getThreadMXBean();
    }

    public long getCpuTime() {
        long time = bean.getCurrentThreadCpuTime();
        return bean.isCurrentThreadCpuTimeSupported() ? time / 1000000L : 0L;
    }

    public long getUserTime() {
        long time = bean.getCurrentThreadUserTime();
        return bean.isCurrentThreadCpuTimeSupported() ? time / 1000000L : 0L;
    }

    public long getSystemTime() {
        long time = bean.getCurrentThreadCpuTime() - bean.getCurrentThreadUserTime();
        return bean.isCurrentThreadCpuTimeSupported() ? time / 1000000L : 0L;
    }
}

