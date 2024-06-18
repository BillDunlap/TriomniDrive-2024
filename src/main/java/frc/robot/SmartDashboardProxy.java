/**
 * A class for storing information and perhaps forwarding it to SmakeDashboard.
 */
package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SmartDashboardProxy {
    public class CurrentAndPrevious<E> {
        public E m_current;
        public boolean m_hasPrevious;
        public E m_previous;
        public CurrentAndPrevious(E e){
            m_hasPrevious = false;
            m_current = e;
        }
        public void set(E e) {
            m_hasPrevious = true;
            m_previous = m_current;
            m_current = e;
        }
    }
    private HashMap<String,CurrentAndPrevious<Double>> m_numbers = new HashMap<String,CurrentAndPrevious<Double>>();
    //private HashMap<String,String> m_strings = new HashMap<String,String>();
    //private HashMap<String,Boolean> m_booleans = new HashMap<String,Boolean>();

    public SmartDashboardProxy() {
      // Nothing to do here
    }

    public void putNumber(String key, Double number, boolean forwardToSmartDashboard){
      putNumber(key, number);
      if (forwardToSmartDashboard) {
        SmartDashboard.putNumber(key, number);
      }
    }
    public void putNumber(String key, Double number) {
        CurrentAndPrevious<Double> z = m_numbers.get(key);
        if (z == null) {
            m_numbers.put(key, new CurrentAndPrevious<Double>(number));
        } else {
            z.set(number);
        }
    }    
    public Double getNumber(String key) {
      return m_numbers.get(key).m_current;
    }
    //public Double getOrDefaultNumber(String key, Double defaultValue) {
    //  return m_numbers.getOrDefault(key, defaultValue);
    //
}
