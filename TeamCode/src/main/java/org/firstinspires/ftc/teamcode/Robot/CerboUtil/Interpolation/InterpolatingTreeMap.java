package org.firstinspires.ftc.teamcode.Robot.CerboUtil.Interpolation;

import org.jetbrains.annotations.Nullable;

import java.util.Map;
import java.util.TreeMap;

public class InterpolatingTreeMap <K extends InverseInterpolable<K> & Comparable<K>, V extends Interpolable<V>>
        extends TreeMap<K, V> {
    private static final long serialVersionUID = 8347275262778054124L;

    final int max_;

    public InterpolatingTreeMap(int maximumSize){
        max_ = maximumSize;
    }

    public InterpolatingTreeMap(){
        this(0);
    }

    @Nullable
    @Override
    public V put(K key, V value) {
        if(max_ > 0 && max_ <= size()){
            K first = firstKey();
            remove(first);
        }

        super.put(key, value);

        return value;
    }

    @Override
    public void putAll(Map<? extends  K, ? extends V> map) {
        System.out.println("Unimplemented metod");
    }

    public V getInterpolated(K key){
        V gotval = get(key);
        if(gotval == null){
            // get surrounding keys for interpolation
            K topBound = ceilingKey(key);
            K bottomBound = floorKey(key);

            // if attempting interpolation at ends of tree, return the nearest data point
            if (topBound == null && bottomBound == null) {
                return null;
            } else if (topBound == null) {
                return get(bottomBound);
            } else if (bottomBound == null) {
                return get(topBound);
            }

            // get surrounding values for interpolation
            V topElem = get(topBound);
            V bottomElem = get(bottomBound);
            return bottomElem.interpolate(topElem, bottomBound.inverseInterpolate(topBound, key));
        } else {
            return gotval;
        }
    }
}
