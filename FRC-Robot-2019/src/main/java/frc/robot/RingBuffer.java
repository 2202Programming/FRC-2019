package frc.robot;

/**
 * A RingBuffer object is essentially a double array with a head.
 * The head increments every time a new value is incremented.
 * 
 * @author Kevin Li
 */
public class RingBuffer {
    // TODO: Use this class to replace Deques in other classes

    private double[] array;
    private int head = 0;
    private boolean wrapped = false;

    /**
     * Creates a RingBuffer object.
     * @param length the length of the RingBuffer
     * @throws IllegalArgumentException if length <= 0
     */
    public RingBuffer(int length) {
        if (length <= 0)
            throw new IllegalArgumentException("Length must be positive.");
        array = new double[length];
    }

    /**
     * Inserts a value to the RingBuffer at index head.
     * @param value the value to be inserted into the RingBuffer
     */
    public void add(double value) {
        array[head++] = value;
        if (head >= array.length) {
            head = 0;
            wrapped = true;
        }
    }

    /**
     * Gets the value stored in the RingBuffer at the specified index <i>relative to the head</i>.
     * @param index the index from which to get the value stored in the RingBuffer relative to the head
     * @return the value stored in the index relative to the head
     * @throws IllegalArgumentException if index < 0 or index >= getLength()
     */
    public double get(int index) {
        if (index < 0)
            throw new IllegalArgumentException("Index value must be non-negative.");
        if (index >= array.length)
            throw new IllegalArgumentException("Index value was too high; must be less than the length of the RingBuffer.");
        return array[(index + head) % array.length];
    }

    /**
     * Gets the length of the RingBuffer. //TODO: Change documentation
     * @return the length of the RingBuffer
     */
    public int getLength() {
        if (wrapped) return array.length;
        else return head;
    }


}