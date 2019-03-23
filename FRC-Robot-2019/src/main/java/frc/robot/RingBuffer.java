package frc.robot;

/**
 * A RingBuffer object is essentially an integer array with a head.
 * The head increments every time a new value is incremented.
 * 
 * @author Kevin Li
 */
public class RingBuffer {
    /**
     * The array in which the values are stored
     */
    private int[] array;
    /**
     * The "beginning" of the RingBuffer.
     */
    private int head = 0;
    /**
     * Whether or not the RingBuffer has wrapped all the way to the beginning of the array.
     * Used to figure the length of the RingBuffer.
     */
    private boolean wrapped = false;
    /**
     * The maximum value of the RingBuffer.
     */
    private int max = 0;
    /**
     * The minimum value of the RingBuffer.
     */
    private int min = 0;
    /**
     * Whether or not the maximum and minimum values are updated.
     * Opposite of whether or not <code>compute()</code> is nesessary.
     */
    private boolean justComputed = true;

    /**
     * Creates a RingBuffer object.
     * @param length the length of the RingBuffer
     * @throws IllegalArgumentException if length <= 0
     */
    public RingBuffer(int length) {
        if (length <= 0)
            throw new IllegalArgumentException("Length must be positive.");
        array = new int[length];
    }

    /**
     * Inserts a value to the RingBuffer at index head.
     * @param value the value to be inserted into the RingBuffer
     */
    public void add(int value) {
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
     * Gets the length of the RingBuffer.
     * If the array has wrapped, the full length of the array is returned.
     * Otherwise, the value of the head is returned.
     * @return the length of the RingBuffer
     */
    public int getLength() {
        if (wrapped) return array.length;
        else return head;
    }

    /**
     * Gets the head of the RingBuffer.
     * @return the head
     */
    public int getHead() {
        return head;
    }

    /**
     * Computes and updates the minimum and maximum of all values in the RingBuffer.
     * Using the min or max method (or any method that in any way invokes those methods - e.g. avg)
     * will automatically invoke this method, if necessary.
     */
    public void compute() {
        max = array[0];
        min = array[0];
        for (int i = 1; i < getLength(); i++) {
            if (max < array[i])
                max = array[i];
            if (min < array[i])
                min = array[i];
        }
        justComputed = true;
    }

    /**
     * Gets the minimum value of the RingBuffer.
     * @return the minimum value in the RingBuffer
     */
    public int min() {
        if (!justComputed)
            compute();
        return min;
    }

    /**
     * Gets the maximum value of the RingBuffer.
     * @return the maximum value in the RingBuffer
     */
    public int max() {
        if (!justComputed)
            compute();
        return max;
    }

    /**
     * Gets the sum of all of the values in the RingBuffer.
     * @return the sum of the values in the RingBuffer
     */
    public int total() {
        int sum = 0;
        for (int i = 0; i <= getLength(); i++) {
            if (i >= getLength())
                i = 0;
            sum += array[i];
        }
        return sum;
    }

    /**
     * Gets the mean of all values in the RingBuffer.
     * @return the average of the values in the RingBuffer
     */
    public double avg() {
        if (getLength() == 0)
            return 0;
        else
            return total() / getLength();
    }

    /**
     * Gets the mean of all values in the RingBuffer (ignoring the minimum and maximum).
     * @return the average of the values in the RingBuffer, ignoring the minimum and maximum
     */
    public double olympicAvg() {
        if (getLength() <= 2)
            return 0;
        else
            return (total() - min() - max()) / (getLength() - 2);
    }
}