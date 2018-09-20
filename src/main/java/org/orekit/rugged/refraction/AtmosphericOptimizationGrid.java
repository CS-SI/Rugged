package org.orekit.rugged.refraction;

import org.orekit.rugged.errors.RuggedException;
import org.orekit.rugged.errors.RuggedMessages;
import org.orekit.rugged.linesensor.SensorPixel;

public class AtmosphericOptimizationGrid {

    private double pixelStart = Double.POSITIVE_INFINITY;
    private double lineStart = Double.POSITIVE_INFINITY;
    private int pixelColumns = Integer.MAX_VALUE;
    private int lineRows = Integer.MAX_VALUE;


    public AtmosphericOptimizationGrid(final SensorPixel sensorPixelStart,
                                       final int pixelColumns, final int lineRows) throws RuggedException {

        this.pixelStart = sensorPixelStart.getPixelNumber();
        this.lineStart = sensorPixelStart.getLineNumber();
        this.pixelColumns = pixelColumns;
        this.lineRows = lineRows;
        // TODO change rugged message
        if (this.pixelColumns == 0 || this.lineRows == 0) {
//            throw new RuggedException(RuggedMessages.EMPTY_TILE, this.pixelColumns, this.lineRows);
            throw new RuggedException(RuggedMessages.INTERNAL_ERROR);

        }
    }

    /** Check if the sensorPixel (pixel, line) is inside the grid.
     * @param sensorPixel the sensor pixel
     * @return true if the sensorPixel is inside the grid
     */
    public Boolean isInsideGrid(final SensorPixel sensorPixel) {

        Boolean isInside = true;
        if (sensorPixel.getPixelNumber() < pixelStart || sensorPixel.getPixelNumber() > (pixelStart + pixelColumns) ||
            sensorPixel.getLineNumber() < lineStart || sensorPixel.getLineNumber() > (lineStart + lineRows)) {
            isInside = false;
        }
        return isInside;
    }

    /**
     * @return the pixelStart
     */
    public double getPixelStart() {
        return pixelStart;
    }

    /**
     * @param pixelStart the pixelStart to set
     */
    public void setPixelStart(final double pixelStart) {
        this.pixelStart = pixelStart;
    }

    /**
     * @return the lineStart
     */
    public double getLineStart() {
        return lineStart;
    }

    /**
     * @param lineStart the lineStart to set
     */
    public void setLineStart(final double lineStart) {
        this.lineStart = lineStart;
    }

    /**
     * @return the pixelColumns
     */
    public int getPixelColumns() {
        return pixelColumns;
    }

    /**
     * @param pixelColumns the pixelColumns to set
     */
    public void setPixelColumns(final int pixelColumns) {
        this.pixelColumns = pixelColumns;
    }

    /**
     * @return the lineRows
     */
    public int getLineRows() {
        return lineRows;
    }

    /**
     * @param lineRows the lineRows to set
     */
    public void setLineRows(final int lineRows) {
        this.lineRows = lineRows;
    }
}
