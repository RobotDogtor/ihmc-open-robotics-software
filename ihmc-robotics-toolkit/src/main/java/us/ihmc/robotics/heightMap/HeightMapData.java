package us.ihmc.robotics.heightMap;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.heightMap.HeightMapTools;

import java.util.Arrays;

public class HeightMapData
{
   /* Unordered list of the keys of all occupied cells */
   private final TIntArrayList occupiedCells = new TIntArrayList();
   /* List of heights indexed by key */
   private final double[] heights;

   private final int centerIndex;
   private final int cellsPerAxis;
   private final double gridResolutionXY;
   private final double gridSizeXY;
   private final Point2D gridCenter = new Point2D();
   private double estimatedGroundHeight = Double.NaN;

   private final double minX, maxX, minY, maxY;

   public HeightMapData(double gridResolutionXY, double gridSizeXY, double gridCenterX, double gridCenterY)
   {
      this.gridResolutionXY = gridResolutionXY;
      this.gridSizeXY = gridSizeXY;
      this.centerIndex = HeightMapTools.computeCenterIndex(gridSizeXY, gridResolutionXY);
      this.cellsPerAxis = 2 * centerIndex + 1;
      this.heights = new double[cellsPerAxis * cellsPerAxis];
      this.gridCenter.set(gridCenterX, gridCenterY);

      double epsilon = 1e-8;
      double halfWidth = 0.5 * (gridSizeXY + gridResolutionXY) - epsilon;
      minX = gridCenterX - halfWidth;
      maxX = gridCenterX + halfWidth;
      minY = gridCenterY - halfWidth;
      maxY = gridCenterY + halfWidth;

      reset();
   }

   public void reset()
   {
      occupiedCells.clear();
      Arrays.fill(heights, Double.NaN);
      estimatedGroundHeight = Double.NaN;
   }

   public double getGridResolutionXY()
   {
      return gridResolutionXY;
   }

   public double getGridSizeXY()
   {
      return gridSizeXY;
   }

   public int getNumberOfOccupiedCells()
   {
      return occupiedCells.size();
   }

   public double getHeight(int i)
   {
      return heights[occupiedCells.get(i)];
   }

   public Point2D getCellPosition(int i)
   {
      int key = occupiedCells.get(i);
      return new Point2D(HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), gridResolutionXY, centerIndex),
                         HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), gridResolutionXY, centerIndex));
   }

   /**
    * Returns height at the given (x,y) position, or NaN if there is no height at the given point
    */
   public double getHeightAt(double x, double y)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         return Double.NaN;
      }

      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), gridResolutionXY, centerIndex);
      if (occupiedCells.contains(key))
      {
         return heights[key];
      }
      else
      {
         return estimatedGroundHeight;
      }
   }

   public void setHeightAt(int key, double height)
   {
      if (key >= 0 && key < heights.length)
      {
         if (Double.isNaN(heights[key]))
         {
            occupiedCells.add(key);
         }

         heights[key] = height;
      }
   }

   public void setHeightAt(double x, double y, double z)
   {
      if (!MathTools.intervalContains(x, minX, maxX) || !MathTools.intervalContains(y, minY, maxY))
      {
         return;
      }

      int key = HeightMapTools.coordinateToKey(x, y, gridCenter.getX(), gridCenter.getY(), gridResolutionXY, centerIndex);
      if (Double.isNaN(heights[key]))
      {
         occupiedCells.add(key);
      }

      heights[key] = z;
   }

   public double getHeightAt(int key)
   {
      double height = heights[key];
      return Double.isNaN(height) ? estimatedGroundHeight : height;
   }

   public double getHeightAt(int xIndex, int yIndex)
   {
      if (xIndex < 0 || yIndex < 0 || xIndex >= cellsPerAxis || yIndex >= cellsPerAxis)
      {
         return Double.NaN;
      }

      double height = heights[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)];
      return Double.isNaN(height) ? estimatedGroundHeight : height;
   }

   public boolean isCellAtGroundPlane(int xIndex, int yIndex)
   {
      return Double.isNaN(heights[HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex)]);
   }

   public void setEstimatedGroundHeight(double estimatedGroundHeight)
   {
      this.estimatedGroundHeight = estimatedGroundHeight;
   }

   public double getEstimatedGroundHeight()
   {
      return estimatedGroundHeight;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public Point2D getGridCenter()
   {
      return gridCenter;
   }

   public double getMinHeight()
   {
      double minValue = Double.POSITIVE_INFINITY;
      for (int i = 0; i < heights.length; i++)
      {
         if (!Double.isNaN(heights[i]) && heights[i] < minValue)
            minValue = heights[i];
      }

      return minValue;
   }

   public int getKey(int i)
   {
      return occupiedCells.get(i);
   }

   public void markGroundCell(int i)
   {
      heights[occupiedCells.get(i)] = Double.NaN;
      occupiedCells.remove(i);
   }
}
