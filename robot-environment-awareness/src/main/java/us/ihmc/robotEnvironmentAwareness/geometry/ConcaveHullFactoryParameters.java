package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Scanner;

import us.ihmc.jOctoMap.tools.ScannerTools;

public class ConcaveHullFactoryParameters
{
   private double edgeLengthThreshold;
   private boolean removeAllTrianglesWithTwoBorderEdges;
   private boolean allowSplittingConcaveHull;
   private int maxNumberOfIterations;
   private double triangulationTolerance;

   public ConcaveHullFactoryParameters()
   {
      setDefaultParameters();
   }

   public ConcaveHullFactoryParameters(ConcaveHullFactoryParameters other)
   {
      set(other);
   }

   public void set(ConcaveHullFactoryParameters other)
   {
      edgeLengthThreshold = other.edgeLengthThreshold;
      removeAllTrianglesWithTwoBorderEdges = other.removeAllTrianglesWithTwoBorderEdges;
      allowSplittingConcaveHull = other.allowSplittingConcaveHull;
      maxNumberOfIterations = other.maxNumberOfIterations;
      triangulationTolerance = other.triangulationTolerance;
   }

   public void setDefaultParameters()
   {
      edgeLengthThreshold = 0.10;
      removeAllTrianglesWithTwoBorderEdges = true;
      allowSplittingConcaveHull = true;
      maxNumberOfIterations = 5000;
      triangulationTolerance = 0.0;
   }

   public double getEdgeLengthThreshold()
   {
      return edgeLengthThreshold;
   }

   public boolean doRemoveAllTrianglesWithTwoBorderEdges()
   {
      return removeAllTrianglesWithTwoBorderEdges;
   }

   public boolean isSplittingConcaveHullAllowed()
   {
      return allowSplittingConcaveHull;
   }

   public int getMaxNumberOfIterations()
   {
      return maxNumberOfIterations;
   }

   public double getTriangulationTolerance()
   {
      return triangulationTolerance;
   }

   public void setEdgeLengthThreshold(double edgeLengthThreshold)
   {
      this.edgeLengthThreshold = edgeLengthThreshold;
   }

   public void setRemoveAllTrianglesWithTwoBorderEdges(boolean removeAllTrianglesWithTwoBorderEdges)
   {
      this.removeAllTrianglesWithTwoBorderEdges = removeAllTrianglesWithTwoBorderEdges;
   }

   public void setAllowSplittingConcaveHull(boolean allowSplittingConcaveHull)
   {
      this.allowSplittingConcaveHull = allowSplittingConcaveHull;
   }

   public void setMaxNumberOfIterations(int maxNumberOfIterations)
   {
      this.maxNumberOfIterations = maxNumberOfIterations;
   }

   public void setTriangulationTolerance(double triangulationTolerance)
   {
      this.triangulationTolerance = triangulationTolerance;
   }

   @Override
   public String toString()
   {
      return "edge length threshold: " + edgeLengthThreshold + ", remove any triangle with two borderedges: " + removeAllTrianglesWithTwoBorderEdges
            + ", allow splitting concave hull: " + allowSplittingConcaveHull + ", maximum number of iterations: " + maxNumberOfIterations
            + ", triangulation tolerance: " + triangulationTolerance;
   }

   public void setFromString(String parametersAsString)
   {
      parametersAsString = parametersAsString.replace(",", "");
      Scanner scanner = new Scanner(parametersAsString);
      setEdgeLengthThreshold(ScannerTools.readNextDouble(scanner, getEdgeLengthThreshold()));
      setRemoveAllTrianglesWithTwoBorderEdges(ScannerTools.readNextBoolean(scanner, doRemoveAllTrianglesWithTwoBorderEdges()));
      setAllowSplittingConcaveHull(ScannerTools.readNextBoolean(scanner, isSplittingConcaveHullAllowed()));
      setMaxNumberOfIterations(ScannerTools.readNextInt(scanner, getMaxNumberOfIterations()));
      setTriangulationTolerance(ScannerTools.readNextDouble(scanner, getTriangulationTolerance()));
      scanner.close();
   }

   public static ConcaveHullFactoryParameters parse(String parametersAsString)
   {
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setFromString(parametersAsString);
      return parameters;
   }
}
